#!/usr/bin/env python3
"""Standalone MuJoCo exploration — VLM goal source.

Identical to main_champ.py except the goal source is a VLM (Vision-Language
Model) instead of the CFPA2 frontier allocator. A* planner + pure-pursuit
controller + CHAMP gait + replan/blacklist all stay the same.

Usage:
    mjpython standalone/main_vlm.py [--config path] [--headless] [--no-map]
                                    [--provider auto|xai|openai|anthropic|mock]
                                    [--model MODEL]
                                    [--mission "find blue boxes"]
                                    [--cycle-sec 12.0]

Env vars for API keys:
    GEMINI_API_KEY     (Google AI Studio — Gemma / Gemini; .env.gemini supported)
    XAI_API_KEY        (Grok via x.ai)
    OPENAI_API_KEY     (GPT-4o, etc.)
    ANTHROPIC_API_KEY  (Claude)

If no key is set, provider auto-resolves to 'mock' (picks highest-info
frontier deterministically — useful for offline testing).

Compare with `mjpython standalone/main_champ.py` (CFPA2-driven baseline)
for VLM-vs-traditional ablations.
"""
from __future__ import annotations

import argparse
import atexit
import logging
import math
import os
import subprocess
import sys
import time
from pathlib import Path

_REPO_ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(_REPO_ROOT))
sys.path.insert(0, str(_REPO_ROOT / "standalone"))

import yaml
import numpy as np
import mujoco
import mujoco.viewer

from standalone.core.bus import BUS, TIMERS
from standalone.core.ros_compat import OccupancyGrid, Odometry, String, now_stamp
from standalone.sim.mujoco_env import MuJoCoEnv
from standalone.sim.champ_controller import ChampController, ChampParams
from standalone.sim.sensor.lidar import Lidar
from standalone.mapping.occupancy_grid import OccupancyMapper
from standalone.nav.tf_manager import TF
from standalone.nav.planner import AStarPlanner
from standalone.nav.controller import PurePursuitController
from standalone.control.hybrid_cmd_router import HybridCmdRouter
from standalone.control.stuck_watchdog import StuckWatchdog
from standalone.observability.metrics_logger import ExplorationMetricsLogger
from standalone.observability.map_writer import MapWriter
from standalone.vlm.explorer import VLMExplorer

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-5s %(name)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("main_vlm")


def load_config(path: str | None) -> dict:
    cfg_path = path or str(_REPO_ROOT / "standalone" / "config.yaml")
    with open(cfg_path) as f:
        return yaml.safe_load(f)


def publish_odom(env: MuJoCoEnv, ns: str) -> None:
    x, y, yaw = env.get_pose2d()
    msg = Odometry()
    msg.header.frame_id = "map"
    msg.pose.pose.position.x = x
    msg.pose.pose.position.y = y
    msg.pose.pose.orientation.z = math.sin(0.5 * yaw)
    msg.pose.pose.orientation.w = math.cos(0.5 * yaw)
    BUS.publish(f"/{ns}/odom/nav", msg)


def publish_map(mapper: OccupancyMapper, ns: str) -> None:
    grid = mapper.to_occupancy_grid()
    BUS.publish(f"/{ns}/map", grid)


def _spawn_map_viewer(memmap_path: Path) -> "subprocess.Popen | None":
    cmd = [sys.executable, "-m", "standalone.viz.map_viewer", str(memmap_path)]
    env = os.environ.copy()
    env.setdefault("PYTHONUNBUFFERED", "1")
    try:
        return subprocess.Popen(
            cmd, cwd=str(_REPO_ROOT), env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    except Exception as exc:
        log.warning(f"[VLM] Map viewer spawn failed: {exc}")
        return None


def _spawn_vlm_dashboard(session_dir: Path,
                          camera_png: Path) -> "subprocess.Popen | None":
    """Launch the two-panel VLM dashboard (map render + camera) as a subprocess."""
    cmd = [
        sys.executable, "-m", "standalone.viz.vlm_dashboard",
        str(session_dir), str(camera_png),
    ]
    env = os.environ.copy()
    env.setdefault("PYTHONUNBUFFERED", "1")
    try:
        return subprocess.Popen(
            cmd, cwd=str(_REPO_ROOT), env=env,
            stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL,
        )
    except Exception as exc:
        log.warning(f"[VLM] Dashboard spawn failed: {exc}")
        return None


def publish_nav_status(controller: PurePursuitController, ns: str,
                       goal_seq: int = 0, active_goal=None) -> None:
    import json
    # Only publish "goal_reached" when the controller actually signalled
    # arrival at a goal we were navigating to. "no path" is NOT goal_reached
    # — it's just idle, which VLMExplorer should not treat as a trigger.
    if controller.goal_reached and active_goal is not None:
        state = "goal_reached"
    elif not controller.has_path:
        state = "idle"
    else:
        state = "navigating"
    msg = String(data=json.dumps({"state": state, "goal_seq": goal_seq}))
    BUS.publish(f"/{ns}/nav_status", msg)


def main() -> None:
    parser = argparse.ArgumentParser(description="Standalone MuJoCo exploration — VLM goal source")
    parser.add_argument("--config", default=None, help="Path to config.yaml")
    parser.add_argument("--headless", action="store_true")
    parser.add_argument("--no-map", action="store_true",
                        help="Disable live occupancy map subprocess")
    parser.add_argument("--provider", default="auto",
                        choices=["auto", "xai", "openai", "anthropic",
                                 "google", "gemini", "groq", "mock"])
    parser.add_argument("--model", default="",
                        help="Override provider default model")
    parser.add_argument("--mission", default="",
                        help="Override default mission prompt")
    parser.add_argument("--cycle-sec", type=float, default=12.0,
                        help="VLM query period (seconds)")
    parser.add_argument("--timeout-sec", type=float, default=25.0)
    parser.add_argument("--strategy", default="mock",
                        choices=["mock", "info_gain", "random", "greedy_nearest"],
                        help="Goal selection strategy when provider=mock "
                             "(random | greedy_nearest | mock/info_gain)")
    # Ablation flags (Pri 4/6/7) — strip optional inputs to the VLM to
    # quantify per-component contribution.
    parser.add_argument("--no-info-gain", action="store_true",
                        help="Strip 'info_gain' field from each frontier in scene JSON")
    parser.add_argument("--no-camera", action="store_true",
                        help="Skip robot front-camera image in VLM input")
    parser.add_argument("--no-history", action="store_true",
                        help="Skip exploration history injection in VLM prompt")
    args = parser.parse_args()

    cfg = load_config(args.config)
    sim_cfg   = cfg["sim"]
    lidar_cfg = cfg["lidar"]
    map_cfg   = cfg["mapping"]
    nav_cfg   = cfg["nav"]
    wd_cfg    = cfg["watchdog"]
    champ_cfg = cfg["champ"]
    cfpa2_cfg = cfg.get("cfpa2", {"namespace": "robot"})

    ns = cfpa2_cfg.get("namespace", "robot")

    # ── Scene ────────────────────────────────────────────────────────────────
    scene_path = _REPO_ROOT / sim_cfg["scene"]
    if not scene_path.exists():
        log.error(f"MJCF not found: {scene_path}")
        sys.exit(1)
    log.info(f"[VLM] Loading scene: {scene_path}")

    env = MuJoCoEnv(scene_path)
    dt = env.timestep
    params = ChampParams.from_dict(champ_cfg)
    leg_ctrl = ChampController(env, params)

    lidar = Lidar(
        env,
        hz_samples=lidar_cfg["hz_samples"],
        vt_samples=lidar_cfg["vt_samples"],
        h_fov_deg=lidar_cfg["h_fov_deg"],
        v_min_deg=lidar_cfg["v_min_deg"],
        v_max_deg=lidar_cfg["v_max_deg"],
        range_min=lidar_cfg["range_min"],
        range_max=lidar_cfg["range_max"],
    )

    env.forward()
    start_x, start_y, _ = env.get_pose2d()
    # Map centre: use explicit config value when provided (scene-specific),
    # otherwise fall back to robot spawn so small single-room scenes still work.
    map_cx = map_cfg.get("map_center_x", start_x)
    map_cy = map_cfg.get("map_center_y", start_y)

    mapper = OccupancyMapper(
        resolution=map_cfg["resolution"],
        width_m=map_cfg["width_m"],
        height_m=map_cfg["height_m"],
        origin_x=map_cx,
        origin_y=map_cy,
        hit_log_odds=map_cfg["hit_log_odds"],
        miss_log_odds=map_cfg["miss_log_odds"],
        clip_min=map_cfg["clip_min"],
        clip_max=map_cfg["clip_max"],
        occ_threshold=map_cfg["occ_threshold"],
        free_threshold=map_cfg["free_threshold"],
        max_scan_height=map_cfg["max_scan_height"],
    )

    show_map = (not args.headless) and (not args.no_map)
    map_writer: MapWriter | None = None
    map_viewer_proc: subprocess.Popen | None = None
    dashboard_proc: subprocess.Popen | None = None
    # Live camera PNG path — written every ~2 s in the sim loop so the
    # dashboard subprocess can display the robot's current POV.
    _live_camera_png = Path("/tmp/vlm_robot_pov.png")
    _last_camera_save_t: list[float] = [-1e9]
    _CAMERA_SAVE_INTERVAL = 2.0  # seconds between camera PNG saves

    if show_map:
        try:
            map_writer = MapWriter(mapper)
            map_viewer_proc = _spawn_map_viewer(map_writer.path)
            if map_viewer_proc is not None:
                log.info(f"[VLM] Live map viewer pid={map_viewer_proc.pid}")
        except Exception as exc:
            log.warning(f"[VLM] Map viewer unavailable: {exc}")
            map_writer = None

    planner = AStarPlanner(inflation_cells=nav_cfg["planner_inflation_cells"])
    controller = PurePursuitController(
        lookahead_m=nav_cfg["lookahead_m"],
        max_vx=nav_cfg["max_vx"],
        max_wz=nav_cfg["max_wz"],
        goal_tolerance_m=nav_cfg["goal_tolerance_m"],
        heading_gain=nav_cfg["heading_gain"],
    )
    router = HybridCmdRouter()
    watchdog = StuckWatchdog(
        ns,
        stuck_window_sec=wd_cfg["stuck_window_sec"],
        stuck_threshold_m=wd_cfg["stuck_threshold_m"],
        backup_distance_m=wd_cfg["backup_distance_m"],
        cooldown_sec=wd_cfg["cooldown_sec"],
    )
    # VLM cycle = 12 s; coverage stagnation needs a longer window than
    # CFPA2's default 30 s or it kills the run after 2-3 VLM cycles.
    metrics = ExplorationMetricsLogger(
        ns,
        coverage_stagnant_window_sec=120.0,
        consec_no_reachable_threshold=6,
        run_name=(f"vlm_{args.provider}"
                  if args.provider != "mock"
                  else f"vlm_mock_{args.strategy}"),
    )

    # ── Camera capture function (renders front_camera from main thread) ──────
    def _capture_camera() -> "str | None":
        """Render the robot's front_camera to a base64 PNG string.

        Called from the main sim loop before dispatching the background
        VLM thread. mujoco.Renderer creates its own OpenGL context —
        safe on the main thread, not safe from background threads.
        """
        import base64
        rgb = env.render_camera("front_camera", width=320, height=240)
        if rgb is None:
            return None
        from standalone.vlm.renderer import MapRenderer
        png_bytes = MapRenderer.rgb_array_to_png(rgb)
        return base64.b64encode(png_bytes).decode("ascii")

    def _save_live_camera() -> None:
        """Save robot camera to a shared PNG so the dashboard subprocess can read it."""
        from standalone.vlm.renderer import MapRenderer
        rgb = env.render_camera("front_camera", width=320, height=240)
        if rgb is None:
            return
        png_bytes = MapRenderer.rgb_array_to_png(rgb)
        tmp = Path(str(_live_camera_png) + ".tmp")
        tmp.write_bytes(png_bytes)
        tmp.replace(_live_camera_png)  # atomic rename avoids partial-read by dashboard

    # ── VLM explorer (replaces CFPA2 node) ───────────────────────────────────
    vlm_explorer = VLMExplorer(
        mapper,
        namespace=ns,
        provider=args.provider,
        model=args.model,
        cycle_period_sec=args.cycle_sec,
        timeout_sec=args.timeout_sec,
        mission=(args.mission or (
            "Explore the entire environment. While navigating, look for distinctly "
            "coloured spheres, boxes, and cylinders placed around the scene: "
            "a RED sphere, a BLUE box, a YELLOW cylinder, and a GREEN sphere. "
            "When you spot one in the camera image, call log_artifact() immediately "
            "with its world (x, y) position and colour/shape description."
        )),
        planner_inflation_cells=nav_cfg["planner_inflation_cells"],
        planner=planner,
        camera_fn=None if args.no_camera else _capture_camera,
        strategy=args.strategy,
        strip_info_gain=args.no_info_gain,
        disable_history=args.no_history,
    )
    log.info(f"[VLM] explorer ready: provider={vlm_explorer.provider} "
             f"model={vlm_explorer.model} cycle={args.cycle_sec:.1f}s")

    # ── VLM dashboard (robot POV + VLM map render) ────────────────────────────
    if show_map:
        try:
            dashboard_proc = _spawn_vlm_dashboard(
                vlm_explorer._log_dir, _live_camera_png
            )
            if dashboard_proc is not None:
                log.info(f"[VLM] Dashboard pid={dashboard_proc.pid}  "
                         f"session={vlm_explorer._log_dir}")

            def _cleanup_all_viewers() -> None:
                # Write closed.flag so the dashboard exits cleanly.
                try:
                    (vlm_explorer._log_dir / "closed.flag").touch()
                except Exception:
                    pass
                for proc in (map_viewer_proc, dashboard_proc):
                    if proc is not None and proc.poll() is None:
                        try:
                            proc.terminate()
                            proc.wait(timeout=1.5)
                        except Exception:
                            try: proc.kill()
                            except Exception: pass
                if map_writer is not None:
                    try: map_writer.close()
                    except Exception: pass
            atexit.register(_cleanup_all_viewers)
        except Exception as exc:
            log.warning(f"[VLM] Dashboard unavailable: {exc}")

    # ── Replan + active-goal state (mirrors main_champ.py Step D/E) ──────────
    _current_grid: list = [None]
    _active_goal: list = [None]
    _last_replan_t: list = [-1.0]
    _goal_seq: list = [0]
    _fail_count: list = [0]
    _last_fail_log_t: list = [-1e9]
    REPLAN_PERIOD_SEC = 1.0
    PATH_CHECK_SAMPLE = 3
    UNREACHABLE_FAIL_LIMIT = 3
    FAIL_LOG_PERIOD_SEC = 5.0

    def _on_status(msg) -> None:
        state = getattr(msg, "data", "") or ""
        if map_writer is not None:
            try: map_writer.set_status(state)
            except Exception: pass
    BUS.subscribe(f"/{ns}/exploration_status", _on_status)

    def _publish_unreachable(gx: float, gy: float) -> None:
        import json
        payload = {"state": "unreachable", "goal_seq": _goal_seq[0],
                   "goal_x": gx, "goal_y": gy}
        BUS.publish(f"/{ns}/nav_status", String(data=json.dumps(payload)))

    def _plan_and_apply(gx: float, gy: float, reason: str) -> bool:
        pose = env.get_pose2d()
        grid = _current_grid[0]
        if grid is None:
            return False
        path = planner.plan(grid, (pose[0], pose[1]), (gx, gy))
        if path:
            controller.set_path(path)
            if map_writer is not None:
                map_writer.set_path(path)
                map_writer.set_goal(gx, gy)
            log.info(f"[VLM] [{reason}] path to ({gx:.2f},{gy:.2f}) — {len(path)} wps")
            _fail_count[0] = 0
            return True
        _fail_count[0] += 1
        now = env.time
        if now - _last_fail_log_t[0] >= FAIL_LOG_PERIOD_SEC or _fail_count[0] == 1:
            log.warning(f"[VLM] [{reason}] no path to ({gx:.2f},{gy:.2f}) (fails={_fail_count[0]})")
            _last_fail_log_t[0] = now
        # Two paths to unreachable:
        #   - reason == "new_goal" AND first attempt failed: goal is bad from
        #     the start (VLM picked something A* refuses). Tell VLM right away
        #     so the next cycle picks a different point instead of cycling.
        #   - reason == "periodic" with N≥UNREACHABLE_FAIL_LIMIT failures:
        #     goal was good initially but the map grew a wall mid-flight.
        force_unreachable = (reason == "new_goal")
        threshold_hit = _fail_count[0] >= UNREACHABLE_FAIL_LIMIT
        if (force_unreachable or threshold_hit) and _active_goal[0] is not None:
            log.warning(f"[VLM] goal ({gx:.2f},{gy:.2f}) unreachable — clearing")
            _publish_unreachable(gx, gy)
            _active_goal[0] = None
            controller.clear()
            if map_writer is not None:
                map_writer.clear_path(); map_writer.clear_goal()
        return False

    def _path_blocked(grid, path) -> bool:
        if grid is None or not path:
            return False
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        W = grid.info.width; H = grid.info.height
        data = grid.data
        for i, (wx, wy) in enumerate(path):
            if i % PATH_CHECK_SAMPLE:
                continue
            cx = int((wx - ox) / res); cy = int((wy - oy) / res)
            if 0 <= cx < W and 0 <= cy < H:
                if data[cy * W + cx] >= 50:
                    return True
        return False

    def _on_goal(msg) -> None:
        gx = float(msg.pose.position.x); gy = float(msg.pose.position.y)
        prev = _active_goal[0]
        if prev is not None and math.hypot(gx - prev[0], gy - prev[1]) < 0.30:
            return
        _active_goal[0] = (gx, gy)
        _goal_seq[0] += 1
        _fail_count[0] = 0
        _last_fail_log_t[0] = -1e9
        if _plan_and_apply(gx, gy, "new_goal"):
            _last_replan_t[0] = env.time

    BUS.subscribe(f"/{ns}/goal_pose", _on_goal)

    def _maybe_replan() -> None:
        goal = _active_goal[0]
        if goal is None or _current_grid[0] is None:
            return
        if controller.goal_reached:
            return
        now = env.time
        blocked = _path_blocked(_current_grid[0], list(controller._path))
        if blocked or now - _last_replan_t[0] >= REPLAN_PERIOD_SEC:
            reason = "blocked" if blocked else "periodic"
            ok = _plan_and_apply(goal[0], goal[1], reason)
            if ok:
                _last_replan_t[0] = now
            elif blocked and _active_goal[0] is not None:
                controller.clear()
                if map_writer is not None:
                    map_writer.clear_path()

    # ── Watchdog backup cmd_vel override ─────────────────────────────────────
    _backup_cmd: list = [None]
    def _on_cmd_vel(msg) -> None:
        _backup_cmd[0] = (msg.linear.x, msg.linear.y, msg.angular.z)
    BUS.subscribe(f"/{ns}/cmd_vel", _on_cmd_vel)

    # ── Timing ───────────────────────────────────────────────────────────────
    lidar_period    = 1.0 / lidar_cfg["publish_rate_hz"]
    odom_period     = 0.05
    watchdog_period = 0.50
    vlm_tick_period = 0.50    # 2 Hz — checks if VLM cycle ready or finished
    camera_period   = 2.0     # seconds between live camera PNG saves for dashboard
    t_last_lidar    = -lidar_period
    t_last_odom     = -odom_period
    t_last_watchdog = -watchdog_period
    t_last_metrics  = -1.0
    t_last_vlm      = -vlm_tick_period
    t_last_camera   = -camera_period

    log.info("[VLM] Simulation started. Ctrl+C to exit.")

    def run_loop(viewer=None) -> None:
        nonlocal t_last_lidar, t_last_odom, t_last_watchdog, t_last_metrics, t_last_vlm, t_last_camera

        while True:
            t = env.time

            if t - t_last_odom >= odom_period:
                publish_odom(env, ns)
                pose = env.get_pose2d()
                TF.update("robot", pose, t)
                t_last_odom = t

            if t - t_last_lidar >= lidar_period:
                pose = env.get_pose2d()
                pts = lidar.scan()
                body_z = env.get_pose3d()[2]
                mapper.update(pts, pose, robot_z=body_z)
                grid = mapper.to_occupancy_grid()
                _current_grid[0] = grid
                publish_map(mapper, ns)
                if map_writer is not None:
                    map_writer.tick(pose)
                _maybe_replan()
                if controller.goal_reached:
                    if _active_goal[0] is not None:
                        _active_goal[0] = None
                        if map_writer is not None:
                            map_writer.clear_goal(); map_writer.clear_path()
                t_last_lidar = t

            # CFPA2 TIMERS not used. VLM has its own tick cadence.
            if t - t_last_vlm >= vlm_tick_period:
                vlm_explorer.tick(t)
                t_last_vlm = t

            # Live camera PNG for dashboard (2 Hz, only when show_map is on).
            if show_map and t - t_last_camera >= camera_period:
                try:
                    _save_live_camera()
                except Exception:
                    pass
                t_last_camera = t

            publish_nav_status(controller, ns, goal_seq=_goal_seq[0],
                               active_goal=_active_goal[0])

            pose = env.get_pose2d()
            backup = _backup_cmd[0]
            if backup is not None:
                vx, vy, wz = backup
                wheel_mode = True
                _backup_cmd[0] = None
            else:
                vx, wz = controller.compute(pose)
                router.set_cmd_vel(vx, 0.0, wz)
                vx, wz, wheel_mode = router.tick()
                vy = 0.0

            leg_ctrl.set_cmd_vel(vx, vy, wz, wheel_mode)
            leg_ctrl.step()
            env.step()

            if t - t_last_watchdog >= watchdog_period:
                watchdog.tick()
                t_last_watchdog = t

            if t - t_last_metrics >= 0.5:
                done = metrics.tick(sim_time=t)
                if done:
                    log.info("[VLM] Exploration complete — stopping.")
                    leg_ctrl.set_cmd_vel(0.0, 0.0, 0.0, False)
                    for _ in range(50):
                        leg_ctrl.step(); env.step()
                    break
                t_last_metrics = t

            if viewer is not None:
                viewer.sync()
                if not viewer.is_running():
                    break

    if args.headless:
        try: run_loop(viewer=None)
        except KeyboardInterrupt:
            log.info("[VLM] Interrupted.")
            metrics.flush_timeseries()
    else:
        with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
            try: run_loop(viewer=viewer)
            except KeyboardInterrupt:
                log.info("[VLM] Interrupted.")
                metrics.flush_timeseries()

    # Final artifact summary
    arts = vlm_explorer.artifacts
    if arts:
        log.info(f"[VLM] {len(arts)} artifacts logged:")
        for a in arts:
            log.info(f"  cycle={a['cycle']} pos=({a['pos'][0]:.2f},{a['pos'][1]:.2f}) "
                     f"reason={a['reason'][:70]!r}")
    else:
        log.info("[VLM] No artifacts reported by VLM.")

    # ── Comparison report ────────────────────────────────────────────────────
    import json as _json
    t_end = env.time
    known_cells = int(np.sum(mapper._observed))
    known_m2 = round(known_cells * (mapper.resolution ** 2), 2)
    free_cells = int(np.sum(mapper._observed & (mapper._log_odds <= mapper._free_thr)))
    occ_cells  = int(np.sum(mapper._observed & (mapper._log_odds >= mapper._occ_thr)))
    free_m2 = round(free_cells * (mapper.resolution ** 2), 2)
    # known_m2 is the primary comparison metric: total observed area (free + walls).
    # Don't compute a percentage here — the map grid is 28×20=560m² but demo3 scene
    # is 24×16=384m², so any denominator choice is misleading. Compare runs by known_m2.
    # Provider-aware run label so report aggregation can group by condition.
    _prov = vlm_explorer.provider
    if _prov == "mock":
        _run_label = f"mock_{args.strategy}"
    else:
        _run_label = f"vlm_{_prov}"
    # Ablation suffixes for run_label so different conditions don't overwrite each other.
    _abl = []
    if args.no_info_gain: _abl.append("noig")
    if args.no_camera:    _abl.append("nocam")
    if args.no_history:   _abl.append("nohist")
    if _abl:
        _run_label = f"{_run_label}_" + "_".join(_abl)
    report = {
        "run": _run_label,
        "provider": _prov,
        "model": vlm_explorer.model,
        "strategy": args.strategy,
        "ablations": {"no_info_gain": args.no_info_gain,
                      "no_camera": args.no_camera,
                      "no_history": args.no_history},
        "scene": str(scene_path.name),
        "cycle_sec": float(args.cycle_sec),
        "sim_time_sec": round(t_end, 1),
        "known_m2": known_m2,
        "free_m2": free_m2,
        "wall_m2": round(occ_cells * (mapper.resolution ** 2), 2),
        "goals_published": _goal_seq[0],
        "vlm_cycles": vlm_explorer._cycle_counter,
        "artifacts_found": len(arts),
        "artifacts": [{"pos": a["pos"], "desc": a["reason"][:80]} for a in arts],
    }
    # Honour STANDALONE_RESULTS_DIR when set so batch runners can collect
    # reports directly; fall back to /tmp for ad-hoc runs.
    _report_root = Path(os.environ.get("STANDALONE_RESULTS_DIR", "/tmp"))
    _report_root.mkdir(parents=True, exist_ok=True)
    report_path = _report_root / f"{_run_label}_report_{time.strftime('%Y%m%d_%H%M%S')}.json"
    report_path.write_text(_json.dumps(report, indent=2))
    log.info(f"[VLM] Report saved → {report_path}")
    log.info(f"[VLM] known={known_m2}m²  free={free_m2}m²"
             f"  goals={report['goals_published']}  cycles={report['vlm_cycles']}"
             f"  artifacts={report['artifacts_found']}  sim_time={report['sim_time_sec']}s")


if __name__ == "__main__":
    main()
