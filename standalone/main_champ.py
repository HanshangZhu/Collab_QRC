#!/usr/bin/env python3
"""Standalone MuJoCo exploration stack — CHAMP gait variant.

Identical pipeline to main.py except the leg/wheel controller is
ChampController (full trot gait + IK + body-height PD) instead of
the simple PD standing controller in LegController.

Run alongside main.py for A/B comparison:
    python3 standalone/main.py        --headless   # baseline PD+wheels
    python3 standalone/main_champ.py  --headless   # CHAMP trot gait

Usage:
    python3 standalone/main_champ.py [--config path/to/config.yaml] [--headless] [--no-map]

Viewer controls (when not headless):
    Space  — pause/resume physics
    Ctrl+C — clean exit

Map view (shown by default when not headless, disable with --no-map):
    Opens the map PNG in macOS Preview, which auto-refreshes on every
    LiDAR scan (mjpython runs Python in a pthread so Tk/Qt crash; cv2
    renders to a file instead and Preview shows the live result).
    - Gray  = unknown, White = free, Black = obstacle
    - Red dot = robot, green line = A* path, yellow X = active goal
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
from standalone.control.cfpa2_bridge import CFPA2Bridge
from standalone.control.stuck_watchdog import StuckWatchdog
from standalone.observability.metrics_logger import ExplorationMetricsLogger
from standalone.observability.map_writer import MapWriter
from standalone.exploration.cfpa2_single_robot import CFPA2SingleRobotNode

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-5s %(name)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("main_champ")


# ── Config loading ────────────────────────────────────────────────────────────

def load_config(path: str | None) -> dict:
    cfg_path = path or str(_REPO_ROOT / "standalone" / "config.yaml")
    with open(cfg_path) as f:
        return yaml.safe_load(f)


# ── Odometry bridge: publish GT pose to EventBus so CFPA2 sees it ─────────────

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


# ── Live occupancy-map visualiser ─────────────────────────────────────────────
#
# Memmap-backed live viewer. Parent (mjpython, background pthread) writes
# OccupancyGrid + robot pose + path + goal into a memmap via MapWriter; a
# subprocess running standalone.viz.map_viewer (regular python3, own main
# thread) reads the memmap and renders a matplotlib FuncAnimation window.
# This sidesteps the macOS "GUI must be on main thread" crash that hits
# Tk/Qt under mjpython.

def _spawn_map_viewer(memmap_path: Path) -> "subprocess.Popen | None":
    """Launch the child viewer process. Returns None if it can't start."""
    # Use plain python3, not mjpython — child does GUI, doesn't need MuJoCo.
    cmd = [
        sys.executable, "-m", "standalone.viz.map_viewer",
        str(memmap_path),
    ]
    env = os.environ.copy()
    env.setdefault("PYTHONUNBUFFERED", "1")
    try:
        proc = subprocess.Popen(
            cmd,
            cwd=str(_REPO_ROOT),
            env=env,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return proc
    except Exception as exc:
        log.warning(f"[CHAMP] Could not spawn map viewer ({exc}); continuing without it.")
        return None


def publish_nav_status(controller: PurePursuitController, ns: str,
                       goal_seq: int = 0) -> None:
    import json
    state = "goal_reached" if (controller.goal_reached or not controller.has_path) else "navigating"
    msg = String(data=json.dumps({"state": state, "goal_seq": goal_seq}))
    BUS.publish(f"/{ns}/nav_status", msg)


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Standalone MuJoCo exploration — CHAMP gait")
    parser.add_argument("--config", default=None, help="Path to config.yaml")
    parser.add_argument("--headless", action="store_true", help="Run without MuJoCo viewer")
    parser.add_argument("--no-map", action="store_true",
                        help="Disable the live occupancy-map window")
    args = parser.parse_args()

    cfg       = load_config(args.config)
    sim_cfg   = cfg["sim"]
    lidar_cfg = cfg["lidar"]
    map_cfg   = cfg["mapping"]
    nav_cfg   = cfg["nav"]
    cfpa2_cfg = cfg["cfpa2"]
    wd_cfg    = cfg["watchdog"]
    champ_cfg = cfg["champ"]

    ns = cfpa2_cfg["namespace"]

    # ── Scene ────────────────────────────────────────────────────────────────
    scene_rel  = sim_cfg["scene"]
    scene_path = _REPO_ROOT / scene_rel
    if not scene_path.exists():
        log.error(f"MJCF not found: {scene_path}")
        sys.exit(1)
    log.info(f"[CHAMP] Loading scene: {scene_path}")

    # ── Components ───────────────────────────────────────────────────────────
    env    = MuJoCoEnv(scene_path)
    dt     = env.timestep
    params = ChampParams.from_dict(champ_cfg)

    # CHAMP controller — replaces the PD LegController
    leg_ctrl = ChampController(env, params)
    log.info(f"[CHAMP] Controller ready (gait_period={params.gait_period}s, "
             f"swing_height={params.swing_height}m, "
             f"body_height={params.body_height}m)")

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

    mapper = OccupancyMapper(
        resolution=map_cfg["resolution"],
        width_m=map_cfg["width_m"],
        height_m=map_cfg["height_m"],
        origin_x=start_x,
        origin_y=start_y,
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
    if show_map:
        try:
            map_writer = MapWriter(mapper)
            map_viewer_proc = _spawn_map_viewer(map_writer.path)
            if map_viewer_proc is not None:
                log.info(f"[CHAMP] Live map viewer pid={map_viewer_proc.pid} "
                         f"memmap={map_writer.path}")

            def _cleanup_viewer() -> None:
                try:
                    if map_writer is not None:
                        map_writer.close()
                except Exception:
                    pass
                if map_viewer_proc is not None and map_viewer_proc.poll() is None:
                    try:
                        map_viewer_proc.terminate()
                        map_viewer_proc.wait(timeout=1.5)
                    except Exception:
                        try:
                            map_viewer_proc.kill()
                        except Exception:
                            pass
            atexit.register(_cleanup_viewer)
        except Exception as exc:
            log.warning(f"[CHAMP] Map viewer unavailable ({exc}); continuing without it.")
            map_writer = None
            map_viewer_proc = None

    planner    = AStarPlanner(inflation_cells=nav_cfg["planner_inflation_cells"])
    controller = PurePursuitController(
        lookahead_m=nav_cfg["lookahead_m"],
        max_vx=nav_cfg["max_vx"],
        max_wz=nav_cfg["max_wz"],
        goal_tolerance_m=nav_cfg["goal_tolerance_m"],
        heading_gain=nav_cfg["heading_gain"],
    )
    router  = HybridCmdRouter()
    bridge  = CFPA2Bridge(namespace=ns)
    watchdog = StuckWatchdog(
        ns,
        stuck_window_sec=wd_cfg["stuck_window_sec"],
        stuck_threshold_m=wd_cfg["stuck_threshold_m"],
        backup_distance_m=wd_cfg["backup_distance_m"],
        cooldown_sec=wd_cfg["cooldown_sec"],
    )
    metrics = ExplorationMetricsLogger(ns)

    cfpa2 = CFPA2SingleRobotNode()
    cfpa2.set_parameter("namespaces", [ns])
    cfpa2.set_parameter("robot_namespace", ns)
    cfpa2.set_parameter("publish_rate", cfpa2_cfg["publish_rate"])
    cfpa2.set_parameter("sensor_range", cfpa2_cfg["sensor_range"])
    cfpa2.set_parameter("verbose_logs", cfpa2_cfg["verbose_logs"])
    cfpa2.set_parameter("planning_map_topic_suffix", "/map")

    # ── Goal pose → planner bridge + periodic replan ─────────────────────────
    # Original ROS2 default_nav.py uses an AsyncGridPlanner with
    # replan_interval_sec=2.0 + a control-loop tick that requests a fresh
    # plan against the latest map every iteration. Standalone previously
    # ran A* ONCE on goal arrival → once the map grew and revealed walls
    # beyond an unknown region the cached path drove the robot through them.
    # Now: cache active goal, replan at ~1 Hz, force replan whenever the
    # current path is invalidated by new obstacles in the latest grid.
    _current_grid: list = [None]
    _active_goal: list = [None]            # (gx, gy) or None
    _last_replan_t: list = [-1.0]
    _goal_seq: list = [0]                  # increments per new goal
    _fail_count: list = [0]                # consecutive plan failures for active goal
    _last_fail_log_t: list = [-1e9]
    REPLAN_PERIOD_SEC = 1.0
    PATH_CHECK_SAMPLE = 3                  # every Nth waypoint for re-validation
    UNREACHABLE_FAIL_LIMIT = 3             # after N failures, publish state=unreachable
    FAIL_LOG_PERIOD_SEC = 5.0              # de-spam "no path" warnings

    # ── Status pump: drop active goal + refresh viewer on CFPA2 state ───────
    def _on_status(msg) -> None:
        state = getattr(msg, "data", "") or ""
        if map_writer is not None:
            try:
                map_writer.set_status(state)
            except Exception:
                pass
        # If CFPA2 declares no_frontiers / paused, drop the stale active
        # goal so the planner stops trying to reach it.
        if state in ("no_frontiers", "paused"):
            if _active_goal[0] is not None:
                _active_goal[0] = None
                controller.clear()
                if map_writer is not None:
                    try:
                        map_writer.clear_path()
                        map_writer.clear_goal()
                    except Exception:
                        pass
    BUS.subscribe(f"/{ns}/exploration_status", _on_status)

    def _publish_unreachable(gx: float, gy: float) -> None:
        """Signal CFPA2 fast-blacklist on /<ns>/nav_status (state=unreachable)."""
        import json
        payload = {
            "state": "unreachable",
            "goal_seq": _goal_seq[0],
            "goal_x": gx,
            "goal_y": gy,
        }
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
            log.info(f"[CHAMP] [{reason}] path to ({gx:.2f},{gy:.2f}) — {len(path)} wps")
            _fail_count[0] = 0
            return True
        _fail_count[0] += 1
        now = env.time
        if now - _last_fail_log_t[0] >= FAIL_LOG_PERIOD_SEC or _fail_count[0] == 1:
            log.warning(f"[CHAMP] [{reason}] no path to ({gx:.2f},{gy:.2f}) "
                        f"(fails={_fail_count[0]})")
            _last_fail_log_t[0] = now
        if _fail_count[0] >= UNREACHABLE_FAIL_LIMIT and _active_goal[0] is not None:
            log.warning(f"[CHAMP] goal ({gx:.2f},{gy:.2f}) unreachable after "
                        f"{_fail_count[0]} attempts — telling CFPA2 to blacklist")
            _publish_unreachable(gx, gy)
            _active_goal[0] = None
            controller.clear()
            if map_writer is not None:
                map_writer.clear_path()
                map_writer.clear_goal()
        return False

    def _path_blocked(grid, path) -> bool:
        """Return True if any waypoint on the active path is now occupied."""
        if grid is None or not path:
            return False
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        W = grid.info.width
        H = grid.info.height
        data = grid.data
        for i, (wx, wy) in enumerate(path):
            if i % PATH_CHECK_SAMPLE:
                continue
            cx = int((wx - ox) / res)
            cy = int((wy - oy) / res)
            if 0 <= cx < W and 0 <= cy < H:
                v = data[cy * W + cx]
                if v >= 50:        # OCC_THRESHOLD == 50 (planner.py)
                    return True
        return False

    def _on_goal(msg) -> None:
        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)
        # Treat as new only if it differs from active by ≥ 0.3 m; CFPA2
        # re-publishes the same frontier on every tick.
        prev = _active_goal[0]
        if prev is not None:
            dx = gx - prev[0]; dy = gy - prev[1]
            if math.hypot(dx, dy) < 0.30:
                return
        _active_goal[0] = (gx, gy)
        _goal_seq[0] += 1
        _fail_count[0] = 0
        _last_fail_log_t[0] = -1e9
        if _plan_and_apply(gx, gy, "new_goal"):
            _last_replan_t[0] = env.time

    BUS.subscribe(f"/{ns}/goal_pose", _on_goal)

    def _maybe_replan() -> None:
        """Called at ~lidar tick (5 Hz). Replans if periodic timer fires or
        if current path is now blocked by a freshly-discovered obstacle.
        _plan_and_apply handles unreachable-goal escalation internally."""
        goal = _active_goal[0]
        if goal is None:
            return
        grid = _current_grid[0]
        if grid is None:
            return
        if controller.goal_reached:
            return
        now = env.time
        path = list(controller._path)
        blocked = _path_blocked(grid, path)
        elapsed = now - _last_replan_t[0]
        if blocked or elapsed >= REPLAN_PERIOD_SEC:
            reason = "blocked" if blocked else "periodic"
            ok = _plan_and_apply(goal[0], goal[1], reason)
            if ok:
                _last_replan_t[0] = now
            elif blocked and _active_goal[0] is not None:
                # Path is now blocked but goal not yet flagged unreachable.
                # Stop controller so robot doesn't ram the new obstacle;
                # next replan attempt within REPLAN_PERIOD_SEC may succeed.
                controller.clear()
                if map_writer is not None:
                    map_writer.clear_path()

    # ── Watchdog backup cmd_vel override ─────────────────────────────────────
    _backup_cmd: list = [None]

    def _on_cmd_vel(msg) -> None:
        _backup_cmd[0] = (msg.linear.x, msg.linear.y, msg.angular.z)

    BUS.subscribe(f"/{ns}/cmd_vel", _on_cmd_vel)

    # ── Timing ───────────────────────────────────────────────────────────────
    lidar_period     = 1.0 / lidar_cfg["publish_rate_hz"]
    odom_period      = 0.05    # 20 Hz
    watchdog_period  = 0.50    # 2 Hz
    t_last_lidar     = -lidar_period
    t_last_odom      = -odom_period
    t_last_watchdog  = -watchdog_period
    t_last_metrics   = -1.0

    log.info("[CHAMP] Simulation started (1.5s settle → trot). Ctrl+C to exit.")

    # ── Simulation loop ───────────────────────────────────────────────────────
    def run_loop(viewer=None) -> None:
        nonlocal t_last_lidar, t_last_odom, t_last_watchdog, t_last_metrics

        while True:
            t = env.time

            # 1. Odometry (20 Hz)
            if t - t_last_odom >= odom_period:
                publish_odom(env, ns)
                pose = env.get_pose2d()
                TF.update("robot", pose, t)
                t_last_odom = t

            # 2. LiDAR + mapping
            if t - t_last_lidar >= lidar_period:
                pose   = env.get_pose2d()
                pts    = lidar.scan()
                body_z = env.get_pose3d()[2]
                mapper.update(pts, pose, robot_z=body_z)
                grid = mapper.to_occupancy_grid()
                _current_grid[0] = grid
                publish_map(mapper, ns)
                if map_writer is not None:
                    map_writer.tick(pose)
                # Replan against the fresh grid (every ~1 s, or sooner if
                # the cached path crosses a newly-discovered obstacle).
                _maybe_replan()
                # If controller declared goal reached, drop the cached goal so
                # the next CFPA2 publish triggers _on_goal again.
                if controller.goal_reached:
                    if _active_goal[0] is not None:
                        _active_goal[0] = None
                        if map_writer is not None:
                            map_writer.clear_goal()
                            map_writer.clear_path()
                t_last_lidar = t

            # 3. CFPA2 timers (event-driven via TIMERS registry)
            TIMERS.tick(t)

            # 4. Nav status → CFPA2
            publish_nav_status(controller, ns, goal_seq=_goal_seq[0])

            # 5. Controller → router → CHAMP
            pose   = env.get_pose2d()
            backup = _backup_cmd[0]
            if backup is not None:
                vx, vy, wz = backup
                wheel_mode  = True   # watchdog backup always uses wheel drive
                _backup_cmd[0] = None
            else:
                vx, wz = controller.compute(pose)
                router.set_cmd_vel(vx, 0.0, wz)
                # Honor router's wheel_mode flag — matches original ROS2
                # go2w_hybrid_cmd_router: cruise = wheel mode (legs locked,
                # wheels drive); trot only for low-speed pivots / idle.
                # Hardcoding wheel_mode=False forces always-trot, which
                # freewheels wheels against condim=6 friction="0.8 0.02 0.01"
                # → 88% slip at cruise. See docs/claude/champ_fixes_plan.md.
                vx, wz, wheel_mode = router.tick()
                vy = 0.0

            leg_ctrl.set_cmd_vel(vx, vy, wz, wheel_mode)

            # 6. CHAMP step + physics
            leg_ctrl.step()
            env.step()

            # 7. Watchdog (2 Hz)
            if t - t_last_watchdog >= watchdog_period:
                watchdog.tick()
                t_last_watchdog = t

            # 8. Metrics + stop trigger (2 Hz)
            if t - t_last_metrics >= 0.5:
                done = metrics.tick()
                if done:
                    log.info("[CHAMP] Exploration complete — stopping.")
                    leg_ctrl.set_cmd_vel(0.0, 0.0, 0.0, False)
                    # Apply zero cmd to physics so the robot halts, then exit.
                    for _ in range(50):
                        leg_ctrl.step()
                        env.step()
                    break
                t_last_metrics = t

            # 9. Viewer
            if viewer is not None:
                viewer.sync()
                if not viewer.is_running():
                    break

    if args.headless:
        try:
            run_loop(viewer=None)
        except KeyboardInterrupt:
            log.info("[CHAMP] Interrupted — exiting.")
    else:
        with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
            try:
                run_loop(viewer=viewer)
            except KeyboardInterrupt:
                log.info("[CHAMP] Interrupted — exiting.")


if __name__ == "__main__":
    main()
