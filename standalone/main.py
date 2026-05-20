#!/usr/bin/env python3
"""Standalone MuJoCo exploration stack — no ROS2 required.

Phases wired together in a single sim loop:
  - MuJoCo physics + GT pose
  - LiDAR raycast → 2D occupancy map
  - CFPA2 frontier exploration → waypoints
  - A* planner + pure-pursuit controller
  - Hybrid cmd router → leg PD + wheel velocity commands

Usage:
    python3 standalone/main.py [--config path/to/config.yaml] [--headless]

Viewer controls (when not headless):
    Space  — pause/resume physics
    Ctrl+C — clean exit
"""
from __future__ import annotations

import argparse
import logging
import math
import os
import sys
import time
from pathlib import Path

# ── repo root on sys.path ─────────────────────────────────────────────────────
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
from standalone.sim.leg_controller import LegController
from standalone.sim.sensor.lidar import Lidar
from standalone.mapping.occupancy_grid import OccupancyMapper
from standalone.nav.tf_manager import TF
from standalone.nav.planner import AStarPlanner
from standalone.nav.controller import PurePursuitController
from standalone.control.hybrid_cmd_router import HybridCmdRouter
from standalone.control.cfpa2_bridge import CFPA2Bridge
from standalone.control.stuck_watchdog import StuckWatchdog
from standalone.observability.metrics_logger import ExplorationMetricsLogger
from standalone.exploration.cfpa2_single_robot import CFPA2SingleRobotNode

logging.basicConfig(
    level=logging.INFO,
    format="%(asctime)s %(levelname)-5s %(name)s | %(message)s",
    datefmt="%H:%M:%S",
)
log = logging.getLogger("main")


# ── Config loading ────────────────────────────────────────────────────────────

def load_config(path: str | None) -> dict:
    cfg_path = path or str(_REPO_ROOT / "standalone" / "config.yaml")
    with open(cfg_path) as f:
        return yaml.safe_load(f)


# ── Odometry bridge: publish to BUS so CFPA2 sees it ─────────────────────────

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


# ── Nav status bridge: simple "navigating" string for CFPA2 ──────────────────

def publish_nav_status(controller: PurePursuitController, ns: str) -> None:
    import json
    if controller.goal_reached or not controller.has_path:
        state = "goal_reached" if not controller.has_path else "goal_reached"
    else:
        state = "navigating"
    msg = String(data=json.dumps({"state": state}))
    BUS.publish(f"/{ns}/nav_status", msg)


# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    parser = argparse.ArgumentParser(description="Standalone MuJoCo exploration")
    parser.add_argument("--config", default=None, help="Path to config.yaml")
    parser.add_argument("--headless", action="store_true",
                        help="Run without MuJoCo viewer")
    args = parser.parse_args()

    cfg = load_config(args.config)
    sim_cfg  = cfg["sim"]
    lidar_cfg = cfg["lidar"]
    map_cfg  = cfg["mapping"]
    nav_cfg  = cfg["nav"]
    robot_cfg = cfg["robot"]
    cfpa2_cfg = cfg["cfpa2"]
    wd_cfg   = cfg["watchdog"]

    ns = cfpa2_cfg["namespace"]

    # ── Scene path ───────────────────────────────────────────────────────────
    scene_rel = sim_cfg["scene"]
    scene_path = _REPO_ROOT / scene_rel
    if not scene_path.exists():
        log.error(f"MJCF not found: {scene_path}")
        sys.exit(1)
    log.info(f"Loading scene: {scene_path}")

    # ── Init components ───────────────────────────────────────────────────────
    env = MuJoCoEnv(scene_path)
    dt = env.timestep

    leg_ctrl = LegController(
        env,
        stand_abduction=robot_cfg["stand_abduction"],
        stand_thigh=robot_cfg["stand_thigh"],
        stand_calf=robot_cfg["stand_calf"],
        kp_ab=robot_cfg["kp_ab"],
        kp_thigh=robot_cfg["kp_thigh"],
        kp_calf=robot_cfg["kp_calf"],
        kd=robot_cfg["kd"],
        wheel_radius_m=robot_cfg["wheel_radius_m"],
        wheel_track_m=robot_cfg["wheel_track_m"],
        wheel_max_omega=robot_cfg["wheel_max_omega"],
    )

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

    # Robot start position from first GT pose (after forward kinematics)
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

    planner = AStarPlanner(inflation_cells=nav_cfg["planner_inflation_cells"])
    controller = PurePursuitController(
        lookahead_m=nav_cfg["lookahead_m"],
        max_vx=nav_cfg["max_vx"],
        max_wz=nav_cfg["max_wz"],
        goal_tolerance_m=nav_cfg["goal_tolerance_m"],
        heading_gain=nav_cfg["heading_gain"],
    )
    router = HybridCmdRouter()
    bridge = CFPA2Bridge(namespace=ns)
    watchdog = StuckWatchdog(
        ns,
        stuck_window_sec=wd_cfg["stuck_window_sec"],
        stuck_threshold_m=wd_cfg["stuck_threshold_m"],
        backup_distance_m=wd_cfg["backup_distance_m"],
        cooldown_sec=wd_cfg["cooldown_sec"],
    )
    metrics = ExplorationMetricsLogger(ns)

    # ── CFPA2 node (uses NodeBase + TIMERS internally) ─────────────────────────
    cfpa2 = CFPA2SingleRobotNode()
    # Override params from config
    cfpa2.set_parameter("namespaces", [ns])
    cfpa2.set_parameter("robot_namespace", ns)
    cfpa2.set_parameter("publish_rate", cfpa2_cfg["publish_rate"])
    cfpa2.set_parameter("sensor_range", cfpa2_cfg["sensor_range"])
    cfpa2.set_parameter("verbose_logs", cfpa2_cfg["verbose_logs"])
    cfpa2.set_parameter("planning_map_topic_suffix", "/map")

    # ── Goal pose → planner bridge + periodic replan ─────────────────────────
    # See main_champ.py for design notes. Mirrors the original ROS2
    # default_nav.py AsyncGridPlanner (replan every ~1 s + path-blocked check).
    _current_grid: list = [None]
    _active_goal: list = [None]
    _last_replan_t: list = [-1.0]
    REPLAN_PERIOD_SEC = 1.0
    PATH_CHECK_SAMPLE = 3

    def _plan_and_apply(gx: float, gy: float, reason: str) -> bool:
        pose = env.get_pose2d()
        grid = _current_grid[0]
        if grid is None:
            return False
        path = planner.plan(grid, (pose[0], pose[1]), (gx, gy))
        if path:
            controller.set_path(path)
            log.info(f"[{reason}] path to ({gx:.2f},{gy:.2f}) — {len(path)} wps")
            return True
        log.warning(f"[{reason}] no path to ({gx:.2f},{gy:.2f})")
        return False

    def _path_blocked(grid, path) -> bool:
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
                if data[cy * W + cx] >= 50:
                    return True
        return False

    def _on_goal(msg) -> None:
        gx = float(msg.pose.position.x)
        gy = float(msg.pose.position.y)
        _active_goal[0] = (gx, gy)
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
            elif blocked:
                log.warning("blocked path + no replan — stopping; watchdog will recover")
                controller.clear()

    # ── cmd_vel from watchdog backup → router bypass ───────────────────────────
    _backup_cmd: list = [None]

    def _on_cmd_vel(msg) -> None:
        _backup_cmd[0] = (msg.linear.x, msg.linear.y, msg.angular.z)

    BUS.subscribe(f"/{ns}/cmd_vel", _on_cmd_vel)

    # ── Timing state ──────────────────────────────────────────────────────────
    lidar_period = 1.0 / lidar_cfg["publish_rate_hz"]
    odom_period  = 0.05   # 20 Hz odom
    watchdog_period = 0.5  # 2 Hz watchdog
    t_last_lidar = -lidar_period
    t_last_odom  = -odom_period
    t_last_watchdog = -watchdog_period
    t_last_metrics  = -1.0
    steps_done = 0

    log.info("Simulation started. Ctrl+C to exit.")

    # ── Sim loop ──────────────────────────────────────────────────────────────
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
                _maybe_replan()
                if controller.goal_reached and _active_goal[0] is not None:
                    _active_goal[0] = None
                t_last_lidar = t

            # 3. CFPA2 timers (1 Hz, driven by TIMERS registry)
            TIMERS.tick(t)

            # 4. Navigation status → CFPA2
            publish_nav_status(controller, ns)

            # 5. Controller → router
            pose = env.get_pose2d()
            backup = _backup_cmd[0]
            if backup is not None:
                vx, vy, wz = backup
                wheel_mode = False
                _backup_cmd[0] = None
            else:
                vx, wz = controller.compute(pose)
                router.set_cmd_vel(vx, 0.0, wz)
                vx, wz, wheel_mode = router.tick()

            leg_ctrl.set_cmd_vel(vx, 0.0, wz, wheel_mode)

            # 6. Physics step + leg control
            leg_ctrl.step()
            env.step()

            # 7. Watchdog (2 Hz)
            if t - t_last_watchdog >= watchdog_period:
                watchdog.tick()
                t_last_watchdog = t

            # 8. Metrics (2 Hz)
            if t - t_last_metrics >= 0.5:
                done = metrics.tick()
                if done:
                    log.info("Exploration complete — stopping.")
                    leg_ctrl.set_cmd_vel(0.0, 0.0, 0.0, False)
                t_last_metrics = t

            # 9. Viewer sync
            if viewer is not None:
                viewer.sync()
                if not viewer.is_running():
                    break
            else:
                # Headless: yield briefly to avoid 100% CPU spin
                # (mj_step is bounded by timestep, so this is fine)
                pass

    if args.headless:
        try:
            run_loop(viewer=None)
        except KeyboardInterrupt:
            log.info("Interrupted — exiting.")
    else:
        with mujoco.viewer.launch_passive(env.model, env.data) as viewer:
            try:
                run_loop(viewer=viewer)
            except KeyboardInterrupt:
                log.info("Interrupted — exiting.")


if __name__ == "__main__":
    main()
