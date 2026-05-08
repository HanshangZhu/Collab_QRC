#!/usr/bin/env python3
"""Comparison harness — traditional CFPA2+A* arm vs VLM arm (stub).

Usage:
    python3 -m native.harness --arm traditional \
        --scene src/go2w/go2_gazebo_sim/mujoco/vlm_exploration_scene.xml \
        --trials 3 --duration 60 --out /tmp/native_bench/cfgA

Each trial writes: <out>/trial_<n>.json
Summary written to: <out>/summary.json

Metrics per trial (matches archived scripts/bench/session_reporter.py schema):
  coverage_ratio    fraction of total cells observed
  distance_m        total odometry distance travelled
  contacts          total contact events (floor contacts excluded)
  tipped            True if body pitch|roll > tip_threshold_deg at any point
  completed         True if no crash and ran full duration
  sim_time_s        actual sim time elapsed
  wall_time_s       wall clock time for this trial
"""
from __future__ import annotations

import argparse
import json
import math
import time
from pathlib import Path

import mujoco
import numpy as np

from .sensors import LiDARSensor, read_pose, read_contacts
from .control import WheelDirectController
from .mapping import OccupancyMapper
from .cfpa2 import FrontierExplorer
from .planning import (DefaultNavCoordinator, DefaultNavConfig,
                       RobotState, GoalState, NavRuntimeState, make_scan)

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_SCENE = REPO_ROOT / "src/go2w/go2_gazebo_sim/mujoco/vlm_exploration_scene.xml"

TIP_THRESHOLD_DEG = 30.0
FLOOR_BODIES = {"world", "floor", "ground"}  # contact pairs involving these are ignored
MAP_RES = 0.05
MAP_W = 500
MAP_H = 500
MAP_OX = -12.5
MAP_OY = -12.5


def _quat_to_rpy(qw, qx, qy, qz):
    """Returns (roll, pitch, yaw) in radians."""
    roll  = math.atan2(2*(qw*qx + qy*qz), 1 - 2*(qx*qx + qy*qy))
    pitch = math.asin(max(-1.0, min(1.0, 2*(qw*qy - qz*qx))))
    yaw   = math.atan2(2*(qw*qz + qx*qy), 1 - 2*(qy*qy + qz*qz))
    return roll, pitch, yaw


def _body_name(model, bid):
    try:
        name = mujoco.mj_id2name(model, mujoco.mjtObj.mjOBJ_BODY, bid)
        return name if name else ""
    except Exception:
        return ""


def run_traditional_trial(model: mujoco.MjModel, duration: float) -> dict:
    """Run one traditional-arm trial. Returns metrics dict."""
    data = mujoco.MjData(model)
    lidar = LiDARSensor(model)
    ctrl = WheelDirectController(model)
    mapper = OccupancyMapper(resolution=MAP_RES, width=MAP_W, height=MAP_H,
                             origin_x=MAP_OX, origin_y=MAP_OY)
    explorer = FrontierExplorer(mapper, blacklist_ttl_s=20.0)
    cfg = DefaultNavConfig(
        startup_delay=0.0,
        require_settle_before_motion=False,
        max_linear_speed=0.3,
        max_angular_speed=0.8,
        goal_tolerance=0.8,
        planner_replan_sec=1.0,
    )
    nav = DefaultNavCoordinator(cfg)
    rt = NavRuntimeState()

    dt = model.opt.timestep
    total_steps = int(duration / dt)
    scan_iv = max(1, int(0.1 / dt))     # ~10 Hz sensor + map updates
    goal_iv = max(1, int(2.0 / dt))     # frontier re-select every 2s

    goal_state = GoalState()
    prev_x, prev_y = 0.0, 0.0
    distance_m = 0.0
    contact_count = 0
    tipped = False
    contacts_this_step: list = []

    # Settle
    for _ in range(500):
        ctrl.tick(model, data)
        mujoco.mj_step(model, data)

    pose0 = read_pose(model, data)
    prev_x, prev_y = pose0.x, pose0.y

    t_wall_start = time.monotonic()

    for step in range(total_steps):
        ctrl.tick(model, data)
        mujoco.mj_step(model, data)

        if step % scan_iv == 0:
            pose = read_pose(model, data)

            # Odometry distance
            distance_m += math.hypot(pose.x - prev_x, pose.y - prev_y)
            prev_x, prev_y = pose.x, pose.y

            # Tipped check
            r, p, _ = _quat_to_rpy(pose.qw, pose.qx, pose.qy, pose.qz)
            if max(abs(math.degrees(r)), abs(math.degrees(p))) > TIP_THRESHOLD_DEG:
                tipped = True

            # LiDAR + map
            hits = lidar.tick(model, data)
            if len(hits) > 0:
                mapper.update(np.array([pose.x, pose.y]), hits)

            # Contacts (non-floor)
            contacts = read_contacts(model, data)
            for c in contacts:
                b1 = _body_name(model, c.body1).lower()
                b2 = _body_name(model, c.body2).lower()
                if not any(f in b1 or f in b2 for f in FLOOR_BODIES):
                    contact_count += 1

            # Scan for nav
            scan = make_scan(hits if len(hits) > 0 else np.empty((0, 3), dtype=np.float32), pose)

            # Nav tick
            yaw = _quat_to_rpy(pose.qw, pose.qx, pose.qy, pose.qz)[2]
            rs = RobotState(x=pose.x, y=pose.y, yaw=yaw, speed=0.0)
            tick = nav.tick(data.time, rt, rs, goal_state, scan, external_stop=0)
            ctrl.set_cmd_vel(tick.linear_x, tick.angular_z)

        if step % goal_iv == 0:
            pose = read_pose(model, data)
            goal = explorer.select_goal(pose, sim_time=data.time)
            if goal:
                goal_state.x, goal_state.y = goal[0], goal[1]
                # Blacklist if robot is stuck near this goal for too long
            else:
                goal_state.x = goal_state.y = None

    wall_time = time.monotonic() - t_wall_start

    return {
        "coverage_ratio": float(mapper.coverage_ratio),
        "distance_m": float(distance_m),
        "contacts": int(contact_count),
        "tipped": bool(tipped),
        "completed": True,
        "sim_time_s": float(duration),
        "wall_time_s": float(wall_time),
    }


def run_vlm_trial(model: mujoco.MjModel, duration: float) -> dict:
    raise NotImplementedError("VLM arm not implemented yet — Phase B")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--arm", choices=["traditional", "vlm"], default="traditional")
    ap.add_argument("--scene", default=str(DEFAULT_SCENE))
    ap.add_argument("--trials", type=int, default=3)
    ap.add_argument("--duration", type=float, default=60.0, help="Sim seconds per trial")
    ap.add_argument("--out", default="/tmp/native_bench/cfgA")
    args = ap.parse_args()

    scene = Path(args.scene)
    if not scene.exists():
        raise FileNotFoundError(f"Scene not found: {scene}")

    out_dir = Path(args.out)
    out_dir.mkdir(parents=True, exist_ok=True)

    model = mujoco.MjModel.from_xml_path(str(scene))
    run_fn = run_traditional_trial if args.arm == "traditional" else run_vlm_trial

    print(f"Harness: arm={args.arm} scene={scene.name} trials={args.trials} duration={args.duration}s")

    results = []
    for i in range(1, args.trials + 1):
        print(f"  Trial {i}/{args.trials} ...", end="", flush=True)
        metrics = run_fn(model, args.duration)
        metrics["trial"] = i
        metrics["arm"] = args.arm
        path = out_dir / f"trial_{i}.json"
        path.write_text(json.dumps(metrics, indent=2))
        results.append(metrics)
        print(f" coverage={metrics['coverage_ratio']:.1%} dist={metrics['distance_m']:.1f}m "
              f"contacts={metrics['contacts']} tipped={metrics['tipped']}")

    # Summary
    cov = [r["coverage_ratio"] for r in results]
    summary = {
        "arm": args.arm,
        "scene": str(scene),
        "trials": args.trials,
        "duration_s": args.duration,
        "coverage_mean": float(np.mean(cov)),
        "coverage_std": float(np.std(cov)),
        "coverage_min": float(np.min(cov)),
        "coverage_max": float(np.max(cov)),
        "contacts_total": sum(r["contacts"] for r in results),
        "tipped_count": sum(r["tipped"] for r in results),
        "completed_count": sum(r["completed"] for r in results),
    }
    (out_dir / "summary.json").write_text(json.dumps(summary, indent=2))
    print(f"\nSummary: coverage {summary['coverage_mean']:.1%} ± {summary['coverage_std']:.1%}")
    print(f"Written to {out_dir}")


if __name__ == "__main__":
    main()
