#!/usr/bin/env python3
"""Interactive demo — MuJoCo viewer + live occupancy map.

Run with mjpython (required on macOS for viewer):
    mjpython -m native.demo
    mjpython -m native.demo --fast          # faster than real-time
    mjpython -m native.demo --duration 120  # stop after 120 sim-seconds

Two windows:
  1. MuJoCo 3D viewer — robot in scene (this window)
  2. macOS Preview    — live occupancy map PNG (auto-refreshes every 2s)
                        grey=unknown  white=free  black=wall  blue=trail  red=goal
"""
from __future__ import annotations

import argparse
import math
import subprocess
import time
from pathlib import Path

import matplotlib
matplotlib.use("Agg")   # non-interactive — no GUI thread conflict with mjpython
import matplotlib.pyplot as plt
import mujoco
import mujoco.viewer
import numpy as np

from .sensors import LiDARSensor, read_pose
from .control import WheelDirectController
from .mapping import OccupancyMapper
from .cfpa2 import FrontierExplorer
from .artifacts import ArtifactDetector
from .planning import (DefaultNavCoordinator, DefaultNavConfig,
                       RobotState, GoalState, NavRuntimeState, make_scan)

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_SCENE = REPO_ROOT / "src/go2w/go2_gazebo_sim/mujoco/vlm_exploration_scene.xml"
MAP_PNG = Path("/tmp/native_sim_map.png")

MAP_RES = 0.05
MAP_W   = 500
MAP_H   = 500
MAP_OX  = -12.5
MAP_OY  = -12.5


def _yaw(pose) -> float:
    return math.atan2(
        2.0 * (pose.qw * pose.qz + pose.qx * pose.qy),
        1.0 - 2.0 * (pose.qy ** 2 + pose.qz ** 2),
    )


def _save_map(mapper, trail_x, trail_y, pose, goal, sim_time):
    """Render occupancy map to PNG. macOS Preview auto-refreshes on file change."""
    g = mapper.grid   # (H,W) int8: -1=unknown, 0=free, 100=occ
    display = np.where(g == -1, 0.5, np.where(g == 100, 0.0, 1.0))

    fig, ax = plt.subplots(figsize=(7, 7), dpi=100)
    extent = [MAP_OX, MAP_OX + MAP_W * MAP_RES,
              MAP_OY, MAP_OY + MAP_H * MAP_RES]
    ax.imshow(display, origin="lower", extent=extent,
              cmap="gray", vmin=0, vmax=1, interpolation="nearest")

    if trail_x:
        ax.plot(trail_x, trail_y, "b-", linewidth=0.8, alpha=0.7, label="trail")
    ax.plot([pose.x], [pose.y], "bo", markersize=7, label="robot")
    if goal:
        ax.plot([goal[0]], [goal[1]], "r*", markersize=14, label=f"goal ({goal[0]:.1f},{goal[1]:.1f})")

    ax.set_aspect("equal")
    ax.set_title(
        f"t={sim_time:.0f}s   coverage={mapper.coverage_ratio:.1%}\n"
        f"grey=unknown  white=free  black=wall",
        fontsize=9
    )
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.legend(loc="upper right", fontsize=8)
    fig.tight_layout()
    fig.savefig(MAP_PNG, dpi=100)
    plt.close(fig)


def run(scene: Path, duration: float, realtime: bool) -> None:
    model = mujoco.MjModel.from_xml_path(str(scene))
    data  = mujoco.MjData(model)

    lidar    = LiDARSensor(model)
    ctrl     = WheelDirectController(model)
    mapper   = OccupancyMapper(resolution=MAP_RES, width=MAP_W, height=MAP_H,
                                origin_x=MAP_OX, origin_y=MAP_OY)
    explorer = FrontierExplorer(mapper, blacklist_ttl_s=20.0)
    cfg = DefaultNavConfig(
        startup_delay=0.0,
        require_settle_before_motion=False,
        max_linear_speed=0.22,
        max_angular_speed=0.8,
        goal_tolerance=1.0,          # declare reached within 1m — don't wedge at exact point
        obstacle_stop_dist=0.30,     # body front ~0.35m from base; 0.30m scan = 0.46m from base
        obstacle_slow_dist=0.65,     # start slowing 0.65m out
        front_half_angle_deg=45.0,   # narrower cone — don't stop for corridor side walls
        side_check_angle_deg=60.0,
        avoidance_gain=2.0,
        planner_enabled=False,       # direct heading + reactive avoidance — no local A* (unreliable without costmap)
    )
    nav = DefaultNavCoordinator(cfg)
    rt  = NavRuntimeState()

    dt      = model.opt.timestep
    scan_iv = max(1, int(0.1  / dt))   # 10 Hz
    goal_iv = max(1, int(2.0  / dt))   # frontier every 2 s
    map_iv  = max(1, int(2.0  / dt))   # PNG save every 2 s

    detector = ArtifactDetector(model, detect_dist=2.0)

    goal_state   = GoalState()
    current_goal = None
    trail_x: list[float] = []
    trail_y: list[float] = []

    # Stuck detection
    STUCK_SEC      = 12.0   # window length
    STUCK_DIST     = 0.15   # m — truly stuck if moved less than this in STUCK_SEC
    stuck_window: list[tuple[float, float, float]] = []  # (sim_time, x, y)
    stuck_cooldown = 0.0    # don't re-fire stuck immediately after clearing

    TIP_DEG        = 20.0   # deg — cut wheels if roll or pitch exceeds this
    tipped_until   = 0.0    # sim_time after which wheel drive resumes

    print(f"Scene    : {scene.name}")
    print(f"Duration : {duration}s (0=unlimited)")
    print(f"Realtime : {realtime}")
    print(f"Map PNG  : {MAP_PNG}  (opens in Preview, auto-refreshes)")

    # Write blank map + open Preview before viewer starts
    pose0 = read_pose(model, data)
    _save_map(mapper, [], [], pose0, None, 0.0)
    subprocess.Popen(["open", str(MAP_PNG)])

    with mujoco.viewer.launch_passive(model, data) as v:
        v.cam.azimuth   = 135
        v.cam.elevation = -25
        v.cam.distance  = 12.0

        print("Settling...")
        for _ in range(500):
            ctrl.tick(model, data)
            mujoco.mj_step(model, data)
        v.sync()
        print("Running — watch MuJoCo window + Preview map.")

        step        = 0
        t_wall_prev = time.monotonic()

        while v.is_running() and (duration <= 0 or data.time < duration):
            ctrl.tick(model, data)
            mujoco.mj_step(model, data)
            step += 1

            if step % scan_iv == 0:
                pose = read_pose(model, data)
                trail_x.append(pose.x)
                trail_y.append(pose.y)

                hits = lidar.tick(model, data)
                if len(hits) > 0:
                    mapper.update(np.array([pose.x, pose.y]), hits)

                # Tip detection: cut wheels when roll/pitch too large
                roll  = math.atan2(2*(pose.qw*pose.qx + pose.qy*pose.qz),
                                   1 - 2*(pose.qx**2 + pose.qy**2))
                pitch = math.asin(max(-1.0, min(1.0, 2*(pose.qw*pose.qy - pose.qz*pose.qx))))
                if max(abs(math.degrees(roll)), abs(math.degrees(pitch))) > TIP_DEG:
                    tipped_until = data.time + 3.0
                    print(f"  [tip!] roll={math.degrees(roll):.0f}° pitch={math.degrees(pitch):.0f}°"
                          f" — cutting wheels for 3s")

                if data.time < tipped_until:
                    ctrl.set_cmd_vel(0.0, 0.0)
                else:
                    scan = make_scan(
                        hits if len(hits) > 0 else np.empty((0, 3), dtype=np.float32),
                        pose,
                    )
                    yaw  = _yaw(pose)
                    rs   = RobotState(x=pose.x, y=pose.y, yaw=yaw, speed=0.0)
                    tick = nav.tick(data.time, rt, rs, goal_state, scan, external_stop=0)
                    ctrl.set_cmd_vel(tick.linear_x, tick.angular_z)

            if step % goal_iv == 0:
                pose = read_pose(model, data)

                # Artifact detection — dynamic, no hardcoded positions
                for a in detector.tick(data, pose.x, pose.y, data.time):
                    print(f"  [ARTIFACT FOUND] {a.name}  "
                          f"world=({a.x:.2f}, {a.y:.2f}, {a.z:.2f})  "
                          f"t={a.found_at_t:.0f}s  dist={a.found_dist:.2f}m  "
                          f"total={len(detector.found)}/{len(detector.all_artifacts)}")

                # Stuck detection — only check when window is full (>= STUCK_SEC old)
                stuck_window.append((data.time, pose.x, pose.y))
                stuck_window[:] = [(t, x, y) for t, x, y in stuck_window
                                   if data.time - t <= STUCK_SEC]
                window_age = data.time - stuck_window[0][0] if stuck_window else 0
                if (window_age >= STUCK_SEC * 0.8          # window nearly full
                        and current_goal is not None
                        and data.time > stuck_cooldown):
                    oldest = stuck_window[0]
                    moved = math.hypot(pose.x - oldest[1], pose.y - oldest[2])
                    if moved < STUCK_DIST:
                        print(f"  [stuck] moved {moved:.2f}m in {window_age:.0f}s "
                              f"— blacklisting ({current_goal[0]:.1f},{current_goal[1]:.1f})")
                        explorer.blacklist_goal(current_goal, data.time)
                        current_goal = None
                        goal_state.x = goal_state.y = None
                        rt = NavRuntimeState()
                        stuck_window.clear()
                        stuck_cooldown = data.time + 4.0   # 4s before stuck can fire again

                current_goal = explorer.select_goal(pose, sim_time=data.time)
                if current_goal:
                    goal_state.x, goal_state.y = current_goal
                else:
                    goal_state.x = goal_state.y = None

            if step % map_iv == 0:
                pose = read_pose(model, data)
                _save_map(mapper, trail_x, trail_y, pose, current_goal, data.time)
                print(f"  t={data.time:6.1f}s  coverage={mapper.coverage_ratio:.1%}"
                      f"  artifacts={len(detector.found)}/{len(detector.all_artifacts)}"
                      f"  goal={f'({current_goal[0]:.1f},{current_goal[1]:.1f})' if current_goal else 'none'}")

            v.sync()

            if realtime:
                now   = time.monotonic()
                sleep = dt - (now - t_wall_prev)
                if sleep > 0:
                    time.sleep(sleep)
                t_wall_prev = time.monotonic()

    print(f"\nDone. Final coverage: {mapper.coverage_ratio:.1%}")
    print(f"Final map saved to {MAP_PNG}")
    print(f"\n{detector.summary()}")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=str(DEFAULT_SCENE))
    ap.add_argument("--duration", type=float, default=0)
    ap.add_argument("--fast", action="store_true")
    args = ap.parse_args()
    scene = Path(args.scene)
    if not scene.exists():
        raise FileNotFoundError(f"Scene not found: {scene}")
    run(scene, args.duration, realtime=not args.fast)


if __name__ == "__main__":
    main()
