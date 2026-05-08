#!/usr/bin/env python3
"""Native MuJoCo simulation runner — no ROS, no Docker.

Usage:
    python3 -m native.sim_runner --scene <mjcf-path>
    python3 -m native.sim_runner --scene <mjcf-path> --headless --duration 5
    python3 -m native.sim_runner --scene <mjcf-path> --fast
"""
from __future__ import annotations

import argparse
import time
from pathlib import Path

import mujoco
import mujoco.viewer

REPO_ROOT = Path(__file__).resolve().parent.parent.parent
DEFAULT_SCENE = REPO_ROOT / "src/go2w/go2_gazebo_sim/mujoco/vlm_exploration_scene.xml"


def run(scene: Path, duration: float, headless: bool, realtime: bool) -> None:
    model = mujoco.MjModel.from_xml_path(str(scene))
    data = mujoco.MjData(model)

    print(f"Loaded: {scene.name}  |  timestep={model.opt.timestep:.4f}s  |  bodies={model.nbody}")

    if headless:
        steps = int(duration / model.opt.timestep)
        t0 = time.monotonic()
        for _ in range(steps):
            mujoco.mj_step(model, data)
        elapsed = time.monotonic() - t0
        sim_time = steps * model.opt.timestep
        print(f"Headless done: {sim_time:.1f}s sim in {elapsed:.2f}s wall ({sim_time/elapsed:.1f}x realtime)")
        return

    with mujoco.viewer.launch_passive(model, data) as v:
        v.cam.azimuth = 135
        v.cam.elevation = -20
        v.cam.distance = 8.0
        deadline = time.monotonic() + duration
        wall_prev = time.monotonic()
        while v.is_running() and (duration <= 0 or time.monotonic() < deadline):
            mujoco.mj_step(model, data)
            v.sync()
            if realtime:
                now = time.monotonic()
                step_wall = now - wall_prev
                sleep = model.opt.timestep - step_wall
                if sleep > 0:
                    time.sleep(sleep)
                wall_prev = time.monotonic()


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--scene", default=str(DEFAULT_SCENE))
    ap.add_argument("--duration", type=float, default=0,
                    help="Max sim seconds (0 = run until window closed)")
    ap.add_argument("--headless", action="store_true",
                    help="No viewer — pure physics")
    ap.add_argument("--fast", action="store_true",
                    help="Run as fast as possible (no realtime throttle). Default when --headless.")
    args = ap.parse_args()
    scene = Path(args.scene)
    if not scene.exists():
        raise FileNotFoundError(f"Scene not found: {scene}")
    realtime = not args.fast and not args.headless
    dur = args.duration if args.duration > 0 else (30.0 if args.headless else 0)
    run(scene, dur, args.headless, realtime)


if __name__ == "__main__":
    main()
