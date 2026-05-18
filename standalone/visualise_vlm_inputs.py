"""Visualise the two images the VLM receives every cycle.

Run with:
    mjpython standalone/visualise_vlm_inputs.py               # live capture
    mjpython standalone/visualise_vlm_inputs.py --session latest  # from saved logs
    mjpython standalone/visualise_vlm_inputs.py --session /tmp/standalone_vlm/session_20260518_154827

What you'll see
---------------
Left  — Image 1: top-down occupancy map (gray=unknown, white=free, black=wall,
        red dot=robot, yellow circles=frontier candidates, blue trail=path history)
Right — Image 2: robot front camera RGB frame (first-person view used for
        artifact detection)

The bottom panel shows the scene JSON the VLM receives as text alongside the images.
"""
from __future__ import annotations

import argparse
import base64
import json
import os
import sys
from io import BytesIO
from pathlib import Path

# ── resolve repo root ─────────────────────────────────────────────────────────
_HERE = Path(__file__).resolve().parent
_REPO = _HERE.parent
sys.path.insert(0, str(_REPO))
sys.path.insert(0, str(_HERE))


def _b64_to_array(b64: str):
    import numpy as np
    from PIL import Image
    data = base64.b64decode(b64)
    img = Image.open(BytesIO(data)).convert("RGB")
    return np.array(img)


def _show_from_session(session_dir: Path, cycle: int = 1) -> None:
    """Load saved map PNG + scene JSON from a log session and display."""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")  # mjpython owns the main thread; use non-interactive backend
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec

    map_png = session_dir / f"cycle_{cycle:04d}_map.png"
    scene_json = session_dir / f"cycle_{cycle:04d}_scene.json"
    decision_json = session_dir / f"cycle_{cycle:04d}_decision.json"
    camera_png = session_dir / f"cycle_{cycle:04d}_camera.png"

    if not map_png.exists():
        print(f"[viz] No map PNG found at {map_png}")
        print(f"      Available cycles: {sorted(p.stem for p in session_dir.glob('cycle_*_map.png'))}")
        return

    from PIL import Image
    map_img = np.array(Image.open(map_png).convert("RGB"))

    scene = json.loads(scene_json.read_text()) if scene_json.exists() else {}
    decision = json.loads(decision_json.read_text()) if decision_json.exists() else {}

    has_camera = camera_png.exists()
    cam_img = np.array(Image.open(camera_png).convert("RGB")) if has_camera else None

    fig = plt.figure(figsize=(16, 10), facecolor="#1a1a2e")
    fig.suptitle(
        f"VLM Inputs — session: {session_dir.name}  cycle: {cycle}",
        color="white", fontsize=14, fontweight="bold"
    )

    if has_camera:
        gs = gridspec.GridSpec(2, 2, figure=fig, height_ratios=[3, 1],
                               hspace=0.35, wspace=0.15)
        ax_map = fig.add_subplot(gs[0, 0])
        ax_cam = fig.add_subplot(gs[0, 1])
        ax_json = fig.add_subplot(gs[1, :])
    else:
        gs = gridspec.GridSpec(2, 1, figure=fig, height_ratios=[3, 1], hspace=0.35)
        ax_map = fig.add_subplot(gs[0])
        ax_cam = None
        ax_json = fig.add_subplot(gs[1])

    # ── Image 1: occupancy map ────────────────────────────────────────────────
    ax_map.imshow(map_img)
    ax_map.set_title("Image 1 — Occupancy Map\n(gray=unknown  white=free  black=wall  "
                     "red=robot  yellow=frontiers  blue=trail)",
                     color="white", fontsize=9, pad=8)
    ax_map.axis("off")

    # Annotate frontier candidates directly on the map
    if scene.get("frontier_candidates"):
        # Map image coordinates: we'd need pixel mapping — just show count
        n = len(scene["frontier_candidates"])
        ax_map.set_xlabel(f"{n} frontier candidates (yellow circles)",
                          color="#ffdd44", fontsize=8)

    # ── Image 2: front camera ─────────────────────────────────────────────────
    if ax_cam is not None and cam_img is not None:
        ax_cam.imshow(cam_img)
        ax_cam.set_title("Image 2 — Robot Front Camera\n"
                         "(first-person RGB — VLM scans for artifacts here)",
                         color="white", fontsize=9, pad=8)
        ax_cam.axis("off")
    elif ax_cam is not None:
        ax_cam.set_facecolor("#2a2a3e")
        ax_cam.text(0.5, 0.5,
                    "Camera frame not saved\nRun again to capture",
                    ha="center", va="center", color="#888888", fontsize=11,
                    transform=ax_cam.transAxes)
        ax_cam.set_title("Image 2 — Robot Front Camera (not saved yet)",
                         color="#888888", fontsize=9, pad=8)
        ax_cam.axis("off")

    # ── Scene JSON summary ────────────────────────────────────────────────────
    ax_json.set_facecolor("#0d0d1a")
    ax_json.axis("off")

    robot = scene.get("robot", {})
    coverage = scene.get("coverage", {})
    frontiers = scene.get("frontier_candidates", [])
    dec_goal = decision.get("goal")
    dec_reason = decision.get("reason", "")
    dec_dur = decision.get("duration_sec", "?")

    summary = (
        f"SCENE JSON (text sent alongside images)\n"
        f"{'─'*70}\n"
        f"  robot:     x={robot.get('x','?'):.2f}  y={robot.get('y','?'):.2f}  "
        f"yaw={robot.get('yaw_deg','?'):.1f}°\n"
        f"  coverage:  {coverage.get('known_m2','?'):.1f} m²  known\n"
        f"  frontiers: {len(frontiers)} candidates  "
        + (f"→ top: ({frontiers[0]['x']:.2f}, {frontiers[0]['y']:.2f}) "
           f"info_gain={frontiers[0]['info_gain']:.1f}" if frontiers else "none")
        + f"\n{'─'*70}\n"
        f"VLM DECISION  ({dec_dur}s)\n"
        f"  goal:    {dec_goal}\n"
        f"  reason:  {dec_reason[:100]}"
    )

    ax_json.text(0.02, 0.95, summary,
                 transform=ax_json.transAxes,
                 color="#00ff88", fontsize=8.5,
                 fontfamily="monospace",
                 verticalalignment="top")

    out = session_dir / f"cycle_{cycle:04d}_visualised.png"
    plt.savefig(str(out), dpi=150, bbox_inches="tight", facecolor="#1a1a2e")
    plt.close()
    print(f"[viz] saved → {out}")
    import subprocess
    subprocess.Popen(["open", str(out)])  # open with macOS Preview


def _show_live() -> None:
    """Spin up the sim for a few seconds, capture both images live, display."""
    import numpy as np
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    import matplotlib.gridspec as gridspec

    print("[viz] Starting MuJoCo sim (headless) to capture live images…")

    scene_xml = _REPO / "src/go2w/go2_gazebo_sim/mujoco/demo1.xml"
    if not scene_xml.exists():
        print(f"[viz] Scene not found at {scene_xml}")
        return

    from sim.mujoco_env import MuJoCoEnv
    from mapping.occupancy_grid import OccupancyMapper
    from nav.lidar_processor import LidarProcessor
    from vlm.renderer import MapRenderer
    from vlm.explorer import _extract_frontiers
    import math

    env = MuJoCoEnv(str(scene_xml))
    mapper = OccupancyMapper(resolution=0.1, width=250, height=250,
                             origin_x=-5.0, origin_y=-5.0)
    lidar = LidarProcessor(env)
    renderer = MapRenderer(mapper)

    print("[viz] Stepping sim for 5 s to build up map…")
    # Drive forward slowly while collecting LiDAR
    for i in range(1000):
        env.set_wheel_velocity(0.5, 0.5, 0.5, 0.5)
        env.step()
        if i % 20 == 0:
            pose = env.get_pose()
            scan = lidar.get_scan()
            if scan is not None:
                mapper.update(pose, scan)
            renderer.append_trail(pose[0], pose[1])

    pose = env.get_pose()
    print(f"[viz] Robot pose: x={pose[0]:.2f}  y={pose[1]:.2f}  yaw={math.degrees(pose[2]):.1f}°")

    # ── Image 1: render occupancy map ─────────────────────────────────────────
    import numpy as np
    grid = np.full((mapper.height, mapper.width), -1, dtype=np.int8)
    obs = mapper._observed
    log_odds = mapper._log_odds
    grid[obs & (log_odds <= mapper._free_thr)] = 0
    grid[obs & (log_odds >= mapper._occ_thr)] = 100

    frontiers = _extract_frontiers(
        grid, mapper.resolution, mapper.origin_x, mapper.origin_y,
        (pose[0], pose[1]), max_targets=10, sensor_range_m=3.5,
    )

    map_b64 = renderer.render_b64(
        pose, frontiers=frontiers, goal=None, title="Live capture — VLM input"
    )
    from PIL import Image
    map_img = _b64_to_array(map_b64)

    # ── Image 2: render front camera ──────────────────────────────────────────
    cam_b64 = env.render_camera("front_camera", width=640, height=480)
    cam_img = _b64_to_array(cam_b64) if cam_b64 else None

    # ── Plot ──────────────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(16, 8), facecolor="#1a1a2e")
    fig.suptitle("VLM Inputs — Live Capture (demo1.xml, 5 s of driving)",
                 color="white", fontsize=14, fontweight="bold")

    gs = gridspec.GridSpec(1, 2, figure=fig, wspace=0.1)
    ax_map = fig.add_subplot(gs[0])
    ax_cam = fig.add_subplot(gs[1])

    ax_map.imshow(map_img)
    ax_map.set_title(
        "Image 1 — Occupancy Map\n"
        "gray=unknown  white=free  black=wall  red=robot  yellow=frontiers  blue=trail",
        color="white", fontsize=9, pad=8
    )
    ax_map.axis("off")

    if cam_img is not None:
        ax_cam.imshow(cam_img)
        ax_cam.set_title(
            "Image 2 — Robot Front Camera (RGB)\n"
            "First-person view — VLM scans this for artifacts (colored objects, boxes, etc.)",
            color="white", fontsize=9, pad=8
        )
    else:
        ax_cam.set_facecolor("#2a2a3e")
        ax_cam.text(0.5, 0.5,
                    "render_camera() returned None\n"
                    "(requires mjpython + OpenGL context)",
                    ha="center", va="center", color="#888888", fontsize=11,
                    transform=ax_cam.transAxes)
        ax_cam.set_title("Image 2 — Robot Front Camera (unavailable)",
                         color="#888888", fontsize=9, pad=8)
    ax_cam.axis("off")

    out = Path("/tmp/vlm_inputs_live.png")
    plt.savefig(str(out), dpi=150, bbox_inches="tight", facecolor="#1a1a2e")
    plt.close()
    print(f"[viz] saved → {out}")
    import subprocess
    subprocess.Popen(["open", str(out)])


def main() -> None:
    import numpy as np

    ap = argparse.ArgumentParser(description="Visualise VLM inputs (map + camera)")
    ap.add_argument("--session", default="",
                    help="Session dir path or 'latest'. Omit for live capture.")
    ap.add_argument("--cycle", type=int, default=1,
                    help="Which cycle to show from session logs (default: 1)")
    args = ap.parse_args()

    if args.session:
        base = Path("/tmp/standalone_vlm")
        if args.session == "latest":
            sessions = sorted(base.glob("session_*"))
            if not sessions:
                print("[viz] No sessions found in /tmp/standalone_vlm/")
                return
            session_dir = sessions[-1]
        else:
            session_dir = Path(args.session)
        print(f"[viz] Loading session: {session_dir}")
        _show_from_session(session_dir, cycle=args.cycle)
    else:
        _show_live()


if __name__ == "__main__":
    main()
