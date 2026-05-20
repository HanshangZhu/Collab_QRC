#!/usr/bin/env python3
"""Live VLM exploration dashboard — subprocess companion to main_vlm.py.

Two-panel window:
  Left  — VLM's MAP VIEW: the annotated occupancy map sent to the LLM each cycle
           (frontiers as yellow circles, robot trail, last goal).
  Right — ROBOT POV: the front-camera image saved every ~2 s from the sim loop.

Bottom strip shows the last VLM decision: cycle number, goal coordinates, reason,
and any artifacts logged so far.

Launched automatically by main_vlm.py (unless --headless or --no-map).
Can also be run manually to replay a saved session:

    python3 -m standalone.viz.vlm_dashboard <session_dir> [camera_png_path]

Arguments:
    session_dir     Path to the VLM session log directory (contains cycle_*.png etc.)
    camera_png      (optional) Path where main_vlm.py writes the live camera frame.
                    Defaults to /tmp/vlm_robot_pov.png
"""
from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import matplotlib.gridspec as gridspec
from matplotlib.animation import FuncAnimation


def _find_latest_cycle_file(session_dir: Path, suffix: str) -> Path | None:
    """Return the highest-numbered cycle_XXXX_<suffix> file, or None."""
    files = sorted(session_dir.glob(f"cycle_*_{suffix}"))
    return files[-1] if files else None


def _load_png(path: Path | None) -> np.ndarray | None:
    """Load a PNG as an RGBA numpy array, or return None on failure."""
    if path is None or not path.exists():
        return None
    try:
        return plt.imread(str(path))
    except Exception:
        return None


def _load_decision(session_dir: Path) -> dict | None:
    """Load the latest cycle_XXXX_decision.json."""
    path = _find_latest_cycle_file(session_dir, "decision.json")
    if path is None or not path.exists():
        return None
    try:
        return json.loads(path.read_text())
    except Exception:
        return None


def _placeholder(label: str) -> np.ndarray:
    """Gray placeholder with white centered text rendered as a tiny numpy array."""
    arr = np.full((120, 240, 3), 0.22, dtype=np.float32)
    return arr


def main() -> None:
    parser = argparse.ArgumentParser(description="Live VLM exploration dashboard")
    parser.add_argument("session_dir",
                        help="Session log directory written by VLMExplorer")
    parser.add_argument("camera_png", nargs="?",
                        default="/tmp/vlm_robot_pov.png",
                        help="Path to live camera PNG (updated by main_vlm.py)")
    parser.add_argument("--fps", type=float, default=2.0,
                        help="Refresh rate in Hz (default: 2)")
    args = parser.parse_args()

    session_dir = Path(args.session_dir)
    camera_path = Path(args.camera_png)
    closed_flag = session_dir / "closed.flag"

    # ── Figure layout ────────────────────────────────────────────────────────
    fig = plt.figure(figsize=(14, 7.5), dpi=100)
    fig.canvas.manager.set_window_title("VLM Exploration Dashboard")
    fig.patch.set_facecolor("#1a1a2e")

    gs = gridspec.GridSpec(
        2, 2,
        height_ratios=[9, 1],
        hspace=0.06,
        wspace=0.04,
        left=0.02, right=0.98, top=0.96, bottom=0.02,
    )

    ax_map = fig.add_subplot(gs[0, 0])
    ax_cam = fig.add_subplot(gs[0, 1])
    ax_info = fig.add_subplot(gs[1, :])

    for ax in (ax_map, ax_cam, ax_info):
        ax.set_facecolor("#1a1a2e")
        ax.tick_params(left=False, bottom=False,
                       labelleft=False, labelbottom=False)
        for spine in ax.spines.values():
            spine.set_edgecolor("#444466")

    # Panel titles
    ax_map.set_title("VLM MAP VIEW  (last cycle)", color="#aaaacc",
                     fontsize=11, pad=4, loc="left")
    ax_cam.set_title("ROBOT CAMERA  (live)", color="#aaaacc",
                     fontsize=11, pad=4, loc="left")

    # Initial placeholder images
    ph_map = _placeholder("waiting for first VLM cycle…")
    ph_cam = _placeholder("waiting for camera frame…")

    im_map = ax_map.imshow(ph_map, aspect="auto")
    im_cam = ax_cam.imshow(ph_cam, aspect="auto")

    # Watermark text while waiting
    txt_map_wait = ax_map.text(
        0.5, 0.5, "waiting for first VLM cycle…",
        transform=ax_map.transAxes, ha="center", va="center",
        color="#666688", fontsize=12, style="italic"
    )
    txt_cam_wait = ax_cam.text(
        0.5, 0.5, "waiting for camera frame…",
        transform=ax_cam.transAxes, ha="center", va="center",
        color="#666688", fontsize=12, style="italic"
    )

    # Bottom info bar
    ax_info.set_xlim(0, 1)
    ax_info.set_ylim(0, 1)
    info_text = ax_info.text(
        0.01, 0.5, "No VLM decision yet.",
        transform=ax_info.transAxes,
        ha="left", va="center",
        color="#ccccee", fontsize=9.5, family="monospace",
        bbox=dict(facecolor="#252540", edgecolor="none", pad=4)
    )

    # Track last cycle so we don't re-read the same files.
    last_cycle_name: list[str] = [""]
    last_cam_mtime: list[float] = [0.0]

    def _fmt_decision(d: dict | None, session_dir: Path) -> str:
        if d is None:
            return "No VLM decision yet."
        cycle = d.get("cycle_id", "?")
        goal = d.get("goal")
        reason = d.get("reason", "")[:90]
        dur = d.get("duration_sec", 0.0)
        artifact = d.get("artifact_seen", False)

        # Count artifacts
        art_file = session_dir / "artifacts.json"
        n_arts = 0
        art_str = ""
        if art_file.exists():
            try:
                arts = json.loads(art_file.read_text())
                n_arts = len(arts)
                if arts:
                    last_art = arts[-1]
                    p = last_art.get("pos", [0, 0])
                    art_str = (f"  |  artifact #{n_arts}: ({p[0]:.1f}, {p[1]:.1f})"
                               f" — {last_art.get('reason','')[:40]}")
            except Exception:
                pass

        goal_str = (f"({goal[0]:.2f}, {goal[1]:.2f})" if isinstance(goal, (list, tuple))
                    else str(goal))
        return (f"Cycle {cycle}  |  goal: {goal_str}  |  {reason}  "
                f"[{dur:.1f}s]{art_str}")

    def update(_):
        if closed_flag.exists():
            plt.close(fig)
            return

        # ── Left: VLM map render ──────────────────────────────────────────
        map_path = _find_latest_cycle_file(session_dir, "map.png")
        cycle_name = map_path.name if map_path else ""
        if cycle_name != last_cycle_name[0]:
            img = _load_png(map_path)
            if img is not None:
                im_map.set_data(img)
                im_map.set_extent([0, img.shape[1], 0, img.shape[0]])
                im_map.axes.set_xlim(0, img.shape[1])
                im_map.axes.set_ylim(0, img.shape[0])
                txt_map_wait.set_visible(False)
                last_cycle_name[0] = cycle_name

            # Update decision bar
            decision = _load_decision(session_dir)
            info_text.set_text(_fmt_decision(decision, session_dir))

        # ── Right: live robot camera ──────────────────────────────────────
        try:
            cam_mtime = camera_path.stat().st_mtime if camera_path.exists() else 0.0
        except Exception:
            cam_mtime = 0.0

        if cam_mtime != last_cam_mtime[0]:
            img = _load_png(camera_path)
            if img is not None:
                im_cam.set_data(img)
                im_cam.set_extent([0, img.shape[1], 0, img.shape[0]])
                im_cam.axes.set_xlim(0, img.shape[1])
                im_cam.axes.set_ylim(0, img.shape[0])
                txt_cam_wait.set_visible(False)
                last_cam_mtime[0] = cam_mtime

        fig.canvas.draw_idle()

    interval_ms = int(1000.0 / max(args.fps, 0.5))
    anim = FuncAnimation(fig, update, interval=interval_ms,
                         blit=False, cache_frame_data=False)

    plt.tight_layout(pad=0.5)
    plt.show()


if __name__ == "__main__":
    main()
