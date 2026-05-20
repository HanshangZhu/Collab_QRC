#!/usr/bin/env python3
"""Live occupancy-map viewer — subprocess companion to standalone/main_champ.py.

Runs as a separate Python process (NOT mjpython) so the GUI lives on this
process's main thread, sidestepping macOS's "AppKit must be on main thread"
crash that hits matplotlib/Tk/Qt under mjpython's background pthread.

Reads a memmap written by standalone.observability.map_writer.MapWriter and
shows the live grid + robot + path + goal at ~10 Hz via matplotlib
FuncAnimation.

Usage (invoked by main_champ.py, but also runnable manually):
    python3 -m standalone.viz.map_viewer /tmp/standalone_map_XXXX.bin
"""
from __future__ import annotations

import argparse
import math
import struct
import sys
import time
from pathlib import Path

import numpy as np

import matplotlib
matplotlib.use("TkAgg")  # macOS-friendly default; child has its own main thread
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from matplotlib.colors import ListedColormap, BoundaryNorm
from matplotlib.lines import Line2D
from matplotlib.patches import FancyArrow, Circle

# Header offsets — must match MapWriter exactly.
MAGIC = 0xCAFE5A11
HEADER_SIZE = 256
STATUS_OFFSET = 52
STATUS_SIZE = 32
FRAME_OFFSET = 84


def _read_header(mm: np.ndarray) -> dict:
    magic, = struct.unpack_from("<I", mm, 0)
    if magic != MAGIC:
        raise RuntimeError(f"Bad magic: 0x{magic:08x}, expected 0x{MAGIC:08x}")
    width, = struct.unpack_from("<i", mm, 4)
    height, = struct.unpack_from("<i", mm, 8)
    resolution, = struct.unpack_from("<f", mm, 12)
    origin_x, = struct.unpack_from("<f", mm, 16)
    origin_y, = struct.unpack_from("<f", mm, 20)
    robot_x, = struct.unpack_from("<f", mm, 24)
    robot_y, = struct.unpack_from("<f", mm, 28)
    robot_yaw, = struct.unpack_from("<f", mm, 32)
    goal_x, = struct.unpack_from("<f", mm, 36)
    goal_y, = struct.unpack_from("<f", mm, 40)
    has_goal, = struct.unpack_from("<B", mm, 44)
    closed, = struct.unpack_from("<B", mm, 45)
    path_len, = struct.unpack_from("<i", mm, 48)
    status_bytes = bytes(mm[STATUS_OFFSET: STATUS_OFFSET + STATUS_SIZE])
    status = status_bytes.split(b"\0", 1)[0].decode("ascii", errors="replace")
    frame_id, = struct.unpack_from("<Q", mm, FRAME_OFFSET)
    return dict(
        width=width, height=height, resolution=resolution,
        origin_x=origin_x, origin_y=origin_y,
        robot_x=robot_x, robot_y=robot_y, robot_yaw=robot_yaw,
        goal_x=goal_x, goal_y=goal_y, has_goal=bool(has_goal),
        closed=bool(closed), path_len=path_len,
        status=status, frame_id=frame_id,
    )


def main() -> None:
    parser = argparse.ArgumentParser(description="Live occupancy map viewer")
    parser.add_argument("memmap_path", help="Path to the memmap file produced by MapWriter")
    parser.add_argument("--fps", type=float, default=10.0,
                        help="Refresh rate (Hz)")
    args = parser.parse_args()

    mm_path = Path(args.memmap_path)
    # Wait briefly for the parent to create the file.
    deadline = time.time() + 5.0
    while not mm_path.exists() and time.time() < deadline:
        time.sleep(0.05)
    if not mm_path.exists():
        print(f"[map_viewer] memmap not found: {mm_path}", file=sys.stderr)
        sys.exit(1)

    # Wait for header to be initialised (magic written).
    deadline = time.time() + 5.0
    while time.time() < deadline:
        try:
            with open(mm_path, "rb") as f:
                head = f.read(4)
            if len(head) == 4 and struct.unpack("<I", head)[0] == MAGIC:
                break
        except Exception:
            pass
        time.sleep(0.05)

    size = mm_path.stat().st_size
    mm = np.memmap(mm_path, dtype=np.uint8, mode="r", shape=(size,))
    hdr = _read_header(mm)
    W, H = hdr["width"], hdr["height"]
    res = hdr["resolution"]

    grid_start = HEADER_SIZE
    grid_end = grid_start + W * H
    path_view = mm[grid_end:].view(np.float32).reshape(-1, 2)
    grid_view = mm[grid_start: grid_end].view(np.int8).reshape(H, W)

    # ── Set up figure ───────────────────────────────────────────────────────
    fig, ax = plt.subplots(figsize=(7.5, 7.5), dpi=110)
    fig.canvas.manager.set_window_title("Standalone — Live Occupancy Map")

    # Colormap: -1 unknown=gray, 0 free=white, 100 occ=black
    cmap = ListedColormap(["#888888", "#f5f5f5", "#202020"])
    norm = BoundaryNorm([-1.5, -0.5, 50, 101], cmap.N)

    extent = (
        hdr["origin_x"],
        hdr["origin_x"] + W * res,
        hdr["origin_y"],
        hdr["origin_y"] + H * res,
    )
    im = ax.imshow(grid_view, cmap=cmap, norm=norm, origin="lower",
                   extent=extent, interpolation="nearest")

    path_line, = ax.plot([], [], "-", color="#00b050", linewidth=1.6, label="Path")
    goal_marker, = ax.plot([], [], "x", color="#d4a017", markersize=12,
                           markeredgewidth=2.5, label="Goal")
    robot_dot = Circle((0, 0), 0.18, color="#cc0000", zorder=10)
    ax.add_patch(robot_dot)
    heading_line, = ax.plot([], [], "-", color="#cc0000", linewidth=2.2)

    status_text = ax.text(
        0.02, 0.98, "", transform=ax.transAxes,
        ha="left", va="top", fontsize=9, family="monospace",
        bbox=dict(facecolor="white", alpha=0.7, edgecolor="none", pad=3))

    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_aspect("equal")
    ax.grid(True, linestyle=":", alpha=0.3)
    ax.legend(loc="upper right", fontsize=8, framealpha=0.85)

    last_frame_id = [-1]
    start_t = time.monotonic()

    def update(_):
        h = _read_header(mm)
        if h["closed"]:
            plt.close(fig)
            return im, path_line, goal_marker, robot_dot, heading_line, status_text
        if h["frame_id"] == last_frame_id[0]:
            return im, path_line, goal_marker, robot_dot, heading_line, status_text
        last_frame_id[0] = h["frame_id"]

        # Re-sync origin if recentred.
        new_extent = (
            h["origin_x"], h["origin_x"] + h["width"] * h["resolution"],
            h["origin_y"], h["origin_y"] + h["height"] * h["resolution"],
        )
        im.set_extent(new_extent)
        im.set_data(grid_view)

        rx, ry, ryaw = h["robot_x"], h["robot_y"], h["robot_yaw"]
        robot_dot.center = (rx, ry)
        hx = rx + 0.35 * math.cos(ryaw)
        hy = ry + 0.35 * math.sin(ryaw)
        heading_line.set_data([rx, hx], [ry, hy])

        n = max(0, min(h["path_len"], path_view.shape[0]))
        if n > 0:
            path_line.set_data(path_view[:n, 0], path_view[:n, 1])
        else:
            path_line.set_data([], [])

        if h["has_goal"]:
            goal_marker.set_data([h["goal_x"]], [h["goal_y"]])
        else:
            goal_marker.set_data([], [])

        elapsed = time.monotonic() - start_t
        status_text.set_text(
            f"status: {h['status'] or '-'}\n"
            f"frame:  {h['frame_id']}\n"
            f"robot:  ({rx:+.2f}, {ry:+.2f}, {math.degrees(ryaw):+.0f}°)\n"
            f"t:      {elapsed:6.1f} s"
        )

        return im, path_line, goal_marker, robot_dot, heading_line, status_text

    interval_ms = int(1000.0 / max(args.fps, 1.0))
    anim = FuncAnimation(fig, update, interval=interval_ms, blit=False,
                         cache_frame_data=False)
    plt.show()


if __name__ == "__main__":
    main()
