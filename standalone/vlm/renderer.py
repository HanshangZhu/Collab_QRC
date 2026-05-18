"""OccupancyMapper grid → annotated PNG bytes for VLM consumption.

Renders a top-down map with:
  - free / occupied / unknown cells
  - robot pose marker + heading
  - recent trajectory trail
  - frontier candidate markers (optional)
  - axis labels in metres
  - origin reference

Output: bytes of a PNG image, base64-encoded by caller if shipping to VLM.

Uses matplotlib's Agg backend (no display), safe under mjpython.
"""
from __future__ import annotations

import base64
import io
import math
from collections import deque
from typing import Iterable, List, Optional, Sequence, Tuple

import numpy as np

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.colors import ListedColormap, BoundaryNorm

from ..mapping.occupancy_grid import OccupancyMapper

Pose2D = Tuple[float, float, float]


class MapRenderer:
    """Render OccupancyMapper state to PNG bytes."""

    def __init__(
        self,
        mapper: OccupancyMapper,
        *,
        trail_max_len: int = 400,
        dpi: int = 96,
        fig_size_in: Tuple[float, float] = (6.0, 6.0),
    ) -> None:
        self._mapper = mapper
        self._trail: deque = deque(maxlen=trail_max_len)
        self._dpi = dpi
        self._figsize = fig_size_in

    # ── State input ─────────────────────────────────────────────────────────

    def append_trail(self, x: float, y: float) -> None:
        if not self._trail or math.hypot(x - self._trail[-1][0], y - self._trail[-1][1]) > 0.05:
            self._trail.append((float(x), float(y)))

    def clear_trail(self) -> None:
        self._trail.clear()

    # ── Render ──────────────────────────────────────────────────────────────

    def render_png(
        self,
        robot_pose: Pose2D,
        *,
        frontiers: Optional[Sequence[Tuple[float, float, float]]] = None,
        goal: Optional[Tuple[float, float]] = None,
        title: str = "Standalone exploration map",
    ) -> bytes:
        """Render the map + overlays to PNG bytes.

        frontiers: list of (x, y, info_gain). info_gain controls marker size.
        """
        m = self._mapper
        # int8 grid: -1 unknown, 0 free, 100 occupied
        grid = np.full((m.height, m.width), -1, dtype=np.int8)
        observed = m._observed
        log_odds = m._log_odds
        grid[observed & (log_odds <= m._free_thr)] = 0
        grid[observed & (log_odds >= m._occ_thr)] = 100

        extent = (
            m.origin_x,
            m.origin_x + m.width * m.resolution,
            m.origin_y,
            m.origin_y + m.height * m.resolution,
        )

        fig, ax = plt.subplots(figsize=self._figsize, dpi=self._dpi)
        cmap = ListedColormap(["#888888", "#f5f5f5", "#202020"])
        norm = BoundaryNorm([-1.5, -0.5, 50, 101], cmap.N)
        ax.imshow(grid, cmap=cmap, norm=norm, origin="lower",
                  extent=extent, interpolation="nearest")

        # Trail
        if len(self._trail) >= 2:
            xs = [p[0] for p in self._trail]
            ys = [p[1] for p in self._trail]
            ax.plot(xs, ys, "-", color="#1f6feb", linewidth=1.2, alpha=0.85,
                    label="trail")

        # Frontiers
        if frontiers:
            fx = [f[0] for f in frontiers]
            fy = [f[1] for f in frontiers]
            sizes = [max(20.0, min(120.0, 20.0 + 8.0 * f[2])) for f in frontiers]
            ax.scatter(fx, fy, s=sizes, c="#d4a017", marker="o",
                       edgecolors="black", linewidths=0.5,
                       alpha=0.85, label="frontiers")

        # Robot
        rx, ry, ryaw = robot_pose
        ax.plot([rx], [ry], "o", color="#cc0000", markersize=10,
                markeredgecolor="black", zorder=10)
        hx = rx + 0.4 * math.cos(ryaw)
        hy = ry + 0.4 * math.sin(ryaw)
        ax.plot([rx, hx], [ry, hy], "-", color="#cc0000", linewidth=2.2,
                zorder=10)

        # Goal
        if goal is not None:
            ax.plot([goal[0]], [goal[1]], "x", color="#00b050", markersize=14,
                    markeredgewidth=3.0, zorder=11, label="goal")

        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title(title, fontsize=10)
        ax.set_aspect("equal")
        ax.grid(True, linestyle=":", alpha=0.3)
        ax.legend(loc="upper right", fontsize=8, framealpha=0.85)

        buf = io.BytesIO()
        fig.tight_layout()
        fig.savefig(buf, format="png", dpi=self._dpi)
        plt.close(fig)
        return buf.getvalue()

    def render_b64(self, *args, **kwargs) -> str:
        return base64.b64encode(self.render_png(*args, **kwargs)).decode("ascii")

    @staticmethod
    def rgb_array_to_png(rgb: "np.ndarray") -> bytes:
        """Convert (H, W, 3) uint8 numpy array to PNG bytes via matplotlib imsave."""
        buf = io.BytesIO()
        plt.imsave(buf, rgb, format="png")
        return buf.getvalue()

    @staticmethod
    def rgb_array_to_b64(rgb: "np.ndarray") -> str:
        """Convert (H, W, 3) uint8 numpy array to base64 PNG string."""
        return base64.b64encode(MapRenderer.rgb_array_to_png(rgb)).decode("ascii")
