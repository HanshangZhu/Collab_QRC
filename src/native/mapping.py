"""Occupancy grid mapper — native, no ROS.

Input per update:
  - sensor_xy: (2,) float — LiDAR sensor position in world XY plane
  - hits_world: (N,3) float32 — world-frame 3D LiDAR hit points from sensors.LiDARSensor

Output:
  - grid: np.ndarray[H,W] int8
      -1 = unknown, 0 = free, 100 = occupied

Core algorithm ported from:
  src/collaborative_exploration/go2_nav_algorithms/scripts/simple_scan_mapper.py
  (Bresenham ray carving + score-based evidence integration — identical logic)
"""
from __future__ import annotations

import math
from typing import Optional

import numpy as np


def _bresenham_cells(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
    """Bresenham line from (x0,y0) to (x1,y1), endpoints inclusive."""
    pts: list[tuple[int, int]] = []
    dx, dy = abs(x1 - x0), abs(y1 - y0)
    x, y = x0, y0
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    if dx > dy:
        err = dx / 2.0
        while x != x1:
            pts.append((x, y))
            err -= dy
            if err < 0:
                y += sy
                err += dx
            x += sx
    else:
        err = dy / 2.0
        while y != y1:
            pts.append((x, y))
            err -= dx
            if err < 0:
                x += sx
                err += dy
            y += sy
    pts.append((x1, y1))
    return pts


class OccupancyMapper:
    """2D occupancy grid updated from world-frame LiDAR hit points."""

    def __init__(self,
                 resolution: float = 0.05,
                 width: int = 500,
                 height: int = 500,
                 origin_x: float = -12.5,
                 origin_y: float = -12.5,
                 max_range: float = 20.0,
                 max_clear_distance: float = 5.0,
                 hit_increment: int = 3,
                 miss_decrement: int = 1,
                 score_min: int = -20,
                 score_max: int = 20,
                 occupied_threshold: int = 3,
                 free_threshold: int = -3):
        self.res = resolution
        self.W = width
        self.H = height
        self.ox = origin_x
        self.oy = origin_y
        self.max_range = max_range
        self.max_clear = max_clear_distance
        self.hit_inc = hit_increment
        self.miss_dec = miss_decrement
        self.s_min = score_min
        self.s_max = score_max
        self.occ_thresh = occupied_threshold
        self.free_thresh = free_threshold

        self._scores = np.zeros((height, width), dtype=np.int16)
        self._observed = np.zeros((height, width), dtype=bool)

    # ── Core update ──────────────────────────────────────────────────────────

    def update(self, sensor_xy: np.ndarray, hits_world: np.ndarray) -> None:
        """Update map from one LiDAR sweep.

        sensor_xy: (2,) world-frame sensor origin
        hits_world: (N,3) or (N,2) world-frame hit points (only XY used)
        """
        sx, sy = float(sensor_xy[0]), float(sensor_xy[1])
        ogx, ogy = self._world_to_grid(sx, sy)
        if ogx is None:
            return

        if hits_world.shape[1] >= 2:
            hit_xy = hits_world[:, :2]
        else:
            return

        for i in range(len(hit_xy)):
            hx, hy = float(hit_xy[i, 0]), float(hit_xy[i, 1])
            dist = math.hypot(hx - sx, hy - sy)
            if dist < 0.01 or dist > self.max_range:
                continue

            # Endpoint cell
            gx, gy = self._world_to_grid(hx, hy)

            # Clear-ray endpoint (capped at max_clear_distance)
            clear_dist = min(dist, self.max_clear)
            t = clear_dist / dist
            cex = sx + (hx - sx) * t
            cey = sy + (hy - sy) * t
            cgx, cgy = self._world_to_grid(cex, cey)
            if cgx is None:
                continue

            # Carve free cells along ray (exclude last cell)
            cells = _bresenham_cells(ogx, ogy, cgx, cgy)
            for cx, cy in cells[:-1]:
                self._apply(cy, cx, -self.miss_dec)

            # Mark hit
            if gx is not None:
                self._apply(gy, gx, self.hit_inc)

    def _world_to_grid(self, x: float, y: float) -> tuple[Optional[int], Optional[int]]:
        gx = int((x - self.ox) / self.res)
        gy = int((y - self.oy) / self.res)
        if 0 <= gx < self.W and 0 <= gy < self.H:
            return gx, gy
        return None, None

    def _apply(self, row: int, col: int, delta: int) -> None:
        self._observed[row, col] = True
        s = int(self._scores[row, col]) + delta
        self._scores[row, col] = max(self.s_min, min(self.s_max, s))

    # ── Read-out ─────────────────────────────────────────────────────────────

    @property
    def grid(self) -> np.ndarray:
        """Returns (H,W) int8 grid: -1=unknown, 0=free, 100=occupied."""
        g = np.full((self.H, self.W), -1, dtype=np.int8)
        g[self._observed & (self._scores >= self.occ_thresh)] = 100
        g[self._observed & (self._scores <= self.free_thresh)] = 0
        return g

    @property
    def coverage_ratio(self) -> float:
        """Fraction of cells observed (free or occupied) out of total."""
        return float(np.sum(self._observed)) / (self.W * self.H)

    def grid_to_world(self, gx: int, gy: int) -> tuple[float, float]:
        return self.ox + (gx + 0.5) * self.res, self.oy + (gy + 0.5) * self.res

    def reset(self) -> None:
        self._scores[:] = 0
        self._observed[:] = False
