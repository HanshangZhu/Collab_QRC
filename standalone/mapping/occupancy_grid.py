"""2D log-odds occupancy grid — replaces octomap_server + map projection.

Ingests 3D world-frame point clouds from the LiDAR, projects them to 2D
(ground plane), and maintains a log-odds belief map.

Output: OccupancyGrid-compatible object (same field paths CFPA2 reads).
"""
from __future__ import annotations

import math
from typing import Tuple

import numpy as np

from ..core.ros_compat import OccupancyGrid, MapMetaData, Point, Header, now_stamp


class OccupancyMapper:
    """Rolling 2D occupancy map updated from 3D point clouds.

    Coordinate convention: map frame = world XY plane.
    Cell value semantics (int8): -1 = unknown, 0 = free, 100 = occupied.
    This matches nav_msgs/OccupancyGrid and what CFPA2 expects.
    """

    def __init__(
        self,
        *,
        resolution: float = 0.10,
        width_m: float = 20.0,
        height_m: float = 20.0,
        origin_x: float = 0.0,
        origin_y: float = 0.0,
        hit_log_odds: float = 0.9,
        miss_log_odds: float = -0.4,
        clip_min: float = -5.0,
        clip_max: float = 5.0,
        occ_threshold: float = 0.5,
        free_threshold: float = -0.5,
        max_scan_height: float = 1.0,   # m above robot; filter ceiling hits
        min_scan_height: float = -0.05, # m below robot; filter ground hits
    ) -> None:
        self.resolution = resolution
        self.width = int(math.ceil(width_m / resolution))
        self.height = int(math.ceil(height_m / resolution))
        self.origin_x = origin_x - (self.width  / 2) * resolution
        self.origin_y = origin_y - (self.height / 2) * resolution

        self._hit = hit_log_odds
        self._miss = miss_log_odds
        self._clip_min = clip_min
        self._clip_max = clip_max
        self._occ_thr = occ_threshold
        self._free_thr = free_threshold
        self._max_h = max_scan_height
        self._min_h = min_scan_height

        # Log-odds grid: float32
        self._log_odds = np.zeros((self.height, self.width), dtype=np.float32)
        # Known mask: True once a cell has been observed
        self._observed = np.zeros((self.height, self.width), dtype=bool)

    # ── Update ───────────────────────────────────────────────────────────────

    def update(self, points_world: np.ndarray,
               robot_pose: Tuple[float, float, float]) -> None:
        """Update map from a 3D world-frame point cloud.

        robot_pose = (x, y, yaw).  Points are in absolute world frame.
        """
        rx, ry, _ = robot_pose
        rz = 0.0  # assume robot height ~ 0 for now

        if points_world.size == 0:
            return

        pts = points_world.astype(np.float64)

        # Height filter: keep only points that are in a reasonable vertical band
        # around robot. pts[:, 2] is world-frame z.
        height_above_robot = pts[:, 2] - rz
        mask_h = (height_above_robot >= self._min_h) & \
                 (height_above_robot <= self._max_h)
        pts = pts[mask_h]
        if pts.size == 0:
            return

        # Cast ray from robot → each hit point, marking free cells along ray
        robot_cx, robot_cy = self._world_to_cell(rx, ry)

        for pt in pts:
            hx, hy = self._world_to_cell(pt[0], pt[1])
            self._trace_ray(robot_cx, robot_cy, hx, hy)

    def _world_to_cell(self, wx: float, wy: float) -> Tuple[int, int]:
        cx = int((wx - self.origin_x) / self.resolution)
        cy = int((wy - self.origin_y) / self.resolution)
        return cx, cy

    def _cell_to_world(self, cx: int, cy: int) -> Tuple[float, float]:
        wx = self.origin_x + (cx + 0.5) * self.resolution
        wy = self.origin_y + (cy + 0.5) * self.resolution
        return wx, wy

    def _in_bounds(self, cx: int, cy: int) -> bool:
        return 0 <= cx < self.width and 0 <= cy < self.height

    def _trace_ray(self, x0: int, y0: int, x1: int, y1: int) -> None:
        """Bresenham ray trace: miss updates along ray, hit update at end."""
        dx = abs(x1 - x0)
        dy = abs(y1 - y0)
        sx = 1 if x1 > x0 else -1
        sy = 1 if y1 > y0 else -1
        err = dx - dy
        x, y = x0, y0
        steps = 0
        max_steps = max(dx, dy) + 1

        while steps < max_steps:
            if x == x1 and y == y1:
                # Hit cell
                if self._in_bounds(x, y):
                    self._log_odds[y, x] = float(np.clip(
                        self._log_odds[y, x] + self._hit,
                        self._clip_min, self._clip_max))
                    self._observed[y, x] = True
                break
            else:
                # Free cell along ray
                if self._in_bounds(x, y):
                    self._log_odds[y, x] = float(np.clip(
                        self._log_odds[y, x] + self._miss,
                        self._clip_min, self._clip_max))
                    self._observed[y, x] = True

            e2 = 2 * err
            if e2 > -dy:
                err -= dy
                x += sx
            if e2 < dx:
                err += dx
                y += sy
            steps += 1

    # ── Output ───────────────────────────────────────────────────────────────

    def to_occupancy_grid(self) -> OccupancyGrid:
        """Produce an OccupancyGrid object compatible with CFPA2."""
        grid = OccupancyGrid()
        grid.header = Header(frame_id="map")
        grid.header.stamp = now_stamp()
        grid.info.resolution = self.resolution
        grid.info.width = self.width
        grid.info.height = self.height
        grid.info.origin.position.x = self.origin_x
        grid.info.origin.position.y = self.origin_y

        data = np.full(self.height * self.width, -1, dtype=np.int8)
        flat = self._log_odds.ravel()
        obs_flat = self._observed.ravel()

        # Free cells
        data[(obs_flat) & (flat <= self._free_thr)] = 0
        # Occupied cells
        data[(obs_flat) & (flat >= self._occ_thr)] = 100

        grid.data = data.tolist()
        return grid

    def get_log_odds_array(self) -> np.ndarray:
        """Raw (H, W) float32 log-odds grid for visualization."""
        return self._log_odds.copy()

    def get_observed_mask(self) -> np.ndarray:
        """(H, W) bool array — True where the cell has been observed."""
        return self._observed.copy()

    def recenter(self, robot_x: float, robot_y: float) -> None:
        """Shift map origin so robot stays near centre (optional, rolling map)."""
        new_ox = robot_x - (self.width  / 2) * self.resolution
        new_oy = robot_y - (self.height / 2) * self.resolution
        self.origin_x = new_ox
        self.origin_y = new_oy
        self._log_odds[:] = 0.0
        self._observed[:] = False
