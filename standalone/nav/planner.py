"""A* planner on a 2D occupancy grid — replaces Nav2 SmacPlannerLattice.

Input:  OccupancyGrid (from OccupancyMapper.to_occupancy_grid())
        start (x, y) and goal (x, y) in world metres.
Output: list of (x, y) waypoints in world metres, or [] if no path found.

Inflation is applied to the occupancy grid before planning: any cell within
`inflation_cells` of an occupied cell is treated as blocked.  This gives a
conservative clearance equivalent to Nav2's costmap inflation layer.
"""
from __future__ import annotations

import heapq
import math
from typing import List, Optional, Tuple

import numpy as np

from ..core.ros_compat import OccupancyGrid

Cell = Tuple[int, int]
Path = List[Tuple[float, float]]


class AStarPlanner:
    """4-connected A* with Euclidean heuristic on an OccupancyGrid."""

    OCC_THRESHOLD = 50   # cells ≥ this are blocked (matches CFPA2 occ_thresh)
    UNKNOWN_PASSABLE = True   # allow traversal through unknown space (exploration)

    def __init__(self, inflation_cells: int = 3) -> None:
        self.inflation_cells = inflation_cells
        self._last_grid: Optional[OccupancyGrid] = None
        self._inflated: Optional[np.ndarray] = None

    # ── Public API ────────────────────────────────────────────────────────────

    def plan(self, grid: OccupancyGrid,
             start: Tuple[float, float],
             goal: Tuple[float, float]) -> Path:
        """Return path (list of world-frame (x, y)) from start to goal.

        Returns [] if no path exists.
        """
        if grid is None or not grid.data:
            return []

        raw = self._grid_array(grid)
        inflated = self._inflate(raw)

        sx, sy = self._world_to_cell(grid, start[0], start[1])
        gx, gy = self._world_to_cell(grid, goal[0], goal[1])

        W, H = grid.info.width, grid.info.height
        if not (0 <= sx < W and 0 <= sy < H):
            return []
        if not (0 <= gx < W and 0 <= gy < H):
            return []

        cell_path = self._astar(inflated, (sx, sy), (gx, gy), W, H)
        if not cell_path:
            return []

        return [self._cell_to_world(grid, cx, cy) for cx, cy in cell_path]

    # ── Grid helpers ──────────────────────────────────────────────────────────

    @staticmethod
    def _grid_array(grid: OccupancyGrid) -> np.ndarray:
        """Return (H, W) int8 array matching grid.data row-major order."""
        return np.array(grid.data, dtype=np.int8).reshape(
            grid.info.height, grid.info.width)

    def _inflate(self, raw: np.ndarray) -> np.ndarray:
        """Binary dilation: any cell within `inflation_cells` of occ → blocked."""
        if self.inflation_cells <= 0:
            return raw.copy()
        from scipy.ndimage import binary_dilation
        occ = (raw >= self.OCC_THRESHOLD)
        struct = np.ones((2 * self.inflation_cells + 1,) * 2, dtype=bool)
        dilated = binary_dilation(occ, structure=struct)
        result = raw.copy()
        result[dilated] = 100
        return result

    @staticmethod
    def _world_to_cell(grid: OccupancyGrid,
                       wx: float, wy: float) -> Tuple[int, int]:
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        cx = int((wx - ox) / res)
        cy = int((wy - oy) / res)
        return cx, cy

    @staticmethod
    def _cell_to_world(grid: OccupancyGrid,
                       cx: int, cy: int) -> Tuple[float, float]:
        res = grid.info.resolution
        ox = grid.info.origin.position.x
        oy = grid.info.origin.position.y
        return (ox + (cx + 0.5) * res, oy + (cy + 0.5) * res)

    def _passable(self, grid: np.ndarray, cx: int, cy: int) -> bool:
        v = int(grid[cy, cx])
        if v >= self.OCC_THRESHOLD:
            return False
        if v == -1 and not self.UNKNOWN_PASSABLE:
            return False
        return True

    # ── A* ────────────────────────────────────────────────────────────────────

    def _astar(self, grid: np.ndarray, start: Cell, goal: Cell,
               W: int, H: int) -> List[Cell]:
        if not self._passable(grid, start[0], start[1]):
            return []
        if not self._passable(grid, goal[0], goal[1]):
            return []

        def h(c: Cell) -> float:
            return math.hypot(c[0] - goal[0], c[1] - goal[1])

        open_heap: list = []
        heapq.heappush(open_heap, (h(start), 0.0, start))
        came_from: dict[Cell, Optional[Cell]] = {start: None}
        g_score: dict[Cell, float] = {start: 0.0}

        neighbors = [(1, 0), (-1, 0), (0, 1), (0, -1)]

        while open_heap:
            _, g, cur = heapq.heappop(open_heap)
            if cur == goal:
                return self._reconstruct(came_from, goal)

            if g > g_score.get(cur, float("inf")) + 1e-9:
                continue

            for dx, dy in neighbors:
                nx, ny = cur[0] + dx, cur[1] + dy
                nb: Cell = (nx, ny)
                if not (0 <= nx < W and 0 <= ny < H):
                    continue
                if not self._passable(grid, nx, ny):
                    continue
                ng = g + 1.0
                if ng < g_score.get(nb, float("inf")):
                    g_score[nb] = ng
                    came_from[nb] = cur
                    heapq.heappush(open_heap, (ng + h(nb), ng, nb))

        return []

    @staticmethod
    def _reconstruct(came_from: dict, goal: Cell) -> List[Cell]:
        path: List[Cell] = []
        node: Optional[Cell] = goal
        while node is not None:
            path.append(node)
            node = came_from[node]
        path.reverse()
        # Thin the path: keep every 3rd cell to reduce waypoint density
        return path[::3] + ([path[-1]] if len(path) % 3 != 1 else [])
