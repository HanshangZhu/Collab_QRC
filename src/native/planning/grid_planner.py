"""Grid planner on the global occupancy grid — D* Lite with A* fallback.

Plans in world coordinates using the OccupancyGrid published by
simple_scan_mapper_cpp.  Uses D* Lite for incremental replanning when
only the map changes (avoids full A* restarts on every replan cycle).
Falls back to A* when the goal changes or D* Lite is unavailable.

Performance notes:
- D* Lite incrementally repairs the search tree on map changes (~1-5ms).
- Full A* only runs on goal change or first plan.
- A* inner loop can run in C++ via ctypes (~5ms for a 16m path on 400×400 grid).
- Inflation uses scipy.ndimage.binary_dilation (~9ms).
- Proximity cost gradient uses distance_transform_edt (~2ms) to push paths
  away from walls/corners.
- Planning runs in a background thread so it never blocks the control loop.
"""

from __future__ import annotations

import ctypes
import heapq
import math
import os
import threading
from dataclasses import dataclass, field
from typing import Optional

import numpy as np

# Try scipy for fast dilation + distance transform; fall back to manual numpy if unavailable
try:
    from scipy.ndimage import binary_dilation, distance_transform_edt

    _HAS_SCIPY = True
except ImportError:
    _HAS_SCIPY = False

# Load C++ A* shared library
_astar_lib = None
try:
    _so_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "astar_grid.so")
    _astar_lib = ctypes.CDLL(_so_path)
    _astar_lib.astar_grid.restype = ctypes.c_int
    _astar_lib.astar_grid.argtypes = [
        ctypes.POINTER(ctypes.c_uint8),  # blocked
        ctypes.c_int, ctypes.c_int,       # W, H
        ctypes.c_int, ctypes.c_int,       # sx, sy
        ctypes.c_int, ctypes.c_int,       # gx, gy
        ctypes.POINTER(ctypes.c_int),     # path_x_out
        ctypes.POINTER(ctypes.c_int),     # path_y_out
        ctypes.c_int,                     # max_path_len
        ctypes.c_int,                     # max_cells
        ctypes.POINTER(ctypes.c_int),     # cells_explored_out
        ctypes.POINTER(ctypes.c_uint8),   # cost_grid (nullable)
    ]
except Exception:
    _astar_lib = None


@dataclass
class OccGridInfo:
    """Lightweight mirror of nav_msgs/OccupancyGrid metadata."""

    resolution: float
    width: int
    height: int
    origin_x: float
    origin_y: float
    data: np.ndarray  # 2D bool array: True = blocked


@dataclass
class GridPlanResult:
    waypoints_world: list[tuple[float, float]] = field(default_factory=list)
    success: bool = False
    cells_explored: int = 0
    time_ms: float = 0.0


def _inflate_grid(occupied: np.ndarray, inflate_cells: int) -> np.ndarray:
    """Return a boolean grid with obstacles inflated by `inflate_cells`."""
    if inflate_cells <= 0:
        return occupied.copy()

    if _HAS_SCIPY:
        # Build circular structuring element
        d = 2 * inflate_cells + 1
        y, x = np.ogrid[-inflate_cells : inflate_cells + 1, -inflate_cells : inflate_cells + 1]
        kernel = (x * x + y * y) <= inflate_cells * inflate_cells
        return binary_dilation(occupied, structure=kernel).astype(bool)

    # Numpy fallback: shift-and-OR for each offset in the inflation disc
    result = occupied.copy()
    for dy in range(-inflate_cells, inflate_cells + 1):
        for dx in range(-inflate_cells, inflate_cells + 1):
            if dx * dx + dy * dy > inflate_cells * inflate_cells:
                continue
            shifted = np.roll(np.roll(occupied, dy, axis=0), dx, axis=1)
            if dy > 0:
                shifted[:dy, :] = False
            elif dy < 0:
                shifted[dy:, :] = False
            if dx > 0:
                shifted[:, :dx] = False
            elif dx < 0:
                shifted[:, dx:] = False
            result |= shifted
    return result


def _compute_cost_map(
    blocked: np.ndarray,
    raw_occupied: np.ndarray,
    decay_cells: int,
    cost_weight: float = 1.0,
) -> np.ndarray:
    """Compute a uint8 proximity cost map using distance transform.

    Cells within `decay_cells` of any raw obstacle (pre-inflation) get
    a traversal penalty that decreases linearly with distance.  Blocked
    cells are set to 0 (they're impassable anyway).

    Returns uint8 array [0, 252] where 252 = maximum penalty.
    """
    if decay_cells <= 0 or not _HAS_SCIPY:
        return np.zeros_like(blocked, dtype=np.uint8)

    # Distance from each free cell to nearest obstacle (in cells)
    dist = distance_transform_edt(~raw_occupied)

    # Linear decay: cost = weight * max(0, 1 - dist / decay_cells)
    cost_float = np.clip(1.0 - dist / decay_cells, 0.0, 1.0) * cost_weight

    # Scale to uint8 [0, 252] and zero out blocked cells
    cost_u8 = (cost_float * 252.0).astype(np.uint8)
    cost_u8[blocked] = 0
    return cost_u8


def occupancy_to_blocked(
    data: list[int] | np.ndarray,
    width: int,
    height: int,
    inflate_cells: int = 0,
) -> np.ndarray:
    """Convert ROS OccupancyGrid.data to inflated 2D boolean array."""
    arr = np.array(data, dtype=np.int8).reshape(height, width)
    occupied = arr == 100  # OccupancyGrid: 100 = occupied
    if inflate_cells > 0:
        occupied = _inflate_grid(occupied, inflate_cells)
    return occupied


def _astar_cpp(blocked: np.ndarray, sx: int, sy: int, gx: int, gy: int,
               max_cells: int,
               cost_map: np.ndarray | None = None) -> tuple[list[tuple[int, int]], int]:
    """Run A* via C++ shared library. Returns (path_cells, cells_explored)."""
    H, W = blocked.shape
    # Ensure contiguous uint8 row-major
    grid_flat = np.ascontiguousarray(blocked.astype(np.uint8)).ravel()

    max_path = W * H  # worst case
    if max_path > 200000:
        max_path = 200000
    path_x = (ctypes.c_int * max_path)()
    path_y = (ctypes.c_int * max_path)()
    explored = ctypes.c_int(0)

    # Prepare cost grid pointer (nullable)
    cost_ptr = None
    if cost_map is not None:
        cost_flat = np.ascontiguousarray(cost_map.astype(np.uint8)).ravel()
        cost_ptr = cost_flat.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8))

    path_len = _astar_lib.astar_grid(
        grid_flat.ctypes.data_as(ctypes.POINTER(ctypes.c_uint8)),
        W, H, sx, sy, gx, gy,
        path_x, path_y, max_path, max_cells,
        ctypes.byref(explored),
        cost_ptr,
    )

    cells = [(path_x[i], path_y[i]) for i in range(path_len)]
    return cells, explored.value


def _astar_python(blocked: np.ndarray, sx: int, sy: int, gx: int, gy: int,
                  max_cells: int,
                  cost_map: np.ndarray | None = None) -> tuple[list[tuple[int, int]], int]:
    """Pure-Python A* fallback."""
    H, W = blocked.shape
    SQRT2 = 1.4142135623730951
    COST_SCALE = 1.0 / 252.0
    neighbors = (
        (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
        (-1, -1, SQRT2), (-1, 1, SQRT2), (1, -1, SQRT2), (1, 1, SQRT2),
    )

    open_heap: list[tuple[float, float, int, int]] = []
    heapq.heappush(open_heap, (math.hypot(sx - gx, sy - gy), 0.0, sx, sy))
    g_score = np.full((H, W), np.inf, dtype=np.float32)
    g_score[sy, sx] = 0.0
    closed = np.zeros((H, W), dtype=bool)
    came_from = {}

    found = False
    cells_explored = 0

    while open_heap and cells_explored < max_cells:
        _, cur_g, x, y = heapq.heappop(open_heap)
        if closed[y, x]:
            continue
        closed[y, x] = True
        cells_explored += 1

        if x == gx and y == gy:
            found = True
            break

        for dx, dy, cost in neighbors:
            nx, ny = x + dx, y + dy
            if not (0 <= nx < W and 0 <= ny < H):
                continue
            if blocked[ny, nx] or closed[ny, nx]:
                continue
            ng = cur_g + cost
            if cost_map is not None:
                ng += float(cost_map[ny, nx]) * COST_SCALE
            if ng < g_score[ny, nx]:
                g_score[ny, nx] = ng
                came_from[(nx, ny)] = (x, y)
                heapq.heappush(open_heap, (ng + math.hypot(nx - gx, ny - gy), ng, nx, ny))

    if not found:
        return [], cells_explored

    path_cells = []
    c = (gx, gy)
    while c in came_from:
        path_cells.append(c)
        c = came_from[c]
    path_cells.append((sx, sy))
    path_cells.reverse()
    return path_cells, cells_explored


def _elastic_band_smooth(
    path_world: list[tuple[float, float]],
    blocked: np.ndarray,
    origin_x: float,
    origin_y: float,
    resolution: float,
    iterations: int = 5,
    smooth_weight: float = 0.3,
    obstacle_weight: float = 0.15,
    min_clearance_cells: float = 5.0,
) -> list[tuple[float, float]]:
    """Elastic band path optimization — smooth + push away from obstacles.

    For each iteration, each interior waypoint is moved by:
      F_smooth  = smooth_weight * (midpoint_of_neighbors - waypoint)
      F_repel   = obstacle_weight * repulsion  (only if within min_clearance_cells)

    Repulsion uses the distance transform gradient to push perpendicular to
    the nearest obstacle surface.  Collision-checked: if the new position
    lands in a blocked cell, the move is skipped.

    This is a one-shot post-process — deterministic, ~0.5ms, no replanning.
    """
    if len(path_world) < 3:
        return list(path_world)

    H, W = blocked.shape

    # Compute distance transform on the free space (distance to nearest blocked cell)
    if _HAS_SCIPY:
        dist_field = distance_transform_edt(~blocked)
    else:
        # Fallback: no repulsion, just smoothing
        dist_field = np.full((H, W), min_clearance_cells + 1, dtype=np.float32)

    # Precompute gradient of distance field (points away from obstacles)
    # gradient[0] = d(dist)/d(row), gradient[1] = d(dist)/d(col)
    grad_y = np.zeros_like(dist_field)
    grad_x = np.zeros_like(dist_field)
    grad_y[1:-1, :] = (dist_field[2:, :] - dist_field[:-2, :]) / 2.0
    grad_x[:, 1:-1] = (dist_field[:, 2:] - dist_field[:, :-2]) / 2.0

    inv_res = 1.0 / resolution

    def w2c(wx: float, wy: float) -> tuple[int, int]:
        return (int(math.floor((wx - origin_x) * inv_res)),
                int(math.floor((wy - origin_y) * inv_res)))

    def is_free(cx: int, cy: int) -> bool:
        return 0 <= cx < W and 0 <= cy < H and not blocked[cy, cx]

    def get_dist(cx: int, cy: int) -> float:
        if 0 <= cx < W and 0 <= cy < H:
            return float(dist_field[cy, cx])
        return 0.0

    def get_grad(cx: int, cy: int) -> tuple[float, float]:
        """Returns gradient in world coords (gx_world, gy_world)."""
        if 0 <= cx < W and 0 <= cy < H:
            return float(grad_x[cy, cx]), float(grad_y[cy, cx])
        return 0.0, 0.0

    # Work with mutable list of [x, y]
    pts = [[p[0], p[1]] for p in path_world]
    n = len(pts)

    for _ in range(iterations):
        for i in range(1, n - 1):  # skip start and end
            px, py = pts[i]

            # Smoothing force: pull toward midpoint of neighbors
            mx = (pts[i - 1][0] + pts[i + 1][0]) * 0.5
            my = (pts[i - 1][1] + pts[i + 1][1]) * 0.5
            fx = smooth_weight * (mx - px)
            fy = smooth_weight * (my - py)

            # Obstacle repulsion force
            cx, cy_cell = w2c(px, py)
            d = get_dist(cx, cy_cell)
            if d < min_clearance_cells and d > 0.1:
                # Repulsion magnitude: stronger when closer
                strength = obstacle_weight * (1.0 / d - 1.0 / min_clearance_cells)
                gx_w, gy_w = get_grad(cx, cy_cell)
                gn = math.hypot(gx_w, gy_w)
                if gn > 1e-6:
                    # Gradient points away from obstacles — scale by strength
                    fx += strength * (gx_w / gn) * resolution
                    fy += strength * (gy_w / gn) * resolution

            # Apply force
            new_x = px + fx
            new_y = py + fy

            # Collision check
            ncx, ncy = w2c(new_x, new_y)
            if is_free(ncx, ncy):
                pts[i][0] = new_x
                pts[i][1] = new_y

    return [(p[0], p[1]) for p in pts]


def plan_on_grid(
    info: OccGridInfo,
    robot_x: float,
    robot_y: float,
    goal_x: float,
    goal_y: float,
    inflation_m: float = 0.25,
    waypoint_spacing_m: float = 0.5,
    max_cells: int = 80000,
    decay_m: float = 0.0,
    cost_weight: float = 1.0,
    cost_map: np.ndarray | None = None,
) -> GridPlanResult:
    """Run A* from robot to goal on the occupancy grid."""
    import time

    t0 = time.monotonic()
    result = GridPlanResult()

    inflate_cells = max(0, int(math.ceil(inflation_m / info.resolution)))
    blocked = info.data

    if blocked.dtype != bool:
        blocked = occupancy_to_blocked(blocked.ravel(), info.width, info.height, inflate_cells)
    elif inflate_cells > 0 and not hasattr(info, "_inflated"):
        blocked = _inflate_grid(blocked, inflate_cells)

    H, W = blocked.shape

    def w2c(wx: float, wy: float) -> tuple[int, int] | None:
        cx = int(math.floor((wx - info.origin_x) / info.resolution))
        cy = int(math.floor((wy - info.origin_y) / info.resolution))
        if 0 <= cx < W and 0 <= cy < H:
            return (cx, cy)
        return None

    start_cell = w2c(robot_x, robot_y)
    goal_cell = w2c(goal_x, goal_y)
    if start_cell is None or goal_cell is None:
        result.time_ms = (time.monotonic() - t0) * 1000
        return result

    sx, sy = start_cell
    if blocked[sy, sx]:
        r = inflate_cells + 2
        y_lo, y_hi = max(0, sy - r), min(H, sy + r + 1)
        x_lo, x_hi = max(0, sx - r), min(W, sx + r + 1)
        blocked = blocked.copy()
        blocked[y_lo:y_hi, x_lo:x_hi] = False

    gx, gy = goal_cell
    if blocked[gy, gx]:
        goal_cell = _nearest_free(blocked, gx, gy, W, H, search_radius=15)
        if goal_cell is None:
            result.time_ms = (time.monotonic() - t0) * 1000
            return result
        gx, gy = goal_cell

    # Run A* — C++ if available, else Python
    if _astar_lib is not None:
        path_cells, cells_explored = _astar_cpp(blocked, sx, sy, gx, gy, max_cells, cost_map)
    else:
        path_cells, cells_explored = _astar_python(blocked, sx, sy, gx, gy, max_cells, cost_map)

    result.cells_explored = cells_explored

    if not path_cells:
        result.time_ms = (time.monotonic() - t0) * 1000
        return result

    if len(path_cells) < 2:
        result.success = True
        result.time_ms = (time.monotonic() - t0) * 1000
        return result

    # Convert to world, resample, then smooth corners
    res = info.resolution
    ox, oy = info.origin_x, info.origin_y
    path_world = [(ox + (cx + 0.5) * res, oy + (cy + 0.5) * res) for cx, cy in path_cells]

    resampled = [path_world[0]]
    accum = 0.0
    for i in range(1, len(path_world)):
        dx = path_world[i][0] - path_world[i - 1][0]
        dy = path_world[i][1] - path_world[i - 1][1]
        accum += math.hypot(dx, dy)
        if accum >= waypoint_spacing_m:
            resampled.append(path_world[i])
            accum = 0.0
    if resampled[-1] != path_world[-1]:
        resampled.append(path_world[-1])

    # Elastic band smoothing on resampled waypoints
    resampled = _elastic_band_smooth(resampled, blocked, ox, oy, res)

    result.waypoints_world = resampled[1:]  # skip start (robot pos)
    result.success = True
    result.time_ms = (time.monotonic() - t0) * 1000
    return result


def _nearest_free(
    blocked: np.ndarray,
    gx: int,
    gy: int,
    W: int,
    H: int,
    search_radius: int = 10,
) -> tuple[int, int] | None:
    best = None
    best_d = float("inf")
    for dy in range(-search_radius, search_radius + 1):
        for dx in range(-search_radius, search_radius + 1):
            nx, ny = gx + dx, gy + dy
            if 0 <= nx < W and 0 <= ny < H and not blocked[ny, nx]:
                d = dx * dx + dy * dy
                if d < best_d:
                    best_d = d
                    best = (nx, ny)
    return best


class AsyncGridPlanner:
    """D* Lite grid planner with A* fallback; runs in a background thread.

    When the goal stays the same and only the map changes, D* Lite
    incrementally repairs the search tree instead of running a full A*.
    A full restart only happens on goal change or first plan.
    """

    def __init__(
        self,
        inflation_m: float = 0.25,
        waypoint_spacing_m: float = 0.5,
        replan_interval_sec: float = 2.0,
        goal_shift_threshold_m: float = 0.3,
        use_dstar_lite: bool = True,
        decay_m: float = 0.6,
        cost_weight: float = 1.0,
    ):
        self.inflation_m = inflation_m
        self.waypoint_spacing_m = waypoint_spacing_m
        self.replan_interval_sec = replan_interval_sec
        self.goal_shift_threshold_m = goal_shift_threshold_m
        self.use_dstar_lite = use_dstar_lite
        self.decay_m = decay_m
        self.cost_weight = cost_weight

        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None
        self._last_result: GridPlanResult | None = None
        self._last_plan_time: float | None = None
        self._last_plan_goal: tuple[float, float] | None = None

        # Cache inflated grid and cost map to avoid recomputing every cycle
        self._cached_blocked: np.ndarray | None = None
        self._cached_cost_map: np.ndarray | None = None
        self._cached_raw_occupied: np.ndarray | None = None
        self._cached_map_stamp: float | None = None

        # D* Lite persistent state
        self._dstar: Optional["DStarLite"] = None
        self._prev_blocked: np.ndarray | None = None
        self._dstar_goal_cell: tuple[int, int] | None = None

    def request_plan(
        self,
        now_sec: float,
        info: OccGridInfo,
        robot_x: float,
        robot_y: float,
        goal_x: float,
        goal_y: float,
        map_stamp_sec: float = 0.0,
    ) -> GridPlanResult | None:
        """Non-blocking: start a plan if needed, return latest result if ready."""
        goal_xy = (goal_x, goal_y)

        # Check if we need to re-plan
        need_replan = False
        if self._last_plan_goal is None:
            need_replan = True
        elif self._last_plan_time is None or (now_sec - self._last_plan_time) >= self.replan_interval_sec:
            need_replan = True
        elif math.hypot(goal_xy[0] - self._last_plan_goal[0], goal_xy[1] - self._last_plan_goal[1]) > self.goal_shift_threshold_m:
            need_replan = True

        if need_replan and (self._thread is None or not self._thread.is_alive()):
            self._last_plan_time = now_sec
            self._last_plan_goal = goal_xy

            # Pre-compute inflated grid + cost map (cache if map hasn't changed)
            map_changed = False
            if self._cached_blocked is None or self._cached_map_stamp != map_stamp_sec:
                inflate_cells = max(0, int(math.ceil(self.inflation_m / info.resolution)))
                # Extract raw occupied before inflation for distance transform
                if info.data.dtype == bool:
                    raw_occupied = info.data
                    blocked = _inflate_grid(raw_occupied, inflate_cells) if inflate_cells > 0 else raw_occupied.copy()
                else:
                    arr = np.array(info.data.ravel(), dtype=np.int8).reshape(info.height, info.width)
                    raw_occupied = arr == 100
                    blocked = _inflate_grid(raw_occupied, inflate_cells) if inflate_cells > 0 else raw_occupied.copy()
                self._cached_blocked = blocked
                self._cached_raw_occupied = raw_occupied
                # Compute proximity cost map
                decay_cells = max(0, int(math.ceil(self.decay_m / info.resolution)))
                self._cached_cost_map = _compute_cost_map(
                    blocked, raw_occupied, decay_cells, self.cost_weight,
                )
                self._cached_map_stamp = map_stamp_sec
                map_changed = True

            blocked_snapshot = self._cached_blocked
            cost_map_snapshot = self._cached_cost_map

            if self.use_dstar_lite:
                self._run_dstar_plan(
                    info, blocked_snapshot, robot_x, robot_y,
                    goal_x, goal_y, map_changed, cost_map_snapshot,
                )
            else:
                # Legacy A* path
                plan_info = OccGridInfo(
                    resolution=info.resolution,
                    width=info.width,
                    height=info.height,
                    origin_x=info.origin_x,
                    origin_y=info.origin_y,
                    data=blocked_snapshot,
                )
                _cost_snap = cost_map_snapshot

                def _run():
                    r = plan_on_grid(
                        plan_info, robot_x, robot_y, goal_x, goal_y,
                        inflation_m=0.0,
                        waypoint_spacing_m=self.waypoint_spacing_m,
                        cost_map=_cost_snap,
                    )
                    with self._lock:
                        self._last_result = r

                self._thread = threading.Thread(target=_run, daemon=True)
                self._thread.start()

        # Return latest completed result
        with self._lock:
            r = self._last_result
            self._last_result = None
            return r

    def _run_dstar_plan(
        self,
        info: OccGridInfo,
        blocked: np.ndarray,
        robot_x: float,
        robot_y: float,
        goal_x: float,
        goal_y: float,
        map_changed: bool,
        cost_map: np.ndarray | None = None,
    ) -> None:
        """Launch D* Lite planning in background thread."""
        from .dstar_lite import DStarLite

        H, W = blocked.shape
        res = info.resolution
        ox, oy = info.origin_x, info.origin_y

        def w2c(wx: float, wy: float) -> tuple[int, int] | None:
            cx = int(math.floor((wx - ox) / res))
            cy = int(math.floor((wy - oy) / res))
            if 0 <= cx < W and 0 <= cy < H:
                return (cx, cy)
            return None

        start_cell = w2c(robot_x, robot_y)
        goal_cell = w2c(goal_x, goal_y)
        if start_cell is None or goal_cell is None:
            return

        sx, sy = start_cell
        gx, gy = goal_cell

        # Determine if we need full re-init or incremental update
        goal_changed = (self._dstar_goal_cell is None or
                        self._dstar_goal_cell != (gx, gy))
        grid_resized = (self._dstar is not None and
                        (self._dstar.W != W or self._dstar.H != H))
        need_full_init = (self._dstar is None or goal_changed or grid_resized)

        # Capture state for background thread
        prev_blocked = self._prev_blocked
        dstar = self._dstar
        spacing = self.waypoint_spacing_m
        inflate_cells = max(0, int(math.ceil(self.inflation_m / res)))

        def _run():
            import time
            nonlocal dstar

            t0 = time.monotonic()
            result = GridPlanResult()

            # Handle start in obstacle
            local_blocked = blocked
            _sx, _sy = sx, sy
            if local_blocked[_sy, _sx]:
                local_blocked = local_blocked.copy()
                r = inflate_cells + 2
                y_lo, y_hi = max(0, _sy - r), min(H, _sy + r + 1)
                x_lo, x_hi = max(0, _sx - r), min(W, _sx + r + 1)
                local_blocked[y_lo:y_hi, x_lo:x_hi] = False

            # Handle goal in obstacle
            _gx, _gy = gx, gy
            if local_blocked[_gy, _gx]:
                free = _nearest_free(local_blocked, _gx, _gy, W, H, search_radius=15)
                if free is None:
                    result.time_ms = (time.monotonic() - t0) * 1000
                    with self._lock:
                        self._last_result = result
                    return
                _gx, _gy = free

            if need_full_init:
                # Full D* Lite initialization
                dstar = DStarLite(W, H)
                dstar.initialize(local_blocked, _sx, _sy, _gx, _gy, cost_map=cost_map)
            else:
                # Incremental update: diff blocked grids and feed changes
                dstar.update_start(_sx, _sy)
                if map_changed and prev_blocked is not None:
                    diff = np.argwhere(local_blocked != prev_blocked)
                    if len(diff) > 0:
                        changes = [
                            (int(row[1]), int(row[0]), bool(local_blocked[row[0], row[1]]))
                            for row in diff
                        ]
                        dstar.update_cells(changes)

            found = dstar.compute_shortest_path()

            if not found:
                result.time_ms = (time.monotonic() - t0) * 1000
                with self._lock:
                    self._last_result = result
                    self._dstar = dstar
                    self._prev_blocked = local_blocked.copy()
                    self._dstar_goal_cell = (_gx, _gy)
                return

            path_cells = dstar.extract_path()
            if len(path_cells) < 2:
                result.success = True
                result.time_ms = (time.monotonic() - t0) * 1000
                with self._lock:
                    self._last_result = result
                    self._dstar = dstar
                    self._prev_blocked = local_blocked.copy()
                    self._dstar_goal_cell = (_gx, _gy)
                return

            # Convert to world coords, resample, then smooth corners
            path_world = [(ox + (cx + 0.5) * res, oy + (cy + 0.5) * res)
                          for cx, cy in path_cells]

            resampled = [path_world[0]]
            accum = 0.0
            for i in range(1, len(path_world)):
                dx = path_world[i][0] - path_world[i - 1][0]
                dy = path_world[i][1] - path_world[i - 1][1]
                accum += math.hypot(dx, dy)
                if accum >= spacing:
                    resampled.append(path_world[i])
                    accum = 0.0
            if resampled[-1] != path_world[-1]:
                resampled.append(path_world[-1])

            # Elastic band smoothing on resampled waypoints
            resampled = _elastic_band_smooth(resampled, local_blocked, ox, oy, res)

            result.waypoints_world = resampled[1:]  # skip start (robot pos)
            result.success = True
            result.time_ms = (time.monotonic() - t0) * 1000

            with self._lock:
                self._last_result = result
                self._dstar = dstar
                self._prev_blocked = local_blocked.copy()
                self._dstar_goal_cell = (_gx, _gy)

        self._thread = threading.Thread(target=_run, daemon=True)
        self._thread.start()

    def force_replan(self):
        """Force replan on next request (resets D* Lite state)."""
        self._last_plan_goal = None
        self._last_plan_time = None
        self._cached_blocked = None
        self._cached_cost_map = None
        self._cached_raw_occupied = None
        # Reset D* Lite — will do full re-init on next plan
        self._dstar = None
        self._prev_blocked = None
        self._dstar_goal_cell = None
