"""D* Lite incremental path planner (Koenig & Likhachev, 2002).

Maintains a persistent search tree rooted at the goal. When the map changes
(new obstacles or cleared cells), only the affected portion of the tree is
repaired — O(changed cells) instead of a full A* restart.

Uses lazy deletion with per-cell version counters for an efficient pure-Python
priority queue implementation.

Usage:
    dstar = DStarLite(width, height)
    dstar.initialize(blocked, sx, sy, gx, gy)
    dstar.compute_shortest_path()
    path = dstar.extract_path()

    # Robot moves, map changes:
    dstar.update_start(new_sx, new_sy)
    dstar.update_cells([(cx, cy, is_blocked), ...])
    dstar.compute_shortest_path()
    path = dstar.extract_path()
"""

from __future__ import annotations

import heapq
import math

import numpy as np

_SQRT2 = math.sqrt(2)
_INF = float("inf")

# 8-connected neighbors: (dx, dy, cost)
_NEIGHBORS = (
    (-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
    (-1, -1, _SQRT2), (-1, 1, _SQRT2), (1, -1, _SQRT2), (1, 1, _SQRT2),
)


class DStarLite:
    """Optimized D* Lite on a 2D grid with 8-connected movement.

    Coordinates: (x, y) where x is column, y is row.
    Internal arrays are indexed [y, x] (row-major, matching numpy convention).

    Uses lazy deletion: each cell has a version counter. When a cell is
    re-inserted, its version increments. Stale heap entries (wrong version)
    are skipped on pop.
    """

    __slots__ = (
        "W", "H", "g", "rhs", "blocked", "cost_map",
        "_sx", "_sy", "_gx", "_gy",
        "_km", "_s_last_x", "_s_last_y",
        "_open", "_version", "_counter",
        "_initialized", "max_cells",
    )

    def __init__(self, width: int, height: int, max_cells: int = 200_000):
        self.W = width
        self.H = height
        self.max_cells = max_cells
        self._initialized = False

        # Allocate arrays — float64 for precision
        self.g = np.full((height, width), _INF, dtype=np.float64)
        self.rhs = np.full((height, width), _INF, dtype=np.float64)
        self.blocked = np.zeros((height, width), dtype=bool)
        self.cost_map: np.ndarray | None = None  # optional uint8 proximity cost

        # Priority queue: (k1, k2, counter, version, x, y)
        # version is used for lazy deletion — stale entries have wrong version
        self._open: list[tuple[float, float, int, int, int, int]] = []
        self._version = np.zeros((height, width), dtype=np.int32)
        self._counter = 0  # tiebreaker for heap stability

        self._sx = self._sy = 0
        self._gx = self._gy = 0
        self._km = 0.0
        self._s_last_x = self._s_last_y = 0

    def initialize(
        self,
        blocked: np.ndarray,
        sx: int, sy: int,
        gx: int, gy: int,
        cost_map: np.ndarray | None = None,
    ) -> None:
        """Set up a fresh search from (sx,sy) to (gx,gy) on the given grid."""
        H, W = self.H, self.W
        assert blocked.shape == (H, W), f"Grid shape mismatch: {blocked.shape} vs ({H}, {W})"

        self.blocked = blocked.astype(bool, copy=True)
        self.cost_map = cost_map  # uint8 array or None
        self.g[:] = _INF
        self.rhs[:] = _INF
        self._version[:] = 0
        self._open.clear()
        self._counter = 0
        self._km = 0.0

        self._sx, self._sy = sx, sy
        self._gx, self._gy = gx, gy
        self._s_last_x, self._s_last_y = sx, sy

        # D* Lite searches backward from goal
        self.rhs[gy, gx] = 0.0
        self._queue_insert(gx, gy)
        self._initialized = True

    def _heuristic(self, x: int, y: int) -> float:
        """Octile distance from (x,y) to current start."""
        dx = abs(x - self._sx)
        dy = abs(y - self._sy)
        return max(dx, dy) + (_SQRT2 - 1.0) * min(dx, dy)

    def _calculate_key(self, x: int, y: int) -> tuple[float, float]:
        g_val = self.g[y, x]
        rhs_val = self.rhs[y, x]
        mn = g_val if g_val < rhs_val else rhs_val
        return (mn + self._heuristic(x, y) + self._km, mn)

    def _queue_insert(self, x: int, y: int) -> None:
        """Insert (or re-insert) cell into the priority queue.

        Increments the cell's version to invalidate any previous entries.
        """
        self._version[y, x] += 1
        ver = int(self._version[y, x])
        k1, k2 = self._calculate_key(x, y)
        self._counter += 1
        heapq.heappush(self._open, (k1, k2, self._counter, ver, x, y))

    def _queue_remove(self, x: int, y: int) -> None:
        """Logically remove cell from queue by incrementing its version."""
        self._version[y, x] += 1

    def _update_vertex(self, x: int, y: int) -> None:
        """Standard D* Lite UpdateVertex: recompute rhs, update queue."""
        if x == self._gx and y == self._gy:
            return

        W, H = self.W, self.H
        best = _INF
        blocked = self.blocked
        g = self.g
        cost_map = self.cost_map
        _COST_SCALE = 1.0 / 252.0

        # rhs(u) = min over successors s' of { c(u, s') + g(s') }
        for dx, dy, cost in _NEIGHBORS:
            nx, ny = x + dx, y + dy
            if 0 <= nx < W and 0 <= ny < H and not blocked[ny, nx]:
                edge = cost
                if cost_map is not None:
                    edge += float(cost_map[ny, nx]) * _COST_SCALE
                val = edge + g[ny, nx]
                if val < best:
                    best = val

        self.rhs[y, x] = best

        # Remove from queue (invalidate old entry)
        self._queue_remove(x, y)

        # Re-insert if inconsistent
        if g[y, x] != best:
            self._queue_insert(x, y)

    def _pop_valid(self) -> tuple[float, float, int, int] | None:
        """Pop the next valid (non-stale) entry from the queue.

        Returns (k1, k2, x, y) or None if queue is empty.
        """
        version = self._version
        while self._open:
            k1, k2, _, ver, x, y = heapq.heappop(self._open)
            if ver == version[y, x]:
                return (k1, k2, x, y)
        return None

    def _peek_key(self) -> tuple[float, float]:
        """Return the key of the top valid entry without removing it.

        Returns (inf, inf) if queue is empty.
        """
        version = self._version
        while self._open:
            k1, k2, _, ver, x, y = self._open[0]
            if ver == version[y, x]:
                return (k1, k2)
            # Stale entry — discard
            heapq.heappop(self._open)
        return (_INF, _INF)

    def compute_shortest_path(self) -> bool:
        """Expand nodes until the start is locally consistent.

        Returns True if a path exists from start to goal.
        """
        if not self._initialized:
            return False

        g = self.g
        rhs = self.rhs
        blocked = self.blocked
        W, H = self.W, self.H
        sx, sy = self._sx, self._sy
        cells_expanded = 0

        while True:
            top_key = self._peek_key()
            start_key = self._calculate_key(sx, sy)

            # Termination: top key >= start key AND start is consistent
            if top_key >= start_key and rhs[sy, sx] == g[sy, sx]:
                break

            # Queue exhausted
            if top_key[0] >= _INF:
                break

            entry = self._pop_valid()
            if entry is None:
                break

            k_old1, k_old2, x, y = entry
            k_new = self._calculate_key(x, y)

            if (k_old1, k_old2) < k_new:
                # Key is outdated due to km change — re-insert with correct key
                self._queue_insert(x, y)
            elif g[y, x] > rhs[y, x]:
                # Overconsistent: make consistent
                g[y, x] = rhs[y, x]
                cells_expanded += 1
                # Update all predecessors
                for dx, dy, cost in _NEIGHBORS:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < W and 0 <= ny < H and not blocked[ny, nx]:
                        self._update_vertex(nx, ny)
            else:
                # Underconsistent: reset g and propagate
                g[y, x] = _INF
                cells_expanded += 1
                # Update self and all predecessors
                self._update_vertex(x, y)
                for dx, dy, cost in _NEIGHBORS:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < W and 0 <= ny < H and not blocked[ny, nx]:
                        self._update_vertex(nx, ny)

            if cells_expanded >= self.max_cells:
                break

        return g[sy, sx] < _INF

    def update_start(self, new_sx: int, new_sy: int) -> None:
        """Update the robot's position (start of the path)."""
        if new_sx == self._sx and new_sy == self._sy:
            return
        # Accumulate km so we don't need to re-key the entire queue
        dx = abs(new_sx - self._s_last_x)
        dy = abs(new_sy - self._s_last_y)
        self._km += max(dx, dy) + (_SQRT2 - 1.0) * min(dx, dy)
        self._s_last_x, self._s_last_y = new_sx, new_sy
        self._sx, self._sy = new_sx, new_sy

    def update_cells(self, changes: list[tuple[int, int, bool]]) -> None:
        """Batch-update cell blocked status and repair the search tree.

        changes: list of (x, y, now_blocked)
        """
        if not self._initialized:
            return

        W, H = self.W, self.H
        blocked = self.blocked

        for x, y, is_blocked in changes:
            if not (0 <= x < W and 0 <= y < H):
                continue
            if blocked[y, x] == is_blocked:
                continue

            blocked[y, x] = is_blocked

            if is_blocked:
                # Cell became blocked: invalidate it and update neighbors
                self.g[y, x] = _INF
                self.rhs[y, x] = _INF
                self._queue_remove(x, y)
                for dx, dy, cost in _NEIGHBORS:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < W and 0 <= ny < H and not blocked[ny, nx]:
                        self._update_vertex(nx, ny)
            else:
                # Cell became free: update it and neighbors
                self._update_vertex(x, y)
                for dx, dy, cost in _NEIGHBORS:
                    nx, ny = x + dx, y + dy
                    if 0 <= nx < W and 0 <= ny < H and not blocked[ny, nx]:
                        self._update_vertex(nx, ny)

    def extract_path(self, max_len: int = 10000) -> list[tuple[int, int]]:
        """Trace the shortest path from start to goal by greedy descent.

        Returns list of (x, y) cells from start to goal inclusive,
        or empty list if no path exists.
        """
        if not self._initialized:
            return []

        g = self.g
        blocked = self.blocked
        cost_map = self.cost_map
        _COST_SCALE = 1.0 / 252.0
        W, H = self.W, self.H
        sx, sy = self._sx, self._sy

        if g[sy, sx] >= _INF:
            return []

        path = [(sx, sy)]
        x, y = sx, sy

        for _ in range(max_len):
            if x == self._gx and y == self._gy:
                break

            best_x, best_y = -1, -1
            best_cost = _INF

            for dx, dy, edge_cost in _NEIGHBORS:
                nx, ny = x + dx, y + dy
                if 0 <= nx < W and 0 <= ny < H and not blocked[ny, nx]:
                    ec = edge_cost
                    if cost_map is not None:
                        ec += float(cost_map[ny, nx]) * _COST_SCALE
                    val = ec + g[ny, nx]
                    if val < best_cost:
                        best_cost = val
                        best_x, best_y = nx, ny

            if best_x < 0 or best_cost >= _INF:
                return []  # no path

            x, y = best_x, best_y
            path.append((x, y))
        else:
            return []  # path too long, likely a loop

        return path

    @property
    def path_cost(self) -> float:
        """Cost of the shortest path from start to goal (inf if none)."""
        if not self._initialized:
            return _INF
        return self.g[self._sy, self._sx]
