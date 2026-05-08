"""CFPA2 frontier exploration — pure Python, no ROS.

Single-robot subset extracted from:
  cfpa2_coordinator_node.py (algorithm methods only)

Known gap inherited from original: _distance_transform uses raw-map BFS
(not costmap-inflated), so reachability can over-estimate — a frontier may
be marked reachable that the planner can't path through. Documented in
CLAUDE.md; fix deferred to Phase B (feed inflated costmap).

Public API:
  explorer = FrontierExplorer(mapper)
  goal = explorer.select_goal(pose)   # returns (x,y) or None
"""
from __future__ import annotations

import math
from collections import deque
from typing import Optional

import numpy as np


# Grid cell values (matching OccupancyMapper)
FREE     = 0
UNKNOWN  = -1
OCC_MIN  = 100   # occupied threshold


class FrontierExplorer:
    """Stateful single-robot frontier selector."""

    def __init__(self,
                 mapper,                           # native.mapping.OccupancyMapper
                 min_cluster_area_m2: float = 0.1,
                 clearance_m: float = 0.20,
                 unknown_check_radius_m: float = 0.50,
                 min_live_unknowns: int = 3,
                 cluster_radius_m: float = 1.0,
                 blacklist_ttl_s: float = 30.0,
                 min_assign_dist_m: float = 0.50):
        self.mapper = mapper
        self.min_cluster_area = min_cluster_area_m2
        self.clearance_m = clearance_m
        self.unknown_radius_m = unknown_check_radius_m
        self.min_live_unknowns = min_live_unknowns
        self.cluster_radius = cluster_radius_m
        self.blacklist_ttl = blacklist_ttl_s
        self.min_assign_dist = min_assign_dist_m

        self._blacklist: dict[tuple[int,int], float] = {}   # key → expiry sim_time
        self._current_goal: Optional[tuple[float,float]] = None

    # ── Public entry point ────────────────────────────────────────────────────

    def select_goal(self, pose, sim_time: float = 0.0) -> Optional[tuple[float,float]]:
        """Pick best reachable frontier given current pose and map.

        Returns world-frame (x, y) or None if no frontier found.
        """
        grid = self.mapper.grid   # (H,W) int8
        res = self.mapper.res
        ox, oy = self.mapper.ox, self.mapper.oy
        W, H = self.mapper.W, self.mapper.H

        # Prune blacklist
        self._blacklist = {k: t for k, t in self._blacklist.items() if t > sim_time}

        # Frontier extraction
        frontiers = self._extract_frontiers(grid, W, H, res, ox, oy)
        if not frontiers:
            return None

        # Reachability (BFS on raw map — known gap, see module docstring)
        rx, ry = float(pose.x), float(pose.y)
        dist_map = self._distance_transform(grid, W, H, res, ox, oy, rx, ry)

        # Score and pick best reachable, non-blacklisted frontier
        best: Optional[tuple[float,float]] = None
        best_score = float("-inf")

        for fx, fy in frontiers:
            # Blacklist check
            bkey = self._blacklist_key(fx, fy, res)
            if bkey in self._blacklist:
                continue

            # Reachability
            gx = int((fx - ox) / res)
            gy = int((fy - oy) / res)
            if not (0 <= gx < W and 0 <= gy < H):
                continue
            flat = gy * W + gx
            if flat not in dist_map:
                continue

            dist = dist_map[flat] * res
            if dist < self.min_assign_dist:
                continue

            # Score: info_gain / distance (favour close, unknown-rich frontiers)
            ig = self._info_gain(grid, W, H, gx, gy, self.unknown_radius_m, res)
            score = ig / max(dist, 0.1)

            if score > best_score:
                best_score = score
                best = (fx, fy)

        self._current_goal = best
        return best

    def blacklist_goal(self, goal: tuple[float,float], sim_time: float) -> None:
        key = self._blacklist_key(goal[0], goal[1], self.mapper.res)
        self._blacklist[key] = sim_time + self.blacklist_ttl

    # ── Frontier extraction ────────────────────────────────────────────────────

    def _extract_frontiers(self, grid: np.ndarray, W: int, H: int,
                           res: float, ox: float, oy: float) -> list[tuple[float,float]]:
        data = grid.ravel()  # (H*W,) int8 (row-major: idx = row*W + col)

        # Mark frontier cells: free cells adjacent to unknown
        NEIGHBOR8 = ((1,0),(-1,0),(0,1),(0,-1),(1,1),(-1,1),(1,-1),(-1,-1))
        frontier_mask = np.zeros(H * W, dtype=bool)
        for gy in range(1, H - 1):
            base = gy * W
            for gx in range(1, W - 1):
                idx = base + gx
                if data[idx] != FREE:
                    continue
                for dx, dy in NEIGHBOR8:
                    nidx = (gy + dy) * W + (gx + dx)
                    if data[nidx] == UNKNOWN:
                        frontier_mask[idx] = True
                        break

        # Connected-component clustering
        frontier_cells = list(zip(*np.where(frontier_mask.reshape(H, W))))
        if not frontier_cells:
            return []

        clearance_cells = max(0, int(math.ceil(self.clearance_m / res)))
        min_area = self.min_cluster_area

        visited = np.zeros(H * W, dtype=bool)
        raw: list[tuple[float,float]] = []

        for seed_row, seed_col in frontier_cells:
            seed_idx = seed_row * W + seed_col
            if visited[seed_idx]:
                continue
            if not frontier_mask[seed_idx]:
                continue

            # BFS over frontier component
            visited[seed_idx] = True
            q: deque[tuple[int,int]] = deque([(seed_col, seed_row)])
            component: list[tuple[int,int]] = []

            while q:
                cx, cy = q.popleft()
                component.append((cx, cy))
                for dx, dy in NEIGHBOR8:
                    nx, ny = cx + dx, cy + dy
                    if not (0 < nx < W - 1 and 0 < ny < H - 1):
                        continue
                    nidx = ny * W + nx
                    if visited[nidx] or not frontier_mask[nidx]:
                        continue
                    visited[nidx] = True
                    q.append((nx, ny))

            if len(component) * res * res < min_area:
                continue

            for gx, gy in component:
                if clearance_cells > 0 and not self._has_clearance(data, gx, gy, W, H, clearance_cells):
                    continue
                wx = ox + (gx + 0.5) * res
                wy = oy + (gy + 0.5) * res
                raw.append((wx, wy))

        # Merge nearby representatives into cluster centroids
        return self._cluster(raw, self.cluster_radius)

    def _has_clearance(self, data, gx: int, gy: int, W: int, H: int, r: int) -> bool:
        r2 = r * r
        for dy in range(-r, r + 1):
            ny = gy + dy
            if ny < 0 or ny >= H:
                return False
            for dx in range(-r, r + 1):
                if dx * dx + dy * dy > r2:
                    continue
                nx = gx + dx
                if nx < 0 or nx >= W:
                    return False
                if data[ny * W + nx] >= OCC_MIN:
                    return False
        return True

    @staticmethod
    def _cluster(points: list[tuple[float,float]], radius: float) -> list[tuple[float,float]]:
        if not points or radius <= 0:
            return points
        r2 = radius * radius
        clusters: list[list] = []
        for px, py in points:
            joined = False
            for c in clusters:
                if (px - c[0])**2 + (py - c[1])**2 <= r2:
                    c[2].append((px, py))
                    n = len(c[2])
                    c[0] = sum(p[0] for p in c[2]) / n
                    c[1] = sum(p[1] for p in c[2]) / n
                    joined = True
                    break
            if not joined:
                clusters.append([px, py, [(px, py)]])
        return [(c[0], c[1]) for c in clusters]

    # ── Distance transform (BFS on free cells) ───────────────────────────────

    def _distance_transform(self, grid: np.ndarray, W: int, H: int,
                             res: float, ox: float, oy: float,
                             rx: float, ry: float) -> dict[int,int]:
        sx = int((rx - ox) / res)
        sy = int((ry - oy) / res)
        sx = max(0, min(W - 1, sx))
        sy = max(0, min(H - 1, sy))
        data = grid.ravel()

        sidx = sy * W + sx
        if data[sidx] != FREE:
            # Snap to nearest free cell
            found = None
            for r in range(1, 13):
                for dy in range(-r, r + 1):
                    for dx in range(-r, r + 1):
                        nx, ny = sx + dx, sy + dy
                        if not (0 <= nx < W and 0 <= ny < H):
                            continue
                        nidx = ny * W + nx
                        if data[nidx] == FREE:
                            found = (nx, ny, nidx)
                            break
                    if found:
                        break
                if found:
                    break
            if not found:
                return {}
            sx, sy, sidx = found

        q: deque[tuple[int,int]] = deque([(sx, sy)])
        dist: dict[int,int] = {sidx: 0}
        while q:
            cx, cy = q.popleft()
            cidx = cy * W + cx
            d = dist[cidx]
            for dx, dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nx, ny = cx + dx, cy + dy
                if not (0 <= nx < W and 0 <= ny < H):
                    continue
                nidx = ny * W + nx
                if nidx in dist or data[nidx] != FREE:
                    continue
                dist[nidx] = d + 1
                q.append((nx, ny))
        return dist

    # ── Info gain ────────────────────────────────────────────────────────────

    @staticmethod
    def _info_gain(grid: np.ndarray, W: int, H: int,
                   gx: int, gy: int, radius_m: float, res: float) -> float:
        r = max(1, int(radius_m / res))
        data = grid.ravel()
        count = 0
        for dy in range(-r, r + 1):
            ny = gy + dy
            if ny < 0 or ny >= H:
                continue
            for dx in range(-r, r + 1):
                nx = gx + dx
                if nx < 0 or nx >= W:
                    continue
                if data[ny * W + nx] == UNKNOWN:
                    count += 1
        return float(count)

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _blacklist_key(wx: float, wy: float, res: float) -> tuple[int,int]:
        q = max(res, 0.05)
        return int(round(wx / q)), int(round(wy / q))
