from __future__ import annotations

import heapq
import math
from dataclasses import dataclass, field
from typing import Optional

from .geometry import (
    bresenham,
    grid_to_local,
    local_to_grid,
    local_to_world,
    resample_path,
    smooth_path_chaikin,
    world_delta_to_local,
    wrap_angle,
)


@dataclass
class PlannerTargetResult:
    target_world: Optional[tuple[float, float]] = None
    path_world: list[tuple[float, float]] = field(default_factory=list)
    planner_mode: str = "direct_fallback"
    replanned: bool = False
    events: list[tuple[str, str]] = field(default_factory=list)


class LocalPlanner:
    def __init__(self, cfg) -> None:
        self.cfg = cfg

    def clear_stale_escape_target(self, now_sec: float, robot_state, runtime_state) -> None:
        if runtime_state.escape_target_world is None:
            return

        if (
            runtime_state.escape_target_until_sec is not None
            and now_sec > runtime_state.escape_target_until_sec
        ):
            runtime_state.escape_target_world = None
            runtime_state.escape_target_until_sec = None
            return

        if math.hypot(
            runtime_state.escape_target_world[0] - robot_state.x,
            runtime_state.escape_target_world[1] - robot_state.y,
        ) <= self.cfg.escape_waypoint_reach_tol:
            runtime_state.escape_target_world = None
            runtime_state.escape_target_until_sec = None

    def set_temporary_escape_target(
        self,
        now_sec: float,
        runtime_state,
        robot_state,
        scan,
        goal_dx_world: float,
        goal_dy_world: float,
        reason: str,
    ) -> tuple[bool, list[tuple[str, str]]]:
        fallback_local = self._select_scan_escape_target(scan, robot_state, goal_dx_world, goal_dy_world)
        if fallback_local is None:
            return (False, [])

        runtime_state.escape_target_world = local_to_world(
            robot_state.x,
            robot_state.y,
            robot_state.yaw,
            fallback_local[0],
            fallback_local[1],
        )
        runtime_state.escape_target_until_sec = now_sec + max(0.5, self.cfg.escape_waypoint_hold_sec)
        return (
            True,
            [
                (
                    "info",
                    f"Temporary escape target set ({reason}): "
                    f"({runtime_state.escape_target_world[0]:.2f}, {runtime_state.escape_target_world[1]:.2f})",
                )
            ],
        )

    def planner_target_world(
        self,
        now_sec: float,
        runtime_state,
        robot_state,
        goal_state,
        scan,
        goal_dx_world: float,
        goal_dy_world: float,
    ) -> PlannerTargetResult:
        result = PlannerTargetResult()

        # Replan only when necessary:
        #  1. No path exists (consumed or never planned)
        #  2. Goal shifted significantly
        #  3. Current path is blocked by a new obstacle (checked periodically)
        # Obstacle *avoidance* (slow/stop/steer) is handled by the controller
        # independently — it always runs from live scan data.

        has_path = bool(runtime_state.plan_waypoints_world)
        need_replan = not has_path

        goal_now = (goal_state.x, goal_state.y)
        if runtime_state.plan_last_goal is None:
            need_replan = True
        else:
            goal_shift = math.hypot(
                goal_now[0] - runtime_state.plan_last_goal[0],
                goal_now[1] - runtime_state.plan_last_goal[1],
            )
            if goal_shift >= self.cfg.planner_goal_replan_delta:
                need_replan = True

        # Periodic validity check: only replan if path is actually blocked.
        if not need_replan and has_path and runtime_state.plan_last_time_sec is not None:
            elapsed = now_sec - runtime_state.plan_last_time_sec
            if elapsed >= self.cfg.planner_replan_sec:
                runtime_state.plan_last_time_sec = now_sec
                if not self._is_plan_still_valid(scan, robot_state, runtime_state.plan_waypoints_world):
                    runtime_state.plan_invalid_streak += 1
                    if runtime_state.plan_invalid_streak >= self.cfg.planner_invalid_replan_count:
                        need_replan = True
                        runtime_state.plan_invalid_streak = 0
                else:
                    runtime_state.plan_invalid_streak = 0

        if need_replan:
            path_local = self._plan_local_path(scan, robot_state, goal_dx_world, goal_dy_world)
            runtime_state.plan_last_time_sec = now_sec
            runtime_state.plan_last_goal = goal_now
            runtime_state.plan_invalid_streak = 0
            result.replanned = True

            if path_local:
                runtime_state.plan_waypoints_world = [
                    local_to_world(robot_state.x, robot_state.y, robot_state.yaw, px, py)
                    for (px, py) in path_local
                ]
            else:
                runtime_state.plan_waypoints_world = []
                now_bucket = int(now_sec)
                if now_bucket != runtime_state.plan_last_warn_sec:
                    runtime_state.plan_last_warn_sec = now_bucket
                    result.events.append(
                        (
                            "warn",
                            "Local planner failed to find path; falling back to direct goal heading.",
                        )
                    )

                if self.cfg.planner_escape_enabled:
                    fallback_local = self._select_scan_escape_target(
                        scan,
                        robot_state,
                        goal_dx_world,
                        goal_dy_world,
                    )
                    if fallback_local is not None:
                        result.target_world = local_to_world(
                            robot_state.x,
                            robot_state.y,
                            robot_state.yaw,
                            fallback_local[0],
                            fallback_local[1],
                        )
                        result.planner_mode = "escape_fallback"
                        return result

                result.planner_mode = "direct_fallback"
                return result

        # ── Path shortcut: find closest waypoint, skip all prior ones ──
        # O(n) scan — negligible for typical 10-50 waypoint paths.
        # If the robot drifts near waypoint #5, skip #1-#4 instead of
        # backtracking.  Then drop any waypoints within reach radius.
        wps = runtime_state.plan_waypoints_world
        if wps:
            best_idx = 0
            best_d2 = float("inf")
            for i, (wx, wy) in enumerate(wps):
                d2 = (wx - robot_state.x) ** 2 + (wy - robot_state.y) ** 2
                if d2 < best_d2:
                    best_d2 = d2
                    best_idx = i
            # Keep from closest onward, then drop any within reach radius
            wps = wps[best_idx:]
            reach = self.cfg.planner_waypoint_spacing * 0.6
            while wps and math.hypot(wps[0][0] - robot_state.x, wps[0][1] - robot_state.y) < reach:
                wps.pop(0)
        runtime_state.plan_waypoints_world = wps

        result.path_world = list(runtime_state.plan_waypoints_world)

        if not runtime_state.plan_waypoints_world:
            result.planner_mode = "direct_fallback"
            return result

        for wx, wy in runtime_state.plan_waypoints_world:
            if math.hypot(wx - robot_state.x, wy - robot_state.y) >= self.cfg.planner_waypoint_lookahead:
                result.target_world = (wx, wy)
                result.planner_mode = "planned_waypoint"
                return result

        result.target_world = runtime_state.plan_waypoints_world[-1]
        result.planner_mode = "planned_waypoint"
        return result

    def _plan_local_path(self, scan, robot_state, goal_dx_world: float, goal_dy_world: float) -> list[tuple[float, float]]:
        goal_lx, goal_ly = world_delta_to_local(goal_dx_world, goal_dy_world, robot_state.yaw)
        goal_dist = math.hypot(goal_lx, goal_ly)
        max_plan_dist = min(self.cfg.planner_goal_clip_distance, self.cfg.planner_grid_radius * 0.9)
        if goal_dist > max_plan_dist and goal_dist > 1e-6:
            scale = max_plan_dist / goal_dist
            goal_lx *= scale
            goal_ly *= scale

        grid = self._build_local_grid(scan)
        c = self.cfg.planner_cells // 2
        start = (c, c)
        goal = local_to_grid(
            goal_lx,
            goal_ly,
            self.cfg.planner_resolution,
            self.cfg.planner_cells,
        )
        if goal is None:
            return []

        goal_search_cells = max(8, int(round(self.cfg.planner_goal_search_radius / self.cfg.planner_resolution)))
        goal = self._nearest_traversable(grid, goal, search_radius=goal_search_cells)
        if goal is None:
            return []

        path_cells = self._astar(grid, start, goal)
        if not path_cells:
            return []

        # Convert entire A* path to local coordinates for smoothing.
        path_local = [
            grid_to_local(gx, gy, self.cfg.planner_resolution, self.cfg.planner_cells)
            for gx, gy in path_cells
        ]

        # Chaikin corner-cutting: smooth sharp 8-connected staircase into arcs.
        smoothing_passes = getattr(self.cfg, "path_smoothing_passes", 0)
        if smoothing_passes > 0 and len(path_local) >= 3:
            path_local = smooth_path_chaikin(path_local, iterations=smoothing_passes)

            # Collision check: discard any smoothed point that landed in an obstacle.
            # The waypoint follower handles gaps via lookahead.
            n = self.cfg.planner_cells
            safe_path: list[tuple[float, float]] = []
            for lx, ly in path_local:
                cell = local_to_grid(lx, ly, self.cfg.planner_resolution, n)
                if cell is not None and self._is_traversable(grid, cell[0], cell[1]):
                    safe_path.append((lx, ly))
            path_local = safe_path if len(safe_path) >= 2 else path_local

            path_local = resample_path(path_local, self.cfg.planner_waypoint_spacing)
        else:
            step_cells = max(1, int(round(self.cfg.planner_waypoint_spacing / self.cfg.planner_resolution)))
            sampled_cells = path_cells[::step_cells]
            if sampled_cells[-1] != path_cells[-1]:
                sampled_cells.append(path_cells[-1])
            path_local = [
                grid_to_local(gx, gy, self.cfg.planner_resolution, self.cfg.planner_cells)
                for gx, gy in sampled_cells
            ]

        # Wall-repulsion: push waypoints away from nearby obstacles.
        safety_clearance = getattr(self.cfg, "planner_safety_clearance", 0.45)
        path_local = self._push_waypoints_from_walls(
            path_local, grid, safety_clearance,
        )

        return path_local

    def _push_waypoints_from_walls(
        self,
        path_local: list[tuple[float, float]],
        grid: list[list[int]],
        safety_clearance: float,
    ) -> list[tuple[float, float]]:
        """Push waypoints away from nearby obstacles to avoid corner clipping.

        For each waypoint, find the nearest obstacle cells within
        safety_clearance.  If any are found, compute a repulsion vector and
        move the waypoint away from the obstacle cluster.
        """
        if len(path_local) < 2 or safety_clearance <= 0:
            return path_local

        res = self.cfg.planner_resolution
        n = self.cfg.planner_cells
        search_r = int(math.ceil(safety_clearance / res)) + 1
        result: list[tuple[float, float]] = [path_local[0]]  # keep start

        for lx, ly in path_local[1:-1]:
            cell = local_to_grid(lx, ly, res, n)
            if cell is None:
                result.append((lx, ly))
                continue
            cx, cy = cell

            # Accumulate repulsion from nearby obstacle cells
            repel_x, repel_y = 0.0, 0.0
            threat_count = 0
            for dy in range(-search_r, search_r + 1):
                for dx in range(-search_r, search_r + 1):
                    nx, ny = cx + dx, cy + dy
                    if 0 <= nx < n and 0 <= ny < n and grid[ny][nx] == 100:
                        dist_cells = math.hypot(dx, dy)
                        dist_m = dist_cells * res
                        if dist_m < safety_clearance:
                            # Weight closer obstacles more heavily
                            weight = 1.0 - (dist_m / safety_clearance)
                            if dist_cells > 0.01:
                                repel_x -= dx / dist_cells * weight
                                repel_y -= dy / dist_cells * weight
                            threat_count += 1

            if threat_count == 0:
                result.append((lx, ly))
                continue

            # Normalize repulsion vector and push waypoint
            mag = math.hypot(repel_x, repel_y)
            if mag < 0.01:
                result.append((lx, ly))
                continue

            push_dist = min(safety_clearance * 0.5, res * 3)  # max push ~3 cells
            new_lx = lx + (repel_x / mag) * push_dist
            new_ly = ly + (repel_y / mag) * push_dist

            # Only keep if the pushed position is traversable
            new_cell = local_to_grid(new_lx, new_ly, res, n)
            if new_cell is not None and self._is_traversable(grid, new_cell[0], new_cell[1]):
                result.append((new_lx, new_ly))
            else:
                result.append((lx, ly))

        result.append(path_local[-1])  # keep goal
        return result

    def _is_plan_still_valid(
        self,
        scan,
        robot_state,
        plan_world: list[tuple[float, float]],
    ) -> bool:
        """Check existing waypoints against the current scan grid.

        Returns True if every remaining waypoint is still in free space,
        meaning the plan doesn't need to be replaced.
        """
        grid = self._build_local_grid(scan)
        n = self.cfg.planner_cells
        for wx, wy in plan_world:
            dx = wx - robot_state.x
            dy = wy - robot_state.y
            lx, ly = world_delta_to_local(dx, dy, robot_state.yaw)
            cell = local_to_grid(lx, ly, self.cfg.planner_resolution, n)
            if cell is None:
                # Waypoint outside the local grid — can't verify, keep the plan.
                continue
            if not self._is_traversable(grid, cell[0], cell[1]):
                return False
        return True

    def _build_local_grid(self, scan) -> list[list[int]]:
        n = self.cfg.planner_cells
        c = n // 2
        grid = [[-1 for _ in range(n)] for _ in range(n)]
        grid[c][c] = 0

        max_clear = max(
            0.05,
            min(scan.range_max, self.cfg.planner_grid_radius - self.cfg.planner_resolution),
        )
        angle = scan.angle_min
        for r in scan.ranges:
            valid_hit = math.isfinite(r) and r >= 0.05
            dist = min(r, max_clear) if valid_hit else max_clear
            end = local_to_grid(dist * math.cos(angle), dist * math.sin(angle), self.cfg.planner_resolution, n)
            angle += scan.angle_increment
            if end is None:
                continue
            ray = bresenham(c, c, end[0], end[1])
            for rx, ry in ray[:-1]:
                if 0 <= rx < n and 0 <= ry < n:
                    grid[ry][rx] = 0

            ex, ey = ray[-1]
            if 0 <= ex < n and 0 <= ey < n:
                if valid_hit and r < max_clear * 0.995:
                    grid[ey][ex] = 100
                elif grid[ey][ex] != 100:
                    grid[ey][ex] = 0

        inflate_cells = max(0, int(math.ceil(self.cfg.planner_inflation_radius / self.cfg.planner_resolution)))
        if inflate_cells > 0:
            occupied = [(x, y) for y in range(n) for x in range(n) if grid[y][x] == 100]
            for ox, oy in occupied:
                for dy in range(-inflate_cells, inflate_cells + 1):
                    for dx in range(-inflate_cells, inflate_cells + 1):
                        if dx * dx + dy * dy > inflate_cells * inflate_cells:
                            continue
                        nx = ox + dx
                        ny = oy + dy
                        if 0 <= nx < n and 0 <= ny < n:
                            grid[ny][nx] = 100

        clear_cells = max(0, int(round(self.cfg.planner_start_clearance_radius / self.cfg.planner_resolution)))
        if clear_cells > 0:
            for dy in range(-clear_cells, clear_cells + 1):
                for dx in range(-clear_cells, clear_cells + 1):
                    if dx * dx + dy * dy > clear_cells * clear_cells:
                        continue
                    nx = c + dx
                    ny = c + dy
                    if 0 <= nx < n and 0 <= ny < n:
                        grid[ny][nx] = 0

        return grid

    def _nearest_traversable(
        self,
        grid: list[list[int]],
        goal: tuple[int, int],
        search_radius: int,
    ) -> Optional[tuple[int, int]]:
        if self._is_traversable(grid, goal[0], goal[1]):
            return goal

        n = len(grid)
        gx, gy = goal
        best = None
        best_cost = float("inf")
        for dy in range(-search_radius, search_radius + 1):
            for dx in range(-search_radius, search_radius + 1):
                nx = gx + dx
                ny = gy + dy
                if not (0 <= nx < n and 0 <= ny < n):
                    continue
                if not self._is_traversable(grid, nx, ny):
                    continue
                cost = dx * dx + dy * dy
                if cost < best_cost:
                    best_cost = cost
                    best = (nx, ny)
        return best

    def _astar(
        self,
        grid: list[list[int]],
        start: tuple[int, int],
        goal: tuple[int, int],
    ) -> list[tuple[int, int]]:
        n = len(grid)
        if not self._is_traversable(grid, start[0], start[1]):
            return []
        if not self._is_traversable(grid, goal[0], goal[1]):
            return []

        open_heap: list[tuple[float, float, int, int]] = []
        heapq.heappush(open_heap, (self._heuristic(start, goal), 0.0, start[0], start[1]))
        came_from: dict[tuple[int, int], tuple[int, int]] = {}
        g_score: dict[tuple[int, int], float] = {start: 0.0}
        closed: set[tuple[int, int]] = set()

        neighbors = [
            (-1, 0, 1.0),
            (1, 0, 1.0),
            (0, -1, 1.0),
            (0, 1, 1.0),
            (-1, -1, math.sqrt(2.0)),
            (-1, 1, math.sqrt(2.0)),
            (1, -1, math.sqrt(2.0)),
            (1, 1, math.sqrt(2.0)),
        ]

        while open_heap:
            _, cur_g, x, y = heapq.heappop(open_heap)
            cur = (x, y)
            if cur in closed:
                continue
            if cur == goal:
                return self._reconstruct_path(came_from, cur)
            closed.add(cur)

            for dx, dy, step in neighbors:
                nx = x + dx
                ny = y + dy
                if nx < 0 or ny < 0 or nx >= n or ny >= n:
                    continue

                if dx != 0 and dy != 0:
                    if not self._is_traversable(grid, x + dx, y) or not self._is_traversable(grid, x, y + dy):
                        continue

                if not self._is_traversable(grid, nx, ny):
                    continue

                nxt = (nx, ny)
                unknown_penalty = 0.25 if grid[ny][nx] < 0 and not self.cfg.planner_unknown_is_obstacle else 0.0
                tentative_g = cur_g + step + unknown_penalty
                if tentative_g >= g_score.get(nxt, float("inf")):
                    continue

                came_from[nxt] = cur
                g_score[nxt] = tentative_g
                heapq.heappush(
                    open_heap,
                    (tentative_g + self._heuristic(nxt, goal), tentative_g, nx, ny),
                )

        return []

    def _is_traversable(self, grid: list[list[int]], x: int, y: int) -> bool:
        val = grid[y][x]
        if val >= 100:
            return False
        if self.cfg.planner_unknown_is_obstacle and val < 0:
            return False
        return True

    @staticmethod
    def _heuristic(a: tuple[int, int], b: tuple[int, int]) -> float:
        return math.hypot(a[0] - b[0], a[1] - b[1])

    @staticmethod
    def _reconstruct_path(
        came_from: dict[tuple[int, int], tuple[int, int]],
        cur: tuple[int, int],
    ) -> list[tuple[int, int]]:
        path = [cur]
        while cur in came_from:
            cur = came_from[cur]
            path.append(cur)
        path.reverse()
        return path

    def _select_scan_escape_target(
        self,
        scan,
        robot_state,
        goal_dx_world: float,
        goal_dy_world: float,
    ) -> Optional[tuple[float, float]]:
        goal_lx, goal_ly = world_delta_to_local(goal_dx_world, goal_dy_world, robot_state.yaw)
        goal_heading = math.atan2(goal_ly, goal_lx)

        blocked_front = False
        min_front = float("inf")
        front_window = max(0.15, self.cfg.front_half * 0.9)

        angle = scan.angle_min
        for r in scan.ranges:
            if math.isfinite(r) and r >= 0.05:
                if abs(angle) < front_window and r < min_front:
                    min_front = r
            angle += scan.angle_increment
        if min_front < (self.cfg.obstacle_stop_dist + 0.08):
            blocked_front = True

        best_score = float("-inf")
        best_local = None

        angle = scan.angle_min
        for r in scan.ranges:
            if not math.isfinite(r) or r < self.cfg.planner_escape_min_range:
                angle += scan.angle_increment
                continue

            clearance = r - self.cfg.obstacle_stop_dist
            if clearance <= 0.0:
                angle += scan.angle_increment
                continue

            step = min(clearance * 0.85, self.cfg.planner_escape_max_step)
            if step < self.cfg.planner_escape_min_step:
                angle += scan.angle_increment
                continue

            heading_align = math.cos(wrap_angle(angle - goal_heading))
            clearance_score = min(clearance / max(self.cfg.obstacle_slow_dist, 0.1), 1.5)
            turn_penalty = abs(angle) / math.pi
            blocked_front_penalty = 0.0
            if blocked_front and abs(angle) <= (self.cfg.front_half * 1.2):
                blocked_front_penalty = self.cfg.planner_escape_blocked_front_penalty

            score = (
                step
                + self.cfg.planner_escape_goal_align_gain * heading_align
                + self.cfg.planner_escape_clearance_gain * clearance_score
                - self.cfg.planner_escape_turn_penalty_gain * turn_penalty
                - blocked_front_penalty
            )
            if score > best_score:
                best_score = score
                best_local = (step * math.cos(angle), step * math.sin(angle))

            angle += scan.angle_increment

        return best_local
