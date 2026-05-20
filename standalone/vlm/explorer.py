"""Standalone VLM exploration coordinator — LangGraph edition.

Replaces the CFPA2 frontier allocator as the goal source. Each cycle:

    1. Sample current OccupancyGrid + robot pose + frontier candidates.
    2. Render occupancy PNG + optionally capture RGB camera frame.
    3. Run a LangGraph ReAct agent (background thread) that can:
       - list_frontiers()          — query reachable candidates
       - validate_path(x, y)       — A* reachability check
       - get_coverage()            — coverage stats
       - log_artifact(x, y, desc)  — record spotted object
       then emits a final goal JSON.
    4. On agent completion, parse → publish PoseStamped on /<ns>/goal_pose.
    5. Log the full round-trip to a session folder for inspection.

Adaptive triggers (new vs original):
    - Goal reached   → request next VLM cycle immediately (no timer wait)
    - Stuck detected → request VLM cycle for an escape goal (with cooldown)
    - Fallback timer → fire every cycle_period_sec if neither event occurred

For offline / no-API-key use:  --provider mock  → deterministic frontier pick,
no LangGraph, no network. Useful for integration testing the full pipeline.
"""
from __future__ import annotations

import json
import math
import os
import threading
import time
from pathlib import Path
from typing import Callable, List, Optional, Tuple

import numpy as np

from ..core.bus import BUS
from ..core.ros_compat import OccupancyGrid, PoseStamped, String, now_stamp
from ..mapping.occupancy_grid import OccupancyMapper
from ..nav.planner import AStarPlanner
from . import backend as vlm_backend
from .prompts import build_system_prompt, build_user_prompt, parse_response
from .renderer import MapRenderer

Pose2D = Tuple[float, float, float]


# ── Lightweight frontier extractor (unchanged from original) ──────────────────

def _reachability_distances(grid_int8: np.ndarray,
                             start_xy: Tuple[int, int],
                             inflation_cells: int = 0,
                             ) -> np.ndarray:
    """4-conn BFS from start through cells that are free OR unknown.

    Treats only KNOWN-occupied cells (value >= 50) as walls; obstacles
    are inflated by `inflation_cells` to mirror A*'s costmap clearance.
    Unknown is traversable because frontiers sit at the free-unknown
    boundary. Returns (H, W) int32 grid: -1 unreachable, else step distance.
    """
    H, W = grid_int8.shape
    occupied = grid_int8 >= 50
    if inflation_cells > 0:
        try:
            from scipy.ndimage import binary_dilation
            k = 2 * inflation_cells + 1
            struct = np.ones((k, k), dtype=bool)
            occupied = binary_dilation(occupied, structure=struct)
        except ImportError:
            pass
    sx, sy = start_xy
    dist = np.full((H, W), -1, dtype=np.int32)
    if not (0 <= sx < W and 0 <= sy < H):
        return dist
    if occupied[sy, sx]:
        seeded = False
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nx, ny = sx + dx, sy + dy
            if 0 <= nx < W and 0 <= ny < H and not occupied[ny, nx]:
                dist[ny, nx] = 1
                seeded = True
        if not seeded:
            return dist
    else:
        dist[sy, sx] = 0
    from collections import deque
    queue = deque()
    ys, xs = np.where(dist >= 0)
    for x, y in zip(xs.tolist(), ys.tolist()):
        queue.append((x, y))
    while queue:
        x, y = queue.popleft()
        d = dist[y, x]
        for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
            nx, ny = x + dx, y + dy
            if not (0 <= nx < W and 0 <= ny < H):
                continue
            if dist[ny, nx] != -1:
                continue
            if occupied[ny, nx]:
                continue
            dist[ny, nx] = d + 1
            queue.append((nx, ny))
    return dist


def _extract_frontiers(grid_int8: np.ndarray, resolution: float,
                       origin_x: float, origin_y: float,
                       robot_xy: Tuple[float, float],
                       max_targets: int = 12,
                       stride: int = 2,
                       sensor_range_m: float = 3.5,
                       inflation_cells: int = 0) -> List[Tuple[float, float, float]]:
    """Find boundary cells (free with ≥1 unknown 4-neighbor). Cluster,
    filter by REACHABILITY from robot through free+unknown cells, &
    rank by info-gain proxy: unknown cells within sensor_range.

    Returns list of (wx, wy, info_gain), only candidates A* can reach.
    """
    H, W = grid_int8.shape
    free = (grid_int8 == 0)
    unknown = (grid_int8 == -1)
    nbr_unk = np.zeros_like(unknown)
    nbr_unk[:-1, :] |= unknown[1:, :]
    nbr_unk[1:, :]  |= unknown[:-1, :]
    nbr_unk[:, :-1] |= unknown[:, 1:]
    nbr_unk[:, 1:]  |= unknown[:, :-1]
    boundary = free & nbr_unk

    ys, xs = np.where(boundary)
    if len(xs) == 0:
        return []

    xs = xs[::stride]
    ys = ys[::stride]
    if len(xs) == 0:
        return []

    rcx = int((robot_xy[0] - origin_x) / resolution)
    rcy = int((robot_xy[1] - origin_y) / resolution)
    dist = _reachability_distances(grid_int8, (rcx, rcy),
                                    inflation_cells=inflation_cells)

    cells = list(zip(xs.tolist(), ys.tolist()))
    clusters: List[List[Tuple[int, int]]] = []
    seen = set()
    for cx, cy in cells:
        if (cx, cy) in seen:
            continue
        cluster = []
        stack = [(cx, cy)]
        while stack:
            x, y = stack.pop()
            if (x, y) in seen:
                continue
            seen.add((x, y))
            if 0 <= x < W and 0 <= y < H and boundary[y, x]:
                cluster.append((x, y))
                for dx, dy in ((-1, 0), (1, 0), (0, -1), (0, 1)):
                    stack.append((x + dx, y + dy))
        if cluster:
            clusters.append(cluster)

    radius_cells = int(sensor_range_m / max(resolution, 1e-3))

    cands: List[Tuple[float, float, float]] = []
    for cluster in clusters:
        mx = sum(c[0] for c in cluster) / len(cluster)
        my = sum(c[1] for c in cluster) / len(cluster)
        cx0 = int(mx); cy0 = int(my)
        if not (0 <= cx0 < W and 0 <= cy0 < H):
            continue
        if dist[cy0, cx0] < 0:
            reachable_member = None
            for cx, cy in cluster:
                if 0 <= cx < W and 0 <= cy < H and dist[cy, cx] >= 0:
                    reachable_member = (cx, cy); break
            if reachable_member is None:
                continue
            cx0, cy0 = reachable_member
            mx, my = float(cx0), float(cy0)

        wx = origin_x + (mx + 0.5) * resolution
        wy = origin_y + (my + 0.5) * resolution

        # Push the goal 0.6 m back toward the robot so it lands inside clearly
        # free space, not at the exact free-unknown boundary. Without this,
        # A* initially accepts the goal (29 wps) but as the robot approaches
        # and the wall is mapped precisely, the goal falls into the inflated
        # obstacle zone and A* rejects it — causing repeated unreachable fails.
        _inset_m = 0.6
        _dx = robot_xy[0] - wx
        _dy = robot_xy[1] - wy
        _d = math.sqrt(_dx * _dx + _dy * _dy)
        if _d > _inset_m + 0.01:
            wx += (_dx / _d) * _inset_m
            wy += (_dy / _d) * _inset_m

        x0 = max(0, cx0 - radius_cells); x1 = min(W, cx0 + radius_cells + 1)
        y0 = max(0, cy0 - radius_cells); y1 = min(H, cy0 + radius_cells + 1)
        patch = unknown[y0:y1, x0:x1]
        info_gain = float(patch.sum()) * resolution * resolution

        path_cells = float(dist[cy0, cx0])
        utility = info_gain - 0.1 * path_cells * resolution
        cands.append((wx, wy, info_gain, utility))

    cands.sort(key=lambda c: c[3], reverse=True)
    return [(c[0], c[1], c[2]) for c in cands[:max_targets]]


# ── VLMExplorer ──────────────────────────────────────────────────────────────

class VLMExplorer:
    """LangGraph-powered goal source for the standalone exploration loop.

    Drop-in replacement for CFPA2: publishes goals on /<ns>/goal_pose
    when the LangGraph agent returns a fresh decision. The agent runs in
    a background thread so the 500 Hz sim loop stays at full rate.

    Adaptive triggers:
        - goal_reached on /<ns>/nav_status  → fires next query immediately
        - stuck_detected on /<ns>/recovery_event → fires query with cooldown
        - fallback: fixed cycle_period_sec timer
    """

    def __init__(
        self,
        mapper: OccupancyMapper,
        *,
        namespace: str = "robot",
        provider: str = "auto",
        model: str = "",
        cycle_period_sec: float = 12.0,
        min_replan_dist_m: float = 0.3,
        max_frontier_cands: int = 10,
        sensor_range_m: float = 3.5,
        planner_inflation_cells: int = 4,
        planner: Optional[AStarPlanner] = None,
        timeout_sec: float = 25.0,
        temperature: float = 0.1,
        max_tokens: int = 2048,
        mission: Optional[str] = None,
        log_dir: Optional[str] = None,
        camera_fn: Optional[Callable[[], Optional[str]]] = None,
        strategy: str = "mock",
        strip_info_gain: bool = False,
        disable_history: bool = False,
    ) -> None:
        self._mapper = mapper
        self._ns = namespace
        self._provider_arg = provider
        self._provider = vlm_backend.resolve_provider(provider)
        self._model = model or (
            "gemini-2.5-flash-lite" if self._provider == "google"
            else vlm_backend.default_model(self._provider)
        )
        self._cycle_period = max(2.0, float(cycle_period_sec))
        self._min_replan_dist = max(0.0, float(min_replan_dist_m))
        self._max_cands = int(max_frontier_cands)
        self._sensor_range = float(sensor_range_m)
        self._inflation_cells = int(planner_inflation_cells)
        self._planner = planner or AStarPlanner(inflation_cells=self._inflation_cells)
        self._timeout_sec = float(timeout_sec)
        self._temperature = float(temperature)
        self._max_tokens = int(max_tokens)
        self._mission = mission
        self._camera_fn = camera_fn  # callable() → base64 PNG str or None
        # Strategy used when provider=='mock' (non-VLM baselines):
        #   'mock' / 'info_gain'   — highest info_gain frontier (default)
        #   'random'               — uniform random from A*-valid candidates
        #   'greedy_nearest'       — nearest frontier by Euclidean distance
        self._strategy = strategy
        # Ablation knobs (Pri 4/6/7).
        self._strip_info_gain = bool(strip_info_gain)
        self._disable_history = bool(disable_history)

        self._renderer = MapRenderer(mapper)

        self._robot_pose: Pose2D = (0.0, 0.0, 0.0)
        self._last_query_t: float = -1e9
        self._last_goal: Optional[Tuple[float, float]] = None
        self._artifacts: List[dict] = []
        self._failed_goals: List[dict] = []
        self._failed_goals_max = 8
        self._failed_radius_m = 0.5

        # Exploration history: last N {cycle, goal, reason} — injected into
        # each LangGraph cycle as memory of where the robot has already been.
        self._history: List[dict] = []
        self._history_max = 20


        # Adaptive trigger flag: set by goal_reached or stuck_detected events.
        self._trigger_now: bool = False
        # Minimum wall-clock seconds between any two dispatches — prevents API
        # rate limits (Groq 6000 TPM, Gemini 15 RPM) from creating a tight retry
        # loop when a goal is cleared and immediately re-triggers a new cycle.
        # 20s gives the per-minute token quota meaningful recovery time.
        self._min_dispatch_interval: float = max(20.0, cycle_period_sec * 0.5)
        # Separate cooldown for stuck-triggered queries (avoid thrash).
        self._last_stuck_trigger_t: float = -1e9
        self._stuck_trigger_cooldown = 10.0

        self._worker: Optional[threading.Thread] = None
        self._pending_lock = threading.Lock()
        self._pending_result: Optional[dict] = None
        self._cycle_counter = 0

        # Snapshot of the last ROS grid built in _build_scene — shared with
        # tool closures so tools use the same grid the agent was dispatched with.
        self._last_ros_grid = None

        # Tracks consecutive cycles where A* rejected all BFS candidates.
        # On the first occurrence we attempt a trail-retreat before giving up.
        self._consecutive_all_rejected: int = 0

        # Session log dir
        if log_dir is None:
            base = os.environ.get("STANDALONE_VLM_LOG",
                                  str(Path("/tmp") / "standalone_vlm"))
            ts = time.strftime("%Y%m%d_%H%M%S")
            self._log_dir = Path(base) / f"session_{ts}"
        else:
            self._log_dir = Path(log_dir)
        self._log_dir.mkdir(parents=True, exist_ok=True)

        # BUS subscriptions
        BUS.subscribe(f"/{self._ns}/odom/nav", self._on_odom)
        BUS.subscribe(f"/{self._ns}/nav_status", self._on_nav_status)
        BUS.subscribe(f"/{self._ns}/recovery_event", self._on_recovery_event)

        self._goal_topic = f"/{self._ns}/goal_pose"
        self._status_topic = f"/{self._ns}/exploration_status"
        self._artifact_topic = f"/{self._ns}/artifact_seen"
        self._status_pub_state: str = "init"

    # ── Public API ──────────────────────────────────────────────────────────

    @property
    def provider(self) -> str:
        return self._provider

    @property
    def model(self) -> str:
        return self._model

    @property
    def artifacts(self) -> List[dict]:
        return list(self._artifacts)

    def tick(self, sim_time: float) -> None:
        """Called periodically from the main sim loop (~2 Hz).

        Ingests completed agent results and fires new queries when:
          - a goal_reached or stuck_detected event set _trigger_now, OR
          - the fixed cycle_period_sec timer has elapsed.
        """
        self._ingest_result()

        if self._worker is not None and self._worker.is_alive():
            return

        timer_elapsed = (sim_time - self._last_query_t) >= self._cycle_period
        min_gap_ok = (sim_time - self._last_query_t) >= self._min_dispatch_interval
        if (self._trigger_now or timer_elapsed) and min_gap_ok:
            self._trigger_now = False
            self._last_query_t = sim_time
            self._dispatch_query()

    # ── BUS callbacks ───────────────────────────────────────────────────────

    def _on_odom(self, msg) -> None:
        x = float(msg.pose.pose.position.x)
        y = float(msg.pose.pose.position.y)
        qz = float(msg.pose.pose.orientation.z)
        qw = float(msg.pose.pose.orientation.w)
        yaw = 2.0 * math.atan2(qz, qw)
        self._robot_pose = (x, y, yaw)
        self._renderer.append_trail(x, y)

    def _on_nav_status(self, msg) -> None:
        try:
            payload = json.loads(getattr(msg, "data", "") or "{}")
        except Exception:
            return
        if not isinstance(payload, dict):
            return
        state = str(payload.get("state", ""))

        if state == "goal_reached":
            # Only trigger early if we actually had an active goal that was
            # reached. If _last_goal is None (no goal set yet, or after a
            # failed VLM cycle), "goal_reached" just means "no path" — don't
            # treat that as a trigger or every 429 fires the next cycle
            # immediately.
            if self._last_goal is not None:
                self._trigger_now = True

        elif state in ("unreachable", "failed"):
            gx = payload.get("goal_x")
            gy = payload.get("goal_y")
            if gx is None or gy is None:
                if self._last_goal is None:
                    return
                gx, gy = self._last_goal
            gx = float(gx); gy = float(gy)
            for f in self._failed_goals[-self._failed_goals_max:]:
                if math.hypot(gx - f["x"], gy - f["y"]) < self._failed_radius_m:
                    return
            self._failed_goals.append({
                "x": round(gx, 3), "y": round(gy, 3),
                "reason": "planner unreachable",
                "cycle": self._cycle_counter,
            })
            if len(self._failed_goals) > self._failed_goals_max:
                self._failed_goals = self._failed_goals[-self._failed_goals_max:]
            self._last_goal = None

    def _on_recovery_event(self, msg) -> None:
        """Watchdog publishes stuck_detected → trigger an immediate VLM query
        so the agent can pick an escape goal from the new wedged position."""
        event = str(getattr(msg, "data", "") or "")
        if event != "stuck_detected":
            return
        t = time.monotonic()
        if t - self._last_stuck_trigger_t < self._stuck_trigger_cooldown:
            return
        self._last_stuck_trigger_t = t
        self._trigger_now = True

    def _publish_status(self, state: str) -> None:
        if state == self._status_pub_state:
            return
        self._status_pub_state = state
        BUS.publish(self._status_topic, String(data=state))

    # ── Scene building ───────────────────────────────────────────────────────

    def _filter_unreachable(self, frontiers: List[Tuple[float, float, float]]
                            ) -> List[Tuple[float, float, float]]:
        if not self._failed_goals:
            return frontiers
        kept = []
        for x, y, ig in frontiers:
            skip = any(
                math.hypot(x - f["x"], y - f["y"]) < self._failed_radius_m
                for f in self._failed_goals
            )
            if not skip:
                kept.append((x, y, ig))
        return kept

    def _build_scene(self) -> Tuple[dict, list]:
        """Build scene dict + validated frontier list. Also caches _last_ros_grid."""
        m = self._mapper
        grid = np.full((m.height, m.width), -1, dtype=np.int8)
        obs = m._observed
        log_odds = m._log_odds
        grid[obs & (log_odds <= m._free_thr)] = 0
        grid[obs & (log_odds >= m._occ_thr)] = 100

        bfs_candidates = _extract_frontiers(
            grid, m.resolution, m.origin_x, m.origin_y,
            (self._robot_pose[0], self._robot_pose[1]),
            max_targets=self._max_cands * 3,
            sensor_range_m=self._sensor_range,
            inflation_cells=self._inflation_cells,
        )
        bfs_candidates = self._filter_unreachable(bfs_candidates)

        ros_grid = m.to_occupancy_grid()
        self._last_ros_grid = ros_grid  # snapshot for tool closures

        pose_xy = (self._robot_pose[0], self._robot_pose[1])
        frontiers: List[Tuple[float, float, float]] = []
        for cand in bfs_candidates:
            if len(frontiers) >= self._max_cands:
                break
            path = self._planner.plan(ros_grid, pose_xy, (cand[0], cand[1]))
            if path:
                frontiers.append(cand)

        if bfs_candidates and not frontiers:
            # A* rejected every BFS candidate — robot is wedged against a wall
            # or all remaining frontiers are in inflation-blocked zones.
            # Increment counter; _dispatch_query will attempt a trail retreat on
            # the first occurrence before declaring no_frontiers.
            self._consecutive_all_rejected += 1
            print(f"[VLM] A* rejected all {len(bfs_candidates)} BFS candidates "
                  f"(occurrence #{self._consecutive_all_rejected}).")
        else:
            self._consecutive_all_rejected = 0

        known = int(np.sum(obs))
        known_m2 = known * (m.resolution ** 2)

        scene = {
            "map": {
                "resolution_m": round(m.resolution, 3),
                "width_cells": int(m.width),
                "height_cells": int(m.height),
                "origin_x_m": round(m.origin_x, 3),
                "origin_y_m": round(m.origin_y, 3),
                "extent_xmin_m": round(m.origin_x, 3),
                "extent_xmax_m": round(m.origin_x + m.width * m.resolution, 3),
                "extent_ymin_m": round(m.origin_y, 3),
                "extent_ymax_m": round(m.origin_y + m.height * m.resolution, 3),
            },
            "robot": {
                "x": round(self._robot_pose[0], 3),
                "y": round(self._robot_pose[1], 3),
                "yaw_deg": round(math.degrees(self._robot_pose[2]), 1),
            },
            "coverage": {
                "known_m2": round(known_m2, 2),
                "known_cells": known,
            },
            "frontier_candidates": [
                # Pri 4 ablation: strip info_gain so VLM cannot anchor on it.
                ({"x": round(c[0], 3), "y": round(c[1], 3)}
                 if self._strip_info_gain
                 else {"x": round(c[0], 3), "y": round(c[1], 3),
                       "info_gain": round(c[2], 2)})
                for c in frontiers
            ],
            "last_goal": (
                {"x": round(self._last_goal[0], 3),
                 "y": round(self._last_goal[1], 3)}
                if self._last_goal else None
            ),
            "artifacts_logged": [a["pos"] for a in self._artifacts],
            "recent_failed_goals": list(self._failed_goals),
        }
        return scene, frontiers

    # ── Retreat helper ───────────────────────────────────────────────────────

    def _find_retreat_goal(self) -> Optional[Tuple[float, float]]:
        """Find a reachable point along the recent trail at least 2 m behind
        the robot.  Used when A* rejects all frontier candidates (wedged).

        Walks backward through the renderer trail and A*-checks each candidate
        at least 2 m away from the current pose.  Stops searching after the
        first point that is more than 8 m behind (diminishing returns), or
        after trying 20 candidates.  Returns (x, y) on success, None if the
        trail is too short or every candidate is also unreachable.
        """
        trail = list(self._renderer._trail)
        if len(trail) < 5:
            return None
        ros_grid = self._last_ros_grid
        if ros_grid is None:
            return None
        rx, ry, _ = self._robot_pose
        checked = 0
        for tx, ty in reversed(trail[:-1]):
            dist = math.hypot(tx - rx, ty - ry)
            if dist < 2.0:
                continue
            if dist > 8.0:
                break
            checked += 1
            if checked > 20:
                break
            path = self._planner.plan(ros_grid, (rx, ry), (tx, ty))
            if path:
                return (tx, ty)
        return None

    # ── Dispatch ─────────────────────────────────────────────────────────────

    def _dispatch_query(self) -> None:
        scene, frontiers = self._build_scene()

        if not scene["frontier_candidates"]:
            # First time A* rejects all BFS candidates: attempt a trail retreat
            # to back the robot out of the tight corner before giving up.
            # Only try once — if the retreat itself fails, declare no_frontiers.
            if self._consecutive_all_rejected == 1:
                retreat = self._find_retreat_goal()
                if retreat is not None:
                    rx, ry = retreat
                    print(f"[VLM] Retreat goal: ({rx:.2f}, {ry:.2f}) — "
                          f"backing out of wedge before next frontier query.")
                    msg = PoseStamped.make(rx, ry, 0.0)
                    BUS.publish(self._goal_topic, msg)
                    self._last_goal = (rx, ry)
                    self._publish_status("executing")
                    self._last_query_t = time.time()
                    return
                else:
                    print("[VLM] Retreat: no valid trail point found — "
                          "falling through to no_frontiers.")
            self._publish_status("no_frontiers")
            # No frontiers left — exploration is complete. Publish the
            # completion signal directly rather than waiting for the
            # coverage-stagnation window (which defaults to 120 s).
            BUS.publish(f"/{self._ns}/exploration_complete",
                        String(data="no_frontiers"))
            return

        # ── Render map PNG (main thread — matplotlib Agg, safe) ──────────
        try:
            map_png_b64 = self._renderer.render_b64(
                self._robot_pose,
                frontiers=frontiers,
                goal=self._last_goal,
                title=f"VLM cycle {self._cycle_counter} | "
                      f"{self._provider}/{self._model}",
            )
        except Exception as exc:
            print(f"[VLM] map renderer failed: {exc}")
            return

        # ── Capture camera frame (main thread — mujoco.Renderer) ─────────
        camera_png_b64: Optional[str] = None
        if self._camera_fn is not None:
            try:
                camera_png_b64 = self._camera_fn()
            except Exception as exc:
                print(f"[VLM] camera capture failed (skipping): {exc}")

        # ── Build user text prompt ────────────────────────────────────────
        from .prompts import build_system_prompt, build_user_prompt
        sys_prompt = build_system_prompt(
            mission=self._mission or (
                "Systematically cover as much unknown space as possible — always "
                "move toward the frontier with the most unknown area nearby. "
                "On the way, if the camera shows any colored objects (spheres, "
                "boxes, cylinders, markers), log their world coordinates once."
            )
        )
        user_text = build_user_prompt(scene)

        # ── Snapshot state for thread-safe tool closures ──────────────────
        ros_grid_snapshot = self._last_ros_grid
        pose_snapshot = self._robot_pose
        frontiers_snapshot = [
            {"x": round(c[0], 3), "y": round(c[1], 3), "info_gain": round(c[2], 2)}
            for c in frontiers
        ]
        # Pri 7 ablation: optionally suppress history injection to the VLM.
        history_snapshot = [] if self._disable_history else list(self._history)

        self._cycle_counter += 1
        cycle_id = self._cycle_counter

        # ── Persist cycle artifacts ───────────────────────────────────────
        try:
            (self._log_dir / f"cycle_{cycle_id:04d}_scene.json").write_text(
                json.dumps(scene, indent=2)
            )
            (self._log_dir / f"cycle_{cycle_id:04d}_user_prompt.txt").write_text(user_text)
            (self._log_dir / f"cycle_{cycle_id:04d}_map.png").write_bytes(
                self._renderer.render_png(
                    self._robot_pose, frontiers=frontiers,
                    goal=self._last_goal, title=f"cycle {cycle_id}",
                )
            )
            # Save camera frame so visualise_vlm_inputs.py can replay sessions.
            if camera_png_b64:
                import base64
                (self._log_dir / f"cycle_{cycle_id:04d}_camera.png").write_bytes(
                    base64.b64decode(camera_png_b64)
                )
        except Exception:
            pass

        self._publish_status("querying")
        provider = self._provider_arg   # pass string so worker can re-resolve
        model = self._model
        strategy = self._strategy       # baseline strategy for mock provider
        mapper = self._mapper
        planner = self._planner
        log_dir = self._log_dir
        artifacts_ref = self._artifacts  # for on_artifact callback (append-only)

        def _on_artifact_cb(x: float, y: float, description: str) -> None:
            entry = {
                "cycle": cycle_id,
                "pos": [round(x, 3), round(y, 3)],
                "reason": description,
                "t": time.time(),
            }
            artifacts_ref.append(entry)
            try:
                BUS.publish(self._artifact_topic, String(data=json.dumps(entry)))
                (log_dir / "artifacts.json").write_text(
                    json.dumps(artifacts_ref, indent=2)
                )
            except Exception:
                pass

        def _worker() -> None:
            t0 = time.monotonic()
            try:
                if vlm_backend.resolve_provider(provider) == "mock":
                    # Bypass LangGraph — use configured strategy (random /
                    # greedy_nearest / info_gain). No API call, no LangGraph.
                    raw = vlm_backend._mock_pick_goal(scene, strategy=strategy)
                    decision = parse_response(raw)
                    err = None
                elif vlm_backend.resolve_provider(provider) == "groq":
                    # Groq llama-4-scout returns HTTP 400 "Failed to call a
                    # function" via LangGraph's tool-bind path. Use direct
                    # single-shot call instead — no tools, synthesize artifact
                    # log from the parsed JSON below.
                    from .agent import run_direct
                    decision = run_direct(
                        provider="groq", model=model,
                        system_prompt=sys_prompt,
                        map_png_b64=map_png_b64,
                        camera_png_b64=camera_png_b64,
                        user_text=user_text,
                        history=history_snapshot,
                        temperature=0.1, max_tokens=2048,
                    )
                    # Manually fire the artifact callback if VLM reported one.
                    if decision.get("artifact_seen") and decision.get("artifact_pos"):
                        ap = decision["artifact_pos"]
                        try:
                            _on_artifact_cb(float(ap[0]), float(ap[1]),
                                            decision.get("reason", "")[:80])
                        except Exception:
                            pass
                    err = None
                else:
                    from .tools import make_tools
                    from .agent import run_agent
                    tools = make_tools(
                        ros_grid_snapshot=ros_grid_snapshot,
                        planner=planner,
                        pose_snapshot=pose_snapshot,
                        frontiers_snapshot=frontiers_snapshot,
                        mapper=mapper,
                        on_artifact=_on_artifact_cb,
                    )
                    decision = run_agent(
                        provider=vlm_backend.resolve_provider(provider),
                        model=model,
                        tools=tools,
                        system_prompt=sys_prompt,
                        map_png_b64=map_png_b64,
                        camera_png_b64=camera_png_b64,
                        user_text=user_text,
                        history=history_snapshot,
                        temperature=0.1,
                        max_tokens=2048,
                        max_iterations=2,
                    )
                    err = None
            except Exception as exc:
                decision = {
                    "goal": None, "reason": f"worker_error: {exc}",
                    "artifact_seen": False, "artifact_pos": None, "done": False,
                }
                err = str(exc)

            dur = time.monotonic() - t0
            try:
                (log_dir / f"cycle_{cycle_id:04d}_decision.json").write_text(
                    json.dumps({**decision, "duration_sec": round(dur, 2),
                                "error": err}, indent=2)
                )
            except Exception:
                pass

            with self._pending_lock:
                self._pending_result = {
                    "decision": decision,
                    "error": err,
                    "duration_sec": dur,
                    "cycle_id": cycle_id,
                }

        self._worker = threading.Thread(target=_worker, daemon=True,
                                        name="vlm-agent")
        self._worker.start()

    # ── Result ingestion ─────────────────────────────────────────────────────

    def _ingest_result(self) -> None:
        with self._pending_lock:
            result = self._pending_result
            self._pending_result = None
        if result is None:
            return

        decision = result["decision"]
        err = result["error"]
        cycle = result["cycle_id"]
        dur = result["duration_sec"]
        print(f"[VLM] cycle {cycle} returned in {dur:.1f}s "
              f"goal={decision['goal']} reason={decision['reason'][:60]!r}"
              + (f" ERROR={err}" if err else ""))

        # Update exploration history (for next cycle's memory context)
        self._history.append({
            "cycle": cycle,
            "goal": list(decision["goal"]) if decision["goal"] else None,
            "reason": decision["reason"],
        })
        if len(self._history) > self._history_max:
            self._history = self._history[-self._history_max:]

        if decision.get("done"):
            self._publish_status("done")
            return

        goal_xy = decision["goal"]
        if goal_xy is None:
            self._publish_status("hold")
            return

        # Skip duplicate goals (within min_replan_dist)
        if self._last_goal is not None:
            if math.hypot(goal_xy[0] - self._last_goal[0],
                          goal_xy[1] - self._last_goal[1]) < self._min_replan_dist:
                self._publish_status("hold")
                return

        msg = PoseStamped.make(float(goal_xy[0]), float(goal_xy[1]), 0.0)
        BUS.publish(self._goal_topic, msg)
        self._last_goal = (float(goal_xy[0]), float(goal_xy[1]))
        self._publish_status("executing")
        print(f"[VLM] published goal ({goal_xy[0]:.2f}, {goal_xy[1]:.2f})")
