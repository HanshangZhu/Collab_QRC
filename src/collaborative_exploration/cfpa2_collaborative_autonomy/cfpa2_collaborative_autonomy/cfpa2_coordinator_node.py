#!/usr/bin/env python3
"""CFPA2 ROS2 coordinator with optional space-time A* waypointing."""

from __future__ import annotations

import ctypes
import heapq
import json
import math
import os
import sys
import time
from collections import deque
from pathlib import Path
from typing import Any, Optional

import numpy as np

# ---------- C++ grid-ops acceleration (optional) ----------
_GRID_OPS_LIB = None
try:
    _here = os.path.dirname(os.path.abspath(__file__))
    _here_real = os.path.dirname(os.path.realpath(__file__))
    for _d in (_here, _here_real):
        _so = os.path.join(_d, "cfpa2_grid_ops.so")
        if os.path.isfile(_so):
            _GRID_OPS_LIB = ctypes.CDLL(_so)
            break
    _GRID_OPS_LIB.extract_frontiers.restype = ctypes.c_int
    _GRID_OPS_LIB.extract_frontiers.argtypes = [
        ctypes.POINTER(ctypes.c_int8),   # grid
        ctypes.c_int, ctypes.c_int,       # W, H
        ctypes.c_float, ctypes.c_float, ctypes.c_float,  # res, ox, oy
        ctypes.c_int,                     # stride
        ctypes.c_float,                   # min_cluster_area
        ctypes.c_int,                     # clearance_cells
        ctypes.c_int8, ctypes.c_int8, ctypes.c_int8,  # free_val, unknown_val, occ_threshold
        ctypes.POINTER(ctypes.c_float),   # out_x
        ctypes.POINTER(ctypes.c_float),   # out_y
        ctypes.c_int,                     # max_out
    ]
    _GRID_OPS_LIB.distance_transform.restype = None
    _GRID_OPS_LIB.distance_transform.argtypes = [
        ctypes.POINTER(ctypes.c_int8),   # grid
        ctypes.c_int, ctypes.c_int,       # W, H
        ctypes.c_int, ctypes.c_int,       # sx, sy
        ctypes.c_int8,                    # free_val
        ctypes.POINTER(ctypes.c_int),     # dist_out
    ]
    if hasattr(_GRID_OPS_LIB, "distance_transform_range"):
        _GRID_OPS_LIB.distance_transform_range.restype = None
        _GRID_OPS_LIB.distance_transform_range.argtypes = [
            ctypes.POINTER(ctypes.c_int8),   # grid
            ctypes.c_int, ctypes.c_int,       # W, H
            ctypes.c_int, ctypes.c_int,       # sx, sy
            ctypes.c_int8, ctypes.c_int8,     # unknown_val, occ_threshold
            ctypes.POINTER(ctypes.c_int),     # dist_out
        ]
    _GRID_OPS_LIB.batch_info_gain.restype = None
    _GRID_OPS_LIB.batch_info_gain.argtypes = [
        ctypes.POINTER(ctypes.c_int8),   # grid
        ctypes.c_int, ctypes.c_int,       # W, H
        ctypes.c_float, ctypes.c_float, ctypes.c_float,  # res, ox, oy
        ctypes.POINTER(ctypes.c_float),   # goal_x
        ctypes.POINTER(ctypes.c_float),   # goal_y
        ctypes.c_int,                     # n_goals
        ctypes.c_int,                     # radius
        ctypes.c_int8,                    # unknown_val
        ctypes.POINTER(ctypes.c_float),   # gains_out
    ]
    # 2026-05-19: optional batch floodfill IG path. Older .so versions
    # don't have it; presence-check before binding so legacy builds
    # silently fall back to Python floodfill.
    if hasattr(_GRID_OPS_LIB, "batch_info_gain_floodfill"):
        _GRID_OPS_LIB.batch_info_gain_floodfill.restype = None
        _GRID_OPS_LIB.batch_info_gain_floodfill.argtypes = [
            ctypes.POINTER(ctypes.c_int8),   # grid
            ctypes.c_int, ctypes.c_int,       # W, H
            ctypes.c_float, ctypes.c_float, ctypes.c_float,  # res, ox, oy
            ctypes.POINTER(ctypes.c_float),   # goal_x
            ctypes.POINTER(ctypes.c_float),   # goal_y
            ctypes.c_int,                     # n_goals
            ctypes.c_int,                     # budget
            ctypes.c_int,                     # max_radius_cells
            ctypes.c_int8,                    # unknown_val
            ctypes.POINTER(ctypes.c_int32),   # visited_scratch (W*H ints)
            ctypes.POINTER(ctypes.c_float),   # gains_out
        ]
    # 2026-05-19: optional C++ filter_dead_frontiers + cluster_representatives.
    if hasattr(_GRID_OPS_LIB, "filter_dead_frontiers"):
        _GRID_OPS_LIB.filter_dead_frontiers.restype = ctypes.c_int
        _GRID_OPS_LIB.filter_dead_frontiers.argtypes = [
            ctypes.POINTER(ctypes.c_int8),   # grid
            ctypes.c_int, ctypes.c_int,       # W, H
            ctypes.c_float, ctypes.c_float, ctypes.c_float,  # res, ox, oy
            ctypes.c_int8,                    # occ_threshold
            ctypes.c_int,                     # radius_cells
            ctypes.c_int,                     # min_live_unknowns
            ctypes.POINTER(ctypes.c_float),   # in_x
            ctypes.POINTER(ctypes.c_float),   # in_y
            ctypes.c_int,                     # n_in
            ctypes.POINTER(ctypes.c_float),   # out_x
            ctypes.POINTER(ctypes.c_float),   # out_y
        ]
    if hasattr(_GRID_OPS_LIB, "cluster_representatives"):
        _GRID_OPS_LIB.cluster_representatives.restype = ctypes.c_int
        _GRID_OPS_LIB.cluster_representatives.argtypes = [
            ctypes.POINTER(ctypes.c_float),   # in_x
            ctypes.POINTER(ctypes.c_float),   # in_y
            ctypes.c_int,                     # n_in
            ctypes.c_float,                   # cluster_radius_m
            ctypes.POINTER(ctypes.c_float),   # out_x
            ctypes.POINTER(ctypes.c_float),   # out_y
        ]
except Exception:
    _GRID_OPS_LIB = None

import rclpy
from geometry_msgs.msg import Point, PointStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from std_msgs.msg import Empty, String
from visualization_msgs.msg import Marker, MarkerArray


def _resolve_cfpa2_overlap_penalty_fn():
    candidates: list[Path] = []
    here = Path(__file__).resolve()

    # Source-tree path: .../src/cfpa2_collaborative_autonomy/cfpa2_collaborative_autonomy
    if len(here.parents) >= 3:
        candidates.append(here.parents[2])

    # Generic fallback: search upward for repository root containing cfpa2_demo.
    for parent in here.parents[:8]:
        if (parent / "cfpa2_demo").is_dir():
            candidates.append(parent)

    for root in candidates:
        if not (root / "cfpa2_demo").is_dir():
            continue
        root_str = str(root)
        if root_str not in sys.path:
            sys.path.insert(0, root_str)
        try:
            from cfpa2_demo.core.allocator import overlap_penalty as cfpa2_overlap_penalty
        except Exception:
            continue
        return cfpa2_overlap_penalty
    return None


_CFPA2_OVERLAP_PENALTY_FN = _resolve_cfpa2_overlap_penalty_fn()


def select_first_route_goals(
    *,
    namespaces: list[str],
    routes: dict[int, list[int]],
    exploring_cells: list[tuple[float, float, float]],
    robot_xy: dict[str, tuple[float, float]],
    min_assign_distance: float,
) -> dict[str, tuple[float, float]]:
    goals: dict[str, tuple[float, float]] = {}
    for idx, ns in enumerate(namespaces):
        goals_for_robot = routes.get(idx, [])
        goal = first_goal_for_route(
            goals_for_robot,
            exploring_cells,
            robot_xy=robot_xy.get(ns),
            min_assign_distance=min_assign_distance,
        )
        if goal is not None:
            goals[ns] = goal
    return goals


class CFPA2Coordinator(Node):
    def __init__(
        self,
        *,
        node_name: str = "cfpa2_coordinator",
        default_namespaces: Optional[list[str]] = None,
        startup_label: str = "cfpa2_coordinator",
        planner_desc: str = "Coordinator",
    ) -> None:
        super().__init__(node_name)

        if default_namespaces is None:
            default_namespaces = ["robot_a", "robot_b"]
        self._startup_label = startup_label
        self._planner_desc = planner_desc

        self.declare_parameter("namespaces", default_namespaces)
        self.declare_parameter("publish_rate", 1.0)
        self.declare_parameter("beta", 0.18)
        self.declare_parameter("sensor_range", 3.5)
        self.declare_parameter("frontier_stride", 2)
        self.declare_parameter("max_targets", 800)
        # The C++ frontier extractor streams raw frontier cells in grid scan
        # order. Large robot-centric elevation maps can otherwise fill the raw
        # target buffer with distant border frontiers before local reachable
        # frontiers are seen. Over-fetch raw cells, then cluster/filter them in
        # Python so max_targets limits representative goals, not scan order.
        self.declare_parameter("frontier_raw_overfetch_factor", 20)
        self.declare_parameter("frontier_raw_overfetch_min", 4096)
        self.declare_parameter("goal_topic_suffix", "/way_point_coord")
        self.declare_parameter("nav_status_topic_suffix", "/nav_status")
        # Map source for frontier extraction + BFS reachability. Two practical choices:
        #   "/map"                       — raw octomap projection (legacy default).
        #                                  BFS is "inflation-blind"; can mark frontiers
        #                                  reachable that Nav2 actually can't path to.
        #   "/global_costmap/costmap"    — Nav2's inflated global costmap (recommended).
        #                                  BFS walks the same cost field the planner
        #                                  uses, so CFPA2's reachability matches Nav2's
        #                                  plannability. Inflation gradient cells (cost
        #                                  ≥ occ_thresh, default 50) are blocked.
        # Either way the message type is OccupancyGrid; only the cell-value semantics
        # differ. unknown_value (-1) is the same in both.
        self.declare_parameter("planning_map_topic_suffix", "/map")
        self.declare_parameter("free_value", 0)
        self.declare_parameter("unknown_value", -1)
        self.declare_parameter("occupancy_block_threshold", 50)
        self.declare_parameter("cfpa2_reachability_occ_threshold", 0)
        self.declare_parameter("cfpa2_reachability_allow_unknown", False)
        # Information-gain aggregation mode (always 2D):
        #   "local"     — count unknown cells in a fixed radius square around
        #                 the goal (original ETH-style local IG). Tends to
        #                 undervalue large rooms because the count saturates
        #                 at π·r²; near small nooks then win on distance.
        #   "floodfill" — connected-component count of unknown cells reachable
        #                 from the goal via 4-connectivity through unknown
        #                 territory, capped at `cfpa2_floodfill_budget` cells
        #                 and `cfpa2_floodfill_max_radius_cells` from the goal.
        #                 Big rooms saturate the budget; small nooks finish
        #                 well below it → utility reflects geometric size.
        self.declare_parameter("cfpa2_ig_mode", "local")
        self.declare_parameter("cfpa2_floodfill_budget", 2000)
        self.declare_parameter("cfpa2_floodfill_max_radius_cells", 100)
        self.declare_parameter("switch_hysteresis", 0.02)
        self.declare_parameter("switch_min_dist", 0.35)
        self.declare_parameter("min_assign_distance", 0.30)

        # Level-1 stabilization controls.
        self.declare_parameter("goal_lock_sec", 5.0)
        self.declare_parameter("progress_window_sec", 3.0)
        self.declare_parameter("progress_min_delta_m", 0.15)
        self.declare_parameter("blacklist_fail_count", 2)
        self.declare_parameter("blacklist_ttl_sec", 30.0)
        self.declare_parameter("blacklist_key_resolution", 0.5)
        self.declare_parameter("reached_blacklist_dist", 0.30)
        # Distance at which an exploration frontier is considered satisfied.
        # This is a sensing waypoint contract, not a docking tolerance. If left
        # <=0, it follows max(switch_min_dist, reached_blacklist_dist).
        self.declare_parameter("goal_satisfied_dist", 0.0)
        # The relaxed sensing radius is valid only when the frontier is
        # visible through known-free map cells. This prevents small obstacles
        # from making an occluded frontier look reached by Euclidean distance.
        self.declare_parameter("goal_satisfied_requires_los", True)
        # Very close goals count as reached even if the line check is noisy
        # at the robot footprint. Keep this much smaller than the sensing
        # radius.
        self.declare_parameter("goal_satisfied_direct_dist", 0.30)
        # Optional local-waypoint horizon. Disabled by default for legacy maps;
        # nav_test_3d_explore enables it because elevation maps can expose
        # distant frontier cells before the controller can safely traverse the
        # intermediate terrain.
        self.declare_parameter("cfpa2_max_goal_distance_m", 0.0)
        self.declare_parameter("reached_blacklist_repeat_count", 3)
        self.declare_parameter("reached_blacklist_ttl_sec", 12.0)
        self.declare_parameter("overlap_weight", 1.0)
        self.declare_parameter("cfpa2_w_ig", 1.0)
        self.declare_parameter("cfpa2_w_c", 0.6)
        self.declare_parameter("cfpa2_w_sw", 0.2)
        # 2026-05-09: now interpreted as "max fraction deducted from joint
        # score when goals fully overlap" (multiplicative). Default 0.5 =
        # up to 50% off. See _cfpa2_overlap_penalty / joint allocator block.
        self.declare_parameter("cfpa2_lambda_overlap", 0.5)
        # Outer weight applied to the momentum_bonus term. Bumped 0.8 →
        # 2.0 on 2026-04-25 after demo3_mixed showed a 0.4 Hz goal flap
        # for robot_a despite brake-hold; momentum bonus range was too
        # small relative to info_gain wobble to lock direction.
        self.declare_parameter("cfpa2_w_momentum", 2.0)
        # Inner alpha/beta of momentum_bonus. Previously hardcoded
        # (0.5 / 1.0). Higher alpha = stronger forward bias even when
        # stopped (matters during brake_hold or at goal-reached pause).
        # Higher beta = larger penalty for direction-flip mid-stride.
        self.declare_parameter("cfpa2_momentum_alpha", 1.5)
        self.declare_parameter("cfpa2_momentum_beta",  2.0)
        # Spatial-cluster radius for collapsing the extractor's per-cell
        # frontier samples into one centroid per cluster. 1.0 m merges
        # adjacent stride samples on the same wall edge into a single
        # representative; smaller values risk fragmenting one wall into
        # many "clusters", larger values risk merging two genuine
        # frontiers (different rooms / opposite sides of an obstacle)
        # into one. demo3_mixed corridors are 1.0–2.0 m wide, so 1.0 m
        # keeps separate corridor mouths distinct.
        self.declare_parameter("cfpa2_frontier_cluster_radius_m", 1.0)
        # Dead-frontier filter: drop centroids whose surrounding has
        # too few unknown cells. The radius is the search square
        # (half-side, m). The threshold is the minimum unknown cell
        # count inside that square for the frontier to be considered
        # "live". A trap frontier (unknown cells behind a wall, or
        # outside world bounds) has only 1-3 unknown cells nearby;
        # a real frontier has dozens.
        self.declare_parameter("cfpa2_frontier_unknown_check_radius_m", 0.40)
        self.declare_parameter("cfpa2_frontier_min_unknown_cells", 20)
        # Min-hold lock-in: once a goal is assigned, refuse to switch
        # it for at least this many seconds. Doesn't apply to blacklist
        # or max-lock overrides — those still force a switch. Stops
        # cluster-centroid jitter from reassigning the same robot 5-10×
        # in a few seconds while it's mid-navigation.
        self.declare_parameter("cfpa2_goal_min_hold_sec", 5.0)
        # Stable-challenger override: lets a freshly-seen high-utility frontier
        # preempt the held goal mid-flight, but only if (a) the same candidate
        # has been the top-1 frontier for `streak_required` consecutive ticks
        # (anti-jitter — cluster centroids drift tick-to-tick, real winners
        # persist) AND (b) its score beats utility(held) re-evaluated under
        # the current map by `improvement_factor` AND (c) at least
        # `min_lock_age_sec` has elapsed since the held goal was assigned.
        # Set streak_required=0 to disable the override (legacy commitment-
        # only behavior). Default 1.20× = 20% better; raise to be more
        # conservative, lower to be more reactive.
        self.declare_parameter("cfpa2_challenger_streak_required", 3)
        self.declare_parameter("cfpa2_challenger_improvement_factor", 1.20)
        self.declare_parameter("cfpa2_challenger_min_lock_age_sec", 2.0)
        # Frontier planner: take the top-K frontiers by utility and solve a
        # nearest-neighbor TSP starting from the robot. The head of that
        # tour becomes the candidate. Picks a near frontier when it's on
        # the route to a bigger far frontier, suppressing churn and giving
        # roughly linear sweep motion through clusters of frontiers.
        # K=5 → 120 permutations, brute force in microseconds. K too large
        # would pull in far/low-utility frontiers that drag the tour out of
        # position.
        self.declare_parameter("cfpa2_planner_mode", "greedy")
        self.declare_parameter("cfpa2_tsp_k", 5)
        # Stranded-frontier abort radius: if NO current frontier candidate is
        # within this distance of the held goal, the held goal is declared
        # stale and switched immediately (mid-navigation). Previously the
        # radius was hardcoded max(switch_min_dist*1.5, 0.50) = 0.67m which
        # was too generous — even after exploration revealed the cells that
        # made the goal a frontier, some unrelated frontier within 0.67m
        # would keep it alive. 0.40m (4 cells @ 0.10m res) ties the check
        # to a tight cell neighbourhood.
        self.declare_parameter("cfpa2_stale_frontier_radius_m", 0.40)
        self.declare_parameter("cfpa2_min_utility", -0.5)
        self.declare_parameter("cfpa2_sigma_overlap_m", 0.0)
        self.declare_parameter("cfpa2_stuck_lock_sec", 45.0)
        self.declare_parameter("cfpa2_stuck_min_motion_m", 0.20)
        self.declare_parameter("cfpa2_stuck_blacklist_sec", 60.0)
        # Rolling-window width for the stuck-recovery motion check.
        # Replaces the single goal_lock_start_xy anchor (which latched at
        # goal-set and never re-armed, so any early drift made robot look
        # permanently "in motion"). moved_dist is now max |p_t - p_now|
        # over the last cfpa2_stuck_window_sec odom samples.
        self.declare_parameter("cfpa2_stuck_window_sec", 45.0)
        # Cluster-scale blacklist radius. When a goal is blacklisted,
        # a disk of this radius is also forbidden. Prevents stuck-recovery
        # from re-picking a 0.05 m neighbour in the same V-graph orphan
        # after blacklisting a single cell. Set 0 to disable (cell-only).
        self.declare_parameter("blacklist_cluster_radius_m", 1.0)
        # Hysteresis lock-age override. Once a goal has been held for this
        # many seconds without being reached, stop blocking alternative
        # candidates merely for being close to it — the held goal is
        # almost certainly unreachable. 0 disables the override (legacy).
        self.declare_parameter("switch_hysteresis_max_lock_sec", 20.0)
        self.declare_parameter("local_nav_status_stale_sec", 3.0)
        self.declare_parameter("local_nav_stall_blacklist_sec", 45.0)
        # nav_status/v1 fast-blacklist path — when any planner publishes
        # state=="unreachable" or "failed", blacklist the current goal
        # within ~200 ms instead of waiting for the 45 s legacy stall timer.
        # Set fast_unreachable_enabled=false to fall back to legacy only.
        self.declare_parameter("fast_unreachable_enabled", True)
        self.declare_parameter("fast_unreachable_blacklist_sec", 60.0)
        self.declare_parameter("cfpa2_close_stop_radius_m", 0.35)
        self.declare_parameter("cfpa2_close_stop_speed_epsilon", 0.02)
        self.declare_parameter("cfpa2_space_time_enabled", True)
        self.declare_parameter("cfpa2_space_time_horizon_sec", 5.0)
        self.declare_parameter("cfpa2_space_time_dt_sec", 0.40)
        self.declare_parameter("cfpa2_space_time_safety_radius_m", 0.45)
        self.declare_parameter("cfpa2_space_time_waypoint_lookahead_m", 0.9)
        self.declare_parameter("cfpa2_space_time_window_margin_m", 3.0)
        self.declare_parameter("cfpa2_space_time_max_expansions", 12000)
        self.declare_parameter("cfpa2_space_time_assumed_speed_mps", 0.25)
        self.declare_parameter("cfpa2_space_time_max_speed_mps", 0.60)
        self.declare_parameter("cfpa2_frontier_min_cluster_area_m2", 0.20)
        self.declare_parameter("cfpa2_frontier_obstacle_clearance_m", 0.40)
        # Goal-level clearance is re-evaluated every tick. Frontier extraction
        # can accept a point before a small obstacle is mapped; once the
        # obstacle appears, the active goal must be invalidated instead of held
        # by min-hold / pivot-lock.
        self.declare_parameter("cfpa2_goal_obstacle_clearance_m", 0.0)
        self.declare_parameter("exploration_gain_radius_cells", 4)
        self.declare_parameter("marker_frame_override", "world")
        self.declare_parameter("coordinator_map_topic", "/mtare/coordinator_map")
        self.declare_parameter("robot_markers_topic", "/mtare/robot_markers")
        self.declare_parameter("frontier_markers_topic", "/mtare/frontier_markers")
        self.declare_parameter("trajectory_max_points", 600)
        self.declare_parameter("trajectory_min_point_distance", 0.08)
        self.declare_parameter("robot_marker_scale", 0.35)
        self.declare_parameter("perf_enable", True)
        self.declare_parameter("perf_tick_window_size", 240)
        self.declare_parameter("perf_min_samples", 20)
        self.declare_parameter("perf_tick_warn_p95_ms", 150.0)
        self.declare_parameter("perf_cpu_warn_pct", 15.0)
        self.declare_parameter("adaptive_load_shedding_enabled", False)
        self.declare_parameter("adaptive_budget_utilization", 0.85)
        self.declare_parameter("adaptive_restore_utilization", 0.55)
        self.declare_parameter("adaptive_max_frontier_stride", 8)
        self.declare_parameter("adaptive_min_max_targets", 120)
        self.declare_parameter("adaptive_min_exploration_gain_radius_cells", 2)
        self.declare_parameter("adaptive_max_skip_ticks", 2)
        self.declare_parameter("debug_no_goal_logging", True)
        self.declare_parameter("debug_no_goal_log_interval_sec", 2.0)

        self.namespaces = [str(x) for x in self.get_parameter("namespaces").value]
        self.publish_rate = max(0.2, float(self.get_parameter("publish_rate").value))
        self.beta = float(self.get_parameter("beta").value)
        self.sensor_range = max(0.1, float(self.get_parameter("sensor_range").value))
        self.frontier_stride = max(1, int(self.get_parameter("frontier_stride").value))
        self.max_targets = max(50, int(self.get_parameter("max_targets").value))
        self.frontier_raw_overfetch_factor = max(
            1, int(self.get_parameter("frontier_raw_overfetch_factor").value)
        )
        self.frontier_raw_overfetch_min = max(
            self.max_targets,
            int(self.get_parameter("frontier_raw_overfetch_min").value),
        )
        self.goal_topic_suffix = str(self.get_parameter("goal_topic_suffix").value)
        # Validate planning_map_topic_suffix: must start with '/' and be non-empty
        # (we concatenate with /<ns>{suffix}). Empty / malformed values silently
        # fall back to the legacy /map topic.
        _pm = str(self.get_parameter("planning_map_topic_suffix").value).strip()
        if not _pm.startswith("/"):
            _pm = "/" + _pm if _pm else "/map"
        self.planning_map_topic_suffix = _pm
        self.nav_status_topic_suffix = str(self.get_parameter("nav_status_topic_suffix").value)
        self.free_value = int(self.get_parameter("free_value").value)
        self.unknown_value = int(self.get_parameter("unknown_value").value)
        self.occ_thresh = int(self.get_parameter("occupancy_block_threshold").value)
        reachability_occ_threshold = int(
            self.get_parameter("cfpa2_reachability_occ_threshold").value
        )
        self.cfpa2_reachability_occ_threshold = (
            max(1, min(100, reachability_occ_threshold))
            if reachability_occ_threshold > 0
            else self.occ_thresh
        )
        self.cfpa2_reachability_allow_unknown = bool(
            self.get_parameter("cfpa2_reachability_allow_unknown").value
        )
        _ig_mode = str(self.get_parameter("cfpa2_ig_mode").value).strip().lower()
        if _ig_mode not in ("local", "floodfill"):
            self.get_logger().warn(
                f"cfpa2_ig_mode='{_ig_mode}' invalid, falling back to 'local'")
            _ig_mode = "local"
        self.cfpa2_ig_mode = _ig_mode
        self.cfpa2_floodfill_budget = int(max(1, self.get_parameter("cfpa2_floodfill_budget").value))
        self.cfpa2_floodfill_max_radius_cells = int(max(1,
            self.get_parameter("cfpa2_floodfill_max_radius_cells").value))
        self.switch_hysteresis = max(0.0, float(self.get_parameter("switch_hysteresis").value))
        self.switch_min_dist = max(0.1, float(self.get_parameter("switch_min_dist").value))
        self.min_assign_distance = max(0.0, float(self.get_parameter("min_assign_distance").value))

        self.goal_lock_sec = max(0.0, float(self.get_parameter("goal_lock_sec").value))
        self.progress_window_sec = max(0.5, float(self.get_parameter("progress_window_sec").value))
        self.progress_min_delta_m = max(0.0, float(self.get_parameter("progress_min_delta_m").value))
        self.blacklist_fail_count = max(1, int(self.get_parameter("blacklist_fail_count").value))
        self.blacklist_ttl_sec = max(0.0, float(self.get_parameter("blacklist_ttl_sec").value))
        self.blacklist_key_resolution = max(
            0.05,
            float(self.get_parameter("blacklist_key_resolution").value),
        )
        self.reached_blacklist_dist = max(0.0, float(self.get_parameter("reached_blacklist_dist").value))
        configured_goal_satisfied_dist = float(self.get_parameter("goal_satisfied_dist").value)
        if configured_goal_satisfied_dist > 0.0:
            self.goal_satisfied_dist = max(0.1, configured_goal_satisfied_dist)
        else:
            self.goal_satisfied_dist = max(self.switch_min_dist, self.reached_blacklist_dist)
        self.goal_satisfied_requires_los = bool(
            self.get_parameter("goal_satisfied_requires_los").value
        )
        self.goal_satisfied_direct_dist = min(
            self.goal_satisfied_dist,
            max(0.0, float(self.get_parameter("goal_satisfied_direct_dist").value)),
        )
        self.cfpa2_max_goal_distance_m = max(
            0.0, float(self.get_parameter("cfpa2_max_goal_distance_m").value)
        )
        # A goal that is satisfied by the exploration contract must also be
        # eligible for reached-goal blacklisting; otherwise the coordinator can
        # keep reassigning a point Nav2 already treats as complete.
        self.reached_blacklist_dist = max(self.reached_blacklist_dist, self.goal_satisfied_dist)
        self.reached_blacklist_repeat_count = max(
            1,
            int(self.get_parameter("reached_blacklist_repeat_count").value),
        )
        self.reached_blacklist_ttl_sec = max(
            0.0,
            float(self.get_parameter("reached_blacklist_ttl_sec").value),
        )
        self.overlap_weight = max(0.0, float(self.get_parameter("overlap_weight").value))
        self.cfpa2_w_ig = float(self.get_parameter("cfpa2_w_ig").value)
        self.cfpa2_w_c = max(0.0, float(self.get_parameter("cfpa2_w_c").value))
        self.cfpa2_w_sw = max(0.0, float(self.get_parameter("cfpa2_w_sw").value))
        self.cfpa2_lambda_overlap = max(0.0, float(self.get_parameter("cfpa2_lambda_overlap").value))
        self.cfpa2_w_momentum = max(0.0, float(self.get_parameter("cfpa2_w_momentum").value))
        self.cfpa2_momentum_alpha = max(0.0, float(self.get_parameter("cfpa2_momentum_alpha").value))
        self.cfpa2_momentum_beta  = max(0.0, float(self.get_parameter("cfpa2_momentum_beta").value))
        self.cfpa2_frontier_cluster_radius_m = max(
            0.0, float(self.get_parameter("cfpa2_frontier_cluster_radius_m").value))
        self.cfpa2_frontier_unknown_check_radius_m = max(
            0.0, float(self.get_parameter("cfpa2_frontier_unknown_check_radius_m").value))
        self.cfpa2_frontier_min_unknown_cells = max(
            0, int(self.get_parameter("cfpa2_frontier_min_unknown_cells").value))
        self.cfpa2_goal_min_hold_sec = max(
            0.0, float(self.get_parameter("cfpa2_goal_min_hold_sec").value))
        self.cfpa2_challenger_streak_required = max(
            0, int(self.get_parameter("cfpa2_challenger_streak_required").value))
        self.cfpa2_challenger_improvement_factor = max(
            1.0, float(self.get_parameter("cfpa2_challenger_improvement_factor").value))
        self.cfpa2_challenger_min_lock_age_sec = max(
            0.0, float(self.get_parameter("cfpa2_challenger_min_lock_age_sec").value))
        _planner_mode = str(self.get_parameter("cfpa2_planner_mode").value).strip().lower()
        if _planner_mode not in ("greedy", "tsp_topk"):
            self.get_logger().warn(
                f"cfpa2_planner_mode='{_planner_mode}' invalid, falling back to 'greedy'")
            _planner_mode = "greedy"
        self.cfpa2_planner_mode = _planner_mode
        self.cfpa2_stale_frontier_radius_m = max(
            0.05, float(self.get_parameter("cfpa2_stale_frontier_radius_m").value))
        self.cfpa2_tsp_k = max(1, int(self.get_parameter("cfpa2_tsp_k").value))
        self.cfpa2_min_utility = float(self.get_parameter("cfpa2_min_utility").value)
        self.cfpa2_sigma_overlap_m = max(0.0, float(self.get_parameter("cfpa2_sigma_overlap_m").value))
        self.cfpa2_stuck_lock_sec = max(0.0, float(self.get_parameter("cfpa2_stuck_lock_sec").value))
        self.cfpa2_stuck_min_motion_m = max(
            0.0, float(self.get_parameter("cfpa2_stuck_min_motion_m").value)
        )
        self.cfpa2_stuck_blacklist_sec = max(
            0.0, float(self.get_parameter("cfpa2_stuck_blacklist_sec").value)
        )
        self.cfpa2_stuck_window_sec = max(
            1.0, float(self.get_parameter("cfpa2_stuck_window_sec").value)
        )
        self.blacklist_cluster_radius_m = max(
            0.0, float(self.get_parameter("blacklist_cluster_radius_m").value)
        )
        self.switch_hysteresis_max_lock_sec = max(
            0.0, float(self.get_parameter("switch_hysteresis_max_lock_sec").value)
        )
        self.local_nav_status_stale_sec = max(
            0.0, float(self.get_parameter("local_nav_status_stale_sec").value)
        )
        self.local_nav_stall_blacklist_sec = max(
            0.0, float(self.get_parameter("local_nav_stall_blacklist_sec").value)
        )
        self.fast_unreachable_enabled = bool(
            self.get_parameter("fast_unreachable_enabled").value
        )
        self.fast_unreachable_blacklist_sec = max(
            5.0, float(self.get_parameter("fast_unreachable_blacklist_sec").value)
        )
        # Per-namespace dedup: remembers the goal_seq we already fast-blacklisted
        # so we don't keep firing for every repeat of the same status message.
        self._last_unreachable_goal_seq: dict[str, object] = {}

        # Fast-BL debounce + startup grace (added 2026-04-26).
        # Without these, demo3_mixed dual-robot run at sim t=26-32 s
        # blacklisted 5 of B's first 5 goals because astar returned
        # no_plan_repeated while the local map was still being built.
        # All 5 BLs were 60 s long; by sim t≈130 s every reachable
        # candidate near B was poisoned and B parked permanently
        # (exploration_complete) for the rest of the 11-min run.
        #
        # Two guards:
        #  (a) startup_grace_sec: don't fast-BL during initial period
        #      after node start. Default 15 s — long enough for Fast-LIO
        #      to bootstrap (~5 s) and octomap to populate B's local
        #      grid (~5 s) plus margin.
        #  (b) consecutive_threshold: require K back-to-back unreachable
        #      reports on the SAME (ns, goal) before fast-BL. Default 3.
        #      Real unreachables persist (astar republishes status at
        #      10 Hz so 3 reports = 0.3 s); transient startup fails
        #      typically resolve within 1-2 reports.
        self.declare_parameter("fast_unreachable_startup_grace_sec", 15.0)
        self.declare_parameter("fast_unreachable_consecutive_threshold", 3)
        self.fast_unreachable_startup_grace_sec = max(
            0.0, float(self.get_parameter("fast_unreachable_startup_grace_sec").value)
        )
        self.fast_unreachable_consecutive_threshold = max(
            1, int(self.get_parameter("fast_unreachable_consecutive_threshold").value)
        )
        self._node_start_ns = self.get_clock().now().nanoseconds
        # Per-(ns, goal_key) consecutive count; reset on goal change or
        # when a non-unreachable status arrives.
        self._unreachable_consec: dict[str, dict[tuple, int]] = {}

        # ── Narrow-passage pivot lock ────────────────────────────────
        # When the robot is in a corridor with clearance smaller than
        # its bounding-circle radius (= half-diagonal + tolerance), it
        # CANNOT pivot in place to face a new goal direction without
        # leg/body contact with the wall. demo3_mixed B's spawn corner
        # at (4.1, -6.0) is sandwiched by sw_v_1/sw_v_2 (~0.45 m gap);
        # any CFPA2 goal change while B is in this zone forced B to
        # turn, which scuffed sw_v_2 with the rear thigh.
        #
        # Lock: if pivot-clearance < pivot_lock_radius_m, refuse goal
        # changes that demand reorientation; keep the previous goal so
        # the pathFollower drives the robot OUT of the narrow zone in
        # a straight line. Lock auto-releases the moment clearance
        # opens (robot exits the corridor).
        #
        # Default 0.45 m = max(B half-diag 0.36, A half-diag 0.50) - a
        # bit; chosen as a conservative single value across both bots.
        # Override per-namespace if needed.
        self.declare_parameter("pivot_lock_radius_m", 0.45)
        self.pivot_lock_radius_m = max(
            0.0, float(self.get_parameter("pivot_lock_radius_m").value)
        )
        # Cache map for _set_active_goal pivot check (set each tick).
        self._cur_planning_map: Optional[OccupancyGrid] = None
        # After pivot_lock_max_hold_sec, we release the lock — necessary
        # escape valve when the held goal itself becomes the stale command
        # keeping the executor in lethal space.
        self.declare_parameter("pivot_lock_max_hold_sec", 15.0)
        self.pivot_lock_max_hold_sec = max(
            0.0, float(self.get_parameter("pivot_lock_max_hold_sec").value)
        )
        # Release earlier than max-hold if the held goal is actively
        # regressing. At box corners the pivot lock can be locally safe but
        # globally stale; increasing distance to the held goal is the signal.
        self.declare_parameter("pivot_lock_regress_release_m", 0.50)
        self.pivot_lock_regress_release_m = max(
            0.0, float(self.get_parameter("pivot_lock_regress_release_m").value)
        )
        self.cfpa2_close_stop_radius_m = max(
            0.0, float(self.get_parameter("cfpa2_close_stop_radius_m").value)
        )
        self.cfpa2_close_stop_speed_epsilon = max(
            0.0, float(self.get_parameter("cfpa2_close_stop_speed_epsilon").value)
        )
        self.cfpa2_space_time_enabled = bool(self.get_parameter("cfpa2_space_time_enabled").value)
        self.cfpa2_space_time_horizon_sec = max(
            0.5, float(self.get_parameter("cfpa2_space_time_horizon_sec").value)
        )
        self.cfpa2_space_time_dt_sec = max(
            0.05, float(self.get_parameter("cfpa2_space_time_dt_sec").value)
        )
        self.cfpa2_space_time_safety_radius_m = max(
            0.0, float(self.get_parameter("cfpa2_space_time_safety_radius_m").value)
        )
        self.cfpa2_space_time_waypoint_lookahead_m = max(
            0.1, float(self.get_parameter("cfpa2_space_time_waypoint_lookahead_m").value)
        )
        self.cfpa2_space_time_window_margin_m = max(
            0.0, float(self.get_parameter("cfpa2_space_time_window_margin_m").value)
        )
        self.cfpa2_space_time_max_expansions = max(
            1000, int(self.get_parameter("cfpa2_space_time_max_expansions").value)
        )
        self.cfpa2_space_time_assumed_speed_mps = max(
            0.01, float(self.get_parameter("cfpa2_space_time_assumed_speed_mps").value)
        )
        self.cfpa2_space_time_max_speed_mps = max(
            self.cfpa2_space_time_assumed_speed_mps,
            float(self.get_parameter("cfpa2_space_time_max_speed_mps").value),
        )
        self.cfpa2_frontier_min_cluster_area_m2 = max(
            0.0, float(self.get_parameter("cfpa2_frontier_min_cluster_area_m2").value)
        )
        self.cfpa2_frontier_obstacle_clearance_m = max(
            0.0, float(self.get_parameter("cfpa2_frontier_obstacle_clearance_m").value)
        )
        configured_goal_clearance = float(
            self.get_parameter("cfpa2_goal_obstacle_clearance_m").value
        )
        self.cfpa2_goal_obstacle_clearance_m = (
            max(0.0, configured_goal_clearance)
            if configured_goal_clearance > 0.0
            else self.cfpa2_frontier_obstacle_clearance_m
        )
        self.exploration_gain_radius_cells = max(1, int(self.get_parameter("exploration_gain_radius_cells").value))
        self.marker_frame_override = str(self.get_parameter("marker_frame_override").value).strip()
        self.coordinator_map_topic = str(self.get_parameter("coordinator_map_topic").value).strip()
        self.robot_markers_topic = str(self.get_parameter("robot_markers_topic").value).strip()
        self.frontier_markers_topic = str(self.get_parameter("frontier_markers_topic").value).strip()
        self.trajectory_max_points = max(10, int(self.get_parameter("trajectory_max_points").value))
        self.trajectory_min_point_distance = max(
            0.0, float(self.get_parameter("trajectory_min_point_distance").value)
        )
        self.robot_marker_scale = max(0.05, float(self.get_parameter("robot_marker_scale").value))
        self.perf_enable = bool(self.get_parameter("perf_enable").value)
        self.perf_tick_window_size = max(20, int(self.get_parameter("perf_tick_window_size").value))
        self.perf_min_samples = max(5, int(self.get_parameter("perf_min_samples").value))
        self.perf_tick_warn_p95_ms = max(0.0, float(self.get_parameter("perf_tick_warn_p95_ms").value))
        self.perf_cpu_warn_pct = max(0.0, float(self.get_parameter("perf_cpu_warn_pct").value))
        self.adaptive_load_shedding_enabled = bool(
            self.get_parameter("adaptive_load_shedding_enabled").value
        )
        self.adaptive_budget_utilization = min(
            1.5,
            max(0.2, float(self.get_parameter("adaptive_budget_utilization").value)),
        )
        self.adaptive_restore_utilization = min(
            self.adaptive_budget_utilization,
            max(0.1, float(self.get_parameter("adaptive_restore_utilization").value)),
        )
        self.adaptive_max_frontier_stride = max(
            self.frontier_stride,
            int(self.get_parameter("adaptive_max_frontier_stride").value),
        )
        self.adaptive_min_max_targets = min(
            self.max_targets,
            max(50, int(self.get_parameter("adaptive_min_max_targets").value)),
        )
        self.adaptive_min_exploration_gain_radius_cells = min(
            self.exploration_gain_radius_cells,
            max(1, int(self.get_parameter("adaptive_min_exploration_gain_radius_cells").value)),
        )
        self.adaptive_max_skip_ticks = max(
            0,
            int(self.get_parameter("adaptive_max_skip_ticks").value),
        )
        self.debug_no_goal_logging = bool(self.get_parameter("debug_no_goal_logging").value)
        self.debug_no_goal_log_interval_sec = max(
            0.2, float(self.get_parameter("debug_no_goal_log_interval_sec").value)
        )

        self.maps: dict[str, OccupancyGrid] = {}
        self.odoms: dict[str, Odometry] = {}
        self.nav_status: dict[str, dict[str, Any]] = {}
        self.nav_status_rx_time_ns: dict[str, int] = {}
        self.last_goal: dict[str, tuple[float, float]] = {}
        # Per-ns timestamp (ns) when pivot-lock first started holding the
        # current goal. Used to enforce pivot_lock_max_hold_sec escape:
        # after holding pivot-lock for more than max_hold_sec, release the
        # lock and allow the new goal through. Without this, a robot that
        # enters a narrow corridor whose clearance is permanently <
        # pivot_lock_radius_m (e.g. demo3 0.425 m corridor) can never
        # change goal — it stays in legged/idle indefinitely (CLAUDE.md
        # golden rule #12 — multi-layer safety stacks deadlock easily).
        self._pivot_lock_held_since_ns: dict[str, int] = {}
        self._pivot_lock_start_dist_m: dict[str, float] = {}
        # Stable-challenger streak tracking — see _apply_goal_policy override.
        self._challenger_id: dict[str, tuple[int, int]] = {}
        self._challenger_streak: dict[str, int] = {}
        self.last_goal_set_time_ns: dict[str, int] = {}
        self.goal_progress_samples: dict[str, deque[tuple[int, float]]] = {
            ns: deque() for ns in self.namespaces
        }
        self.goal_fail_counts: dict[str, dict[tuple[int, int], int]] = {
            ns: {} for ns in self.namespaces
        }
        self.goal_blacklist_until_ns: dict[str, dict[tuple[int, int], int]] = {
            ns: {} for ns in self.namespaces
        }
        # Cluster-scale blacklist: list of (x, y, radius_m, until_ns) per ns.
        # Checked in parallel with goal_blacklist_until_ns so any point
        # inside a live disk is rejected regardless of which cell it hits.
        self.goal_blacklist_disks: dict[str, list[tuple[float, float, float, int]]] = {
            ns: [] for ns in self.namespaces
        }
        self.reached_goal_repeat_count: dict[str, int] = {ns: 0 for ns in self.namespaces}
        self.reached_goal_last_key: dict[str, Optional[tuple[int, int]]] = {
            ns: None for ns in self.namespaces
        }
        self.last_policy_reason: dict[str, str] = {ns: "init" for ns in self.namespaces}
        self.odom_rx_time_ns: dict[str, int] = {}
        self.odom_velocity_xy: dict[str, tuple[float, float]] = {ns: (0.0, 0.0) for ns in self.namespaces}
        self.trajectory_history: dict[str, deque[tuple[float, float]]] = {
            ns: deque(maxlen=self.trajectory_max_points) for ns in self.namespaces
        }
        self.goal_lock_start_xy: dict[str, Optional[tuple[float, float]]] = {
            ns: None for ns in self.namespaces
        }
        # Rolling pose window used by stuck-recovery to compute
        # displacement over the last cfpa2_stuck_window_sec seconds.
        # maxlen caps memory; _odom_cb trims by time. Replaces the
        # latched single-sample goal_lock_start_xy for motion checks.
        self.goal_lock_pose_history: dict[str, deque[tuple[int, float, float]]] = {
            ns: deque(maxlen=4096) for ns in self.namespaces
        }
        self.cfpa2_last_stuck_event_ns: dict[str, int] = {ns: 0 for ns in self.namespaces}
        self.local_nav_last_stall_event_count: dict[str, int] = {
            ns: 0 for ns in self.namespaces
        }
        self._frontier_replan_last_bl_ns: dict[str, int] = {
            ns: 0 for ns in self.namespaces
        }

        self._cfpa2_last_close_stop_log_ns = 0
        self._start_ns = self.get_clock().now().nanoseconds
        self._summary_interval_sec = 10.0
        self._last_summary_ns = 0
        self._last_prereq_warn_ns = 0
        self._last_no_goal_debug_ns = 0
        self._tick_period_ms = 1000.0 / self.publish_rate
        self._perf_tick_durations_ms: deque[float] = deque(maxlen=self.perf_tick_window_size)
        self._last_perf_summary_ns = 0
        self._perf_last_cpu_process_sec = time.process_time()
        self._perf_last_cpu_wall_ns = time.perf_counter_ns()
        self._adaptive_frontier_stride = self.frontier_stride
        self._adaptive_max_targets = self.max_targets
        self._adaptive_exploration_gain_radius_cells = self.exploration_gain_radius_cells
        self._adaptive_skip_ticks = 0
        self._adaptive_tick_skip_counter = 0

        self.goal_pubs = {}
        self.goal_marker_pubs = {}
        coordinator_map_qos = QoSProfile(
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.coordinator_map_pub = self.create_publisher(
            OccupancyGrid,
            self.coordinator_map_topic,
            coordinator_map_qos,
        )
        self.robot_markers_pub = self.create_publisher(MarkerArray, self.robot_markers_topic, 10)
        self.frontier_markers_pub = self.create_publisher(
            MarkerArray, self.frontier_markers_topic, 10
        )
        for ns in self.namespaces:
            map_topic = f"/{ns}{self.planning_map_topic_suffix}"
            self.create_subscription(OccupancyGrid, map_topic, lambda m, n=ns: self._map_cb(m, n), 1)
            self.get_logger().info(
                f"[{ns}] planning map ← {map_topic} "
                f"(occ_thresh={self.occ_thresh}, unknown={self.unknown_value})")
            self.create_subscription(Odometry, f"/{ns}/odom/nav", lambda m, n=ns: self._odom_cb(m, n), 10)
            self.create_subscription(
                String,
                f"/{ns}{self.nav_status_topic_suffix}",
                lambda m, n=ns: self._nav_status_cb(m, n),
                10,
            )
            self.create_subscription(
                Empty,
                f"/{ns}/frontier_replan",
                lambda m, n=ns: self._frontier_replan_cb(n),
                10,
            )
            self.goal_pubs[ns] = self.create_publisher(PointStamped, f"/{ns}{self.goal_topic_suffix}", 10)
            self.goal_marker_pubs[ns] = self.create_publisher(Marker, f"/{ns}/mtare_goal_marker", 10)

        self.timer = self.create_timer(1.0 / self.publish_rate, self._tick)
        self.get_logger().info(f"[planner_startup] {self._startup_label} initialized.")
        self.get_logger().info(
            f"{self._planner_desc} started for {self.namespaces}\n"
            f"  ── Utility weights ──\n"
            f"    w_ig={self.cfpa2_w_ig:.2f}  w_c={self.cfpa2_w_c:.2f}  "
            f"w_sw={self.cfpa2_w_sw:.2f}  w_momentum={self.cfpa2_w_momentum:.2f}  "
            f"min_utility={self.cfpa2_min_utility:.2f}\n"
            f"  ── Frontier ──\n"
            f"    sensor_range={self.sensor_range:.1f}m  "
            f"gain_radius={self.exploration_gain_radius_cells}cells  "
            f"min_cluster={self.cfpa2_frontier_min_cluster_area_m2:.2f}m²  "
            f"beta={self.beta:.2f}\n"
            f"  ── Assignment ──\n"
            f"    min_dist={self.min_assign_distance:.2f}m  "
            f"switch_hysteresis={self.switch_hysteresis:.3f}  "
            f"goal_lock={self.goal_lock_sec:.0f}s\n"
            f"  ── Stuck / blacklist ──\n"
            f"    stuck_lock={self.cfpa2_stuck_lock_sec:.0f}s  "
            f"stuck_motion={self.cfpa2_stuck_min_motion_m:.2f}m  "
            f"stuck_window={self.cfpa2_stuck_window_sec:.0f}s  "
            f"bl_ttl={self.blacklist_ttl_sec:.0f}s  "
            f"bl_radius={self.blacklist_cluster_radius_m:.2f}m  "
            f"goal_satisfied={self.goal_satisfied_dist:.2f}m  "
            f"goal_los={'on' if self.goal_satisfied_requires_los else 'off'}  "
            f"reached_bl_dist={self.reached_blacklist_dist:.2f}m  "
            f"hyst_lock_age={self.switch_hysteresis_max_lock_sec:.0f}s"
        )

    def _map_cb(self, msg: OccupancyGrid, ns: str) -> None:
        self.maps[ns] = msg

    def _odom_cb(self, msg: Odometry, ns: str) -> None:
        self.odoms[ns] = msg
        now_ns = self.get_clock().now().nanoseconds
        self.odom_rx_time_ns[ns] = now_ns
        self.odom_velocity_xy[ns] = (
            float(msg.twist.twist.linear.x),
            float(msg.twist.twist.linear.y),
        )
        self._append_trajectory(ns, msg)
        # Rolling pose window for stuck-recovery (Fix #2).
        history = self.goal_lock_pose_history[ns]
        history.append((
            now_ns,
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y),
        ))
        cutoff_ns = now_ns - int(self.cfpa2_stuck_window_sec * 1e9)
        while len(history) >= 2 and history[0][0] < cutoff_ns:
            history.popleft()

    def _nav_status_cb(self, msg: String, ns: str) -> None:
        try:
            payload = json.loads(msg.data)
        except json.JSONDecodeError:
            return
        if not isinstance(payload, dict):
            return
        self.nav_status[ns] = payload
        self.nav_status_rx_time_ns[ns] = self.get_clock().now().nanoseconds

        # ── nav_status/v1 fast-blacklist path ─────────────────────────
        # When the active planner definitively declares the current goal
        # unreachable, blacklist it within ~200 ms instead of waiting for the
        # 45 s legacy stall path. Triggered only on state ∈ {unreachable,
        # failed}; state=="stalled" is a soft signal and keeps using the
        # legacy stall_event_count path downstream.
        if not getattr(self, "fast_unreachable_enabled", True):
            return
        state = str(payload.get("state") or "")
        if state not in ("unreachable", "failed"):
            # Don't reset consec counter here: astar publishes status
            # at 50 Hz interleaving unreachable + brake_hold +
            # navigating:leg as it tries different recovery strategies.
            # A reset on every non-unreachable would mean the counter
            # never accumulates above 1. Counter resets only on goal
            # change (handled in _set_active_goal).
            return
        self._apply_fast_blacklist(ns, payload)

    def _apply_fast_blacklist(self, ns: str, payload: dict) -> None:
        """Blacklist the current goal immediately based on planner-reported
        unreachability. Deduped by (ns, goal_seq); goal coordinates must
        match what CFPA2 last assigned so we don't blacklist on stale
        status reports."""
        now_ns = self.get_clock().now().nanoseconds
        goal_seq = payload.get("goal_seq")
        last_seq = self._last_unreachable_goal_seq.get(ns)
        if goal_seq is not None and last_seq == goal_seq:
            return  # already acted on this exact goal_seq

        reported_goal = payload.get("goal")
        current_goal = self.last_goal.get(ns)
        if current_goal is None or not isinstance(reported_goal, (list, tuple)) or len(reported_goal) < 2:
            return
        # Match by quantised key — if the planner's goal key doesn't match
        # CFPA2's last-assigned goal, CFPA2 has rotated goals and the status
        # is stale.
        try:
            reported_key = self._goal_key((float(reported_goal[0]), float(reported_goal[1])))
        except (TypeError, ValueError):
            return
        current_key = self._goal_key(current_goal)
        if reported_key != current_key:
            return

        # Startup grace: skip fast-BL while map/SLAM are still bootstrapping.
        elapsed_sec = (now_ns - self._node_start_ns) * 1e-9
        if elapsed_sec < self.fast_unreachable_startup_grace_sec:
            self.get_logger().info(
                f"{ns}: skipping fast-BL goal=({current_goal[0]:.2f},"
                f"{current_goal[1]:.2f}) — startup grace "
                f"({elapsed_sec:.1f}/{self.fast_unreachable_startup_grace_sec:.1f}s)"
            )
            return

        # Consecutive-threshold debounce: count repeated unreachable
        # reports on the same (ns, goal) before committing to BL.
        ns_consec = self._unreachable_consec.setdefault(ns, {})
        ns_consec[current_key] = ns_consec.get(current_key, 0) + 1
        if ns_consec[current_key] < self.fast_unreachable_consecutive_threshold:
            self.get_logger().info(
                f"{ns}: pending fast-BL goal=({current_goal[0]:.2f},"
                f"{current_goal[1]:.2f}) — consec "
                f"{ns_consec[current_key]}/"
                f"{self.fast_unreachable_consecutive_threshold}"
            )
            return

        bl_sec = float(self.fast_unreachable_blacklist_sec)
        until_ns = now_ns + int(bl_sec * 1e9)
        self.goal_blacklist_until_ns[ns][current_key] = max(
            self.goal_blacklist_until_ns[ns].get(current_key, 0),
            until_ns,
        )
        # Reset consecutive counter for this key (BL latched).
        ns_consec[current_key] = 0
        # Cluster disk (Fix #1 + #3): when the planner reports unreachable,
        # forbid a whole neighbourhood so re-picks can't land in the same
        # orphan. Pairs with _set_active_goal clearing goal_seq on new goals
        # so the adapter can refire fast-blacklist on stuck-recovery re-picks.
        self._add_blacklist_disk(ns, current_goal, until_ns)
        self.goal_fail_counts[ns][current_key] = 0
        self.goal_progress_samples[ns].clear()
        if goal_seq is not None:
            self._last_unreachable_goal_seq[ns] = goal_seq
        state = payload.get("state", "unreachable")
        reason = payload.get("reason", "?")
        source = payload.get("source", "?")
        self.get_logger().warn(
            f"{ns}: FAST-BL goal=({current_goal[0]:.2f},{current_goal[1]:.2f}) "
            f"state={state} reason={reason} src={source} ttl={bl_sec:.1f}s"
        )

    def _frontier_replan_cb(self, ns: str) -> None:
        """Reactive nav signals it cannot reach the current goal."""
        now_ns = self.get_clock().now().nanoseconds
        current_goal = self.last_goal.get(ns)
        if current_goal is None:
            return

        key = self._goal_key(current_goal)

        # Skip if this goal is already blacklisted.
        if self.goal_blacklist_until_ns[ns].get(key, 0) > now_ns:
            return

        # Cooldown: don't blacklist more often than once per 10s per namespace.
        last_bl_ns = self._frontier_replan_last_bl_ns.get(ns, 0)
        if (now_ns - last_bl_ns) < int(10e9):
            return
        self._frontier_replan_last_bl_ns[ns] = now_ns

        bl_sec = max(self.local_nav_stall_blacklist_sec, 20.0)
        until_ns = now_ns + int(bl_sec * 1e9)
        self.goal_blacklist_until_ns[ns][key] = max(
            self.goal_blacklist_until_ns[ns].get(key, 0),
            until_ns,
        )
        self._add_blacklist_disk(ns, current_goal, until_ns)
        self.goal_fail_counts[ns][key] = 0
        self.goal_progress_samples[ns].clear()
        self.get_logger().warn(
            f"{ns}: frontier_replan received — blacklisting current goal "
            f"({current_goal[0]:.2f},{current_goal[1]:.2f}) for {bl_sec:.1f}s."
        )

    @staticmethod
    def _grid_index(x: int, y: int, w: int) -> int:
        return y * w + x

    def _world_to_grid(self, msg: OccupancyGrid, wx: float, wy: float) -> Optional[tuple[int, int]]:
        gx = int((wx - msg.info.origin.position.x) / msg.info.resolution)
        gy = int((wy - msg.info.origin.position.y) / msg.info.resolution)
        if gx < 0 or gy < 0 or gx >= msg.info.width or gy >= msg.info.height:
            return None
        return (gx, gy)

    def _grid_to_world(self, msg: OccupancyGrid, gx: int, gy: int) -> tuple[float, float]:
        return (
            msg.info.origin.position.x + (gx + 0.5) * msg.info.resolution,
            msg.info.origin.position.y + (gy + 0.5) * msg.info.resolution,
        )

    @staticmethod
    def _grid_line_cells(x0: int, y0: int, x1: int, y1: int) -> list[tuple[int, int]]:
        """Bresenham cells from start to end, inclusive."""
        cells: list[tuple[int, int]] = []
        dx = abs(x1 - x0)
        dy = -abs(y1 - y0)
        sx = 1 if x0 < x1 else -1
        sy = 1 if y0 < y1 else -1
        err = dx + dy
        x, y = x0, y0
        while True:
            cells.append((x, y))
            if x == x1 and y == y1:
                break
            e2 = 2 * err
            if e2 >= dy:
                err += dy
                x += sx
            if e2 <= dx:
                err += dx
                y += sy
        return cells

    def _is_free(self, data: list[int], idx: int) -> bool:
        v = data[idx]
        return v != self.unknown_value and 0 <= v < self.occ_thresh

    def _is_unknown(self, data: list[int], idx: int) -> bool:
        return data[idx] == self.unknown_value

    def _has_frontier_obstacle_clearance(
        self,
        data: list[int],
        gx: int,
        gy: int,
        w: int,
        h: int,
        radius_cells: int,
        occ_threshold: Optional[int] = None,
    ) -> bool:
        """Return True iff a circle of radius_cells around (gx,gy) contains no occupied cell."""
        if radius_cells <= 0:
            return True
        occ_thr = int(self.occ_thresh if occ_threshold is None else occ_threshold)
        r2 = radius_cells * radius_cells
        for dy in range(-radius_cells, radius_cells + 1):
            ny = gy + dy
            if ny < 0 or ny >= h:
                return False
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > r2:
                    continue
                nx = gx + dx
                if nx < 0 or nx >= w:
                    return False
                if data[self._grid_index(nx, ny, w)] >= occ_thr:
                    return False
        return True

    def _goal_has_obstacle_clearance(
        self,
        map_msg: OccupancyGrid,
        goal: tuple[float, float],
        clearance_m: Optional[float] = None,
        occ_threshold: Optional[int] = None,
    ) -> bool:
        """Return True when a goal point has the configured obstacle buffer.

        This intentionally uses the same occupied-cell test as frontier
        extraction, but runs on active goals too. That catches frontiers which
        were valid before a clutter obstacle entered the map and later become
        unsafe while hysteresis or pivot-lock would otherwise keep them alive.
        """
        if clearance_m is None:
            clearance_m = getattr(
                self,
                "cfpa2_goal_obstacle_clearance_m",
                getattr(self, "cfpa2_frontier_obstacle_clearance_m", 0.0),
            )
        if clearance_m <= 0.0:
            return True
        grid = self._world_to_grid(map_msg, goal[0], goal[1])
        if grid is None:
            return False
        res = max(1e-6, float(map_msg.info.resolution))
        radius_cells = int(math.ceil(clearance_m / res))
        return self._has_frontier_obstacle_clearance(
            map_msg.data,
            grid[0],
            grid[1],
            int(map_msg.info.width),
            int(map_msg.info.height),
            radius_cells,
            occ_threshold=occ_threshold,
        )

    def _extract_frontiers(self, msg: OccupancyGrid) -> list[tuple[float, float]]:
        w = int(msg.info.width)
        h = int(msg.info.height)
        res = max(1e-6, float(msg.info.resolution))
        s = max(1, self._adaptive_frontier_stride)
        min_area_m2 = self.cfpa2_frontier_min_cluster_area_m2
        clearance_cells = int(math.ceil(self.cfpa2_frontier_obstacle_clearance_m / res))
        max_targets = self._adaptive_max_targets
        raw_capacity = self._frontier_raw_capacity(w, h, max_targets)

        if _GRID_OPS_LIB is not None:
            raw = self._extract_frontiers_cpp(
                msg, w, h, res, s, min_area_m2, clearance_cells, raw_capacity
            )
        else:
            raw = self._extract_frontiers_py(
                msg, w, h, res, s, min_area_m2, clearance_cells, raw_capacity
            )
        # Collapse per-cell samples into one representative per spatial
        # cluster.
        clustered = self._cluster_representatives(raw, self.cfpa2_frontier_cluster_radius_m)
        # Filter out "dead" / trap frontiers: a frontier is only worth
        # exploring if there's a meaningful patch of unknown cells
        # adjacent. When the unknown side is just 1-2 cells stuck
        # behind a wall (or outside world bounds), driving to it
        # accomplishes nothing — the unknown cells stay unknown
        # forever (LiDAR can't see through walls), and the robot
        # gets pinned to that frontier indefinitely.
        # demo3_mixed 2026-04-26: A reached coverage 97 %, was assigned
        # frontier (1.14, 7.44) just 0.56 m from wall_north. The unknown
        # cells "behind" the frontier are outside the arena (y > 8) —
        # permanently invisible to LiDAR. A drove there, FL_wheel hit
        # wall_north, body climbed; A wedged there for 156 s while
        # CFPA2 kept reassigning the same trap frontier.
        return self._filter_dead_frontiers(clustered, msg)[:max_targets]

    def _frontier_raw_capacity(self, width: int, height: int, max_targets: int) -> int:
        map_cells = max(1, int(width) * int(height))
        factor = max(1, int(getattr(self, "frontier_raw_overfetch_factor", 20)))
        minimum = max(max_targets, int(getattr(self, "frontier_raw_overfetch_min", 4096)))
        requested = max(max_targets, minimum, max_targets * factor)
        return min(map_cells, requested)

    def _filter_dead_frontiers(
        self, points: list[tuple[float, float]], msg: OccupancyGrid
    ) -> list[tuple[float, float]]:
        # Fast path: C++ accelerator. Loop body is O(N · R²) with R up to
        # ~10 cells and N up to ~30 frontier candidates per tick; on aarch64
        # the Python version is the second-largest tick chunk after IG.
        if (
            _GRID_OPS_LIB is not None
            and hasattr(_GRID_OPS_LIB, "filter_dead_frontiers")
            and points
            and self.cfpa2_frontier_min_unknown_cells > 0
        ):
            w = int(msg.info.width)
            h = int(msg.info.height)
            res = float(msg.info.resolution)
            if res > 0:
                ox = float(msg.info.origin.position.x)
                oy = float(msg.info.origin.position.y)
                r_cells = max(1, int(round(self.cfpa2_frontier_unknown_check_radius_m / res)))
                n = len(points)
                in_x = (ctypes.c_float * n)(*(p[0] for p in points))
                in_y = (ctypes.c_float * n)(*(p[1] for p in points))
                out_x = (ctypes.c_float * n)()
                out_y = (ctypes.c_float * n)()
                grid_np = np.array(msg.data, dtype=np.int8)
                n_out = _GRID_OPS_LIB.filter_dead_frontiers(
                    grid_np.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)),
                    w, h, res, ox, oy,
                    ctypes.c_int8(int(self.occ_thresh)),
                    int(r_cells),
                    int(self.cfpa2_frontier_min_unknown_cells),
                    in_x, in_y, n,
                    out_x, out_y,
                )
                return [(float(out_x[i]), float(out_y[i])) for i in range(n_out)]
        return self._filter_dead_frontiers_py(points, msg)

    def _filter_dead_frontiers_py(
        self, points: list[tuple[float, float]], msg: OccupancyGrid
    ) -> list[tuple[float, float]]:
        """Drop centroids whose surrounding has too few "live" unknowns.
        A LIVE unknown is one whose 8-neighbour kernel has NO occupied
        cell — i.e., it sits >=1 cell away from any wall. Trap unknowns
        (the cells right outside the arena, beyond a thin wall like
        wall_north) are EXCLUDED because their immediate neighbours
        include the wall itself. Real exploration unknowns sit in
        open areas where the robot's LiDAR will reach if the robot
        gets close — they have only free + unknown neighbours, no
        occupied. demo3_mixed 2026-04-26: A pinned at wall_north for
        156 s chasing a frontier whose "unknowns" were all 1 cell
        outside the arena boundary; live-unknown count = 0, naive
        count = 50+."""
        if not points or self.cfpa2_frontier_min_unknown_cells <= 0:
            return points
        w = int(msg.info.width)
        h = int(msg.info.height)
        res = float(msg.info.resolution)
        if res <= 0:
            return points
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        data = msg.data
        occ_thr = self.occ_thresh
        R_cells = max(1, int(round(self.cfpa2_frontier_unknown_check_radius_m / res)))
        thr = self.cfpa2_frontier_min_unknown_cells

        def is_live_unknown(nx, ny):
            # Cell at (nx, ny) is unknown AND none of its 8-neighbours
            # is occupied (i.e., the unknown is not against a wall).
            idx = ny * w + nx
            if data[idx] >= 0:
                return False  # not unknown
            for ddy in (-1, 0, 1):
                yy = ny + ddy
                if yy < 0 or yy >= h:
                    continue
                for ddx in (-1, 0, 1):
                    if ddx == 0 and ddy == 0:
                        continue
                    xx = nx + ddx
                    if xx < 0 or xx >= w:
                        continue
                    if data[yy * w + xx] >= occ_thr:
                        return False
            return True

        out = []
        for px, py in points:
            gx = int((px - ox) / res)
            gy = int((py - oy) / res)
            live_n = 0
            y0 = max(1, gy - R_cells); y1 = min(h - 2, gy + R_cells)
            x0 = max(1, gx - R_cells); x1 = min(w - 2, gx + R_cells)
            done = False
            for ny in range(y0, y1 + 1):
                if done: break
                for nx in range(x0, x1 + 1):
                    if is_live_unknown(nx, ny):
                        live_n += 1
                        if live_n >= thr:
                            done = True
                            break
            if live_n >= thr:
                out.append((px, py))
        return out

    def _cluster_representatives(
        self, points: list[tuple[float, float]], cluster_radius_m: float
    ) -> list[tuple[float, float]]:
        # Fast path: C++ accelerator. Greedy O(N²) clustering with N up to
        # ~max_targets (1500). On Orin Nano this was ~25 ms per tick; the
        # C++ port collapses to sub-ms.
        if (
            _GRID_OPS_LIB is not None
            and hasattr(_GRID_OPS_LIB, "cluster_representatives")
            and points
            and cluster_radius_m > 0.0
        ):
            n = len(points)
            in_x = (ctypes.c_float * n)(*(p[0] for p in points))
            in_y = (ctypes.c_float * n)(*(p[1] for p in points))
            out_x = (ctypes.c_float * n)()
            out_y = (ctypes.c_float * n)()
            n_out = _GRID_OPS_LIB.cluster_representatives(
                in_x, in_y, n,
                ctypes.c_float(float(cluster_radius_m)),
                out_x, out_y,
            )
            return [(float(out_x[i]), float(out_y[i])) for i in range(n_out)]

        if not points or cluster_radius_m <= 0.0:
            return points
        r2 = cluster_radius_m * cluster_radius_m
        # Each entry: [cx, cy, [members]]. Greedy: each new point
        # joins the first existing cluster within radius (centroid
        # then updated), else seeds a new cluster. Return the cluster
        # medoid (raw member nearest the centroid), not the centroid
        # itself: raw frontier members already passed obstacle-clearance
        # filtering, while a synthesized centroid can land closer to an
        # obstacle than any sampled member.
        clusters: list[list] = []
        for px, py in points:
            joined = False
            for c in clusters:
                dx = px - c[0]; dy = py - c[1]
                if dx * dx + dy * dy <= r2:
                    c[2].append((px, py))
                    n = len(c[2])
                    c[0] = sum(p[0] for p in c[2]) / n
                    c[1] = sum(p[1] for p in c[2]) / n
                    joined = True
                    break
            if not joined:
                clusters.append([px, py, [(px, py)]])
        reps: list[tuple[float, float]] = []
        for cx, cy, members in clusters:
            reps.append(min(
                members,
                key=lambda p: (p[0] - cx) * (p[0] - cx) + (p[1] - cy) * (p[1] - cy),
            ))
        return reps

    def _extract_frontiers_cpp(self, msg, w, h, res, s, min_area_m2, clearance_cells, max_targets):
        grid_np = np.array(msg.data, dtype=np.int8)
        ox = float(msg.info.origin.position.x)
        oy = float(msg.info.origin.position.y)
        out_x = (ctypes.c_float * max_targets)()
        out_y = (ctypes.c_float * max_targets)()
        n = _GRID_OPS_LIB.extract_frontiers(
            grid_np.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)),
            w, h, res, ox, oy,
            s, min_area_m2, clearance_cells,
            ctypes.c_int8(self.free_value),
            ctypes.c_int8(self.unknown_value),
            ctypes.c_int8(self.occ_thresh),
            out_x, out_y, max_targets,
        )
        return [(float(out_x[i]), float(out_y[i])) for i in range(n)]

    def _extract_frontiers_py(self, msg, w, h, res, s, min_area_m2, clearance_cells, max_targets):
        data = msg.data
        out: list[tuple[float, float]] = []
        neighbor8 = ((1, 0), (-1, 0), (0, 1), (0, -1), (1, 1), (-1, 1), (1, -1), (-1, -1))

        frontier_mask = [False] * (w * h)
        frontier_cells: list[tuple[int, int]] = []
        for gy in range(1, h - 1):
            row = gy * w
            for gx in range(1, w - 1):
                idx = row + gx
                if not self._is_free(data, idx):
                    continue
                found_unknown = False
                for dx, dy in neighbor8:
                    nidx = (gy + dy) * w + (gx + dx)
                    if self._is_unknown(data, nidx):
                        found_unknown = True
                        break
                if not found_unknown:
                    continue
                frontier_mask[idx] = True
                frontier_cells.append((gx, gy))

        if not frontier_cells:
            return out

        visited = [False] * (w * h)
        q: deque[tuple[int, int]] = deque()
        for seed_x, seed_y in frontier_cells:
            seed_idx = self._grid_index(seed_x, seed_y, w)
            if visited[seed_idx] or not frontier_mask[seed_idx]:
                continue

            visited[seed_idx] = True
            q.append((seed_x, seed_y))
            component: list[tuple[int, int]] = []

            while q:
                cx, cy = q.popleft()
                component.append((cx, cy))

                for dx, dy in neighbor8:
                    nx = cx + dx
                    ny = cy + dy
                    if nx <= 0 or ny <= 0 or nx >= (w - 1) or ny >= (h - 1):
                        continue
                    nidx = self._grid_index(nx, ny, w)
                    if visited[nidx] or not frontier_mask[nidx]:
                        continue
                    visited[nidx] = True
                    q.append((nx, ny))

            cluster_area_m2 = len(component) * res * res
            if cluster_area_m2 + 1e-9 < min_area_m2:
                continue

            for i, (gx, gy) in enumerate(component):
                if (i % s) != 0:
                    continue
                if not self._has_frontier_obstacle_clearance(data, gx, gy, w, h, clearance_cells):
                    continue
                out.append(self._grid_to_world(msg, gx, gy))
                if len(out) >= max_targets:
                    return out
        return out

    def _distance_transform(
        self,
        msg: OccupancyGrid,
        start_w: tuple[float, float],
        occ_threshold: Optional[int] = None,
        allow_unknown: bool = False,
    ) -> dict[int, int]:
        start = self._world_to_grid(msg, start_w[0], start_w[1])
        if start is None:
            return {}

        w = int(msg.info.width)
        h = int(msg.info.height)
        sx, sy = start
        occ_thr = int(self.occ_thresh if occ_threshold is None else occ_threshold)

        if _GRID_OPS_LIB is not None and not allow_unknown:
            return self._distance_transform_cpp(msg, w, h, sx, sy, occ_thr)
        return self._distance_transform_py(
            msg,
            w,
            h,
            sx,
            sy,
            occ_thr,
            allow_unknown=allow_unknown,
        )

    def _distance_transform_cpp(self, msg, w, h, sx, sy, occ_threshold):
        grid_np = np.array(msg.data, dtype=np.int8)
        dist_np = np.full(w * h, -1, dtype=np.int32)
        if hasattr(_GRID_OPS_LIB, "distance_transform_range"):
            _GRID_OPS_LIB.distance_transform_range(
                grid_np.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)),
                w,
                h,
                sx,
                sy,
                ctypes.c_int8(self.unknown_value),
                ctypes.c_int8(int(occ_threshold)),
                dist_np.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
            )
        elif int(occ_threshold) == 50:
            _GRID_OPS_LIB.distance_transform(
                grid_np.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)),
                w, h, sx, sy,
                ctypes.c_int8(self.free_value),
                dist_np.ctypes.data_as(ctypes.POINTER(ctypes.c_int)),
            )
        else:
            return self._distance_transform_py(msg, w, h, sx, sy, int(occ_threshold))
        # Store flat array for fast lookup; _grid_path_cost_m uses dict[int, int]
        # Convert to dict only for reachable cells
        indices = np.where(dist_np >= 0)[0]
        return dict(zip(indices.tolist(), dist_np[indices].tolist()))

    def _distance_transform_py(
        self,
        msg,
        w,
        h,
        sx,
        sy,
        occ_threshold,
        *,
        allow_unknown: bool = False,
    ):
        data = msg.data
        sidx = self._grid_index(sx, sy, w)
        occ_thr = int(occ_threshold)

        def is_free_idx(idx: int) -> bool:
            v = data[idx]
            if allow_unknown and v == self.unknown_value:
                return True
            return v != self.unknown_value and 0 <= v < occ_thr

        if not is_free_idx(sidx):
            found = None
            for r in range(1, 13):
                for dy in range(-r, r + 1):
                    ny = sy + dy
                    if ny < 0 or ny >= h:
                        continue
                    for dx in range(-r, r + 1):
                        nx = sx + dx
                        if nx < 0 or nx >= w:
                            continue
                        nidx = self._grid_index(nx, ny, w)
                        if is_free_idx(nidx):
                            found = (nx, ny, nidx)
                            break
                    if found is not None:
                        break
                if found is not None:
                    break
            if found is None:
                return {}
            sx, sy, sidx = found

        q = deque([(sx, sy)])
        dist = {sidx: 0}
        while q:
            cx, cy = q.popleft()
            cidx = self._grid_index(cx, cy, w)
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx = cx + dx
                ny = cy + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                nidx = self._grid_index(nx, ny, w)
                if nidx in dist:
                    continue
                if not is_free_idx(nidx):
                    continue
                dist[nidx] = dist[cidx] + 1
                q.append((nx, ny))
        return dist

    def _merge_targets(self, target_lists: list[list[tuple[float, float]]], merge_resolution: float) -> list[tuple[float, float]]:
        out: list[tuple[float, float]] = []
        seen: set[tuple[int, int]] = set()
        q = max(0.05, float(merge_resolution))
        for targets in target_lists:
            for wx, wy in targets:
                key = (int(round(wx / q)), int(round(wy / q)))
                if key in seen:
                    continue
                seen.add(key)
                out.append((wx, wy))
                if len(out) >= self.max_targets:
                    return out
        return out

    def _goal_key(self, goal: tuple[float, float]) -> tuple[int, int]:
        q = self.blacklist_key_resolution
        return (int(round(goal[0] / q)), int(round(goal[1] / q)))

    def _prune_blacklist(self, ns: str, now_ns: int) -> None:
        entries = self.goal_blacklist_until_ns[ns]
        expired = [k for k, until_ns in entries.items() if until_ns <= now_ns]
        for key in expired:
            entries.pop(key, None)
        disks = self.goal_blacklist_disks.get(ns)
        if disks:
            self.goal_blacklist_disks[ns] = [d for d in disks if d[3] > now_ns]

    def _is_blacklisted(self, ns: str, goal: tuple[float, float], now_ns: int) -> bool:
        self._prune_blacklist(ns, now_ns)
        key = self._goal_key(goal)
        until_ns = self.goal_blacklist_until_ns[ns].get(key, 0)
        if until_ns > now_ns:
            return True
        # Cluster-scale blacklist (Fix #1): reject any point inside a
        # live disk, so the next-best frontier can't come from the same
        # V-graph orphan as the last failure.
        gx = float(goal[0])
        gy = float(goal[1])
        for dx, dy, radius, d_until in self.goal_blacklist_disks[ns]:
            if d_until <= now_ns:
                continue
            if math.hypot(gx - dx, gy - dy) <= radius:
                return True
        return False

    def _add_blacklist_disk(
        self,
        ns: str,
        goal: tuple[float, float],
        until_ns: int,
        radius_m: Optional[float] = None,
    ) -> None:
        """Add a disk-shaped blacklist entry so a whole neighbourhood is
        forbidden, not just the quantised cell of `goal`. A no-op when
        blacklist_cluster_radius_m <= 0."""
        if radius_m is None:
            radius_m = self.blacklist_cluster_radius_m
        if radius_m <= 0.0 or until_ns <= 0:
            return
        self.goal_blacklist_disks[ns].append(
            (float(goal[0]), float(goal[1]), float(radius_m), int(until_ns))
        )

    def _register_goal_failure(self, ns: str, goal: tuple[float, float], now_ns: int, reason: str) -> None:
        key = self._goal_key(goal)
        counts = self.goal_fail_counts[ns]
        counts[key] = counts.get(key, 0) + 1
        if counts[key] < self.blacklist_fail_count:
            return

        counts[key] = 0
        if self.blacklist_ttl_sec <= 0.0:
            return

        until_ns = now_ns + int(self.blacklist_ttl_sec * 1e9)
        self.goal_blacklist_until_ns[ns][key] = until_ns
        self._add_blacklist_disk(ns, goal, until_ns)
        self.get_logger().warn(
            f"{ns}: blacklisting goal ({goal[0]:.2f},{goal[1]:.2f}) for {self.blacklist_ttl_sec:.1f}s "
            f"after repeated {reason} failures."
        )

    def _consume_local_nav_stall_blacklists(self, now_ns: int) -> set[str]:
        if self.local_nav_stall_blacklist_sec <= 0.0 or self.local_nav_status_stale_sec <= 0.0:
            return set()

        forced_switch: set[str] = set()
        stale_ns = int(self.local_nav_status_stale_sec * 1e9)
        for ns in self.namespaces:
            diag = self.nav_status.get(ns)
            rx_time_ns = self.nav_status_rx_time_ns.get(ns, 0)
            if diag is None or rx_time_ns <= 0 or (now_ns - rx_time_ns) > stale_ns:
                continue

            try:
                stall_event_count = int(diag.get("stall_event_count", 0))
            except (TypeError, ValueError):
                continue

            if stall_event_count <= self.local_nav_last_stall_event_count.get(ns, 0):
                continue
            self.local_nav_last_stall_event_count[ns] = stall_event_count

            current_goal = self.last_goal.get(ns)
            if current_goal is None:
                continue

            key = self._goal_key(current_goal)
            until_ns = now_ns + int(self.local_nav_stall_blacklist_sec * 1e9)
            self.goal_blacklist_until_ns[ns][key] = max(
                self.goal_blacklist_until_ns[ns].get(key, 0),
                until_ns,
            )
            self.goal_fail_counts[ns][key] = 0
            self.goal_progress_samples[ns].clear()
            forced_switch.add(ns)

            mode = str(diag.get("mode", "navigate"))
            stall_sec = diag.get("stall_sec", "-")
            self.get_logger().warn(
                f"{ns}: local nav deadlock event #{stall_event_count} "
                f"(mode={mode}, stall_sec={stall_sec}) blacklisting current goal "
                f"({current_goal[0]:.2f},{current_goal[1]:.2f}) for "
                f"{self.local_nav_stall_blacklist_sec:.1f}s."
            )

        return forced_switch

    def _distance_robot_to_goal(self, ns: str, goal: tuple[float, float]) -> float:
        od = self.odoms[ns]
        rx = float(od.pose.pose.position.x)
        ry = float(od.pose.pose.position.y)
        return math.hypot(goal[0] - rx, goal[1] - ry)

    def _goal_too_close(self, ns: str, goal: tuple[float, float]) -> bool:
        if self.min_assign_distance <= 0.0:
            return False
        return self._distance_robot_to_goal(ns, goal) <= self.min_assign_distance

    def _goal_too_far(self, ns: str, goal: tuple[float, float]) -> bool:
        max_distance = float(getattr(self, "cfpa2_max_goal_distance_m", 0.0))
        if max_distance <= 0.0:
            return False
        return self._distance_robot_to_goal(ns, goal) > max_distance

    def _goal_satisfaction_map(
        self,
        ns: str,
        map_msg: Optional[OccupancyGrid] = None,
    ) -> Optional[OccupancyGrid]:
        if map_msg is not None:
            return map_msg
        msg = getattr(self, "maps", {}).get(ns)
        if msg is not None:
            return msg
        return getattr(self, "_cur_planning_map", None)

    def _goal_line_of_sight_clear(
        self,
        ns: str,
        goal: tuple[float, float],
        map_msg: Optional[OccupancyGrid] = None,
    ) -> bool:
        msg = self._goal_satisfaction_map(ns, map_msg)
        if msg is None or ns not in self.odoms:
            return False

        od = self.odoms[ns]
        start = self._world_to_grid(
            msg,
            float(od.pose.pose.position.x),
            float(od.pose.pose.position.y),
        )
        end = self._world_to_grid(msg, goal[0], goal[1])
        if start is None or end is None:
            return False

        w = int(msg.info.width)
        data = msg.data
        for i, (gx, gy) in enumerate(self._grid_line_cells(start[0], start[1], end[0], end[1])):
            if i == 0:
                continue
            v = data[self._grid_index(gx, gy, w)]
            if v == self.unknown_value or v >= self.occ_thresh:
                return False
        return True

    def _goal_satisfied(
        self,
        ns: str,
        goal: tuple[float, float],
        map_msg: Optional[OccupancyGrid] = None,
    ) -> bool:
        dist = self._distance_robot_to_goal(ns, goal)
        if dist > self.goal_satisfied_dist:
            return False
        if dist <= getattr(self, "goal_satisfied_direct_dist", 0.0):
            return True
        if not getattr(self, "goal_satisfied_requires_los", True):
            return True
        return self._goal_line_of_sight_clear(ns, goal, map_msg)

    def _goals_equivalent(
        self,
        a: Optional[tuple[float, float]],
        b: Optional[tuple[float, float]],
        *,
        tol_m: float = 0.35,
    ) -> bool:
        """Two goals count as the same point when within `tol_m` metres.

        Used by the CFPA2 duplicate-goal guard so float noise / sub-cell
        differences don't let both robots chase what is effectively the
        same waypoint. Default tol_m = one robot footprint (~0.35m).
        """
        if a is None or b is None:
            return False
        return math.hypot(a[0] - b[0], a[1] - b[1]) <= tol_m

    def _update_reached_goal_blacklist(
        self,
        ns: str,
        now_ns: int,
        map_msg: Optional[OccupancyGrid] = None,
    ) -> None:
        if self.reached_blacklist_ttl_sec <= 0.0 or self.reached_blacklist_dist <= 0.0:
            return

        goal = self.last_goal.get(ns)
        if goal is None:
            self.reached_goal_last_key[ns] = None
            self.reached_goal_repeat_count[ns] = 0
            return

        key = self._goal_key(goal)
        if not self._goal_satisfied(ns, goal, map_msg):
            self.reached_goal_last_key[ns] = key
            self.reached_goal_repeat_count[ns] = 0
            return

        # Do not repeatedly extend an active blacklist entry for the same key.
        if self.goal_blacklist_until_ns[ns].get(key, 0) > now_ns:
            self.reached_goal_repeat_count[ns] = 0
            return

        if self.reached_goal_last_key[ns] == key:
            self.reached_goal_repeat_count[ns] += 1
        else:
            self.reached_goal_last_key[ns] = key
            self.reached_goal_repeat_count[ns] = 1

        if self.reached_goal_repeat_count[ns] < self.reached_blacklist_repeat_count:
            return

        repeat_count = self.reached_goal_repeat_count[ns]
        self.reached_goal_repeat_count[ns] = 0
        until_ns = now_ns + int(self.reached_blacklist_ttl_sec * 1e9)
        self.goal_blacklist_until_ns[ns][key] = until_ns
        self._add_blacklist_disk(ns, goal, until_ns)
        self.get_logger().warn(
            f"{ns}: blacklisting repeatedly reached goal ({goal[0]:.2f},{goal[1]:.2f}) "
            f"for {self.reached_blacklist_ttl_sec:.1f}s "
            f"after {repeat_count} near-goal repeats "
            f"(dist<={self.reached_blacklist_dist:.2f}m)."
        )
        # Notify subclass-installed listeners (e.g. cfpa2_single_robot_node's
        # ClusterTracker records this as a real attempt — a single blacklist
        # event = one genuine engagement with a cluster, regardless of how
        # many ticks CFPA2 republishes the same goal).
        on_blacklist = getattr(self, "_on_reached_blacklist", None)
        if callable(on_blacklist):
            try:
                on_blacklist(ns, goal)
            except Exception as exc:
                self.get_logger().warn(
                    f"_on_reached_blacklist callback raised: {exc}")

    def _goal_reachable(self, map_msg: OccupancyGrid, dist_map: dict[int, int], goal: tuple[float, float]) -> bool:
        g = self._world_to_grid(map_msg, goal[0], goal[1])
        if g is None:
            return False
        idx = self._grid_index(g[0], g[1], int(map_msg.info.width))
        return idx in dist_map

    def _held_goal_safety_failure(
        self,
        map_msg: OccupancyGrid,
        dist_map: dict[int, int],
        goal: tuple[float, float],
    ) -> Optional[str]:
        if not self._goal_reachable(map_msg, dist_map, goal):
            return "unreachable"
        if not self._goal_has_obstacle_clearance(map_msg, goal):
            return "unsafe_clearance"
        return None

    def _blacklist_active_goal(
        self,
        ns: str,
        goal: tuple[float, float],
        now_ns: int,
        reason: str,
        ttl_sec: Optional[float] = None,
    ) -> None:
        key = self._goal_key(goal)
        if ttl_sec is None:
            ttl_sec = max(30.0, self.blacklist_ttl_sec, self.local_nav_stall_blacklist_sec)
        until_ns = now_ns + int(max(1.0, ttl_sec) * 1e9)
        self.goal_blacklist_until_ns[ns][key] = max(
            self.goal_blacklist_until_ns[ns].get(key, 0),
            until_ns,
        )
        self._add_blacklist_disk(ns, goal, until_ns)
        self.goal_fail_counts[ns][key] = 0
        self.goal_progress_samples[ns].clear()
        self.get_logger().warn(
            f"{ns}: blacklisting active goal ({goal[0]:.2f},{goal[1]:.2f}) "
            f"after {reason}; ttl={max(1.0, ttl_sec):.1f}s."
        )

    def _progress_delta(self, ns: str) -> Optional[float]:
        samples = self.goal_progress_samples[ns]
        if len(samples) < 2:
            return None
        span_ns = samples[-1][0] - samples[0][0]
        if span_ns < int(0.5 * self.progress_window_sec * 1e9):
            return None
        return samples[0][1] - samples[-1][1]

    def _pivot_clearance_blocked(self, ns: str) -> bool:
        """True if robot at current pose is in a corridor too narrow to
        pivot (clearance disk < pivot_lock_radius_m). Uses the most
        recent planning_map cached by _tick_impl.
        """
        if self.pivot_lock_radius_m <= 0.0:
            return False
        if ns not in self.odoms or self._cur_planning_map is None:
            return False
        msg = self._cur_planning_map
        od = self.odoms[ns]
        rx, ry = od.pose.pose.position.x, od.pose.pose.position.y
        g = self._world_to_grid(msg, rx, ry)
        if g is None:
            return False
        radius_cells = max(1, int(math.ceil(
            self.pivot_lock_radius_m / max(0.01, msg.info.resolution))))
        cx, cy = g
        w, h = int(msg.info.width), int(msg.info.height)
        data = msg.data
        radius_sq = radius_cells * radius_cells
        for dy in range(-radius_cells, radius_cells + 1):
            ny = cy + dy
            if ny < 0 or ny >= h:
                continue
            row_off = ny * w
            for dx in range(-radius_cells, radius_cells + 1):
                if dx * dx + dy * dy > radius_sq:
                    continue
                nx = cx + dx
                if nx < 0 or nx >= w:
                    continue
                v = data[row_off + nx]
                if v != self.unknown_value and v >= self.occ_thresh:
                    return True
        return False

    def _set_active_goal(self, ns: str, goal: tuple[float, float], now_ns: int) -> tuple[float, float]:
        prev = self.last_goal.get(ns)
        # Narrow-passage pivot lock: if the robot can't pivot at its
        # current pose without scraping a wall, refuse a goal change
        # that would demand reorientation. Keep the previous goal so
        # the executor drives the robot OUT of the corridor in a
        # straight line. Lock auto-releases as soon as clearance opens.
        goal_changed = (
            prev is not None
            and goal is not None
            and (abs(prev[0] - goal[0]) > 1e-3 or abs(prev[1] - goal[1]) > 1e-3)
        )
        pivot_blocked = (
            self._pivot_clearance_blocked(ns)
            if (goal_changed or ns in self._pivot_lock_held_since_ns)
            else False
        )
        if not pivot_blocked:
            self._pivot_lock_held_since_ns.pop(ns, None)
            self._pivot_lock_start_dist_m.pop(ns, None)

        prev_satisfied = (
            prev is not None
            and ns in self.odoms
            and self._goal_satisfied(ns, prev)
        )
        prev_blacklisted = (
            prev is not None
            and self._is_blacklisted(ns, prev, now_ns)
        )
        if (
            goal_changed
            and pivot_blocked
            and not prev_satisfied
            and not prev_blacklisted
        ):
            held_since_ns = self._pivot_lock_held_since_ns.get(ns)
            if held_since_ns is None:
                held_since_ns = now_ns
                self._pivot_lock_held_since_ns[ns] = held_since_ns
                self._pivot_lock_start_dist_m[ns] = self._distance_robot_to_goal(ns, prev)
            held_sec = max(0.0, (now_ns - held_since_ns) * 1e-9)
            start_dist = self._pivot_lock_start_dist_m.get(
                ns,
                self._distance_robot_to_goal(ns, prev),
            )
            current_dist = self._distance_robot_to_goal(ns, prev)
            regress_m = current_dist - start_dist
            regress_release = (
                self.pivot_lock_regress_release_m > 0.0
                and regress_m >= self.pivot_lock_regress_release_m
            )
            max_hold_release = (
                self.pivot_lock_max_hold_sec > 0.0
                and held_sec >= self.pivot_lock_max_hold_sec
            )
            if max_hold_release or regress_release:
                self._pivot_lock_held_since_ns.pop(ns, None)
                self._pivot_lock_start_dist_m.pop(ns, None)
                if regress_release:
                    reason = "pivot-lock regress release"
                    detail = (
                        f"distance to held goal grew {regress_m:.2f}m "
                        f">= {self.pivot_lock_regress_release_m:.2f}m"
                    )
                    policy = "switch/pivot_lock_regress_release"
                else:
                    reason = "pivot-lock max-hold release"
                    detail = f"held {held_sec:.1f}s >= {self.pivot_lock_max_hold_sec:.1f}s"
                    policy = "switch/pivot_lock_max_hold"
                self.get_logger().warn(
                    f"{ns}: {reason} — {detail} at "
                    f"({self.odoms[ns].pose.pose.position.x:.2f},"
                    f"{self.odoms[ns].pose.pose.position.y:.2f}); accepting "
                    f"({goal[0]:.2f},{goal[1]:.2f}) instead of stale prev goal "
                    f"({prev[0]:.2f},{prev[1]:.2f})."
                )
                self._set_policy_reason(ns, policy)
            else:
                self.get_logger().info(
                    f"{ns}: pivot-lock — clearance < {self.pivot_lock_radius_m:.2f} m "
                    f"at ({self.odoms[ns].pose.pose.position.x:.2f},"
                    f"{self.odoms[ns].pose.pose.position.y:.2f}); "
                    f"keeping prev goal ({prev[0]:.2f},{prev[1]:.2f}) instead of "
                    f"({goal[0]:.2f},{goal[1]:.2f}) "
                    f"(held {held_sec:.1f}/{self.pivot_lock_max_hold_sec:.1f}s)"
                )
                self._set_policy_reason(ns, "hold/narrow_passage_pivot_lock")
                goal = prev
        elif (
            goal_changed
            and prev_satisfied
            and pivot_blocked
        ):
            self._pivot_lock_held_since_ns.pop(ns, None)
            self._pivot_lock_start_dist_m.pop(ns, None)
            self.get_logger().info(
                f"{ns}: pivot-lock released — previous goal "
                f"({prev[0]:.2f},{prev[1]:.2f}) is already satisfied "
                f"(d<={self.goal_satisfied_dist:.2f}m); accepting "
                f"({goal[0]:.2f},{goal[1]:.2f})."
            )
            self._set_policy_reason(ns, "switch/pivot_lock_prev_satisfied")
        elif (
            goal_changed
            and prev_blacklisted
            and pivot_blocked
        ):
            self._pivot_lock_held_since_ns.pop(ns, None)
            self._pivot_lock_start_dist_m.pop(ns, None)
            self.get_logger().info(
                f"{ns}: pivot-lock released — previous goal "
                f"({prev[0]:.2f},{prev[1]:.2f}) is blacklisted; accepting "
                f"({goal[0]:.2f},{goal[1]:.2f})."
            )
            self._set_policy_reason(ns, "switch/pivot_lock_prev_blacklisted")
        elif goal_changed:
            self._pivot_lock_held_since_ns.pop(ns, None)
            self._pivot_lock_start_dist_m.pop(ns, None)
        if prev is None or math.hypot(prev[0] - goal[0], prev[1] - goal[1]) > 1e-6:
            self.last_goal_set_time_ns[ns] = now_ns
            self.goal_progress_samples[ns].clear()
            self.goal_lock_start_xy[ns] = self._robot_xy(ns) if ns in self.odoms else None
            # Re-arm rolling stall window + allow fast-blacklist to refire
            # even when the adapter's goal_seq didn't bump (CFPA2 may have
            # re-picked a neighbour <0.3 m from the previous goal).
            self.goal_lock_pose_history[ns].clear()
            self._last_unreachable_goal_seq.pop(ns, None)
            # Reset per-namespace consecutive-unreachable counts when
            # CFPA2 hands out a different goal: any pending streak no
            # longer applies to the new target.
            self._unreachable_consec.pop(ns, None)
        elif self.goal_lock_start_xy.get(ns) is None and ns in self.odoms:
            self.goal_lock_start_xy[ns] = self._robot_xy(ns)
        self.last_goal[ns] = goal
        return goal

    def _max_displacement_in_window(
        self, ns: str, window_ns: int
    ) -> Optional[float]:
        """Max |p_t - p_now| over odom samples in the last `window_ns` ns.

        Returns None if there aren't enough samples to cover at least half
        the window — otherwise a near-instantaneous check right after a
        goal change would return 0 and trigger a false stuck.
        """
        samples = self.goal_lock_pose_history.get(ns)
        if not samples:
            return None
        now_ns = self.get_clock().now().nanoseconds
        cutoff = now_ns - window_ns
        while len(samples) >= 2 and samples[0][0] < cutoff:
            samples.popleft()
        if len(samples) < 2:
            return None
        span_ns = samples[-1][0] - samples[0][0]
        if span_ns < int(0.5 * window_ns):
            return None
        cx, cy = samples[-1][1], samples[-1][2]
        max_disp = 0.0
        for _t, x, y in samples:
            d = math.hypot(cx - x, cy - y)
            if d > max_disp:
                max_disp = d
        return max_disp

    def _set_policy_reason(self, ns: str, reason: str) -> None:
        self.last_policy_reason[ns] = reason

    def _apply_switch_hysteresis(self, ns: str, goal: tuple[float, float], assignment_score: float) -> tuple[float, float]:
        last = self.last_goal.get(ns)
        if last is None:
            self._set_policy_reason(ns, "switch/no_previous_goal")
            return goal

        # Never hold a blacklisted goal — force switch to the new candidate.
        now_ns = self.get_clock().now().nanoseconds
        if self._is_blacklisted(ns, last, now_ns):
            self._set_policy_reason(ns, "switch/held_goal_blacklisted")
            return goal

        # ── Min-hold lock-in ─────────────────────────────────────
        # Once a goal is assigned, hold it for at least min_hold_sec
        # before allowing a switch to a different cluster. Without
        # this, CFPA2 reassigns the same robot 5-10× in a few seconds
        # (cluster centroids of similar info_gain frontiers oscillate
        # tick-to-tick). The robot's astar can't catch up: each new
        # goal causes a wp_sub callback path-clear, plan_astar
        # attempts the new direction, fails or partially succeeds,
        # next tick CFPA2 reassigns again. Net effect: residual
        # motion from the latest valid path drags the robot toward
        # whichever cluster won early, even when subsequent goals
        # point opposite. demo3_mixed 2026-04-25: A pulled 4 m SW
        # into zigzag_3 by a 13 s string of SW cluster goals despite
        # later NE goals.
        # Exception: never override blacklist (handled above) or
        # max_lock_sec stuck-detection (handled below).
        lock_start_ns = self.last_goal_set_time_ns.get(ns, 0)
        if lock_start_ns > 0 and self.cfpa2_goal_min_hold_sec > 0.0:
            lock_age_sec = max(0.0, (now_ns - lock_start_ns) / 1e9)
            if lock_age_sec < self.cfpa2_goal_min_hold_sec:
                self._set_policy_reason(
                    ns,
                    f"hold/min_hold_{lock_age_sec:.1f}s",
                )
                return last

        od = self.odoms[ns]
        rx = float(od.pose.pose.position.x)
        ry = float(od.pose.pose.position.y)
        dist_to_last = math.hypot(last[0] - rx, last[1] - ry)
        move = math.hypot(goal[0] - last[0], goal[1] - last[1])

        # Lock-age override (Fix #4): if the held goal has been active
        # longer than switch_hysteresis_max_lock_sec without reaching,
        # stop rejecting nearby alternatives — the held goal is almost
        # certainly unreachable and blocking near-duplicates just keeps
        # the robot pinned. Only kicks in while still en route.
        if (
            self.switch_hysteresis_max_lock_sec > 0.0
            and dist_to_last > self.switch_min_dist
        ):
            lock_start_ns = self.last_goal_set_time_ns.get(ns, 0)
            if lock_start_ns > 0:
                lock_age_sec = max(0.0, (now_ns - lock_start_ns) / 1e9)
                if lock_age_sec >= self.switch_hysteresis_max_lock_sec:
                    self._set_policy_reason(ns, "switch/hysteresis_lock_age_override")
                    return goal

        # Only apply hold logic while still traveling to the previous goal.
        if dist_to_last > self.switch_min_dist:
            if move < self.switch_min_dist:
                self._set_policy_reason(ns, "hold/hysteresis_small_move")
                return last
            if assignment_score < self.switch_hysteresis:
                self._set_policy_reason(ns, "hold/hysteresis_low_score")
                return last
        self._set_policy_reason(ns, "switch/hysteresis_ok")
        return goal

    def _tsp_top_k_head(
        self,
        ns: str,
        utilities: dict[tuple[float, float], float],
    ) -> Optional[tuple[float, float]]:
        """Return the first goal in a nearest-neighbor TSP tour through the
        top-K utility frontiers, starting from the current robot pose.

        Why: greedy max-utility flips between a near-small and far-big
        frontier each tick when their scores are close, producing sawtooth
        motion. The TSP head answers "which frontier should I visit FIRST
        if I were going to sweep all top-K?" — typically the one on the
        route to the cluster of high-utility goals. Near-but-on-route
        beats far-but-best-utility. Plumbs naturally into the existing
        commit machinery (challenger streak, stale-frontier checks).
        """
        if not utilities:
            return None
        K = self.cfpa2_tsp_k
        sorted_goals = sorted(utilities.items(), key=lambda kv: -kv[1])[:K]
        candidates = [g for g, _ in sorted_goals]
        if len(candidates) <= 1:
            return candidates[0] if candidates else None
        od = self.odoms.get(ns)
        if od is None:
            return candidates[0]
        rx = float(od.pose.pose.position.x)
        ry = float(od.pose.pose.position.y)
        # Nearest-neighbor heuristic: from robot, repeatedly pick the
        # nearest unvisited candidate. O(K²); for K=5 that's 25 ops.
        remaining = set(range(len(candidates)))
        cur_x, cur_y = rx, ry
        first_idx: Optional[int] = None
        while remaining:
            best_i = -1
            best_d2 = float("inf")
            for i in remaining:
                cx, cy = candidates[i]
                d2 = (cx - cur_x) * (cx - cur_x) + (cy - cur_y) * (cy - cur_y)
                if d2 < best_d2:
                    best_d2 = d2
                    best_i = i
            if best_i < 0:
                break
            if first_idx is None:
                first_idx = best_i
            cur_x, cur_y = candidates[best_i]
            remaining.discard(best_i)
        if first_idx is None:
            return candidates[0]
        return candidates[first_idx]

    def _apply_goal_policy(
        self,
        ns: str,
        candidate_goal: tuple[float, float],
        assignment_score: float,
        map_msg: OccupancyGrid,
        dist_map: dict[int, int],
        now_ns: int,
        current_targets: Optional[list[tuple[float, float]]] = None,
    ) -> tuple[float, float]:
        # Use committed-style policy for ALL modes (progress-based hold,
        # goal_lock, stall detection).  The weaker switch_hysteresis was
        # allowing premature goal switches mid-navigation.

        last = self.last_goal.get(ns)
        if last is None:
            self._set_policy_reason(ns, "switch/no_previous_goal")
            return candidate_goal

        if self._is_blacklisted(ns, candidate_goal, now_ns):
            self._set_policy_reason(ns, "hold/candidate_blacklisted")
            return last

        # ── Stranded-frontier check ──
        # If the held goal is no longer within `stale_frontier_radius_m` of
        # any current frontier, it means the unknown cells that made it a
        # frontier have since been resolved (usually: LiDAR scanned the
        # nearby wall and those cells became occupied). Keeping the old
        # goal in that case drives the robot into a wall. Force a switch
        # to the freshly-computed candidate.
        if current_targets:
            stale_radius_m = self.cfpa2_stale_frontier_radius_m
            stale_r2 = stale_radius_m * stale_radius_m
            still_frontier = False
            for tx, ty in current_targets:
                dx = tx - last[0]; dy = ty - last[1]
                if dx * dx + dy * dy <= stale_r2:
                    still_frontier = True
                    break
            if not still_frontier:
                self._set_policy_reason(ns, "switch/stranded_frontier")
                # Also fast-blacklist the dead goal so we don't re-pick
                # the same spot before the blacklist TTL expires.
                key = self._goal_key(last)
                bl_until = now_ns + int(max(30.0, self.blacklist_ttl_sec) * 1e9)
                self.goal_blacklist_until_ns[ns][key] = max(
                    self.goal_blacklist_until_ns[ns].get(key, 0), bl_until,
                )
                self._add_blacklist_disk(ns, last, bl_until)
                return candidate_goal

        dist_to_last = self._distance_robot_to_goal(ns, last)
        reached_last = self._goal_satisfied(ns, last, map_msg)
        safety_failure = self._held_goal_safety_failure(map_msg, dist_map, last)
        hard_failure = safety_failure is not None

        last_set_ns = self.last_goal_set_time_ns.get(ns, 0)
        lock_active = (
            self.goal_lock_sec > 0.0
            and last_set_ns > 0
            and (now_ns - last_set_ns) < int(self.goal_lock_sec * 1e9)
        )

        # ── Stable-challenger override ─────────────────────────────
        # Placed BEFORE the goal_lock and hold/progressing branches so a
        # *consistent* high-utility alternative can preempt mid-flight.
        # Two layers of jitter rejection:
        #   1. Streak: same _goal_key candidate top-1 for K ticks (rejects
        #      cluster-centroid drift — those flip top-1 every tick).
        #   2. Score: utility(candidate) > utility(last) × improvement_factor,
        #      where utility(last) is RE-EVALUATED under the current map
        #      (catches the case where the held goal's IG dropped to ~0
        #      because nearby unknowns got resolved by recent scans).
        # Plus a min lock-age so it can't fire on the first tick after
        # assignment. Set cfpa2_challenger_streak_required=0 to disable.
        candidate_id = self._goal_key(candidate_goal)
        last_id = self._goal_key(last)
        if candidate_id == last_id:
            self._challenger_id.pop(ns, None)
            self._challenger_streak.pop(ns, None)
        else:
            if self._challenger_id.get(ns) == candidate_id:
                self._challenger_streak[ns] = self._challenger_streak.get(ns, 0) + 1
            else:
                self._challenger_id[ns] = candidate_id
                self._challenger_streak[ns] = 1

            challenger_streak = self._challenger_streak.get(ns, 0)
            challenger_lock_age_sec = (
                max(0.0, (now_ns - last_set_ns) / 1e9) if last_set_ns > 0 else 1e9
            )
            if (
                self.cfpa2_challenger_streak_required > 0
                and challenger_streak >= self.cfpa2_challenger_streak_required
                and challenger_lock_age_sec >= self.cfpa2_challenger_min_lock_age_sec
            ):
                last_score = self._cfpa2_single_utility(
                    ns=ns, goal=last, map_msg=map_msg, dist_map=dist_map,
                )
                if last_score <= -1e17:
                    self._set_policy_reason(ns, "switch/held_goal_dead")
                    self._challenger_id.pop(ns, None)
                    self._challenger_streak.pop(ns, None)
                    return candidate_goal
                # Multiplicative threshold; works for typical positive scores.
                # For the rare negative case (held very bad) any candidate
                # above last_score × k still clears the bar.
                if assignment_score > last_score * self.cfpa2_challenger_improvement_factor:
                    self._set_policy_reason(
                        ns,
                        f"switch/stable_challenger_u={assignment_score:.2f}_vs_held={last_score:.2f}",
                    )
                    self._challenger_id.pop(ns, None)
                    self._challenger_streak.pop(ns, None)
                    return candidate_goal

        if lock_active and not hard_failure and not reached_last:
            self._set_policy_reason(ns, "hold/goal_lock_active")
            return last

        delta = self._progress_delta(ns)
        stalled = delta is not None and delta < self.progress_min_delta_m

        candidate_move = math.hypot(candidate_goal[0] - last[0], candidate_goal[1] - last[1])
        if candidate_move < self.switch_min_dist:
            self._set_policy_reason(ns, "hold/small_candidate_move")
            self._challenger_id.pop(ns, None)
            self._challenger_streak.pop(ns, None)
            return last

        if not reached_last and not hard_failure:
            # Keep commitment while making sufficient progress.
            if not stalled:
                self._set_policy_reason(ns, "hold/progressing")
                return last
            # Only switch on weak assignment scores when already stalled.
            if assignment_score < self.switch_hysteresis:
                self._set_policy_reason(ns, "hold/stalled_but_low_score")
                return last

        if not reached_last and (hard_failure or stalled):
            reason = safety_failure if hard_failure else "stalled"
            self._register_goal_failure(ns, last, now_ns, reason)
            self._set_policy_reason(ns, f"switch/{reason}")
            return candidate_goal

        self._set_policy_reason(ns, "switch/reached_or_improved")
        return candidate_goal

    def _maybe_log_summary(
        self,
        targets_total: int,
        per_ns_frontiers: dict[str, int],
        per_ns_reachable: dict[str, int],
        per_ns_assigned: dict[str, tuple[float, float]],
        per_ns_utilities: dict[str, dict[tuple[float, float], float]] | None = None,
    ) -> None:
        now_ns = self.get_clock().now().nanoseconds
        if self._last_summary_ns == 0:
            self._last_summary_ns = now_ns
            return
        if (now_ns - self._last_summary_ns) < int(self._summary_interval_sec * 1e9):
            return
        self._last_summary_ns = now_ns

        parts = []
        for ns in self.namespaces:
            goal = per_ns_assigned.get(ns)
            goal_txt = "None" if goal is None else f"({goal[0]:.2f},{goal[1]:.2f})"
            dist_txt = "-"
            age_txt = "-"
            if goal is not None and ns in self.odoms:
                dist_txt = f"{self._distance_robot_to_goal(ns, goal):.2f}"
            set_ns = self.last_goal_set_time_ns.get(ns, 0)
            if set_ns > 0:
                age_txt = f"{max(0.0, (now_ns - set_ns) / 1e9):.1f}"
            policy = self.last_policy_reason.get(ns, "-")
            # Show assigned goal's utility if available
            util_txt = "-"
            top_txt = ""
            if per_ns_utilities and ns in per_ns_utilities:
                ns_utils = per_ns_utilities[ns]
                if goal is not None and goal in ns_utils:
                    util_txt = f"{ns_utils[goal]:.2f}"
                # Show top-3 candidates
                sorted_goals = sorted(ns_utils.items(), key=lambda kv: kv[1], reverse=True)[:3]
                if sorted_goals:
                    top_parts = [f"({g[0]:.1f},{g[1]:.1f})={s:.2f}" for g, s in sorted_goals]
                    top_txt = f" top3=[{' '.join(top_parts)}]"
            speed_txt = ""
            vx, vy = self.odom_velocity_xy.get(ns, (0.0, 0.0))
            spd = math.hypot(float(vx), float(vy))
            if spd > 0.03:
                speed_txt = f" spd={spd:.2f}"
            parts.append(
                f"{ns}: fronts={per_ns_frontiers.get(ns, 0)} "
                f"reach={per_ns_reachable.get(ns, 0)} "
                f"goal={goal_txt} d={dist_txt} u={util_txt} "
                f"age={age_txt}s{speed_txt} [{policy}]{top_txt}"
            )
        self.get_logger().info(
            f"ASSIGN [cfpa2] targets={targets_total}\n  " + "\n  ".join(parts)
        )

    def _map_cell_stats(self, msg: OccupancyGrid) -> tuple[int, int, int]:
        free_n = 0
        occ_n = 0
        unknown_n = 0
        for value in msg.data:
            v = int(value)
            if v == self.unknown_value:
                unknown_n += 1
            elif v >= self.occ_thresh:
                occ_n += 1
            elif v == self.free_value:
                free_n += 1
            else:
                unknown_n += 1
        return (free_n, occ_n, unknown_n)

    def _should_log_no_goal_debug(self, now_ns: int) -> bool:
        if not self.debug_no_goal_logging:
            return False
        if (now_ns - self._last_no_goal_debug_ns) < int(self.debug_no_goal_log_interval_sec * 1e9):
            return False
        self._last_no_goal_debug_ns = now_ns
        return True

    def _log_no_goal_debug(
        self,
        *,
        now_ns: int,
        reason: str,
        planning_map: OccupancyGrid,
        per_ns_targets: dict[str, list[tuple[float, float]]],
        dist_maps: Optional[dict[str, dict[int, int]]] = None,
        utilities_sizes: Optional[dict[str, int]] = None,
        candidate_goals: Optional[dict[str, tuple[float, float]]] = None,
        per_ns_assigned: Optional[dict[str, tuple[float, float]]] = None,
    ) -> None:
        if not self._should_log_no_goal_debug(now_ns):
            return

        p_free, p_occ, p_unk = self._map_cell_stats(planning_map)
        parts = [
            f"NO_GOAL[{reason}]",
            f"planning_map(free={p_free} occ={p_occ} unk={p_unk})",
        ]
        for ns in self.namespaces:
            frontier_n = len(per_ns_targets.get(ns, []))
            parts.append(f"{ns}: fronts={frontier_n}")
        self.get_logger().warn(
            f"NO_GOAL [{reason}] map(free={p_free} occ={p_occ} unk={p_unk}) | " + " | ".join(parts)
        )

    @staticmethod
    def _percentile(sorted_values: list[float], quantile: float) -> float:
        if not sorted_values:
            return 0.0
        if len(sorted_values) == 1:
            return sorted_values[0]
        q = min(1.0, max(0.0, quantile))
        idx = q * float(len(sorted_values) - 1)
        lo = int(math.floor(idx))
        hi = int(math.ceil(idx))
        if lo == hi:
            return sorted_values[lo]
        frac = idx - lo
        return sorted_values[lo] * (1.0 - frac) + sorted_values[hi] * frac

    def _maybe_log_perf_summary(self) -> None:
        if not self.perf_enable:
            return

        now_ns = self.get_clock().now().nanoseconds
        if self._last_perf_summary_ns == 0:
            self._last_perf_summary_ns = now_ns
            self._perf_last_cpu_process_sec = time.process_time()
            self._perf_last_cpu_wall_ns = time.perf_counter_ns()
            return
        if (now_ns - self._last_perf_summary_ns) < int(self._summary_interval_sec * 1e9):
            return
        self._last_perf_summary_ns = now_ns

        sample_count = len(self._perf_tick_durations_ms)
        if sample_count < self.perf_min_samples:
            self.get_logger().info(
                f"PERF coordinator: collecting samples ({sample_count}/{self.perf_min_samples})"
            )
            return

        samples = list(self._perf_tick_durations_ms)
        sorted_samples = sorted(samples)
        p50_ms = self._percentile(sorted_samples, 0.50)
        p95_ms = self._percentile(sorted_samples, 0.95)
        mean_ms = sum(samples) / float(sample_count)
        max_ms = sorted_samples[-1]
        over_budget = sum(1 for val in samples if val > self._tick_period_ms)

        current_cpu_process_sec = time.process_time()
        current_cpu_wall_ns = time.perf_counter_ns()
        delta_cpu_sec = max(0.0, current_cpu_process_sec - self._perf_last_cpu_process_sec)
        delta_wall_sec = max(1e-6, (current_cpu_wall_ns - self._perf_last_cpu_wall_ns) / 1e9)
        cpu_pct = 100.0 * delta_cpu_sec / delta_wall_sec
        self._perf_last_cpu_process_sec = current_cpu_process_sec
        self._perf_last_cpu_wall_ns = current_cpu_wall_ns

        self.get_logger().info(
            "PERF coordinator: "
            f"tick_ms[p50={p50_ms:.1f} p95={p95_ms:.1f} mean={mean_ms:.1f} max={max_ms:.1f} "
            f"budget={self._tick_period_ms:.1f} over_budget={over_budget}/{sample_count}] "
            f"cpu={cpu_pct:.1f}% "
            f"adaptive[stride={self._adaptive_frontier_stride} "
            f"targets={self._adaptive_max_targets} "
            f"gain_r={self._adaptive_exploration_gain_radius_cells} "
            f"skip={self._adaptive_skip_ticks}]"
        )

        if self.perf_tick_warn_p95_ms > 0.0 and p95_ms > self.perf_tick_warn_p95_ms:
            self.get_logger().warn(
                "PERF threshold exceeded: "
                f"tick p95 {p95_ms:.1f}ms > {self.perf_tick_warn_p95_ms:.1f}ms"
            )
        if self.perf_cpu_warn_pct > 0.0 and cpu_pct > self.perf_cpu_warn_pct:
            self.get_logger().warn(
                "PERF threshold exceeded: "
                f"CPU {cpu_pct:.1f}% > {self.perf_cpu_warn_pct:.1f}% single-core budget"
            )

        self._update_adaptive_load_shedding(
            p95_ms=p95_ms,
            cpu_pct=cpu_pct,
            over_budget=over_budget,
            sample_count=sample_count,
        )

    def _update_adaptive_load_shedding(
        self,
        *,
        p95_ms: float,
        cpu_pct: float,
        over_budget: int,
        sample_count: int,
    ) -> None:
        if not self.adaptive_load_shedding_enabled:
            return

        budget_ms = max(1.0, self._tick_period_ms)
        over_budget_ratio = float(over_budget) / max(1, sample_count)
        cpu_budget_pct = self.perf_cpu_warn_pct if self.perf_cpu_warn_pct > 0.0 else 100.0
        p95_util = p95_ms / budget_ms
        cpu_util = cpu_pct / max(1.0, cpu_budget_pct)
        overloaded = (
            p95_util > self.adaptive_budget_utilization
            or over_budget_ratio > 0.35
            or cpu_util > 1.0
        )
        healthy = (
            p95_util < self.adaptive_restore_utilization
            and over_budget_ratio < 0.10
            and cpu_util < 0.8
        )

        changed = False
        if overloaded:
            if self._adaptive_frontier_stride < self.adaptive_max_frontier_stride:
                self._adaptive_frontier_stride += 1
                changed = True
            if self._adaptive_max_targets > self.adaptive_min_max_targets:
                reduced_targets = max(
                    self.adaptive_min_max_targets,
                    int(math.floor(self._adaptive_max_targets * 0.75)),
                )
                if reduced_targets != self._adaptive_max_targets:
                    self._adaptive_max_targets = reduced_targets
                    changed = True
            if self._adaptive_exploration_gain_radius_cells > self.adaptive_min_exploration_gain_radius_cells:
                self._adaptive_exploration_gain_radius_cells -= 1
                changed = True
            if self._adaptive_skip_ticks < self.adaptive_max_skip_ticks:
                self._adaptive_skip_ticks += 1
                changed = True
            if changed:
                self.get_logger().warn(
                    "Adaptive CFPA2 load shedding increased: "
                    f"stride={self._adaptive_frontier_stride} "
                    f"targets={self._adaptive_max_targets} "
                    f"gain_r={self._adaptive_exploration_gain_radius_cells} "
                    f"skip={self._adaptive_skip_ticks} "
                    f"(p95={p95_ms:.1f}ms budget={budget_ms:.1f}ms cpu={cpu_pct:.1f}%)"
                )
            return

        if healthy:
            if self._adaptive_skip_ticks > 0:
                self._adaptive_skip_ticks -= 1
                changed = True
            elif self._adaptive_exploration_gain_radius_cells < self.exploration_gain_radius_cells:
                self._adaptive_exploration_gain_radius_cells += 1
                changed = True
            elif self._adaptive_max_targets < self.max_targets:
                self._adaptive_max_targets = min(
                    self.max_targets,
                    int(math.ceil(self._adaptive_max_targets / 0.75)),
                )
                changed = True
            elif self._adaptive_frontier_stride > self.frontier_stride:
                self._adaptive_frontier_stride -= 1
                changed = True
            if changed:
                self.get_logger().info(
                    "Adaptive CFPA2 load shedding relaxed: "
                    f"stride={self._adaptive_frontier_stride} "
                    f"targets={self._adaptive_max_targets} "
                    f"gain_r={self._adaptive_exploration_gain_radius_cells} "
                    f"skip={self._adaptive_skip_ticks}"
                )

    def _record_tick_perf(self, tick_start_ns: int) -> None:
        if not self.perf_enable:
            return
        elapsed_ms = max(0.0, (time.perf_counter_ns() - tick_start_ns) / 1e6)
        self._perf_tick_durations_ms.append(elapsed_ms)
        self._maybe_log_perf_summary()

    def _tick(self) -> None:
        tick_start_ns = time.perf_counter_ns()
        skipped = False
        try:
            if self._adaptive_skip_ticks > 0:
                if self._adaptive_tick_skip_counter < self._adaptive_skip_ticks:
                    self._adaptive_tick_skip_counter += 1
                    skipped = True
                    return
                self._adaptive_tick_skip_counter = 0
            self._tick_impl()
        finally:
            if not skipped:
                self._record_tick_perf(tick_start_ns)

    def _publish_goal(self, ns: str, map_msg: OccupancyGrid, goal_w: tuple[float, float]) -> None:
        if not self._goal_is_finite(goal_w):
            self.get_logger().warn(f"{ns}: dropping non-finite goal {goal_w}")
            return
        msg = PointStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = map_msg.header.frame_id or "world"
        msg.point.x = goal_w[0]
        msg.point.y = goal_w[1]
        msg.point.z = 0.0
        self.goal_pubs[ns].publish(msg)
        self._publish_goal_marker(ns=ns, frame_id=msg.header.frame_id, goal_w=goal_w)

    def _publish_goal_marker(self, ns: str, frame_id: str, goal_w: tuple[float, float]) -> None:
        marker = Marker()
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.header.frame_id = self.marker_frame_override or frame_id or "world"
        marker.ns = "mtare_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = float(goal_w[0])
        marker.pose.position.y = float(goal_w[1])
        marker.pose.position.z = 0.0
        marker.pose.orientation.w = 1.0
        marker.scale.x = 0.35
        marker.scale.y = 0.35
        marker.scale.z = 0.35
        color = self._ns_color(ns)
        marker.color.a = 0.95
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        self.goal_marker_pubs[ns].publish(marker)

    @staticmethod
    def _goal_is_finite(goal_w: tuple[float, float]) -> bool:
        return math.isfinite(float(goal_w[0])) and math.isfinite(float(goal_w[1]))

    def _ns_color(self, ns: str) -> tuple[float, float, float]:
        idx = self.namespaces.index(ns) if ns in self.namespaces else 0
        if idx == 0:
            return (1.0, 0.15, 0.15)
        if idx == 1:
            return (0.15, 1.0, 0.15)
        if idx % 2 == 0:
            return (0.2, 0.6, 1.0)
        return (1.0, 0.8, 0.2)

    def _append_trajectory(self, ns: str, odom_msg: Odometry) -> None:
        x = float(odom_msg.pose.pose.position.x)
        y = float(odom_msg.pose.pose.position.y)
        hist = self.trajectory_history[ns]
        if not hist:
            hist.append((x, y))
            return
        px, py = hist[-1]
        if math.hypot(x - px, y - py) >= self.trajectory_min_point_distance:
            hist.append((x, y))

    def _publish_coordinator_map(self, target_map: OccupancyGrid) -> None:
        self.coordinator_map_pub.publish(target_map)

    def _publish_frontier_markers(
        self,
        target_map: OccupancyGrid,
        targets: list[tuple[float, float]],
    ) -> None:
        """Emit a red SPHERE per current frontier + a DELETEALL sentinel.

        Published on `/mtare/frontier_markers` (MarkerArray). Meant for
        RViz display — not consumed by any planning code. The DELETEALL
        first entry clears stale markers from previous ticks.
        """
        stamp = self.get_clock().now().to_msg()
        frame_id = self.marker_frame_override or target_map.header.frame_id or "world"

        markers = MarkerArray()
        clear = Marker()
        clear.header.stamp = stamp
        clear.header.frame_id = frame_id
        clear.ns = "cfpa2_frontiers"
        clear.action = Marker.DELETEALL
        markers.markers.append(clear)

        for i, (wx, wy) in enumerate(targets):
            if not (math.isfinite(wx) and math.isfinite(wy)):
                continue
            m = Marker()
            m.header.stamp = stamp
            m.header.frame_id = frame_id
            m.ns = "cfpa2_frontiers"
            m.id = i + 1
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = float(wx)
            m.pose.position.y = float(wy)
            m.pose.position.z = 0.05
            m.pose.orientation.w = 1.0
            m.scale.x = 0.18
            m.scale.y = 0.18
            m.scale.z = 0.18
            m.color.a = 0.85
            m.color.r = 1.0
            m.color.g = 0.1
            m.color.b = 0.1
            markers.markers.append(m)

        self.frontier_markers_pub.publish(markers)
        # Throttled confirmation so we can audit publication from the log
        # without needing a live DDS subscriber. Emits roughly every
        # `self._summary_interval_sec` (the same cadence as the ASSIGN log).
        now_ns = self.get_clock().now().nanoseconds
        if (now_ns - getattr(self, "_last_frontier_log_ns", 0)) \
                >= int(self._summary_interval_sec * 1e9):
            self._last_frontier_log_ns = now_ns
            self.get_logger().info(
                f"FRONTIER_MARKERS published {len(markers.markers) - 1} spheres "
                f"on {self.frontier_markers_topic}"
            )

    def _publish_robot_markers(self, target_map: OccupancyGrid) -> None:
        markers = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        frame_id = self.marker_frame_override or target_map.header.frame_id or "world"

        for idx, ns in enumerate(self.namespaces):
            color = self._ns_color(ns)
            pose_id = 100 + idx
            traj_id = 200 + idx
            goal_id = 300 + idx
            label_id = 400 + idx

            od = self.odoms.get(ns)
            if od is not None:
                pose_marker = Marker()
                pose_marker.header.stamp = stamp
                pose_marker.header.frame_id = frame_id
                pose_marker.ns = "mtare_robot_pose"
                pose_marker.id = pose_id
                pose_marker.type = Marker.TRIANGLE_LIST
                pose_marker.action = Marker.ADD
                pose_marker.pose.position.x = float(od.pose.pose.position.x)
                pose_marker.pose.position.y = float(od.pose.pose.position.y)
                pose_marker.pose.position.z = float(od.pose.pose.position.z) + 0.02
                pose_marker.pose.orientation = od.pose.pose.orientation
                pose_marker.scale.x = 1.0
                pose_marker.scale.y = 1.0
                pose_marker.scale.z = 1.0
                pose_marker.color.a = 0.95
                pose_marker.color.r = color[0]
                pose_marker.color.g = color[1]
                pose_marker.color.b = color[2]
                front = Point()
                front.x = 0.70 * self.robot_marker_scale
                front.y = 0.0
                front.z = 0.03
                rear_left = Point()
                rear_left.x = -0.45 * self.robot_marker_scale
                rear_left.y = 0.45 * self.robot_marker_scale
                rear_left.z = 0.03
                rear_right = Point()
                rear_right.x = -0.45 * self.robot_marker_scale
                rear_right.y = -0.45 * self.robot_marker_scale
                rear_right.z = 0.03
                pose_marker.points = [front, rear_left, rear_right]
                markers.markers.append(pose_marker)

                label_marker = Marker()
                label_marker.header.stamp = stamp
                label_marker.header.frame_id = frame_id
                label_marker.ns = "mtare_robot_label"
                label_marker.id = label_id
                label_marker.type = Marker.TEXT_VIEW_FACING
                label_marker.action = Marker.ADD
                label_marker.pose.position.x = float(od.pose.pose.position.x)
                label_marker.pose.position.y = float(od.pose.pose.position.y)
                label_marker.pose.position.z = float(od.pose.pose.position.z) + 0.5
                label_marker.pose.orientation.w = 1.0
                label_marker.scale.z = 0.32
                label_marker.color.a = 1.0
                label_marker.color.r = color[0]
                label_marker.color.g = color[1]
                label_marker.color.b = color[2]
                label_marker.text = ns
                markers.markers.append(label_marker)

            traj_marker = Marker()
            traj_marker.header.stamp = stamp
            traj_marker.header.frame_id = frame_id
            traj_marker.ns = "mtare_robot_traj"
            traj_marker.id = traj_id
            traj_marker.type = Marker.LINE_STRIP
            traj_marker.action = Marker.ADD
            traj_marker.pose.orientation.w = 1.0
            traj_marker.scale.x = 0.08
            traj_marker.color.a = 0.95
            traj_marker.color.r = color[0]
            traj_marker.color.g = color[1]
            traj_marker.color.b = color[2]
            for x, y in self.trajectory_history[ns]:
                pt = Point()
                pt.x = float(x)
                pt.y = float(y)
                pt.z = 0.05
                traj_marker.points.append(pt)
            markers.markers.append(traj_marker)

            goal = self.last_goal.get(ns)
            if goal is not None:
                goal_marker = Marker()
                goal_marker.header.stamp = stamp
                goal_marker.header.frame_id = frame_id
                goal_marker.ns = "mtare_goal_points"
                goal_marker.id = goal_id
                goal_marker.type = Marker.SPHERE
                goal_marker.action = Marker.ADD
                goal_marker.pose.position.x = float(goal[0])
                goal_marker.pose.position.y = float(goal[1])
                goal_marker.pose.position.z = 0.08
                goal_marker.pose.orientation.w = 1.0
                goal_marker.scale.x = 0.24
                goal_marker.scale.y = 0.24
                goal_marker.scale.z = 0.24
                goal_marker.color.a = 0.95
                goal_marker.color.r = color[0]
                goal_marker.color.g = color[1]
                goal_marker.color.b = color[2]
                markers.markers.append(goal_marker)

        self.robot_markers_pub.publish(markers)

    def _robot_xy(self, ns: str) -> tuple[float, float]:
        od = self.odoms[ns]
        return (float(od.pose.pose.position.x), float(od.pose.pose.position.y))

    def _frontier_information_gain(
        self,
        msg: OccupancyGrid,
        goal: tuple[float, float],
        ns: Optional[str] = None,
    ) -> float:
        # Floodfill mode: connected-component size of unknown region reachable
        # from the goal cell. Rewards frontiers that open into large rooms vs
        # small nooks, because nooks saturate well below the budget cap while
        # rooms saturate at the cap.
        if self.cfpa2_ig_mode == "floodfill":
            return self._frontier_information_gain_floodfill(msg, goal)

        # Local-window mode (default, legacy behaviour).
        g = self._world_to_grid(msg, goal[0], goal[1])
        if g is None:
            return 0.0
        gx, gy = g
        w = int(msg.info.width)
        h = int(msg.info.height)
        data = msg.data
        r = self._adaptive_exploration_gain_radius_cells
        gain = 0.0
        for yy in range(max(0, gy - r), min(h, gy + r + 1)):
            row = yy * w
            for xx in range(max(0, gx - r), min(w, gx + r + 1)):
                idx = row + xx
                if data[idx] == self.unknown_value:
                    gain += 1.0
        return gain

    def _frontier_information_gain_floodfill(
        self,
        msg: OccupancyGrid,
        goal: tuple[float, float],
    ) -> float:
        """Count unknown cells connected to the goal via 4-conn BFS.

        Bounded by two limits so worst-case work is O(budget):
        - Stop after `cfpa2_floodfill_budget` cells visited.
        - Stop expanding cells whose Chebyshev distance from the start
          exceeds `cfpa2_floodfill_max_radius_cells`.

        BFS expands into UNKNOWN cells only. Frontiers next to a thin wall
        but open into a 10×10m room will saturate the budget (= big score);
        a frontier opening into a 1×1m closet caps at ~100 cells (= small
        score). Distance cost in the utility formula then prefers the
        nearer big room over a far big room, NOT a near small nook over a
        far big room.
        """
        g = self._world_to_grid(msg, goal[0], goal[1])
        if g is None:
            return 0.0
        sx, sy = g
        w = int(msg.info.width)
        h = int(msg.info.height)
        data = msg.data
        unk = self.unknown_value

        # Seed: the goal cell itself, even if it's not unknown — we want to
        # expand into the adjacent unknown territory. If the goal is on a
        # free cell next to UNK, the first iteration of the BFS will pick
        # up the UNK neighbours.
        budget = self.cfpa2_floodfill_budget
        max_r = self.cfpa2_floodfill_max_radius_cells

        # Visited as a set of linear indices for cheap membership testing.
        seed_idx = sy * w + sx
        visited: set[int] = {seed_idx}
        q: deque[tuple[int, int]] = deque()
        q.append((sx, sy))
        gain = 0
        # The seed cell counts only if it's itself unknown.
        if 0 <= seed_idx < len(data) and data[seed_idx] == unk:
            gain = 1

        while q and gain < budget:
            cx, cy = q.popleft()
            for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                nx, ny = cx + dx, cy + dy
                if nx < 0 or ny < 0 or nx >= w or ny >= h:
                    continue
                # Chebyshev radius cap — keeps work bounded even on a tiny
                # connected unknown blob that spans the whole map (e.g. a
                # corridor with leaks).
                if abs(nx - sx) > max_r or abs(ny - sy) > max_r:
                    continue
                nidx = ny * w + nx
                if nidx in visited:
                    continue
                v = data[nidx]
                if v != unk:
                    # Only expand THROUGH unknown territory. Free cells stop
                    # the wave (otherwise we'd flood the whole free map).
                    visited.add(nidx)  # mark so we don't revisit
                    continue
                visited.add(nidx)
                gain += 1
                if gain >= budget:
                    break
                q.append((nx, ny))
        return float(gain)

    def _batch_frontier_information_gain(
        self,
        msg: OccupancyGrid,
        goals: list[tuple[float, float]],
        ns: Optional[str] = None,
    ) -> list[float]:
        """Batch info-gain for all goals at once (C++ accelerated)."""
        if not goals:
            return []
        # Floodfill mode: prefer the C accelerator when present. On Orin
        # Nano the Python path was the dominant chunk of the tick budget
        # (30 goals × ~40 ms Python BFS = 1.2 s); the C++ batch path
        # collapses that to ~30 ms total. Falls back to Python if the
        # legacy .so doesn't expose the symbol yet.
        if self.cfpa2_ig_mode == "floodfill":
            if (
                _GRID_OPS_LIB is not None
                and hasattr(_GRID_OPS_LIB, "batch_info_gain_floodfill")
            ):
                w = int(msg.info.width)
                h = int(msg.info.height)
                n = len(goals)
                grid_np = np.array(msg.data, dtype=np.int8)
                ox = float(msg.info.origin.position.x)
                oy = float(msg.info.origin.position.y)
                res = float(msg.info.resolution)
                gx_arr = (ctypes.c_float * n)(*(g[0] for g in goals))
                gy_arr = (ctypes.c_float * n)(*(g[1] for g in goals))
                gains_out = (ctypes.c_float * n)()
                # The scratch buffer is reused across calls via a generation
                # counter inside the C++ side, but the buffer itself must
                # persist between calls so we keep it on the instance.
                want = w * h
                cached = getattr(self, "_floodfill_scratch", None)
                if cached is None or cached.size < want:
                    self._floodfill_scratch = np.zeros(want, dtype=np.int32)
                    cached = self._floodfill_scratch
                _GRID_OPS_LIB.batch_info_gain_floodfill(
                    grid_np.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)),
                    w, h, res, ox, oy,
                    gx_arr, gy_arr, n,
                    int(self.cfpa2_floodfill_budget),
                    int(self.cfpa2_floodfill_max_radius_cells),
                    ctypes.c_int8(self.unknown_value),
                    cached.ctypes.data_as(ctypes.POINTER(ctypes.c_int32)),
                    gains_out,
                )
                return [float(gains_out[i]) for i in range(n)]
            return [self._frontier_information_gain_floodfill(msg, g) for g in goals]

        w = int(msg.info.width)
        h = int(msg.info.height)
        r = self._adaptive_exploration_gain_radius_cells
        n = len(goals)

        if _GRID_OPS_LIB is not None:
            grid_np = np.array(msg.data, dtype=np.int8)
            ox = float(msg.info.origin.position.x)
            oy = float(msg.info.origin.position.y)
            res = float(msg.info.resolution)
            gx_arr = (ctypes.c_float * n)(*(g[0] for g in goals))
            gy_arr = (ctypes.c_float * n)(*(g[1] for g in goals))
            gains_out = (ctypes.c_float * n)()
            _GRID_OPS_LIB.batch_info_gain(
                grid_np.ctypes.data_as(ctypes.POINTER(ctypes.c_int8)),
                w, h, res, ox, oy,
                gx_arr, gy_arr, n, r,
                ctypes.c_int8(self.unknown_value),
                gains_out,
            )
            return [float(gains_out[i]) for i in range(n)]

        # Python fallback (2D)
        return [self._frontier_information_gain(msg, g, ns=ns) for g in goals]

    def _grid_path_cost_m(
        self,
        msg: OccupancyGrid,
        dist_map: dict[int, int],
        goal: tuple[float, float],
    ) -> Optional[float]:
        g = self._world_to_grid(msg, goal[0], goal[1])
        if g is None:
            return None
        idx = self._grid_index(g[0], g[1], int(msg.info.width))
        if idx not in dist_map:
            return None
        return float(dist_map[idx]) * msg.info.resolution

    def _cfpa2_switch_penalty(self, ns: str, goal: tuple[float, float]) -> float:
        last = self.last_goal.get(ns)
        if last is None:
            return 0.0
        return 0.0 if self._goal_key(last) == self._goal_key(goal) else 1.0

    def _cfpa2_single_utility(
        self,
        *,
        ns: str,
        goal: tuple[float, float],
        map_msg: OccupancyGrid,
        dist_map: dict[int, int],
    ) -> float:
        if getattr(self, "goal_satisfied_dist", 0.0) > 0.0 and self._goal_satisfied(
            ns,
            goal,
            map_msg,
        ):
            return -1e18
        if not self._goal_has_obstacle_clearance(map_msg, goal):
            return -1e18
        dist_m = self._grid_path_cost_m(map_msg, dist_map, goal)
        if dist_m is None or dist_m <= 0.0:
            return -1e18
        info_gain = self._frontier_information_gain(map_msg, goal, ns=ns)
        # Reject frontiers with negligible info-gain (tiny slivers near walls).
        if info_gain < 3:
            return -1e18
        switch_penalty = self._cfpa2_switch_penalty(ns, goal)
        momentum_bonus = self._cfpa2_momentum_bonus(ns, goal)
        return (
            (self.cfpa2_w_ig * info_gain)
            - (self.cfpa2_w_c * dist_m)
            - (self.cfpa2_w_sw * switch_penalty)
            + (self.cfpa2_w_momentum * momentum_bonus)
        )

    def _cfpa2_overlap_penalty(self, goal_i: tuple[float, float], goal_j: tuple[float, float]) -> float:
        sigma = self.cfpa2_sigma_overlap_m if self.cfpa2_sigma_overlap_m > 0.0 else (2.0 * self.sensor_range)
        sigma = max(1e-3, sigma)
        if _CFPA2_OVERLAP_PENALTY_FN is not None:
            try:
                return float(_CFPA2_OVERLAP_PENALTY_FN(goal_i, goal_j, sigma))
            except Exception:
                pass

        dx = goal_i[0] - goal_j[0]
        dy = goal_i[1] - goal_j[1]
        d2 = (dx * dx) + (dy * dy)
        return math.exp(-d2 / (2.0 * sigma * sigma))

    def _cfpa2_momentum(self, ns: str) -> float:
        vx, vy = self.odom_velocity_xy.get(ns, (0.0, 0.0))
        return math.hypot(float(vx), float(vy))

    def _cfpa2_momentum_bonus(self, ns: str, goal: tuple[float, float]) -> float:
        """Heading + velocity momentum bonus.

        bonus = cos(heading_to_frontier) × (α + β × speed)

        α (base): heading-only component — even when stopped, frontiers
          behind the robot get penalized. Critical at brake_hold (v=0)
          and at goal-reached waypoint stops where the velocity term
          contributes nothing. 2026-04-25 demo3_mixed: at α=0.5 the
          bonus range was only ±0.5 at standstill, easily flipped by
          info_gain wobble (0.4 Hz goal flap). Raised to 1.5.
        β (velocity scale): scales up the bonus when moving. Makes
          mid-stride direction switches very expensive. 1.0 → 2.0.
        Both are parameters now (cfpa2_momentum_alpha / _beta) so they
        can be tuned without a rebuild.
        """
        odom = self.odoms.get(ns)
        if odom is None:
            return 0.0
        rx = float(odom.pose.pose.position.x)
        ry = float(odom.pose.pose.position.y)
        dx = goal[0] - rx
        dy = goal[1] - ry
        d = math.hypot(dx, dy)
        if d < 0.1:
            return 0.0

        # Extract yaw from quaternion
        q = odom.pose.pose.orientation
        siny = 2.0 * (float(q.w) * float(q.z) + float(q.x) * float(q.y))
        cosy = 1.0 - 2.0 * (float(q.y) * float(q.y) + float(q.z) * float(q.z))
        yaw = math.atan2(siny, cosy)

        # cos(angle between heading and frontier direction)
        cos_angle = (math.cos(yaw) * dx + math.sin(yaw) * dy) / d

        # Velocity boost
        vx, vy = self.odom_velocity_xy.get(ns, (0.0, 0.0))
        speed = math.hypot(float(vx), float(vy))

        return cos_angle * (self.cfpa2_momentum_alpha + self.cfpa2_momentum_beta * speed)

    def _find_nearest_free_cell(
        self,
        map_msg: OccupancyGrid,
        cell: tuple[int, int],
        search_radius_cells: int,
    ) -> Optional[tuple[int, int]]:
        w = int(map_msg.info.width)
        h = int(map_msg.info.height)
        data = map_msg.data
        gx, gy = cell
        if gx < 0 or gy < 0 or gx >= w or gy >= h:
            return None
        if self._is_free(data, self._grid_index(gx, gy, w)):
            return (gx, gy)

        best: Optional[tuple[int, int]] = None
        best_d2 = float("inf")
        for r in range(1, search_radius_cells + 1):
            for dy in range(-r, r + 1):
                ny = gy + dy
                if ny < 0 or ny >= h:
                    continue
                for dx in range(-r, r + 1):
                    nx = gx + dx
                    if nx < 0 or nx >= w:
                        continue
                    nidx = self._grid_index(nx, ny, w)
                    if not self._is_free(data, nidx):
                        continue
                    d2 = (nx - gx) * (nx - gx) + (ny - gy) * (ny - gy)
                    if d2 < best_d2:
                        best_d2 = d2
                        best = (nx, ny)
            if best is not None:
                break
        return best

    @staticmethod
    def _dilate_grid_cell(
        gx: int,
        gy: int,
        radius_cells: int,
        w: int,
        h: int,
    ) -> list[tuple[int, int]]:
        if radius_cells <= 0:
            return [(gx, gy)]
        out: list[tuple[int, int]] = []
        r2 = radius_cells * radius_cells
        for dy in range(-radius_cells, radius_cells + 1):
            ny = gy + dy
            if ny < 0 or ny >= h:
                continue
            for dx in range(-radius_cells, radius_cells + 1):
                nx = gx + dx
                if nx < 0 or nx >= w:
                    continue
                if (dx * dx + dy * dy) > r2:
                    continue
                out.append((nx, ny))
        return out

    def _predict_other_robot_blocks(
        self,
        *,
        map_msg: OccupancyGrid,
        ns: str,
        planned_goals: dict[str, tuple[float, float]],
        steps: int,
        dt_sec: float,
        safety_radius_cells: int,
    ) -> list[set[tuple[int, int]]]:
        blocked_by_t: list[set[tuple[int, int]]] = [set() for _ in range(steps + 1)]
        w = int(map_msg.info.width)
        h = int(map_msg.info.height)

        for other in self.namespaces:
            if other == ns or other not in self.odoms:
                continue

            ox, oy = self._robot_xy(other)
            target = planned_goals.get(other) or self.last_goal.get(other) or (ox, oy)
            tx, ty = float(target[0]), float(target[1])

            dx = tx - ox
            dy = ty - oy
            dist = math.hypot(dx, dy)
            dir_x = (dx / dist) if dist > 1e-6 else 0.0
            dir_y = (dy / dist) if dist > 1e-6 else 0.0

            raw_speed = math.hypot(*self.odom_velocity_xy.get(other, (0.0, 0.0)))
            speed = raw_speed if raw_speed > 0.05 else self.cfpa2_space_time_assumed_speed_mps
            speed = max(self.cfpa2_space_time_assumed_speed_mps, min(speed, self.cfpa2_space_time_max_speed_mps))

            for k in range(steps + 1):
                t = dt_sec * k
                travel = min(dist, speed * t)
                px = ox + dir_x * travel
                py = oy + dir_y * travel
                g = self._world_to_grid(map_msg, px, py)
                if g is None:
                    continue
                for cell in self._dilate_grid_cell(g[0], g[1], safety_radius_cells, w, h):
                    blocked_by_t[k].add(cell)

        return blocked_by_t

    def _space_time_astar_cells(
        self,
        *,
        ns: str,
        map_msg: OccupancyGrid,
        final_goal: tuple[float, float],
        planned_goals: dict[str, tuple[float, float]],
    ) -> Optional[list[tuple[int, int]]]:
        if ns not in self.odoms:
            return None

        res = max(1e-6, float(map_msg.info.resolution))
        w = int(map_msg.info.width)
        h = int(map_msg.info.height)
        data = map_msg.data

        start_w = self._robot_xy(ns)
        start = self._world_to_grid(map_msg, start_w[0], start_w[1])
        goal = self._world_to_grid(map_msg, final_goal[0], final_goal[1])
        if start is None or goal is None:
            return None

        near_search = max(4, int(math.ceil(0.8 / res)))
        start = self._find_nearest_free_cell(map_msg, start, near_search)
        goal = self._find_nearest_free_cell(map_msg, goal, near_search)
        if start is None or goal is None:
            return None

        steps = max(2, int(math.ceil(self.cfpa2_space_time_horizon_sec / self.cfpa2_space_time_dt_sec)))
        safety_cells = max(0, int(math.ceil(self.cfpa2_space_time_safety_radius_m / res)))
        blocked_by_t = self._predict_other_robot_blocks(
            map_msg=map_msg,
            ns=ns,
            planned_goals=planned_goals,
            steps=steps,
            dt_sec=self.cfpa2_space_time_dt_sec,
            safety_radius_cells=safety_cells,
        )

        margin_cells = max(
            4,
            int(
                math.ceil(
                    (self.cfpa2_space_time_window_margin_m + self.cfpa2_space_time_max_speed_mps * self.cfpa2_space_time_horizon_sec)
                    / res
                )
            ),
        )
        x_min = max(0, min(start[0], goal[0]) - margin_cells)
        x_max = min(w - 1, max(start[0], goal[0]) + margin_cells)
        y_min = max(0, min(start[1], goal[1]) - margin_cells)
        y_max = min(h - 1, max(start[1], goal[1]) + margin_cells)

        def in_window(x: int, y: int) -> bool:
            return x_min <= x <= x_max and y_min <= y <= y_max

        sx, sy = start
        gx, gy = goal
        if not in_window(sx, sy) or not in_window(gx, gy):
            return None

        neighbors = (
            (1, 0, 1.0),
            (-1, 0, 1.0),
            (0, 1, 1.0),
            (0, -1, 1.0),
            (0, 0, 1.05),  # wait
        )

        start_key = (sx, sy, 0)
        open_heap: list[tuple[float, float, int, int, int]] = []
        heapq.heappush(open_heap, (abs(sx - gx) + abs(sy - gy), 0.0, sx, sy, 0))
        g_score: dict[tuple[int, int, int], float] = {start_key: 0.0}
        parent: dict[tuple[int, int, int], tuple[int, int, int]] = {}

        expansions = 0
        goal_key: Optional[tuple[int, int, int]] = None
        while open_heap and expansions < self.cfpa2_space_time_max_expansions:
            _f, cur_g, x, y, t = heapq.heappop(open_heap)
            key = (x, y, t)
            if cur_g > g_score.get(key, float("inf")) + 1e-9:
                continue
            expansions += 1

            if (x, y) == (gx, gy):
                goal_key = key
                break
            if t >= steps:
                continue

            nt = t + 1
            for dx, dy, move_cost in neighbors:
                nx = x + dx
                ny = y + dy
                if not in_window(nx, ny):
                    continue
                nidx = self._grid_index(nx, ny, w)
                if not self._is_free(data, nidx):
                    continue
                if (nx, ny) in blocked_by_t[nt]:
                    continue
                # Avoid swap-like crossing through a dynamic obstacle trajectory.
                if (nx, ny) in blocked_by_t[t] and (x, y) in blocked_by_t[nt]:
                    continue

                nkey = (nx, ny, nt)
                ng = cur_g + move_cost
                if ng >= g_score.get(nkey, float("inf")) - 1e-9:
                    continue
                g_score[nkey] = ng
                parent[nkey] = key
                h_manhattan = abs(nx - gx) + abs(ny - gy)
                heapq.heappush(open_heap, (ng + h_manhattan, ng, nx, ny, nt))

        if goal_key is None:
            return None

        path: list[tuple[int, int]] = []
        cur = goal_key
        while True:
            path.append((cur[0], cur[1]))
            if cur == start_key:
                break
            cur = parent[cur]
        path.reverse()
        return path

    def _cfpa2_space_time_waypoint(
        self,
        *,
        ns: str,
        map_msg: OccupancyGrid,
        final_goal: tuple[float, float],
        planned_goals: dict[str, tuple[float, float]],
    ) -> Optional[tuple[float, float]]:
        if not self.cfpa2_space_time_enabled or ns not in self.odoms:
            return None

        rx, ry = self._robot_xy(ns)
        if math.hypot(final_goal[0] - rx, final_goal[1] - ry) <= max(self.min_assign_distance, 0.20):
            return None

        path_cells = self._space_time_astar_cells(
            ns=ns,
            map_msg=map_msg,
            final_goal=final_goal,
            planned_goals=planned_goals,
        )
        if not path_cells or len(path_cells) < 2:
            return None

        res = float(map_msg.info.resolution)
        lookahead_m = max(res, self.cfpa2_space_time_waypoint_lookahead_m)
        traveled_m = 0.0
        chosen = path_cells[-1]
        prev = path_cells[0]
        for cell in path_cells[1:]:
            traveled_m += math.hypot(cell[0] - prev[0], cell[1] - prev[1]) * res
            prev = cell
            if traveled_m >= lookahead_m:
                chosen = cell
                break

        return self._grid_to_world(map_msg, chosen[0], chosen[1])

    def _cfpa2_best_available_goal(
        self,
        *,
        ns: str,
        now_ns: int,
        utilities: dict[tuple[float, float], float],
        exclude_goal: Optional[tuple[float, float]] = None,
        fallback_targets: Optional[list[tuple[float, float]]] = None,
        map_msg: Optional[OccupancyGrid] = None,
        dist_map: Optional[dict[int, int]] = None,
    ) -> Optional[tuple[float, float]]:
        # TODO(maybe): Move fallback target prioritization into cfpa2_demo.core for reuse.
        excluded_key = self._goal_key(exclude_goal) if exclude_goal is not None else None

        def is_available(goal: tuple[float, float]) -> bool:
            if excluded_key is not None and self._goal_key(goal) == excluded_key:
                return False
            if self._goal_too_close(ns, goal):
                return False
            if self._goal_too_far(ns, goal):
                return False
            if self._is_blacklisted(ns, goal, now_ns):
                return False
            if map_msg is not None and not self._goal_has_obstacle_clearance(map_msg, goal):
                return False
            if map_msg is not None and dist_map is not None and not self._goal_reachable(
                map_msg, dist_map, goal
            ):
                return False
            return True

        for goal, _score in sorted(utilities.items(), key=lambda kv: kv[1], reverse=True):
            if is_available(goal):
                return goal

        if fallback_targets:
            for goal in sorted(fallback_targets, key=lambda g: self._distance_robot_to_goal(ns, g)):
                if is_available(goal):
                    return goal
        return None

    def _maybe_force_cfpa2_stuck_recovery(
        self,
        *,
        ns: str,
        now_ns: int,
        utilities: dict[tuple[float, float], float],
        fallback_targets: list[tuple[float, float]],
        map_msg: Optional[OccupancyGrid] = None,
    ) -> Optional[tuple[float, float]]:
        if self.cfpa2_stuck_lock_sec <= 0.0 or self.cfpa2_stuck_blacklist_sec <= 0.0:
            return None
        if ns not in self.odoms:
            return None

        current_goal = self.last_goal.get(ns)
        if current_goal is None:
            return None
        if self._goal_satisfied(ns, current_goal, map_msg):
            return None

        lock_start_ns = self.last_goal_set_time_ns.get(ns, 0)
        if lock_start_ns <= 0:
            return None
        lock_age_sec = max(0.0, (now_ns - lock_start_ns) / 1e9)
        if lock_age_sec < self.cfpa2_stuck_lock_sec:
            return None

        # Rolling-window stall check (Fix #2): how much has the robot
        # moved in the last cfpa2_stuck_window_sec seconds? The legacy
        # single-anchor metric latched at goal-set and was never re-armed,
        # so any pre-stall drift >= min_motion disabled recovery forever.
        window_ns = int(self.cfpa2_stuck_window_sec * 1e9)
        moved_dist = self._max_displacement_in_window(ns, window_ns)
        if moved_dist is None:
            # Not enough samples yet — don't false-trigger on a fresh goal.
            return None
        if moved_dist >= self.cfpa2_stuck_min_motion_m:
            return None

        current_key = self._goal_key(current_goal)
        until_ns = now_ns + int(self.cfpa2_stuck_blacklist_sec * 1e9)
        self.goal_blacklist_until_ns[ns][current_key] = max(
            self.goal_blacklist_until_ns[ns].get(current_key, 0),
            until_ns,
        )
        # Cluster disk (Fix #1): forbid the whole orphan neighbourhood so
        # the next-best goal can't come from the same unreachable cluster.
        self._add_blacklist_disk(ns, current_goal, until_ns)

        alternative = self._cfpa2_best_available_goal(
            ns=ns,
            now_ns=now_ns,
            utilities=utilities,
            exclude_goal=current_goal,
            fallback_targets=fallback_targets,
            map_msg=map_msg,
        )

        last_event_ns = self.cfpa2_last_stuck_event_ns.get(ns, 0)
        if alternative is not None:
            if (now_ns - last_event_ns) > int(1e9):
                self.get_logger().warn(
                    f"{ns}: CFPA2 stuck-recovery triggered (lock={lock_age_sec:.1f}s, moved={moved_dist:.2f}m). "
                    f"Blacklisted ({current_goal[0]:.2f},{current_goal[1]:.2f}) for {self.cfpa2_stuck_blacklist_sec:.1f}s "
                    f"and switching to ({alternative[0]:.2f},{alternative[1]:.2f})."
                )
            self.cfpa2_last_stuck_event_ns[ns] = now_ns
            self._set_policy_reason(ns, "switch/cfpa2_stuck_recover")
            return alternative

        if (now_ns - last_event_ns) > int(2e9):
            self.get_logger().warn(
                f"{ns}: CFPA2 stuck-recovery blacklisted current goal but found no alternative frontier."
            )
            self.cfpa2_last_stuck_event_ns[ns] = now_ns
        self._set_policy_reason(ns, "hold/cfpa2_stuck_no_alternative")
        return None

    def _apply_cfpa2_proximity_stop(
        self,
        *,
        candidate_goals: dict[str, tuple[float, float]],
        assignment_scores: dict[str, float],
        now_ns: int,
    ) -> set[str]:
        if self.cfpa2_close_stop_radius_m <= 0.0 or len(self.namespaces) != 2:
            return set()

        ns_a, ns_b = self.namespaces[0], self.namespaces[1]
        if ns_a not in self.odoms or ns_b not in self.odoms:
            return set()

        ax, ay = self._robot_xy(ns_a)
        bx, by = self._robot_xy(ns_b)
        if math.hypot(ax - bx, ay - by) >= self.cfpa2_close_stop_radius_m:
            return set()

        p_a = self._cfpa2_momentum(ns_a)
        p_b = self._cfpa2_momentum(ns_b)
        eps = self.cfpa2_close_stop_speed_epsilon
        if p_a < (p_b - eps):
            stop_ns = ns_a
        elif p_b < (p_a - eps):
            stop_ns = ns_b
        else:
            # Deterministic tie-break keeps one robot moving.
            stop_ns = ns_b

        sx, sy = self._robot_xy(stop_ns)
        candidate_goals[stop_ns] = (sx, sy)
        assignment_scores[stop_ns] = -1e6
        self._set_policy_reason(stop_ns, "hold/cfpa2_close_low_momentum_stop")

        if (now_ns - self._cfpa2_last_close_stop_log_ns) > int(1e9):
            other_ns = ns_b if stop_ns == ns_a else ns_a
            self.get_logger().warn(
                f"CFPA2 close-range arbitration: d<{self.cfpa2_close_stop_radius_m:.2f}m, "
                f"{stop_ns} momentum={self._cfpa2_momentum(stop_ns):.3f} < "
                f"{other_ns} momentum={self._cfpa2_momentum(other_ns):.3f}; forcing {stop_ns} stop."
            )
            self._cfpa2_last_close_stop_log_ns = now_ns
        return {stop_ns}

    def _tick_impl(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        # Each per-namespace decision below reads `self.maps[ns]` directly.
        # We only need one OccupancyGrid for the viz publishers
        # (coordinator_map / robot_markers / frontier_markers) — pick the
        # first available ns's map. Multi-robot fusion (a merged grid
        # across namespaces) was removed 2026-05-19 along with the
        # shared_map feature; the joint allocator never actually needed
        # a fused planning map, just each robot's own.
        if any(ns not in self.maps for ns in self.namespaces):
            if now_ns - self._last_prereq_warn_ns > int(2e9):
                missing_maps = [ns for ns in self.namespaces if ns not in self.maps]
                self.get_logger().warn(
                    f"Waiting for map topics from: {missing_maps}; no CFPA2 goals will be published yet."
                )
                self._last_prereq_warn_ns = now_ns
            return
        planning_map = self.maps[self.namespaces[0]]

        # Stash for _set_active_goal's pivot-clearance check (so the
        # goal-change lock has access to the current map without
        # re-plumbing it through every assignment site).
        self._cur_planning_map = planning_map

        self._publish_coordinator_map(planning_map)
        self._publish_robot_markers(planning_map)

        missing_odoms = [ns for ns in self.namespaces if ns not in self.odoms]
        if missing_odoms:
            if now_ns - self._last_prereq_warn_ns > int(2e9):
                self.get_logger().warn(
                    f"Waiting for odom/nav from: {missing_odoms}; no M-TARE goals will be published yet."
                )
                self._last_prereq_warn_ns = now_ns
            return

        for ns in self.namespaces:
            self._prune_blacklist(ns, now_ns)
            reached_map = self.maps.get(ns)
            self._update_reached_goal_blacklist(ns, now_ns, reached_map)

        per_ns_targets: dict[str, list[tuple[float, float]]] = {}
        for ns in self.namespaces:
            map_msg = self.maps.get(ns)
            per_ns_targets[ns] = self._extract_frontiers(map_msg) if map_msg is not None else []

        merge_res = max(0.1, float(planning_map.info.resolution) * 2.0)
        targets = self._merge_targets([per_ns_targets[ns] for ns in self.namespaces], merge_res)

        self._publish_frontier_markers(planning_map, targets)

        if not targets:
            self._log_no_goal_debug(
                now_ns=now_ns,
                reason="no_frontiers_after_extract",
                planning_map=planning_map,
                per_ns_targets=per_ns_targets,
            )
            # Park each robot at its own current pose so the executor
            # sees a reachable "stay here" goal instead of holding the
            # last assigned (now-unreachable) frontier forever. Without
            # this, FAR adapter reports nav=failed for the stale goal
            # for the rest of the run while CFPA2 returns early every
            # tick. demo3_mixed 2026-04-26: A held (22.86, 1.24) for
            # 60+ s with nav=failed even though all global frontiers
            # had been filtered.
            for ns in self.namespaces:
                if ns not in self.odoms:
                    continue
                od = self.odoms[ns]
                park_goal = (
                    float(od.pose.pose.position.x),
                    float(od.pose.pose.position.y),
                )
                self._set_policy_reason(ns, "hold/exploration_complete")
                park_goal = self._set_active_goal(ns, park_goal, now_ns)
                publish_map = self.maps.get(ns, planning_map)
                self._publish_goal(ns, publish_map, park_goal)
            return

        dist_maps = {}
        for ns in self.namespaces:
            od = self.odoms[ns]
            # When shared map is unavailable we fail-open to per-robot maps.
            cost_map = self.maps.get(ns)
            if cost_map is None:
                dist_maps[ns] = {}
                continue
            dist_maps[ns] = self._distance_transform(
                cost_map,
                (od.pose.pose.position.x, od.pose.pose.position.y),
                occ_threshold=getattr(
                    self,
                    "cfpa2_reachability_occ_threshold",
                    getattr(self, "occ_thresh", 50),
                ),
                allow_unknown=bool(
                    getattr(self, "cfpa2_reachability_allow_unknown", False)
                ),
            )

        local_nav_forced_switch_namespaces = self._consume_local_nav_stall_blacklists(now_ns)

        if len(self.namespaces) == 2:
            ns_a, ns_b = self.namespaces[0], self.namespaces[1]
            map_a = self.maps.get(ns_a)
            map_b = self.maps.get(ns_b)

            utilities_a: dict[tuple[float, float], float] = {}
            utilities_b: dict[tuple[float, float], float] = {}

            if map_a is not None:
                for goal in targets:
                    if self._goal_too_close(ns_a, goal):
                        continue
                    if self._is_blacklisted(ns_a, goal, now_ns):
                        continue
                    score = self._cfpa2_single_utility(
                        ns=ns_a,
                        goal=goal,
                        map_msg=map_a,
                        dist_map=dist_maps.get(ns_a, {}),
                    )
                    if score > -1e17:
                        utilities_a[goal] = score

            if map_b is not None:
                for goal in targets:
                    if self._goal_too_close(ns_b, goal):
                        continue
                    if self._is_blacklisted(ns_b, goal, now_ns):
                        continue
                    score = self._cfpa2_single_utility(
                        ns=ns_b,
                        goal=goal,
                        map_msg=map_b,
                        dist_map=dist_maps.get(ns_b, {}),
                    )
                    if score > -1e17:
                        utilities_b[goal] = score

            if not utilities_a and not utilities_b:
                self._log_no_goal_debug(
                    now_ns=now_ns,
                    reason="cfpa2_no_reachable_utilities",
                    planning_map=planning_map,
                    per_ns_targets=per_ns_targets,
                    dist_maps=dist_maps,
                    utilities_sizes={ns_a: len(utilities_a), ns_b: len(utilities_b)},
                )

            utilities_by_ns: dict[str, dict[tuple[float, float], float]] = {
                ns_a: utilities_a,
                ns_b: utilities_b,
            }

            candidate_goals: dict[str, tuple[float, float]] = {}
            assignment_scores: dict[str, float] = {}
            best_joint = -1e18
            best_pair: Optional[tuple[tuple[float, float], tuple[float, float], float, float]] = None

            for goal_a, score_a in utilities_a.items():
                for goal_b, score_b in utilities_b.items():
                    # Reject literal duplicates and near-duplicates within a
                    # robot footprint — otherwise float noise / a single
                    # shared cell lets both robots pick the same point.
                    if self._goals_equivalent(goal_a, goal_b):
                        continue
                    overlap = self._cfpa2_overlap_penalty(goal_a, goal_b)
                    # Multiplicative penalty (2026-05-09): the previous
                    # additive form `joint = a + b - λ·overlap` made the
                    # penalty a constant ≤ λ regardless of score magnitude;
                    # with IG-dominated scores in the 100-3000 range and
                    # λ=1.0, it deducted ≤ 1.0 from sums of thousands —
                    # effectively zero. Multiplicative makes λ a "max %
                    # deduction when goals fully overlap" (λ=0.5 → up to
                    # 50% off). Scale-invariant w.r.t. IG box size /
                    # cfpa2_w_ig changes. Clamp multiplier to [0, 1] so
                    # λ>1 doesn't flip the sign on positive sums.
                    overlap_multiplier = max(
                        0.0, 1.0 - self.cfpa2_lambda_overlap * overlap)
                    joint = (score_a + score_b) * overlap_multiplier
                    if joint > best_joint:
                        best_joint = joint
                        best_pair = (goal_a, goal_b, score_a, score_b)

            if best_pair is not None:
                goal_a, goal_b, score_a, score_b = best_pair
                candidate_goals[ns_a] = goal_a
                candidate_goals[ns_b] = goal_b
                assignment_scores[ns_a] = score_a
                assignment_scores[ns_b] = score_b
                self._set_policy_reason(ns_a, "switch/cfpa2_joint")
                self._set_policy_reason(ns_b, "switch/cfpa2_joint")
            else:
                # Single-robot fallback (only one ns has viable utilities, or
                # joint allocator found no admissible pair). Pick a single
                # head via greedy max-utility OR TSP-of-top-K depending on
                # cfpa2_planner_mode.
                def _pick_single_head(
                    ns: str, utils: dict[tuple[float, float], float]
                ) -> tuple[Optional[tuple[float, float]], float]:
                    if not utils:
                        return None, -1e18
                    if self.cfpa2_planner_mode == "tsp_topk":
                        head = self._tsp_top_k_head(ns, utils)
                        if head is not None:
                            return head, utils.get(head, max(utils.values()))
                    # Greedy fallback.
                    g, s = max(utils.items(), key=lambda kv: kv[1])
                    return g, s

                best_single_ns: Optional[str] = None
                best_single_goal: Optional[tuple[float, float]] = None
                best_single_score = -1e18
                goal_a, score_a = _pick_single_head(ns_a, utilities_a)
                if goal_a is not None and score_a > best_single_score:
                    best_single_ns = ns_a
                    best_single_goal = goal_a
                    best_single_score = score_a
                goal_b, score_b = _pick_single_head(ns_b, utilities_b)
                if goal_b is not None and score_b > best_single_score:
                    best_single_ns = ns_b
                    best_single_goal = goal_b
                    best_single_score = score_b
                if best_single_ns is not None and best_single_goal is not None:
                    candidate_goals[best_single_ns] = best_single_goal
                    assignment_scores[best_single_ns] = best_single_score
                    reason = ("switch/cfpa2_tsp_topk"
                              if self.cfpa2_planner_mode == "tsp_topk"
                              else "switch/cfpa2_fallback_single")
                    self._set_policy_reason(best_single_ns, reason)

                    # [Duplicate-goal guard] When the joint pair search fails
                    # (typically because utilities_{a,b} share only one common
                    # goal that gets skipped by `goal_a == goal_b`), the other
                    # robot would otherwise fall through to the held-goal
                    # branch below and might inherit the same goal. Force it
                    # onto its best DISTINCT reachable goal, if any exists.
                    # Otherwise leave candidate unset so the held/stop logic
                    # downstream keeps it from colliding.
                    other_ns = ns_b if best_single_ns == ns_a else ns_a
                    other_utils = utilities_by_ns.get(other_ns, {})
                    distinct_goals = {
                        g: s for g, s in other_utils.items()
                        if g != best_single_goal
                    }
                    if distinct_goals:
                        other_goal, other_score = max(
                            distinct_goals.items(), key=lambda kv: kv[1]
                        )
                        candidate_goals[other_ns] = other_goal
                        assignment_scores[other_ns] = other_score
                        self._set_policy_reason(
                            other_ns, "switch/cfpa2_fallback_distinct"
                        )

            forced_switch_namespaces: set[str] = set(local_nav_forced_switch_namespaces)
            for ns in self.namespaces:
                recovery_map = self.maps.get(ns)
                forced_goal = self._maybe_force_cfpa2_stuck_recovery(
                    ns=ns,
                    now_ns=now_ns,
                    utilities=utilities_by_ns.get(ns, {}),
                    fallback_targets=per_ns_targets.get(ns, []),
                    map_msg=recovery_map,
                )
                if forced_goal is None:
                    continue
                candidate_goals[ns] = forced_goal
                assignment_scores[ns] = utilities_by_ns.get(ns, {}).get(
                    forced_goal, assignment_scores.get(ns, 0.0)
                )
                forced_switch_namespaces.add(ns)

            forced_stop_namespaces = self._apply_cfpa2_proximity_stop(
                candidate_goals=candidate_goals,
                assignment_scores=assignment_scores,
                now_ns=now_ns,
            )

            per_ns_assigned: dict[str, tuple[float, float]] = {}
            for ns in self.namespaces:
                candidate = candidate_goals.get(ns)
                held = self.last_goal.get(ns)
                safety_map = self.maps.get(ns)
                if (
                    held is not None
                    and safety_map is not None
                    and not self._is_blacklisted(ns, held, now_ns)
                ):
                    held_failure = self._held_goal_safety_failure(
                        safety_map,
                        dist_maps.get(ns, {}),
                        held,
                    )
                    if held_failure is not None:
                        self._blacklist_active_goal(
                            ns,
                            held,
                            now_ns,
                            f"held_goal_{held_failure}",
                        )
                        forced_switch_namespaces.add(ns)
                        if candidate is None or self._is_blacklisted(ns, candidate, now_ns):
                            fallback = self._cfpa2_best_available_goal(
                                ns=ns,
                                now_ns=now_ns,
                                utilities=utilities_by_ns.get(ns, {}),
                                exclude_goal=held,
                                fallback_targets=per_ns_targets.get(ns, []),
                                map_msg=safety_map,
                                dist_map=dist_maps.get(ns, {}),
                            )
                            if fallback is not None:
                                candidate = fallback
                                candidate_goals[ns] = candidate
                                assignment_scores[ns] = utilities_by_ns.get(ns, {}).get(
                                    fallback, 0.0
                                )
                            else:
                                candidate = self._robot_xy(ns)
                                candidate_goals[ns] = candidate
                                assignment_scores[ns] = -1e6
                                forced_stop_namespaces.add(ns)
                                self._set_policy_reason(
                                    ns, f"hold/held_goal_{held_failure}_stop"
                                )
                        if ns not in forced_stop_namespaces:
                            self._set_policy_reason(ns, f"switch/held_goal_{held_failure}")
                if candidate is None:
                    if held is None:
                        self._set_policy_reason(ns, "hold/cfpa2_no_candidate")
                        continue
                    if self._is_blacklisted(ns, held, now_ns):
                        fallback = self._cfpa2_best_available_goal(
                            ns=ns,
                            now_ns=now_ns,
                            utilities=utilities_by_ns.get(ns, {}),
                            exclude_goal=held,
                            fallback_targets=per_ns_targets.get(ns, []),
                            map_msg=safety_map,
                            dist_map=dist_maps.get(ns, {}),
                        )
                        if fallback is not None:
                            candidate = fallback
                            assignment_scores[ns] = utilities_by_ns.get(ns, {}).get(fallback, 0.0)
                            forced_switch_namespaces.add(ns)
                            self._set_policy_reason(ns, "switch/cfpa2_blacklist_fallback")
                        else:
                            # If current goal is blacklisted and no alternative exists, hold position.
                            candidate = self._robot_xy(ns)
                            forced_stop_namespaces.add(ns)
                            self._set_policy_reason(ns, "hold/cfpa2_blacklisted_stop")
                    else:
                        # [Duplicate-goal guard] Never hold a previous goal
                        # that another robot has just been assigned this
                        # tick — otherwise both robots chase the same point.
                        collision = any(
                            other != ns and
                            self._goals_equivalent(held, candidate_goals.get(other))
                            for other in self.namespaces
                        )
                        if collision:
                            alt = self._cfpa2_best_available_goal(
                                ns=ns,
                                now_ns=now_ns,
                                utilities=utilities_by_ns.get(ns, {}),
                                exclude_goal=held,
                                fallback_targets=per_ns_targets.get(ns, []),
                                map_msg=safety_map,
                                dist_map=dist_maps.get(ns, {}),
                            )
                            if alt is not None:
                                candidate = alt
                                assignment_scores[ns] = utilities_by_ns.get(ns, {}).get(alt, 0.0)
                                forced_switch_namespaces.add(ns)
                                self._set_policy_reason(ns, "switch/cfpa2_avoid_duplicate")
                            else:
                                # No distinct alternative — stop rather than
                                # converge on the same point as the other robot.
                                candidate = self._robot_xy(ns)
                                forced_stop_namespaces.add(ns)
                                self._set_policy_reason(ns, "hold/cfpa2_avoid_duplicate_stop")
                        else:
                            self._set_policy_reason(ns, "hold/cfpa2_keep_previous")
                            goal = held
                if candidate is not None:
                    if ns in forced_switch_namespaces or ns in forced_stop_namespaces:
                        goal = candidate
                    else:
                        goal = self._apply_switch_hysteresis(
                            ns,
                            candidate,
                            assignment_scores.get(ns, 0.0),
                        )

                goal = self._set_active_goal(ns, goal, now_ns)
                publish_map = self.maps.get(ns, planning_map)
                publish_goal = goal
                if (
                    ns not in forced_stop_namespaces
                    and publish_map is not None
                    and self.cfpa2_space_time_enabled
                ):
                    st_waypoint = self._cfpa2_space_time_waypoint(
                        ns=ns,
                        map_msg=publish_map,
                        final_goal=goal,
                        planned_goals=candidate_goals,
                    )
                    if st_waypoint is not None:
                        publish_goal = st_waypoint
                self._publish_goal(ns, publish_map, publish_goal)
                per_ns_assigned[ns] = goal

            per_ns_reachable: dict[str, int] = {}
            for ns in self.namespaces:
                msg = self.maps.get(ns)
                if msg is None:
                    per_ns_reachable[ns] = 0
                    continue
                dist_map = dist_maps.get(ns, {})
                reachable = 0
                for goal in targets:
                    if self._goal_too_close(ns, goal):
                        continue
                    if self._is_blacklisted(ns, goal, now_ns):
                        continue
                    if self._goal_reachable(msg, dist_map, goal):
                        reachable += 1
                per_ns_reachable[ns] = reachable

            per_ns_frontiers = {ns: len(per_ns_targets.get(ns, [])) for ns in self.namespaces}

            # ── exploration-complete / unreachable-park ──────────────
            # If a robot's per_ns_reachable is 0 (no frontier this robot
            # can plan a path to), park it at its current pose. astar
            # then sees d_to_goal < tolerance, declares goal_reached,
            # the goal_reached_brake holds it stationary. Without this
            # the robot sits forever in idle:no_goal because CFPA2
            # never publishes a goal for it; coverage stalls (B was
            # exploring, A spawn-stuck for 190 s in 2026-04-26 run).
            for ns in self.namespaces:
                if per_ns_reachable.get(ns, 0) > 0:
                    continue
                if ns not in self.odoms:
                    continue
                od = self.odoms[ns]
                park_goal = (float(od.pose.pose.position.x),
                             float(od.pose.pose.position.y))
                self._set_policy_reason(ns, "hold/exploration_complete")
                park_goal = self._set_active_goal(ns, park_goal, now_ns)
                publish_map = self.maps.get(ns, planning_map)
                self._publish_goal(ns, publish_map, park_goal)
                per_ns_assigned[ns] = park_goal

            self._maybe_log_summary(
                targets_total=len(targets),
                per_ns_frontiers=per_ns_frontiers,
                per_ns_reachable=per_ns_reachable,
                per_ns_assigned=per_ns_assigned,
            )
            if not per_ns_assigned:
                self._log_no_goal_debug(
                    now_ns=now_ns,
                    reason="cfpa2_no_assignment_published",
                    planning_map=planning_map,
                    per_ns_targets=per_ns_targets,
                    dist_maps=dist_maps,
                    utilities_sizes={ns_a: len(utilities_a), ns_b: len(utilities_b)},
                    candidate_goals=candidate_goals,
                    per_ns_assigned=per_ns_assigned,
                )
            return

        # 3+ robots: fall back to the collaborative greedy allocator below.
        # The 2-robot cfpa2 joint allocator returned earlier. Single-robot
        # path is handled by CFPA2SingleRobotNode (which overrides _tick_impl).
        utilities = [1.0 for _ in targets]
        unassigned = set(self.namespaces)
        assigned: dict[str, int] = {}
        assignment_scores: dict[str, float] = {}
        sigma = max(self.sensor_range * 0.5, 1e-3)

        while unassigned:
            best_pair = None
            best_score = -1e18

            for ns in list(unassigned):
                msg = self.maps.get(ns)
                if msg is None:
                    continue
                dist_map = dist_maps[ns]
                if not dist_map:
                    continue
                for ti, (wx, wy) in enumerate(targets):
                    if self._goal_too_close(ns, (wx, wy)):
                        continue
                    if self._is_blacklisted(ns, (wx, wy), now_ns):
                        continue
                    g = self._world_to_grid(msg, wx, wy)
                    if g is None:
                        continue
                    idx = self._grid_index(g[0], g[1], int(msg.info.width))
                    if idx not in dist_map:
                        continue
                    cost_m = float(dist_map[idx]) * msg.info.resolution
                    score = utilities[ti] - self.beta * cost_m
                    if score > best_score:
                        best_score = score
                        best_pair = (ns, ti, score)

            if best_pair is None:
                break

            ns, ti, score = best_pair
            assigned[ns] = ti
            assignment_scores[ns] = float(score)
            unassigned.remove(ns)

            tx, ty = targets[ti]
            for j, (wx, wy) in enumerate(targets):
                d = math.hypot(wx - tx, wy - ty)
                if d > self.sensor_range:
                    continue
                p = math.exp(-0.5 * (d / sigma) * (d / sigma))
                utilities[j] = max(0.0, utilities[j] - p)

        # Fail-open: if collaborative assignment cannot find a reachable shared target
        # for a robot, use that robot's nearest local frontier.
        candidate_goals: dict[str, tuple[float, float]] = {}
        for ns in list(unassigned):
            local_targets = [
                goal
                for goal in per_ns_targets.get(ns, [])
                if not self._goal_too_close(ns, goal)
                if not self._is_blacklisted(ns, goal, now_ns)
            ]
            if not local_targets:
                continue
            od = self.odoms[ns]
            rx = float(od.pose.pose.position.x)
            ry = float(od.pose.pose.position.y)
            nearest_idx = min(
                range(len(local_targets)),
                key=lambda i: math.hypot(local_targets[i][0] - rx, local_targets[i][1] - ry),
            )
            candidate_goals[ns] = local_targets[nearest_idx]
            assignment_scores.setdefault(ns, 1.0)
            unassigned.remove(ns)

        for ns, ti in assigned.items():
            candidate_goals[ns] = targets[ti]

        per_ns_assigned: dict[str, tuple[float, float]] = {}
        for ns in self.namespaces:
            candidate = candidate_goals.get(ns)
            if candidate is None:
                # No new assignment candidate, hold current goal if any.
                held = self.last_goal.get(ns)
                if held is None:
                    self._set_policy_reason(ns, "hold/no_candidate_no_previous_goal")
                    continue
                self._set_policy_reason(ns, "hold/no_candidate")
                goal = held
            else:
                msg_for_ns = self.maps.get(ns)
                if msg_for_ns is None:
                    held = self.last_goal.get(ns)
                    if held is None:
                        self._set_policy_reason(ns, "hold/no_local_map")
                        continue
                    self._set_policy_reason(ns, "hold/no_local_map")
                    goal = held
                    goal = self._set_active_goal(ns, goal, now_ns)
                    publish_map = self.maps.get(ns, planning_map)
                    self._publish_goal(ns, publish_map, goal)
                    per_ns_assigned[ns] = goal
                    continue
                goal = self._apply_goal_policy(
                    ns=ns,
                    candidate_goal=candidate,
                    assignment_score=assignment_scores.get(ns, 0.0),
                    map_msg=msg_for_ns,
                    dist_map=dist_maps.get(ns, {}),
                    now_ns=now_ns,
                    current_targets=per_ns_targets.get(ns, []),
                )

            goal = self._set_active_goal(ns, goal, now_ns)
            publish_map = self.maps.get(ns, planning_map)
            self._publish_goal(ns, publish_map, goal)
            per_ns_assigned[ns] = goal

        per_ns_reachable: dict[str, int] = {}
        for ns in self.namespaces:
            msg = self.maps.get(ns)
            if msg is None:
                per_ns_reachable[ns] = 0
                continue
            dist_map = dist_maps.get(ns, {})
            if not dist_map:
                per_ns_reachable[ns] = 0
                continue
            reachable = 0
            for wx, wy in targets:
                if self._goal_too_close(ns, (wx, wy)):
                    continue
                if self._is_blacklisted(ns, (wx, wy), now_ns):
                    continue
                g = self._world_to_grid(msg, wx, wy)
                if g is None:
                    continue
                idx = self._grid_index(g[0], g[1], int(msg.info.width))
                if idx in dist_map:
                    reachable += 1
            per_ns_reachable[ns] = reachable

        per_ns_frontiers = {ns: len(per_ns_targets.get(ns, [])) for ns in self.namespaces}

        # ── exploration-complete detection ───────────────────────
        # When per_ns_reachable[ns] is zero, the robot has no
        # frontier it can plan a path to. Re-publishing the same
        # unreachable goal makes astar churn (valid=0) forever and
        # CFPA2 fast-blacklists every retried frontier in a loop.
        # Instead, park the robot at its CURRENT POSE — astar then
        # sees d_to_goal < tolerance, declares goal_reached, and
        # the post-goal-reached brake holds it stationary until the
        # peer discovers a reachable frontier (which may extend the
        # merged map and unstick this robot). Without this, B sat
        # at (23.23, 5.83) for 240 s cycling between two goals it
        # couldn't reach, while coverage was already 78.7 %.
        for ns in self.namespaces:
            if per_ns_reachable.get(ns, 0) > 0:
                continue
            if ns not in self.odoms:
                continue
            # Only park if exploration appears genuinely complete:
            # there ARE frontiers somewhere globally OR map coverage
            # is high. In the early-startup case (no frontiers yet
            # because map is empty), don't park.
            total_reachable = sum(per_ns_reachable.values())
            if total_reachable == 0 and len(targets) > 0:
                # Frontiers exist but none reachable from anyone — park.
                pass
            elif len(targets) == 0:
                # No frontiers at all → exploration complete or
                # not started. Skip parking unless we've seen frontiers
                # before (handled by per-robot last_assigned tracking).
                if per_ns_assigned.get(ns) is None:
                    continue
            od = self.odoms[ns]
            park_goal = (float(od.pose.pose.position.x),
                         float(od.pose.pose.position.y))
            self._set_policy_reason(ns, "hold/exploration_complete")
            park_goal = self._set_active_goal(ns, park_goal, now_ns)
            publish_map = self.maps.get(ns, planning_map)
            self._publish_goal(ns, publish_map, park_goal)
            per_ns_assigned[ns] = park_goal

        self._maybe_log_summary(
            targets_total=len(targets),
            per_ns_frontiers=per_ns_frontiers,
            per_ns_reachable=per_ns_reachable,
            per_ns_assigned=per_ns_assigned,
        )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CFPA2Coordinator()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
