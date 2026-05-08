from __future__ import annotations

import math
from dataclasses import dataclass, field


@dataclass
class DefaultNavConfig:
    max_linear_speed: float = 0.35
    max_angular_speed: float = 0.8
    goal_tolerance: float = 0.8
    goal_reached_replan_cooldown_sec: float = 2.0
    obstacle_slow_dist: float = 0.6
    obstacle_stop_dist: float = 0.25
    front_half_angle_deg: float = 40.0
    side_check_angle_deg: float = 70.0
    avoidance_gain: float = 1.5
    control_rate: float = 10.0
    startup_delay: float = 15.0
    require_settle_before_motion: bool = True
    settle_speed_threshold: float = 0.06
    settle_hold_sec: float = 2.0
    avoidance_deadband: float = 0.08
    avoidance_max_ratio: float = 0.45
    avoidance_conflict_scale: float = 0.30
    turn_in_place_on_block: bool = True
    wall_scan_enabled: bool = True
    wall_scan_trigger_dist: float = 0.25
    wall_scan_turn_speed: float = 0.7
    wall_scan_total_angle_deg: float = 360.0
    wall_scan_rearm_dist: float = 0.55
    wall_scan_blocked_sec: float = 0.9
    wall_scan_rearm_progress_dist: float = 0.30
    wall_scan_pause_after_sec: float = 1.0
    wall_scan_trigger_cooldown_sec: float = 3.0
    frontier_replan_topic: str = "/frontier_replan"
    stop_topic: str = "/stop"
    planner_enabled: bool = True
    planner_replan_sec: float = 0.6
    planner_invalid_replan_count: int = 1
    planner_grid_radius: float = 5.0
    planner_resolution: float = 0.1
    planner_inflation_radius: float = 0.22
    planner_goal_clip_distance: float = 3.0
    planner_waypoint_spacing: float = 0.35
    planner_waypoint_lookahead: float = 0.45
    planner_unknown_is_obstacle: bool = False
    planner_goal_replan_delta: float = 0.2
    planner_goal_search_radius: float = 1.2
    planner_start_clearance_radius: float = 0.20
    planner_safety_clearance: float = 0.45
    planner_escape_enabled: bool = True
    planner_escape_min_range: float = 0.45
    planner_escape_min_step: float = 0.25
    planner_escape_max_step: float = 1.0
    planner_escape_goal_align_gain: float = 0.6
    planner_escape_turn_penalty_gain: float = 0.2
    planner_escape_clearance_gain: float = 0.8
    planner_escape_blocked_front_penalty: float = 1.5
    blocked_replan_sec: float = 2.5
    blocked_replan_cooldown_sec: float = 3.0
    blocked_progress_epsilon: float = 0.22
    stall_recovery_enabled: bool = True
    stall_cmd_min_linear: float = 0.06
    stall_speed_epsilon: float = 0.03
    stall_heading_threshold_deg: float = 35.0
    stall_progress_min_delta_m: float = 0.05
    stall_trigger_sec: float = 2.0
    stall_recovery_cooldown_sec: float = 4.0
    unstick_reverse_enabled: bool = True
    unstick_reverse_speed: float = 0.12
    unstick_reverse_sec: float = 1.2
    unstick_turn_speed: float = 0.55
    unstick_min_rear_clearance: float = 0.40
    escape_waypoint_hold_sec: float = 4.0
    escape_waypoint_reach_tol: float = 0.30
    # Wheeled-robot trajectory smoothing (0 = disabled, 3 = good for wheels).
    path_smoothing_passes: int = 0
    # Minimum fraction of max_linear_speed maintained during turns (0.0 = full cos slowdown).
    min_speed_ratio: float = 0.0
    # Pre-emptive reverse at T-junctions when sharp turn + low front clearance.
    tight_turn_preempt_enabled: bool = False
    tight_turn_angle_threshold_deg: float = 70.0
    tight_turn_min_front_clearance: float = 0.60

    front_half: float = field(init=False)
    side_half: float = field(init=False)
    wall_scan_total_angle: float = field(init=False)
    stall_heading_threshold: float = field(init=False)
    tight_turn_angle_threshold: float = field(init=False)
    planner_cells: int = field(init=False)

    def __post_init__(self) -> None:
        # Non-breaking safety clamps.
        self.control_rate = max(1e-3, float(self.control_rate))
        self.planner_resolution = max(1e-3, float(self.planner_resolution))

        self.front_half = math.radians(float(self.front_half_angle_deg))
        self.side_half = math.radians(float(self.side_check_angle_deg))
        self.wall_scan_total_angle = math.radians(float(self.wall_scan_total_angle_deg))
        self.stall_heading_threshold = math.radians(max(0.0, float(self.stall_heading_threshold_deg)))
        self.tight_turn_angle_threshold = math.radians(max(0.0, float(self.tight_turn_angle_threshold_deg)))

        planner_cells = int(math.ceil((2.0 * self.planner_grid_radius) / self.planner_resolution)) + 1
        self.planner_cells = max(31, planner_cells | 1)

    @classmethod
    def from_node(cls, node) -> "DefaultNavConfig":
        defaults = cls()
        param_defaults = {
            "max_linear_speed": defaults.max_linear_speed,
            "max_angular_speed": defaults.max_angular_speed,
            "goal_tolerance": defaults.goal_tolerance,
            "goal_reached_replan_cooldown_sec": defaults.goal_reached_replan_cooldown_sec,
            "obstacle_slow_dist": defaults.obstacle_slow_dist,
            "obstacle_stop_dist": defaults.obstacle_stop_dist,
            "front_half_angle_deg": defaults.front_half_angle_deg,
            "side_check_angle_deg": defaults.side_check_angle_deg,
            "avoidance_gain": defaults.avoidance_gain,
            "control_rate": defaults.control_rate,
            "startup_delay": defaults.startup_delay,
            "require_settle_before_motion": defaults.require_settle_before_motion,
            "settle_speed_threshold": defaults.settle_speed_threshold,
            "settle_hold_sec": defaults.settle_hold_sec,
            "avoidance_deadband": defaults.avoidance_deadband,
            "avoidance_max_ratio": defaults.avoidance_max_ratio,
            "avoidance_conflict_scale": defaults.avoidance_conflict_scale,
            "turn_in_place_on_block": defaults.turn_in_place_on_block,
            "wall_scan_enabled": defaults.wall_scan_enabled,
            "wall_scan_trigger_dist": defaults.wall_scan_trigger_dist,
            "wall_scan_turn_speed": defaults.wall_scan_turn_speed,
            "wall_scan_total_angle_deg": defaults.wall_scan_total_angle_deg,
            "wall_scan_rearm_dist": defaults.wall_scan_rearm_dist,
            "wall_scan_blocked_sec": defaults.wall_scan_blocked_sec,
            "wall_scan_rearm_progress_dist": defaults.wall_scan_rearm_progress_dist,
            "wall_scan_pause_after_sec": defaults.wall_scan_pause_after_sec,
            "wall_scan_trigger_cooldown_sec": defaults.wall_scan_trigger_cooldown_sec,
            "frontier_replan_topic": defaults.frontier_replan_topic,
            "stop_topic": defaults.stop_topic,
            "planner_enabled": defaults.planner_enabled,
            "planner_replan_sec": defaults.planner_replan_sec,
            "planner_invalid_replan_count": defaults.planner_invalid_replan_count,
            "planner_grid_radius": defaults.planner_grid_radius,
            "planner_resolution": defaults.planner_resolution,
            "planner_inflation_radius": defaults.planner_inflation_radius,
            "planner_goal_clip_distance": defaults.planner_goal_clip_distance,
            "planner_waypoint_spacing": defaults.planner_waypoint_spacing,
            "planner_waypoint_lookahead": defaults.planner_waypoint_lookahead,
            "planner_unknown_is_obstacle": defaults.planner_unknown_is_obstacle,
            "planner_goal_replan_delta": defaults.planner_goal_replan_delta,
            "planner_goal_search_radius": defaults.planner_goal_search_radius,
            "planner_start_clearance_radius": defaults.planner_start_clearance_radius,
            "planner_safety_clearance": defaults.planner_safety_clearance,
            "planner_escape_enabled": defaults.planner_escape_enabled,
            "planner_escape_min_range": defaults.planner_escape_min_range,
            "planner_escape_min_step": defaults.planner_escape_min_step,
            "planner_escape_max_step": defaults.planner_escape_max_step,
            "planner_escape_goal_align_gain": defaults.planner_escape_goal_align_gain,
            "planner_escape_turn_penalty_gain": defaults.planner_escape_turn_penalty_gain,
            "planner_escape_clearance_gain": defaults.planner_escape_clearance_gain,
            "planner_escape_blocked_front_penalty": defaults.planner_escape_blocked_front_penalty,
            "blocked_replan_sec": defaults.blocked_replan_sec,
            "blocked_replan_cooldown_sec": defaults.blocked_replan_cooldown_sec,
            "blocked_progress_epsilon": defaults.blocked_progress_epsilon,
            "stall_recovery_enabled": defaults.stall_recovery_enabled,
            "stall_cmd_min_linear": defaults.stall_cmd_min_linear,
            "stall_speed_epsilon": defaults.stall_speed_epsilon,
            "stall_heading_threshold_deg": defaults.stall_heading_threshold_deg,
            "stall_progress_min_delta_m": defaults.stall_progress_min_delta_m,
            "stall_trigger_sec": defaults.stall_trigger_sec,
            "stall_recovery_cooldown_sec": defaults.stall_recovery_cooldown_sec,
            "unstick_reverse_enabled": defaults.unstick_reverse_enabled,
            "unstick_reverse_speed": defaults.unstick_reverse_speed,
            "unstick_reverse_sec": defaults.unstick_reverse_sec,
            "unstick_turn_speed": defaults.unstick_turn_speed,
            "unstick_min_rear_clearance": defaults.unstick_min_rear_clearance,
            "escape_waypoint_hold_sec": defaults.escape_waypoint_hold_sec,
            "escape_waypoint_reach_tol": defaults.escape_waypoint_reach_tol,
            "path_smoothing_passes": defaults.path_smoothing_passes,
            "min_speed_ratio": defaults.min_speed_ratio,
            "tight_turn_preempt_enabled": defaults.tight_turn_preempt_enabled,
            "tight_turn_angle_threshold_deg": defaults.tight_turn_angle_threshold_deg,
            "tight_turn_min_front_clearance": defaults.tight_turn_min_front_clearance,
        }

        for key, value in param_defaults.items():
            node.declare_parameter(key, value)

        kwargs = {
            key: node.get_parameter(key).value
            for key in param_defaults
        }

        float_fields = {
            "max_linear_speed",
            "max_angular_speed",
            "goal_tolerance",
            "goal_reached_replan_cooldown_sec",
            "obstacle_slow_dist",
            "obstacle_stop_dist",
            "front_half_angle_deg",
            "side_check_angle_deg",
            "avoidance_gain",
            "control_rate",
            "startup_delay",
            "settle_speed_threshold",
            "settle_hold_sec",
            "avoidance_deadband",
            "avoidance_max_ratio",
            "avoidance_conflict_scale",
            "wall_scan_trigger_dist",
            "wall_scan_turn_speed",
            "wall_scan_total_angle_deg",
            "wall_scan_rearm_dist",
            "wall_scan_blocked_sec",
            "wall_scan_rearm_progress_dist",
            "wall_scan_pause_after_sec",
            "wall_scan_trigger_cooldown_sec",
            "planner_replan_sec",
            "planner_grid_radius",
            "planner_resolution",
            "planner_inflation_radius",
            "planner_goal_clip_distance",
            "planner_waypoint_spacing",
            "planner_waypoint_lookahead",
            "planner_goal_replan_delta",
            "planner_goal_search_radius",
            "planner_start_clearance_radius",
            "planner_safety_clearance",
            "planner_escape_min_range",
            "planner_escape_min_step",
            "planner_escape_max_step",
            "planner_escape_goal_align_gain",
            "planner_escape_turn_penalty_gain",
            "planner_escape_clearance_gain",
            "planner_escape_blocked_front_penalty",
            "blocked_replan_sec",
            "blocked_replan_cooldown_sec",
            "blocked_progress_epsilon",
            "stall_cmd_min_linear",
            "stall_speed_epsilon",
            "stall_heading_threshold_deg",
            "stall_progress_min_delta_m",
            "stall_trigger_sec",
            "stall_recovery_cooldown_sec",
            "unstick_reverse_speed",
            "unstick_reverse_sec",
            "unstick_turn_speed",
            "unstick_min_rear_clearance",
            "escape_waypoint_hold_sec",
            "escape_waypoint_reach_tol",
            "min_speed_ratio",
            "tight_turn_angle_threshold_deg",
            "tight_turn_min_front_clearance",
        }
        bool_fields = {
            "require_settle_before_motion",
            "turn_in_place_on_block",
            "wall_scan_enabled",
            "planner_enabled",
            "planner_unknown_is_obstacle",
            "planner_escape_enabled",
            "stall_recovery_enabled",
            "unstick_reverse_enabled",
            "tight_turn_preempt_enabled",
        }

        for field in float_fields:
            kwargs[field] = float(kwargs[field])
        for field in bool_fields:
            kwargs[field] = bool(kwargs[field])

        kwargs["frontier_replan_topic"] = str(kwargs["frontier_replan_topic"])
        kwargs["stop_topic"] = str(kwargs["stop_topic"])
        kwargs["planner_invalid_replan_count"] = max(1, int(kwargs["planner_invalid_replan_count"]))
        kwargs["path_smoothing_passes"] = int(kwargs["path_smoothing_passes"])

        return cls(**kwargs)
