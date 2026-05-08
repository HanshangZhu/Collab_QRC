from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional


@dataclass
class RobotState:
    x: float = 0.0
    y: float = 0.0
    yaw: float = 0.0
    speed: float = float("inf")


@dataclass
class GoalState:
    x: Optional[float] = None
    y: Optional[float] = None


@dataclass
class NavRuntimeState:
    start_time_sec: Optional[float] = None
    settle_start_time_sec: Optional[float] = None
    settle_ready: bool = False

    scan_mode_active: bool = False
    scan_mode_rearm_ready: bool = True
    scan_dir: float = 1.0
    scan_last_yaw: Optional[float] = None
    scan_accum_yaw: float = 0.0
    scan_trigger_xy: Optional[tuple[float, float]] = None
    scan_resume_until_sec: Optional[float] = None
    scan_cooldown_until_sec: Optional[float] = None

    plan_waypoints_world: list[tuple[float, float]] = field(default_factory=list)
    plan_last_time_sec: Optional[float] = None
    plan_last_goal: Optional[tuple[float, float]] = None
    plan_last_warn_sec: int = -1
    plan_invalid_streak: int = 0

    blocked_since_sec: Optional[float] = None
    blocked_anchor_xy: Optional[tuple[float, float]] = None
    last_recovery_replan_time_sec: Optional[float] = None
    last_goal_reached_replan_time_sec: Optional[float] = None
    stall_since_sec: Optional[float] = None
    stall_anchor_goal_dist: Optional[float] = None
    stall_last_event_time_sec: Optional[float] = None
    stall_event_count: int = 0

    escape_target_world: Optional[tuple[float, float]] = None
    escape_target_until_sec: Optional[float] = None

    unstick_until_sec: Optional[float] = None
    unstick_turn_sign: float = 1.0

    tight_turn_reverse_until_sec: Optional[float] = None
    tight_turn_cooldown_until_sec: Optional[float] = None

    external_stop: int = 0


@dataclass
class TickResult:
    linear_x: float
    angular_z: float
    request_replan: bool = False
    events: list[tuple[str, str]] = field(default_factory=list)
    diagnostics: dict = field(default_factory=dict)
