from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional

from .geometry import wrap_angle


@dataclass
class RecoveryAction:
    mode: str = "normal"
    linear_x: float = 0.0
    angular_z: float = 0.0
    consume_tick: bool = False
    request_replan: bool = False
    request_escape_reason: Optional[str] = None
    events: list[tuple[str, str]] = field(default_factory=list)


class RecoveryManager:
    def __init__(self, cfg) -> None:
        self.cfg = cfg

    @staticmethod
    def _reset_stall_state(runtime_state) -> None:
        runtime_state.stall_since_sec = None
        runtime_state.stall_anchor_goal_dist = None

    def update_blocked_state(
        self,
        now_sec: float,
        runtime_state,
        robot_state,
        min_front: float,
        external_stop: int,
    ) -> float:
        blocked = external_stop != 0 or min_front < (self.cfg.obstacle_stop_dist + 0.02)
        if blocked:
            if runtime_state.blocked_since_sec is None:
                runtime_state.blocked_since_sec = now_sec
                runtime_state.blocked_anchor_xy = (robot_state.x, robot_state.y)
            return max(0.0, now_sec - runtime_state.blocked_since_sec)

        runtime_state.blocked_since_sec = None
        runtime_state.blocked_anchor_xy = None
        return 0.0

    def update_stall_state(
        self,
        now_sec: float,
        runtime_state,
        robot_state,
        dist_to_goal: float,
        heading_err: float,
        cmd_lin: float,
        blocked_sec: float,
        external_stop: int,
    ) -> float:
        if not self.cfg.stall_recovery_enabled:
            self._reset_stall_state(runtime_state)
            return 0.0

        if (
            blocked_sec > 0.0
            or dist_to_goal <= (self.cfg.goal_tolerance + 0.1)
            or cmd_lin < self.cfg.stall_cmd_min_linear
            or abs(heading_err) > self.cfg.stall_heading_threshold
        ):
            self._reset_stall_state(runtime_state)
            return 0.0

        if runtime_state.stall_anchor_goal_dist is None:
            runtime_state.stall_anchor_goal_dist = dist_to_goal
            runtime_state.stall_since_sec = now_sec
            return 0.0

        progress = runtime_state.stall_anchor_goal_dist - dist_to_goal
        moving_normally = robot_state.speed > self.cfg.stall_speed_epsilon
        if progress >= self.cfg.stall_progress_min_delta_m or moving_normally:
            runtime_state.stall_anchor_goal_dist = dist_to_goal
            runtime_state.stall_since_sec = now_sec
            return 0.0

        if runtime_state.stall_since_sec is None:
            runtime_state.stall_since_sec = now_sec
        return max(0.0, now_sec - runtime_state.stall_since_sec)

    def update_scan_rearm(
        self,
        runtime_state,
        robot_state,
        min_front: float,
    ) -> None:
        if min_front <= self.cfg.wall_scan_rearm_dist:
            return

        moved_from_scan_origin = True
        if runtime_state.scan_trigger_xy is not None:
            moved_from_scan_origin = (
                math.hypot(
                    robot_state.x - runtime_state.scan_trigger_xy[0],
                    robot_state.y - runtime_state.scan_trigger_xy[1],
                )
                >= self.cfg.wall_scan_rearm_progress_dist
            )
        if moved_from_scan_origin:
            runtime_state.scan_mode_rearm_ready = True

    def maybe_trigger_scan(
        self,
        now_sec: float,
        runtime_state,
        robot_state,
        min_front: float,
        blocked_sec: float,
        heading_err_goal: float,
        dist_to_goal: float,
        external_stop: int,
    ) -> RecoveryAction:
        action = RecoveryAction()
        if not self.cfg.wall_scan_enabled:
            return action
        if not runtime_state.scan_mode_rearm_ready:
            return action
        if dist_to_goal <= (self.cfg.goal_tolerance + 0.1):
            return action

        cooldown_ready = True
        if runtime_state.scan_cooldown_until_sec is not None:
            cooldown_ready = now_sec >= runtime_state.scan_cooldown_until_sec
            if cooldown_ready:
                runtime_state.scan_cooldown_until_sec = None
        if not cooldown_ready:
            return action

        blocked_now = external_stop != 0 or min_front < self.cfg.wall_scan_trigger_dist
        if not blocked_now or blocked_sec < self.cfg.wall_scan_blocked_sec:
            return action

        runtime_state.scan_mode_active = True
        runtime_state.scan_mode_rearm_ready = False
        runtime_state.scan_last_yaw = robot_state.yaw
        runtime_state.scan_accum_yaw = 0.0
        runtime_state.scan_dir = 1.0 if heading_err_goal >= 0.0 else -1.0
        runtime_state.scan_trigger_xy = (robot_state.x, robot_state.y)
        action.events.append(
            (
                "warn",
                f"Wall scan triggered: min_front={min_front:.2f}m, turning "
                f"{math.degrees(self.cfg.wall_scan_total_angle):.0f}deg",
            )
        )
        return action

    def step_scan(self, now_sec: float, runtime_state, robot_state) -> RecoveryAction:
        action = RecoveryAction()
        if runtime_state.scan_mode_active:
            if runtime_state.scan_last_yaw is not None:
                runtime_state.scan_accum_yaw += abs(
                    wrap_angle(robot_state.yaw - runtime_state.scan_last_yaw)
                )
            runtime_state.scan_last_yaw = robot_state.yaw

            if runtime_state.scan_accum_yaw >= self.cfg.wall_scan_total_angle:
                runtime_state.scan_mode_active = False
                runtime_state.scan_resume_until_sec = now_sec + max(0.0, self.cfg.wall_scan_pause_after_sec)
                runtime_state.scan_cooldown_until_sec = now_sec + max(
                    0.0,
                    self.cfg.wall_scan_trigger_cooldown_sec,
                )
                action.mode = "scan_complete"
                action.request_replan = True
                action.request_escape_reason = "after wall scan"
                action.consume_tick = True
                action.events.append(("info", "Wall scan completed; requested frontier replan."))
                return action

            action.mode = "scan_turn"
            action.linear_x = 0.0
            action.angular_z = float(runtime_state.scan_dir * abs(self.cfg.wall_scan_turn_speed))
            action.consume_tick = True
            return action

        if runtime_state.scan_resume_until_sec is not None:
            if now_sec < runtime_state.scan_resume_until_sec:
                action.mode = "scan_pause"
                action.consume_tick = True
                return action
            runtime_state.scan_resume_until_sec = None

        return action

    def step_unstick(self, now_sec: float, runtime_state) -> RecoveryAction:
        action = RecoveryAction()
        if runtime_state.unstick_until_sec is None:
            return action

        if now_sec < runtime_state.unstick_until_sec:
            action.mode = "unstick_reverse"
            action.linear_x = float(-abs(self.cfg.unstick_reverse_speed))
            action.angular_z = float(runtime_state.unstick_turn_sign * abs(self.cfg.unstick_turn_speed))
            action.consume_tick = True
            return action

        runtime_state.unstick_until_sec = None
        action.mode = "unstick_done"
        action.request_replan = True
        action.request_escape_reason = "post-unstick"
        return action

    def maybe_trigger_blocked_recovery(
        self,
        now_sec: float,
        runtime_state,
        robot_state,
        left_push: float,
        right_push: float,
        heading_err_goal: float,
        dist_to_goal: float,
        blocked_sec: float,
        rear_clearance: float,
    ) -> RecoveryAction:
        action = RecoveryAction()

        if runtime_state.blocked_since_sec is None:
            return action
        if dist_to_goal <= (self.cfg.goal_tolerance + 0.1):
            return action

        blocked_progress = 0.0
        if runtime_state.blocked_anchor_xy is not None:
            blocked_progress = math.hypot(
                robot_state.x - runtime_state.blocked_anchor_xy[0],
                robot_state.y - runtime_state.blocked_anchor_xy[1],
            )

        cooldown_ok = (
            runtime_state.last_recovery_replan_time_sec is None
            or (now_sec - runtime_state.last_recovery_replan_time_sec)
            >= self.cfg.blocked_replan_cooldown_sec
        )
        if blocked_sec < self.cfg.blocked_replan_sec or not cooldown_ok:
            return action

        action.request_replan = True
        action.request_escape_reason = "blocked-recovery"
        runtime_state.last_recovery_replan_time_sec = now_sec

        if (
            self.cfg.unstick_reverse_enabled
            and blocked_progress <= self.cfg.blocked_progress_epsilon
            and rear_clearance > self.cfg.unstick_min_rear_clearance
        ):
            turn_sign = -1.0 if left_push > right_push else 1.0
            if abs(left_push - right_push) < 0.02:
                turn_sign = 1.0 if heading_err_goal >= 0.0 else -1.0
            runtime_state.unstick_turn_sign = turn_sign
            runtime_state.unstick_until_sec = now_sec + max(0.2, self.cfg.unstick_reverse_sec)
            runtime_state.scan_mode_rearm_ready = False
            runtime_state.scan_cooldown_until_sec = now_sec + max(
                0.0,
                self.cfg.wall_scan_trigger_cooldown_sec,
            )
            action.events.append(("warn", "Corner-trap detected; running reverse unstick maneuver."))

        return action

    def maybe_trigger_stall_recovery(
        self,
        now_sec: float,
        runtime_state,
        left_push: float,
        right_push: float,
        heading_err_goal: float,
        dist_to_goal: float,
        stall_sec: float,
        rear_clearance: float,
    ) -> RecoveryAction:
        action = RecoveryAction()
        if not self.cfg.stall_recovery_enabled:
            return action
        if stall_sec < self.cfg.stall_trigger_sec:
            return action

        cooldown_ok = (
            runtime_state.stall_last_event_time_sec is None
            or (now_sec - runtime_state.stall_last_event_time_sec) >= self.cfg.stall_recovery_cooldown_sec
        )
        if not cooldown_ok:
            return action

        runtime_state.stall_last_event_time_sec = now_sec
        runtime_state.stall_event_count += 1
        runtime_state.stall_since_sec = None
        runtime_state.stall_anchor_goal_dist = dist_to_goal

        action.request_replan = True
        action.request_escape_reason = "stall-recovery"
        action.consume_tick = True

        if self.cfg.unstick_reverse_enabled and rear_clearance > self.cfg.unstick_min_rear_clearance:
            turn_sign = -1.0 if left_push > right_push else 1.0
            if abs(left_push - right_push) < 0.02:
                turn_sign = 1.0 if heading_err_goal >= 0.0 else -1.0
            runtime_state.unstick_turn_sign = turn_sign
            runtime_state.unstick_until_sec = now_sec + max(0.2, self.cfg.unstick_reverse_sec)
            action.mode = "stall_unstick"
            action.linear_x = float(-abs(self.cfg.unstick_reverse_speed))
            action.angular_z = float(turn_sign * abs(self.cfg.unstick_turn_speed))
        else:
            turn_sign = 1.0 if heading_err_goal >= 0.0 else -1.0
            action.mode = "stall_turn"
            action.linear_x = 0.0
            action.angular_z = float(turn_sign * abs(self.cfg.unstick_turn_speed))

        action.events.append(
            (
                "warn",
                f"No-progress deadlock detected (stall={stall_sec:.1f}s); forcing local recovery.",
            )
        )
        return action
