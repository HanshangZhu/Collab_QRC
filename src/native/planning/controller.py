from __future__ import annotations

import math


class MotionController:
    def __init__(self, cfg) -> None:
        self.cfg = cfg

    def compute_cmd(
        self,
        heading_err: float,
        heading_err_goal: float,
        min_front: float,
        left_push: float,
        right_push: float,
        external_stop: int,
    ) -> tuple[float, float]:
        lin = self.cfg.max_linear_speed

        min_ratio = getattr(self.cfg, "min_speed_ratio", 0.0)
        heading_factor = max(min_ratio, math.cos(heading_err))
        lin *= heading_factor

        if min_front < self.cfg.obstacle_slow_dist:
            denom = max(1e-6, self.cfg.obstacle_slow_dist - self.cfg.obstacle_stop_dist)
            speed_scale = max(0.0, (min_front - self.cfg.obstacle_stop_dist) / denom)
            lin *= speed_scale

        blocked_front = min_front < self.cfg.obstacle_stop_dist
        hard_stop = external_stop != 0
        if blocked_front or hard_stop:
            lin = 0.0

        lin = max(0.0, min(lin, self.cfg.max_linear_speed))

        goal_turn = self.cfg.max_angular_speed * (2.0 / math.pi) * heading_err
        goal_turn = max(-self.cfg.max_angular_speed, min(goal_turn, self.cfg.max_angular_speed))

        avoid_raw = right_push - left_push
        if abs(avoid_raw) < self.cfg.avoidance_deadband:
            avoid_raw = 0.0
        avoid_yaw = self.cfg.avoidance_gain * avoid_raw

        if blocked_front and self.cfg.turn_in_place_on_block:
            # Keep turning when blocked so robot can de-trap instead of outputting dead 0,0.
            if abs(goal_turn) >= 0.12:
                avoid_yaw = 0.0
            else:
                # If goal heading is almost straight into an obstacle, bias using local
                # side pressure; if that is ambiguous, pick goal-side turn.
                if abs(avoid_raw) >= self.cfg.avoidance_deadband:
                    avoid_yaw = self.cfg.avoidance_gain * avoid_raw
                else:
                    avoid_yaw = 0.35 * self.cfg.max_angular_speed * (1.0 if heading_err_goal >= 0.0 else -1.0)
        elif avoid_yaw * heading_err_goal < 0.0:
            avoid_yaw *= self.cfg.avoidance_conflict_scale

        max_avoid = max(0.0, self.cfg.avoidance_max_ratio) * self.cfg.max_angular_speed
        avoid_yaw = max(-max_avoid, min(avoid_yaw, max_avoid))

        ang = goal_turn + avoid_yaw
        ang = max(-self.cfg.max_angular_speed, min(ang, self.cfg.max_angular_speed))

        if hard_stop:
            # External stop kills forward motion but allows turn-in-place
            # so the robot can rotate away from the wall, matching
            # blocked_front behavior above.
            return (0.0, float(ang))

        return (float(lin), float(ang))
