from __future__ import annotations

import math

from .controller import MotionController
from .geometry import wrap_angle
from .perception import ScanAnalyzer
from .planner import LocalPlanner
from .recovery import RecoveryManager
from .state import TickResult


class DefaultNavCoordinator:
    def __init__(self, cfg) -> None:
        self.cfg = cfg
        self.scan_analyzer = ScanAnalyzer()
        self.planner = LocalPlanner(cfg)
        self.recovery = RecoveryManager(cfg)
        self.controller = MotionController(cfg)

    @staticmethod
    def _reset_plan_cache(runtime_state) -> None:
        runtime_state.plan_waypoints_world = []
        runtime_state.plan_last_time_sec = None
        runtime_state.plan_last_goal = None

    def _apply_action_common(
        self,
        action,
        now_sec: float,
        runtime_state,
        robot_state,
        scan,
        goal_dx_world: float,
        goal_dy_world: float,
        events: list[tuple[str, str]],
    ) -> bool:
        events.extend(action.events)
        if action.request_replan:
            self._reset_plan_cache(runtime_state)
        if action.request_escape_reason is not None:
            ok, escape_events = self.planner.set_temporary_escape_target(
                now_sec,
                runtime_state,
                robot_state,
                scan,
                goal_dx_world,
                goal_dy_world,
                reason=action.request_escape_reason,
            )
            events.extend(escape_events)
            if not ok:
                events.append(("warn", "No viable scan-based escape target found."))
        return action.consume_tick

    def tick(
        self,
        now_sec: float,
        runtime_state,
        robot_state,
        goal_state,
        scan,
        external_stop: int,
    ) -> TickResult:
        events: list[tuple[str, str]] = []

        if runtime_state.start_time_sec is None:
            runtime_state.start_time_sec = now_sec
        if (now_sec - runtime_state.start_time_sec) < self.cfg.startup_delay:
            return TickResult(
                0.0,
                0.0,
                events=events,
                diagnostics={
                    "mode": "startup",
                    "ext_stop": external_stop,
                    "stall_sec": 0.0,
                    "stall_active": False,
                    "stall_event_count": runtime_state.stall_event_count,
                },
            )

        if self.cfg.require_settle_before_motion and not runtime_state.settle_ready:
            if robot_state.speed > self.cfg.settle_speed_threshold:
                runtime_state.settle_start_time_sec = None
                return TickResult(
                    0.0,
                    0.0,
                    events=events,
                    diagnostics={
                        "mode": "settling",
                        "speed": round(robot_state.speed, 3),
                        "ext_stop": external_stop,
                        "stall_sec": 0.0,
                        "stall_active": False,
                        "stall_event_count": runtime_state.stall_event_count,
                    },
                )

            if runtime_state.settle_start_time_sec is None:
                runtime_state.settle_start_time_sec = now_sec
                return TickResult(
                    0.0,
                    0.0,
                    events=events,
                    diagnostics={
                        "mode": "settling",
                        "speed": round(robot_state.speed, 3),
                        "ext_stop": external_stop,
                        "stall_sec": 0.0,
                        "stall_active": False,
                        "stall_event_count": runtime_state.stall_event_count,
                    },
                )

            settle_elapsed = now_sec - runtime_state.settle_start_time_sec
            if settle_elapsed < self.cfg.settle_hold_sec:
                return TickResult(
                    0.0,
                    0.0,
                    events=events,
                    diagnostics={
                        "mode": "settling",
                        "speed": round(robot_state.speed, 3),
                        "settle_elapsed": round(settle_elapsed, 2),
                        "ext_stop": external_stop,
                        "stall_sec": 0.0,
                        "stall_active": False,
                        "stall_event_count": runtime_state.stall_event_count,
                    },
                )

            runtime_state.settle_ready = True
            events.append(
                (
                    "info",
                    f"Settle gate passed (speed<{self.cfg.settle_speed_threshold:.2f} m/s "
                    f"for {self.cfg.settle_hold_sec:.1f}s).",
                )
            )

        if goal_state.x is None or goal_state.y is None:
            return TickResult(
                0.0,
                0.0,
                events=events,
                diagnostics={
                    "mode": "no_goal",
                    "ext_stop": external_stop,
                    "stall_sec": 0.0,
                    "stall_active": False,
                    "stall_event_count": runtime_state.stall_event_count,
                },
            )

        if scan is None:
            return TickResult(
                0.0,
                0.0,
                events=events,
                diagnostics={
                    "mode": "no_scan",
                    "goal": [round(goal_state.x, 2), round(goal_state.y, 2)],
                    "ext_stop": external_stop,
                    "stall_sec": 0.0,
                    "stall_active": False,
                    "stall_event_count": runtime_state.stall_event_count,
                },
            )

        goal_dx_world = goal_state.x - robot_state.x
        goal_dy_world = goal_state.y - robot_state.y
        dist_to_goal = math.hypot(goal_dx_world, goal_dy_world)
        if dist_to_goal < self.cfg.goal_tolerance:
            request_replan = False
            last_replan = runtime_state.last_goal_reached_replan_time_sec
            if (
                last_replan is None
                or (now_sec - last_replan) >= self.cfg.goal_reached_replan_cooldown_sec
            ):
                request_replan = True
                runtime_state.last_goal_reached_replan_time_sec = now_sec
            return TickResult(0.0, 0.0, events=events, diagnostics={
                "mode": "goal_reached",
                "goal": [round(goal_state.x, 2), round(goal_state.y, 2)],
                "dist_goal": round(dist_to_goal, 2),
                "ext_stop": external_stop,
                "stall_sec": 0.0,
                "stall_active": False,
                "stall_event_count": runtime_state.stall_event_count,
            }, request_replan=request_replan)

        goal_angle = math.atan2(goal_dy_world, goal_dx_world)
        heading_err_goal = wrap_angle(goal_angle - robot_state.yaw)

        scan_metrics = self.scan_analyzer.analyze(scan, self.cfg)
        blocked_sec = self.recovery.update_blocked_state(
            now_sec,
            runtime_state,
            robot_state,
            scan_metrics.min_front,
            external_stop,
        )
        self.recovery.update_scan_rearm(runtime_state, robot_state, scan_metrics.min_front)

        trigger_action = self.recovery.maybe_trigger_scan(
            now_sec,
            runtime_state,
            robot_state,
            scan_metrics.min_front,
            blocked_sec,
            heading_err_goal,
            dist_to_goal,
            external_stop,
        )
        self._apply_action_common(
            trigger_action,
            now_sec,
            runtime_state,
            robot_state,
            scan,
            goal_dx_world,
            goal_dy_world,
            events,
        )

        scan_action = self.recovery.step_scan(now_sec, runtime_state, robot_state)
        if self._apply_action_common(
            scan_action,
            now_sec,
            runtime_state,
            robot_state,
            scan,
            goal_dx_world,
            goal_dy_world,
            events,
        ):
            mode = "wall_scan" if scan_action.mode == "scan_turn" else "scan_pause"
            return TickResult(
                scan_action.linear_x,
                scan_action.angular_z,
                request_replan=scan_action.request_replan,
                events=events,
                diagnostics={
                    "mode": mode,
                    "goal": [round(goal_state.x, 2), round(goal_state.y, 2)],
                    "dist_goal": round(dist_to_goal, 2),
                    "min_front": round(scan_metrics.min_front, 2),
                    "blocked_sec": round(blocked_sec, 1),
                    "ext_stop": external_stop,
                    "stall_sec": 0.0,
                    "stall_active": False,
                    "stall_event_count": runtime_state.stall_event_count,
                },
            )

        unstick_action = self.recovery.step_unstick(now_sec, runtime_state)
        if self._apply_action_common(
            unstick_action,
            now_sec,
            runtime_state,
            robot_state,
            scan,
            goal_dx_world,
            goal_dy_world,
            events,
        ):
            return TickResult(
                unstick_action.linear_x,
                unstick_action.angular_z,
                request_replan=unstick_action.request_replan,
                events=events,
                diagnostics={
                    "mode": "unstick",
                    "goal": [round(goal_state.x, 2), round(goal_state.y, 2)],
                    "dist_goal": round(dist_to_goal, 2),
                    "min_front": round(scan_metrics.min_front, 2),
                    "blocked_sec": round(blocked_sec, 1),
                    "ext_stop": external_stop,
                    "stall_sec": 0.0,
                    "stall_active": False,
                    "stall_event_count": runtime_state.stall_event_count,
                },
            )

        blocked_action = self.recovery.maybe_trigger_blocked_recovery(
            now_sec,
            runtime_state,
            robot_state,
            scan_metrics.left_push,
            scan_metrics.right_push,
            heading_err_goal,
            dist_to_goal,
            blocked_sec,
            scan_metrics.rear_clearance,
        )
        self._apply_action_common(
            blocked_action,
            now_sec,
            runtime_state,
            robot_state,
            scan,
            goal_dx_world,
            goal_dy_world,
            events,
        )

        self.planner.clear_stale_escape_target(now_sec, robot_state, runtime_state)

        target_x = goal_state.x
        target_y = goal_state.y
        steering_source = "goal"
        if runtime_state.escape_target_world is not None:
            target_x, target_y = runtime_state.escape_target_world
            steering_source = "escape"
        elif self.cfg.planner_enabled:
            plan_result = self.planner.planner_target_world(
                now_sec,
                runtime_state,
                robot_state,
                goal_state,
                scan,
                goal_dx_world,
                goal_dy_world,
            )
            events.extend(plan_result.events)
            if plan_result.target_world is not None:
                target_x, target_y = plan_result.target_world
                steering_source = "planner"

        target_dx = target_x - robot_state.x
        target_dy = target_y - robot_state.y
        target_angle = math.atan2(target_dy, target_dx)
        heading_err = wrap_angle(target_angle - robot_state.yaw)

        # ── Tight-turn give-way: pre-emptive reverse at T-junctions ──
        tt_result = self._maybe_tight_turn_reverse(
            now_sec, runtime_state, robot_state,
            heading_err, scan_metrics.min_front,
            scan_metrics.rear_clearance,
            goal_state, dist_to_goal, blocked_sec, external_stop,
            steering_source, events,
        )
        if tt_result is not None:
            return tt_result

        lin, ang = self.controller.compute_cmd(
            heading_err,
            heading_err_goal,
            scan_metrics.min_front,
            scan_metrics.left_push,
            scan_metrics.right_push,
            external_stop,
        )
        stall_sec = self.recovery.update_stall_state(
            now_sec,
            runtime_state,
            robot_state,
            dist_to_goal,
            heading_err,
            lin,
            blocked_sec,
            external_stop,
        )
        stall_action = self.recovery.maybe_trigger_stall_recovery(
            now_sec,
            runtime_state,
            scan_metrics.left_push,
            scan_metrics.right_push,
            heading_err_goal,
            dist_to_goal,
            stall_sec,
            scan_metrics.rear_clearance,
        )
        if self._apply_action_common(
            stall_action,
            now_sec,
            runtime_state,
            robot_state,
            scan,
            goal_dx_world,
            goal_dy_world,
            events,
        ):
            return TickResult(
                stall_action.linear_x,
                stall_action.angular_z,
                request_replan=stall_action.request_replan,
                events=events,
                diagnostics={
                    "mode": "stall_recovery",
                    "goal": [round(goal_state.x, 2), round(goal_state.y, 2)],
                    "dist_goal": round(dist_to_goal, 2),
                    "target": [round(target_x, 2), round(target_y, 2)],
                    "min_front": round(scan_metrics.min_front, 2),
                    "blocked_sec": round(blocked_sec, 1),
                    "stall_sec": round(stall_sec, 1),
                    "stall_active": stall_sec > 0.0,
                    "stall_event_count": runtime_state.stall_event_count,
                    "ext_stop": external_stop,
                },
            )
        blocked_front = scan_metrics.min_front < self.cfg.obstacle_stop_dist
        hard_stop = external_stop != 0
        zero_reason = None
        if hard_stop:
            zero_reason = "external_stop"
        elif abs(lin) < 1e-6 and abs(ang) < 1e-6:
            if blocked_front:
                zero_reason = "blocked_front_zero_turn"
            else:
                heading_factor = max(0.0, math.cos(heading_err))
                if heading_factor <= 1e-3:
                    zero_reason = "heading_factor_near_zero"
                else:
                    zero_reason = "controller_zero"

        has_plan = len(runtime_state.plan_waypoints_world) > 0
        plan_wps = len(runtime_state.plan_waypoints_world)
        escape = None
        if runtime_state.escape_target_world is not None:
            escape = [round(runtime_state.escape_target_world[0], 2),
                      round(runtime_state.escape_target_world[1], 2)]

        diag = {
            "mode": "navigate",
            "steer": steering_source,
            "goal": [round(goal_state.x, 2), round(goal_state.y, 2)],
            "dist_goal": round(dist_to_goal, 2),
            "target": [round(target_x, 2), round(target_y, 2)],
            "min_front": round(scan_metrics.min_front, 2),
            "blocked_sec": round(blocked_sec, 1),
            "stall_sec": round(stall_sec, 1),
            "stall_active": stall_sec > 0.0,
            "stall_event_count": runtime_state.stall_event_count,
            "has_plan": has_plan,
            "plan_wps": plan_wps,
            "escape": escape,
            "ext_stop": external_stop,
            "blocked_front": blocked_front,
            "hard_stop": hard_stop,
            "heading_err_deg": round(math.degrees(heading_err), 1),
            "heading_err_goal_deg": round(math.degrees(heading_err_goal), 1),
            "zero_reason": zero_reason,
        }

        return TickResult(
            linear_x=lin,
            angular_z=ang,
            request_replan=(
                blocked_action.request_replan
                or unstick_action.request_replan
                or stall_action.request_replan
            ),
            events=events,
            diagnostics=diag,
        )

    def _maybe_tight_turn_reverse(
        self,
        now_sec: float,
        runtime_state,
        robot_state,
        heading_err: float,
        min_front: float,
        rear_clearance: float,
        goal_state,
        dist_to_goal: float,
        blocked_sec: float,
        external_stop: int,
        steering_source: str,
        events: list,
    ):
        """Pre-emptive reverse when a sharp turn is needed but front clearance is low.

        Returns a TickResult if the robot should be reversing, or None to continue
        normal navigation.
        """
        if not self.cfg.tight_turn_preempt_enabled:
            return None

        # Currently executing a tight-turn reverse
        if runtime_state.tight_turn_reverse_until_sec is not None:
            if now_sec < runtime_state.tight_turn_reverse_until_sec:
                remaining = runtime_state.tight_turn_reverse_until_sec - now_sec
                return TickResult(
                    linear_x=-self.cfg.unstick_reverse_speed,
                    angular_z=0.0,
                    events=events,
                    diagnostics={
                        "mode": "tight_turn_reverse",
                        "goal": [round(goal_state.x, 2), round(goal_state.y, 2)],
                        "dist_goal": round(dist_to_goal, 2),
                        "min_front": round(min_front, 2),
                        "heading_err_deg": round(math.degrees(heading_err), 1),
                        "reverse_remaining_sec": round(remaining, 2),
                        "ext_stop": external_stop,
                        "stall_sec": 0.0,
                        "stall_active": False,
                        "stall_event_count": runtime_state.stall_event_count,
                    },
                )
            else:
                # Reverse phase done — enter cooldown
                runtime_state.tight_turn_reverse_until_sec = None
                runtime_state.tight_turn_cooldown_until_sec = now_sec + 3.0
                events.append(("info", "Tight-turn reverse complete, resuming navigation."))
                return None

        # In cooldown — skip detection
        if (runtime_state.tight_turn_cooldown_until_sec is not None
                and now_sec < runtime_state.tight_turn_cooldown_until_sec):
            return None

        # ── Detection ──
        # Sharp turn needed AND front is tight AND rear has room
        sharp_turn = abs(heading_err) > self.cfg.tight_turn_angle_threshold
        front_tight = min_front < self.cfg.tight_turn_min_front_clearance
        rear_ok = rear_clearance > self.cfg.unstick_min_rear_clearance

        if sharp_turn and front_tight and rear_ok:
            duration = self.cfg.unstick_reverse_sec
            runtime_state.tight_turn_reverse_until_sec = now_sec + duration
            events.append((
                "info",
                f"TIGHT-TURN give-way: heading_err={math.degrees(heading_err):.0f}° "
                f"min_front={min_front:.2f}m → reversing {duration:.1f}s",
            ))
            return TickResult(
                linear_x=-self.cfg.unstick_reverse_speed,
                angular_z=0.0,
                events=events,
                diagnostics={
                    "mode": "tight_turn_reverse",
                    "goal": [round(goal_state.x, 2), round(goal_state.y, 2)],
                    "dist_goal": round(dist_to_goal, 2),
                    "min_front": round(min_front, 2),
                    "heading_err_deg": round(math.degrees(heading_err), 1),
                    "reverse_remaining_sec": round(duration, 2),
                    "ext_stop": external_stop,
                    "stall_sec": 0.0,
                    "stall_active": False,
                    "stall_event_count": runtime_state.stall_event_count,
                },
            )

        return None

