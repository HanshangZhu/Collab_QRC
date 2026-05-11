#!/usr/bin/env python3
"""Single-robot CFPA2 frontier planner."""

from __future__ import annotations

from typing import Optional

from ..core.ros_compat import String
from .cfpa2_coordinator import CFPA2Coordinator


class CFPA2SingleRobotNode(CFPA2Coordinator):
    def __init__(self) -> None:
        super().__init__(
            node_name="cfpa2_single_robot",
            default_namespaces=["robot"],
            startup_label="cfpa2_single_robot",
            planner_desc="Single-robot CFPA2",
        )

        default_ns = self.namespaces[0] if self.namespaces else "robot"
        self.declare_parameter("robot_namespace", default_ns)
        self.robot_namespace = str(self.get_parameter("robot_namespace").value).strip().strip("/") or default_ns

        # verbose_logs: gates the loud throttled debug logs (per-tick reasons,
        # full summary). When False (default), only state-change events are
        # logged so the operator can see what's happening at a glance.
        self.declare_parameter("verbose_logs", False)
        self.verbose_logs = bool(self.get_parameter("verbose_logs").value)

        if len(self.namespaces) != 1:
            raise ValueError(
                "cfpa2_single_robot_node requires exactly one namespace; "
                f"received {self.namespaces}"
            )
        if self.robot_namespace != self.namespaces[0]:
            self.get_logger().warn(
                "robot_namespace parameter does not match namespaces[0]; "
                f"using topic namespace '{self.namespaces[0]}'."
            )
        if self.use_shared_map:
            self.get_logger().warn(
                "cfpa2_single_robot_node ignores use_shared_map=true; "
                "single-robot planning always uses the local map."
            )
            self.use_shared_map = False
        if self.output_mode != "waypoint_coord":
            self.get_logger().warn(
                f"cfpa2_single_robot_node requires output_mode=waypoint_coord; "
                f"overriding '{self.output_mode}'."
            )
            self.output_mode = "waypoint_coord"

        # Single source of truth for exploration state; consumed by
        # exploration_metrics_logger to emit structured event lines and to
        # decide when to fire exploration_complete + cancel Nav2.
        ns = self.namespaces[0]
        self._status_pub = self.create_publisher(
            String, f"/{ns}/exploration_status", 10)
        self._last_status: str = ""

        # Pause flag set by exploration_metrics_logger via /<ns>/exploration_complete.
        # When set, _tick_impl returns early and stops publishing goals.
        self._paused: bool = False
        self.create_subscription(
            String, f"/{ns}/exploration_complete",
            self._on_exploration_complete, 10)

    def _publish_status(self, status: str) -> None:
        """Publish a state change. No-op when status unchanged (avoids spam)."""
        if status == self._last_status:
            return
        self._last_status = status
        msg = String()
        msg.data = status
        self._status_pub.publish(msg)
        self.get_logger().info(f"[exploration_status] {status}")

    def _on_exploration_complete(self, msg: String) -> None:
        if self._paused:
            return
        self._paused = True
        reason = msg.data or "unspecified"
        self.get_logger().warn(
            f"[exploration_complete] reason={reason} — pausing CFPA2 "
            f"goal publication. Send empty or 'resume' on /<ns>/exploration_complete "
            f"to re-enable.")
        self._publish_status("paused")

    def _tick_impl(self) -> None:
        now_ns = self.get_clock().now().nanoseconds
        ns = self.namespaces[0]

        # Honour pause flag from exploration_metrics_logger / operator.
        if self._paused:
            self._publish_status("paused")
            return

        if ns not in self.maps:
            if now_ns - self._last_prereq_warn_ns > int(2e9):
                self.get_logger().warn(
                    f"Waiting for map topic from: {ns}; no single-robot CFPA2 goals will be published yet."
                )
                self._last_prereq_warn_ns = now_ns
            return

        planning_map = self.maps[ns]
        self._publish_coordinator_map(planning_map)
        self._publish_robot_markers(planning_map)

        if ns not in self.odoms:
            if now_ns - self._last_prereq_warn_ns > int(2e9):
                self.get_logger().warn(
                    f"Waiting for odom/nav from: {ns}; no single-robot CFPA2 goals will be published yet."
                )
                self._last_prereq_warn_ns = now_ns
            return

        self._prune_blacklist(ns, now_ns)
        self._update_reached_goal_blacklist(ns, now_ns)

        targets = self._extract_frontiers(planning_map)
        per_ns_targets = {ns: targets}
        if not targets:
            self._publish_status("no_frontiers")
            if self.verbose_logs:
                self._log_no_goal_debug(
                    now_ns=now_ns,
                    reason="no_frontiers_after_extract",
                    planning_map=planning_map,
                    per_ns_targets=per_ns_targets,
                )
            return

        odom = self.odoms[ns]
        dist_map = self._distance_transform(
            planning_map,
            (float(odom.pose.pose.position.x), float(odom.pose.pose.position.y)),
        )
        dist_maps = {ns: dist_map}

        utilities: dict[tuple[float, float], float] = {}
        for goal in targets:
            if self._goal_too_close(ns, goal):
                continue
            if self._is_blacklisted(ns, goal, now_ns):
                continue
            score = self._cfpa2_single_utility(
                ns=ns,
                goal=goal,
                map_msg=planning_map,
                dist_map=dist_map,
            )
            if score > -1e17:
                utilities[goal] = score

        if not utilities:
            self._publish_status("no_reachable")
            if self.verbose_logs:
                self._log_no_goal_debug(
                    now_ns=now_ns,
                    reason="cfpa2_no_reachable_utilities",
                    planning_map=planning_map,
                    per_ns_targets=per_ns_targets,
                    dist_maps=dist_maps,
                    utilities_sizes={ns: 0},
                )

        candidate_goal: Optional[tuple[float, float]] = None
        assignment_score = 0.0
        forced_switch = False

        if utilities:
            candidate_goal, assignment_score = max(utilities.items(), key=lambda kv: kv[1])

        # Stop pattern: if even the best frontier is below the min utility
        # threshold, don't assign any goal — robot holds position.
        if candidate_goal is not None and assignment_score < self.cfpa2_min_utility:
            top3 = sorted(utilities.items(), key=lambda kv: kv[1], reverse=True)[:3]
            top_txt = " ".join(f"({g[0]:.1f},{g[1]:.1f})={s:.2f}" for g, s in top3)
            self.get_logger().info(
                f"HOLD [{ns}] best_u={assignment_score:.2f} < min={self.cfpa2_min_utility:.2f} "
                f"| top: {top_txt}"
            )
            candidate_goal = None

        forced_goal = self._maybe_force_cfpa2_stuck_recovery(
            ns=ns,
            now_ns=now_ns,
            utilities=utilities,
            fallback_targets=targets,
        )
        if forced_goal is not None:
            candidate_goal = forced_goal
            assignment_score = utilities.get(forced_goal, assignment_score)
            forced_switch = True

        goal: Optional[tuple[float, float]] = None
        held_goal = self.last_goal.get(ns)

        if candidate_goal is None:
            if held_goal is None:
                self._set_policy_reason(ns, "hold/cfpa2_no_candidate")
                self._publish_status("searching")
                if self.verbose_logs:
                    self._log_no_goal_debug(
                        now_ns=now_ns,
                        reason="cfpa2_no_assignment_published",
                        planning_map=planning_map,
                        per_ns_targets=per_ns_targets,
                        dist_maps=dist_maps,
                        utilities_sizes={ns: len(utilities)},
                        candidate_goals={},
                        per_ns_assigned={},
                    )
                return

            if self._is_blacklisted(ns, held_goal, now_ns):
                fallback = self._cfpa2_best_available_goal(
                    ns=ns,
                    now_ns=now_ns,
                    utilities=utilities,
                    exclude_goal=held_goal,
                    fallback_targets=targets,
                )
                if fallback is not None:
                    candidate_goal = fallback
                    assignment_score = utilities.get(fallback, 0.0)
                    forced_switch = True
                    self._set_policy_reason(ns, "switch/cfpa2_blacklist_fallback")
                else:
                    goal = self._robot_xy(ns)
                    self._set_policy_reason(ns, "hold/cfpa2_blacklisted_stop")
            else:
                goal = held_goal
                self._set_policy_reason(ns, "hold/cfpa2_keep_previous")

        if goal is None and candidate_goal is not None:
            if forced_switch:
                goal = candidate_goal
            else:
                goal = self._apply_goal_policy(
                    ns=ns,
                    candidate_goal=candidate_goal,
                    assignment_score=assignment_score,
                    map_msg=planning_map,
                    dist_map=dist_map,
                    now_ns=now_ns,
                )

        if goal is None:
            self._publish_status("searching")
            if self.verbose_logs:
                self._log_no_goal_debug(
                    now_ns=now_ns,
                    reason="cfpa2_no_assignment_published",
                    planning_map=planning_map,
                    per_ns_targets=per_ns_targets,
                    dist_maps=dist_maps,
                    utilities_sizes={ns: len(utilities)},
                    candidate_goals={ns: candidate_goal} if candidate_goal is not None else {},
                    per_ns_assigned={},
                )
            return

        self._set_active_goal(ns, goal, now_ns)
        self._publish_goal(ns, planning_map, goal)
        self._publish_status("executing")

        reachable = 0
        for frontier in targets:
            if self._goal_too_close(ns, frontier):
                continue
            if self._is_blacklisted(ns, frontier, now_ns):
                continue
            if self._goal_reachable(planning_map, dist_map, frontier):
                reachable += 1

        if self.verbose_logs:
            self._maybe_log_summary(
                targets_total=len(targets),
                per_ns_frontiers={ns: len(targets)},
                per_ns_reachable={ns: reachable},
                per_ns_assigned={ns: goal},
                per_ns_utilities={ns: utilities},
            )


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CFPA2SingleRobotNode()
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
