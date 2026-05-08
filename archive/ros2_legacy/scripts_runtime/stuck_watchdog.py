#!/usr/bin/env python3
"""stuck_watchdog — outer-loop self-recovery when a robot is stuck.

Why this exists
---------------
MPPI / DWB / FAR can each enter a state where they output (vx≈0, ω≈0)
but never report failure to Nav2's BT navigator. The BT therefore never
runs its built-in recovery sequence (Backup / Spin / Wait), and CFPA2's
pivot-lock can refuse to change goal because clearance is small. Net
result: robot sits idle indefinitely while a goal is held.

This watchdog is the outer-loop escape: monitor pose displacement over
a 10s rolling window. When the robot is supposed to be moving (an active
goal exists) but actually hasn't moved more than `stuck_threshold_m`,
trigger a recovery sequence:

  1. Send a Nav2 BackUp action goal (negative x, ~0.4 m). The
     behavior_server (already running per nav2_go2*_full_stack.yaml)
     handles this action — drives the robot backward at a safe speed,
     stops on collision via collision_checker, re-checks costmap.
  2. After backup finishes (success or abort), republish the cached
     goal_pose so bt_navigator picks up a fresh NavigateToPose request.
     SmacPlannerHybrid then replans from the new (post-backup) pose;
     because the planner uses REEDS_SHEPP, the new path naturally
     supports forward+reverse segments to escape the previous wedge.

Combine with CFPA2's pivot-lock: as soon as the robot moves 0.4 m
backward, clearance is recomputed and the pivot-lock typically releases.

CLI:
    python3 stuck_watchdog.py --ros-args \
        -p namespace:=robot_a \
        -p use_sim_time:=true \
        -p stuck_window_sec:=10.0 \
        -p stuck_threshold_m:=0.20 \
        -p backup_distance_m:=0.40 \
        -p backup_speed_mps:=0.10 \
        -p cooldown_sec:=8.0
"""
from __future__ import annotations

import math
import sys
import time
from collections import deque

import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from nav2_msgs.action import BackUp
from std_msgs.msg import String


def _split_ros_argv(argv):
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        return argv[:i], argv[i:]
    return argv, []


class StuckWatchdog(Node):
    def __init__(self) -> None:
        super().__init__("stuck_watchdog")
        self.declare_parameter("namespace", "robot_a")
        self.declare_parameter("odom_topic", "odom/nav")
        self.declare_parameter("goal_topic", "goal_pose")
        self.declare_parameter("backup_action", "backup")
        # Stuck = moved < threshold over the rolling window with an
        # active goal. Tune threshold for IMU/odom noise floor: 0.20 m
        # is generous (robot would normally move 1+ m in 10 s under
        # any reasonable nav).
        self.declare_parameter("stuck_window_sec", 10.0)
        self.declare_parameter("stuck_threshold_m", 0.20)
        # Recovery action: drive backward `backup_distance_m` at
        # `backup_speed_mps`. Nav2 BackUp action takes negative x as
        # reverse direction by default. time_allowance bounds it.
        self.declare_parameter("backup_distance_m", 0.40)
        self.declare_parameter("backup_speed_mps", 0.10)
        self.declare_parameter("backup_time_allowance_sec", 8.0)
        # Cooldown — don't re-trigger immediately after a recovery (let
        # the new plan execute for at least this long before checking
        # stuck again). Without this we'd loop-fire when the post-
        # recovery plan also stalls within the same window.
        self.declare_parameter("cooldown_sec", 8.0)
        # Goal-change reset: when a fresh goal arrives, clear the pose
        # history so we measure stuck-ness against the new goal only.
        self.declare_parameter("goal_change_threshold_m", 0.50)
        # Heartbeat
        self.declare_parameter("check_rate_hz", 2.0)

        ns = str(self.get_parameter("namespace").value)
        odom_topic = f"/{ns}/{self.get_parameter('odom_topic').value}"
        goal_topic = f"/{ns}/{self.get_parameter('goal_topic').value}"
        backup_action = f"/{ns}/{self.get_parameter('backup_action').value}"

        self.window_sec = float(self.get_parameter("stuck_window_sec").value)
        self.threshold_m = float(self.get_parameter("stuck_threshold_m").value)
        self.backup_dist = float(self.get_parameter("backup_distance_m").value)
        self.backup_speed = float(self.get_parameter("backup_speed_mps").value)
        self.backup_timeout = float(
            self.get_parameter("backup_time_allowance_sec").value
        )
        self.cooldown_sec = float(self.get_parameter("cooldown_sec").value)
        self.goal_change_thr = float(
            self.get_parameter("goal_change_threshold_m").value
        )
        check_period = 1.0 / max(0.1,
                                 float(self.get_parameter("check_rate_hz").value))

        # Nav2's bt_navigator subscribes goal_pose with rclcpp::SystemDefaultsQoS
        # = RELIABLE/VOLATILE/KEEP_LAST(10). The earlier BEST_EFFORT mirror was
        # wrong (cfpa2_to_nav2_bridge had the same bug — see its commit) and
        # caused QoS-incompatibility warnings + silent drops of every
        # republished goal during stuck recovery.
        nav2_goal_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=10,
        )

        # Pose history: deque of (t_sec, x, y).
        self._pose_hist: deque = deque()
        self._latest_goal: PoseStamped | None = None
        self._last_recovery_t: float = 0.0
        self._recovery_in_flight: bool = False

        self.create_subscription(Odometry, odom_topic, self._odom_cb, 10)
        self.create_subscription(PoseStamped, goal_topic, self._goal_cb, nav2_goal_qos)
        # Republish goal on the same topic Nav2 listens on, with the
        # same RELIABLE QoS bt_navigator actually expects.
        self._goal_pub = self.create_publisher(PoseStamped, goal_topic, nav2_goal_qos)
        # Recovery event stream — single canonical source for "what did the
        # outer-loop watchdog do?". Consumed by exploration_metrics_logger
        # for the structured event log, instead of grepping log strings.
        # Values: "stuck_detected" | "backup_started" | "backup_done" |
        #         "backup_aborted" | "backup_unavailable".
        self._recovery_pub = self.create_publisher(
            String, f"/{ns}/recovery_event", 10)
        self._backup_client = ActionClient(self, BackUp, backup_action)
        self.create_timer(check_period, self._check_stuck)

        self._ns = ns
        self._backup_action_name = backup_action
        self.get_logger().info(
            f"stuck_watchdog up: ns={ns} window={self.window_sec}s "
            f"threshold={self.threshold_m}m backup={self.backup_dist}m@"
            f"{self.backup_speed}m/s cooldown={self.cooldown_sec}s"
        )

    @staticmethod
    def _now_sec() -> float:
        return time.monotonic()

    def _odom_cb(self, msg: Odometry) -> None:
        t = self._now_sec()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self._pose_hist.append((t, x, y))
        # prune older than window
        cutoff = t - self.window_sec
        while self._pose_hist and self._pose_hist[0][0] < cutoff:
            self._pose_hist.popleft()

    def _goal_cb(self, msg: PoseStamped) -> None:
        # If goal changed substantially, reset the history — we want to
        # measure stuck-ness against the NEW goal, not lingering old data.
        prev = self._latest_goal
        new = msg
        if prev is None:
            self._pose_hist.clear()
        else:
            dx = new.pose.position.x - prev.pose.position.x
            dy = new.pose.position.y - prev.pose.position.y
            if math.hypot(dx, dy) > self.goal_change_thr:
                self._pose_hist.clear()
        self._latest_goal = new

    def _check_stuck(self) -> None:
        if self._recovery_in_flight:
            return
        if self._latest_goal is None:
            return
        if self._now_sec() - self._last_recovery_t < self.cooldown_sec:
            return
        if not self._pose_hist:
            return

        # Need at least window_sec of data to declare stuck — otherwise
        # the robot just started, give it a chance.
        oldest_t = self._pose_hist[0][0]
        latest_t = self._pose_hist[-1][0]
        if (latest_t - oldest_t) < self.window_sec * 0.9:
            return

        # Bounding-box displacement over the window.
        xs = [p[1] for p in self._pose_hist]
        ys = [p[2] for p in self._pose_hist]
        # Use first→last displacement (not bbox) so a slow constant
        # drift doesn't trigger; only true stillness should.
        dx = xs[-1] - xs[0]
        dy = ys[-1] - ys[0]
        moved = math.hypot(dx, dy)
        if moved >= self.threshold_m:
            return

        gx = self._latest_goal.pose.position.x
        gy = self._latest_goal.pose.position.y
        d2g = math.hypot(xs[-1] - gx, ys[-1] - gy)
        # If we're already AT the goal (within 0.5 m), don't recover —
        # nav2 will mark goal succeeded soon. Avoid spurious backup at
        # arrival.
        if d2g < 0.5:
            return

        self.get_logger().warn(
            f"STUCK detected (ns={self._ns}): moved {moved*100:.1f} cm "
            f"in {self.window_sec:.0f} s, goal at ({gx:+.2f},{gy:+.2f}) "
            f"d2g={d2g:.2f} m — triggering BackUp + replan"
        )
        self._emit_recovery("stuck_detected")
        self._trigger_recovery()

    def _emit_recovery(self, kind: str) -> None:
        msg = String()
        msg.data = kind
        self._recovery_pub.publish(msg)

    def _trigger_recovery(self) -> None:
        if not self._backup_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error(
                f"BackUp action server '{self._backup_action_name}' not "
                f"available — is behavior_server running? skipping recovery."
            )
            self._emit_recovery("backup_unavailable")
            self._last_recovery_t = self._now_sec()  # arm cooldown anyway
            return

        self._recovery_in_flight = True
        self._last_recovery_t = self._now_sec()
        self._emit_recovery("backup_started")
        goal = BackUp.Goal()
        # Nav2 BackUp action: target.x is signed distance from current
        # pose along robot's body frame x-axis (forward = +). Negative
        # to reverse.
        goal.target.x = -abs(self.backup_dist)
        goal.target.y = 0.0
        goal.target.z = 0.0
        goal.speed = abs(self.backup_speed)
        goal.time_allowance.sec = int(self.backup_timeout)
        goal.time_allowance.nanosec = int((self.backup_timeout % 1) * 1e9)

        send_future = self._backup_client.send_goal_async(goal)
        send_future.add_done_callback(self._on_backup_accepted)

    def _on_backup_accepted(self, future) -> None:
        gh = future.result()
        if gh is None or not gh.accepted:
            self.get_logger().warn(
                "BackUp goal not accepted; skipping replan, will re-check."
            )
            self._recovery_in_flight = False
            return
        self.get_logger().info("BackUp accepted, awaiting result.")
        gh.get_result_async().add_done_callback(self._on_backup_done)

    def _on_backup_done(self, future) -> None:
        try:
            res = future.result()
            status = res.status if res is not None else None
        except Exception as exc:
            self.get_logger().warn(f"BackUp result error: {exc}")
            status = None

        # GoalStatus codes: 4=SUCCEEDED, 5=CANCELED, 6=ABORTED.
        if status == 4:
            self._emit_recovery("backup_done")
        else:
            self._emit_recovery("backup_aborted")

        # Whether backup succeeded or aborted (collision check rejected
        # halfway), we still want to nudge Nav2 to replan from the new
        # pose. Republish the cached goal: bt_navigator's NavigateToPose
        # action treats a re-published goal_pose as a preempt + replan.
        if self._latest_goal is not None:
            now = self.get_clock().now().to_msg()
            republish = PoseStamped()
            republish.header.stamp = now
            republish.header.frame_id = self._latest_goal.header.frame_id
            republish.pose = self._latest_goal.pose
            self._goal_pub.publish(republish)
            self.get_logger().info(
                f"BackUp finished (status={status}); republished goal "
                f"({self._latest_goal.pose.position.x:+.2f},"
                f"{self._latest_goal.pose.position.y:+.2f}) to force replan."
            )
        # Clear pose history so the next stuck-window starts fresh.
        self._pose_hist.clear()
        self._recovery_in_flight = False


def main(argv=None) -> None:
    user_argv, ros_argv = _split_ros_argv(sys.argv if argv is None else argv)
    rclpy.init(args=ros_argv)
    node = StuckWatchdog()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
