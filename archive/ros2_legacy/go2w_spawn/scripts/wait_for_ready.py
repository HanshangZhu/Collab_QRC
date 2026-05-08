#!/usr/bin/env python3
"""Readiness gate node: blocks until a condition is met, then exits cleanly.

Used with launch_ros OnProcessExit to replace hardcoded TimerAction delays.
The launch system chains dependent actions on this node's exit:

    wait_node = Node(package="go2w_spawn", executable="wait_for_ready.py", ...)
    RegisterEventHandler(OnProcessExit(target_action=wait_node, on_exit=child_actions))

Modes:
    tf         — Wait until a TF transform becomes available.
    topic      — Wait until a topic has active publishers for stable_count checks.
    imu_stable — Wait until IMU angular velocity is below threshold for stable_count
                 consecutive readings. Best signal for "robot finished standing up".

Exits 0 on success (or on timeout — so downstream always starts).
"""

import math
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import Imu
from tf2_ros import Buffer, TransformListener


class WaitForReady(Node):
    def __init__(self):
        super().__init__("wait_for_ready")

        self.declare_parameter("mode", "topic")
        self.declare_parameter("timeout_sec", 60.0)
        self.declare_parameter("gate_name", "")

        # TF mode params
        self.declare_parameter("tf_parent_frame", "odom")
        self.declare_parameter("tf_child_frame", "base_link")

        # Topic mode params
        self.declare_parameter("watch_topic", "")
        self.declare_parameter("stable_count", 3)
        self.declare_parameter("check_interval_sec", 1.0)

        # IMU stable mode params
        self.declare_parameter("imu_topic", "")
        self.declare_parameter("angular_velocity_threshold", 0.15)  # rad/s

        self.mode = self.get_parameter("mode").value
        self.timeout_sec = self.get_parameter("timeout_sec").value
        self.gate_name = self.get_parameter("gate_name").value or self.mode
        self.start_time = self.get_clock().now()
        self._consecutive_ok = 0

        if self.mode == "tf":
            self._setup_tf_mode()
        elif self.mode == "topic":
            self._setup_topic_mode()
        elif self.mode == "imu_stable":
            self._setup_imu_mode()
        else:
            self.get_logger().error(f"Unknown mode: {self.mode}")
            raise SystemExit(1)

        interval = self.get_parameter("check_interval_sec").value
        self.timer = self.create_timer(interval, self._check)

    # ── TF mode ──

    def _setup_tf_mode(self):
        self.tf_parent = self.get_parameter("tf_parent_frame").value
        self.tf_child = self.get_parameter("tf_child_frame").value
        self.stable_count = self.get_parameter("stable_count").value
        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.get_logger().info(
            f"[{self.gate_name}] Waiting for TF: {self.tf_parent} → {self.tf_child} "
            f"(timeout {self.timeout_sec}s)"
        )

    def _check_tf(self):
        try:
            if self.tf_buffer.can_transform(
                self.tf_parent, self.tf_child, rclpy.time.Time()
            ):
                self._consecutive_ok += 1
                if self._consecutive_ok >= self.stable_count:
                    self._exit_ready(
                        f"TF {self.tf_parent} → {self.tf_child}"
                    )
            else:
                self._consecutive_ok = 0
        except Exception:
            self._consecutive_ok = 0

    # ── Topic mode ──

    def _setup_topic_mode(self):
        self.watch_topic = self.get_parameter("watch_topic").value
        self.stable_count = self.get_parameter("stable_count").value

        if not self.watch_topic:
            self.get_logger().error("watch_topic param is required for topic mode")
            raise SystemExit(1)

        self.get_logger().info(
            f"[{self.gate_name}] Waiting for topic: {self.watch_topic} "
            f"({self.stable_count} consecutive checks, timeout {self.timeout_sec}s)"
        )

    def _check_topic(self):
        pub_count = self.count_publishers(self.watch_topic)
        if pub_count > 0:
            self._consecutive_ok += 1
            if self._consecutive_ok >= self.stable_count:
                self._exit_ready(
                    f"{self.watch_topic} has {pub_count} publisher(s)"
                )
        else:
            self._consecutive_ok = 0

    # ── IMU stable mode ──

    def _setup_imu_mode(self):
        self.imu_topic = self.get_parameter("imu_topic").value
        self.stable_count = self.get_parameter("stable_count").value
        self.ang_vel_threshold = self.get_parameter("angular_velocity_threshold").value
        self._last_ang_vel_norm = float("inf")
        self._imu_received = False

        if not self.imu_topic:
            self.get_logger().error("imu_topic param is required for imu_stable mode")
            raise SystemExit(1)

        qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.BEST_EFFORT)
        self.create_subscription(Imu, self.imu_topic, self._on_imu, qos)

        self.get_logger().info(
            f"[{self.gate_name}] Waiting for IMU stability on {self.imu_topic} "
            f"(|ω| < {self.ang_vel_threshold} rad/s for {self.stable_count} checks, "
            f"timeout {self.timeout_sec}s)"
        )

    def _on_imu(self, msg: Imu):
        w = msg.angular_velocity
        self._last_ang_vel_norm = math.sqrt(w.x**2 + w.y**2 + w.z**2)
        self._imu_received = True

    def _check_imu(self):
        if not self._imu_received:
            self._consecutive_ok = 0
            return

        if self._last_ang_vel_norm < self.ang_vel_threshold:
            self._consecutive_ok += 1
            if self._consecutive_ok >= self.stable_count:
                self._exit_ready(
                    f"IMU stable (|ω|={self._last_ang_vel_norm:.3f} rad/s "
                    f"< {self.ang_vel_threshold})"
                )
        else:
            if self._consecutive_ok > 0:
                self.get_logger().debug(
                    f"[{self.gate_name}] IMU not stable yet "
                    f"(|ω|={self._last_ang_vel_norm:.3f} rad/s), resetting"
                )
            self._consecutive_ok = 0

    # ── Common ──

    def _elapsed(self) -> float:
        return (self.get_clock().now() - self.start_time).nanoseconds / 1e9

    def _exit_ready(self, reason: str):
        self.get_logger().info(
            f"[{self.gate_name}] Ready: {reason} (after {self._elapsed():.1f}s)"
        )
        raise SystemExit(0)

    def _check(self):
        elapsed = self._elapsed()

        if elapsed > self.timeout_sec:
            self.get_logger().warn(
                f"[{self.gate_name}] Timeout ({self.timeout_sec}s) — proceeding anyway"
            )
            raise SystemExit(0)

        if self.mode == "tf":
            self._check_tf()
        elif self.mode == "topic":
            self._check_topic()
        elif self.mode == "imu_stable":
            self._check_imu()


def main():
    rclpy.init()
    try:
        node = WaitForReady()
        rclpy.spin(node)
    except SystemExit as e:
        sys.exit(e.code)
    finally:
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
