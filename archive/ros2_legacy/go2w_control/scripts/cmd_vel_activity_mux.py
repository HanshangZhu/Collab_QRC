#!/usr/bin/env python3
"""Select manual or autonomy Twist commands based on recent joystick activity."""

from __future__ import annotations

import copy
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import String


class CmdVelActivityMux(Node):
    def __init__(self) -> None:
        super().__init__("cmd_vel_activity_mux")

        self.declare_parameter("auto_topic", "/robot/cmd_vel_auto")
        self.declare_parameter("manual_topic", "/robot/cmd_vel_manual")
        self.declare_parameter("output_topic", "/cmd_vel")
        self.declare_parameter("status_topic", "/robot/control_source")
        self.declare_parameter("supervisor_state_topic", "/robot/supervisor_state")
        self.declare_parameter("publish_rate", 20.0)
        self.declare_parameter("manual_timeout_sec", 0.35)
        self.declare_parameter("auto_timeout_sec", 0.60)
        self.declare_parameter("linear_activity_threshold", 0.02)
        self.declare_parameter("angular_activity_threshold", 0.05)

        auto_topic = str(self.get_parameter("auto_topic").value)
        manual_topic = str(self.get_parameter("manual_topic").value)
        output_topic = str(self.get_parameter("output_topic").value)
        status_topic = str(self.get_parameter("status_topic").value)
        supervisor_state_topic = str(self.get_parameter("supervisor_state_topic").value)
        publish_rate = max(1.0, float(self.get_parameter("publish_rate").value))
        self.manual_timeout_sec = max(0.0, float(self.get_parameter("manual_timeout_sec").value))
        self.auto_timeout_sec = max(0.0, float(self.get_parameter("auto_timeout_sec").value))
        self.linear_activity_threshold = max(
            0.0, float(self.get_parameter("linear_activity_threshold").value)
        )
        self.angular_activity_threshold = max(
            0.0, float(self.get_parameter("angular_activity_threshold").value)
        )

        self.auto_msg = Twist()
        self.manual_msg = Twist()
        self.last_auto_rx_sec: float | None = None
        self.last_manual_active_sec: float | None = None
        self.control_source = "idle"
        self.panic_active = False

        self.create_subscription(Twist, auto_topic, self._auto_cb, 10)
        self.create_subscription(Twist, manual_topic, self._manual_cb, 10)
        self.create_subscription(String, supervisor_state_topic, self._state_cb, 10)
        self.cmd_pub = self.create_publisher(Twist, output_topic, 10)
        self.status_pub = self.create_publisher(String, status_topic, 10)
        self.timer = self.create_timer(1.0 / publish_rate, self._tick)

        self.get_logger().info(
            "cmd_vel activity mux started: "
            f"manual={manual_topic} auto={auto_topic} -> {output_topic} "
            f"(manual_timeout={self.manual_timeout_sec:.2f}s auto_timeout={self.auto_timeout_sec:.2f}s)"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _is_active(self, msg: Twist) -> bool:
        linear_mag = math.hypot(float(msg.linear.x), float(msg.linear.y))
        angular_mag = abs(float(msg.angular.z))
        return (
            linear_mag > self.linear_activity_threshold
            or angular_mag > self.angular_activity_threshold
        )

    def _auto_cb(self, msg: Twist) -> None:
        self.auto_msg = copy.deepcopy(msg)
        self.last_auto_rx_sec = self._now_sec()

    def _manual_cb(self, msg: Twist) -> None:
        self.manual_msg = copy.deepcopy(msg)
        if self._is_active(msg):
            self.last_manual_active_sec = self._now_sec()

    def _state_cb(self, msg: String) -> None:
        new_panic = (msg.data.strip().lower() == "panic")
        if new_panic != self.panic_active:
            self.panic_active = new_panic
            self.get_logger().warn(
                "PANIC engaged — auto blocked, manual-only" if new_panic
                else "PANIC cleared — auto resumed"
            )

    def _select_source(self, now_sec: float) -> tuple[str, Twist]:
        manual_recent = (
            self.last_manual_active_sec is not None
            and (now_sec - self.last_manual_active_sec) <= self.manual_timeout_sec
        )
        auto_recent = self.last_auto_rx_sec is not None and (now_sec - self.last_auto_rx_sec) <= self.auto_timeout_sec

        # In panic: block auto entirely. Manual passes if active; otherwise
        # publish zero twist as a safe default (robot halts in place).
        if self.panic_active:
            if manual_recent:
                return ("manual_override", self.manual_msg)
            return ("panic", Twist())

        if manual_recent:
            return ("manual", self.manual_msg)
        if auto_recent:
            return ("auto", self.auto_msg)
        return ("idle", Twist())

    def _tick(self) -> None:
        now_sec = self._now_sec()
        source, msg = self._select_source(now_sec)

        if source != self.control_source:
            self.control_source = source
            self.get_logger().info(f"Control source switched to {source}")

        self.cmd_pub.publish(msg)
        status = String()
        status.data = source
        self.status_pub.publish(status)


def main(args=None) -> None:
    rclpy.init(args=args)
    node = CmdVelActivityMux()
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
