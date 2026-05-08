#!/usr/bin/env python3
"""Supervisor panic latch — any-button joystick press triggers emergency override.

Design intent
-------------
The human operator sees a dangerous pose. They press ANY button on the
Unitree official BT controller. Within ~50 ms:
  1. Autonomy cmd_vel is blocked at the mux.
  2. FAR / local planner drop out of autonomyMode (via autonomy_enabler
     zeroing axes[2] on the synthetic /joy when state=panic).
  3. Joystick teleop goes directly through to the Unitree sport API for
     `panic_duration_sec` (default 5 s).
  4. If they press again, the window extends by another 5 s.
  5. On expiry, state → nominal, auto resumes.

Why any-button
--------------
The Unitree BT controller doesn't have a dedicated emergency key, and
forcing the operator to remember one specific button under stress is a
UX hazard. Any-button-press is unambiguous: "I am taking over NOW."

Distinguishing real joystick events from autonomy_enabler's synthetic
/joy is trivial — the synthetic messages always carry `buttons = [0]*N`,
so `any(msg.buttons)` is False for them.

Published topics
----------------
  /robot/supervisor_state  (std_msgs/String)   "nominal" | "panic" at publish_rate_hz
  /robot/panic_cmd_vel     (geometry_msgs/Twist) zero twist at publish_rate_hz
                            — consumed by the mux as a forced-stop fallback when
                            no manual joystick is driving during panic

Consumes
--------
  /joy                              — any-button press → (re)arms the latch
  /supervisor/panic_trigger         — programmatic trigger (std_msgs/Empty)
"""
from __future__ import annotations

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import Empty, String


class SupervisorPanicNode(Node):
    def __init__(self) -> None:
        super().__init__("supervisor_panic")

        self.declare_parameter("panic_duration_sec", 5.0)
        self.declare_parameter("publish_rate_hz", 20.0)
        self.declare_parameter("joy_topic", "/joy")
        self.declare_parameter("trigger_topic", "/supervisor/panic_trigger")
        self.declare_parameter("state_topic", "/robot/supervisor_state")
        self.declare_parameter("panic_cmd_topic", "/robot/panic_cmd_vel")

        self.panic_duration_sec = max(0.1, float(self.get_parameter("panic_duration_sec").value))
        rate = max(1.0, float(self.get_parameter("publish_rate_hz").value))
        joy_topic = str(self.get_parameter("joy_topic").value)
        trigger_topic = str(self.get_parameter("trigger_topic").value)
        state_topic = str(self.get_parameter("state_topic").value)
        panic_cmd_topic = str(self.get_parameter("panic_cmd_topic").value)

        self.panic_until_sec: float = 0.0
        self.last_published_state: str | None = None

        self.create_subscription(Joy, joy_topic, self._joy_cb, 10)
        self.create_subscription(Empty, trigger_topic, self._trigger_cb, 10)
        self.state_pub = self.create_publisher(String, state_topic, 10)
        self.panic_cmd_pub = self.create_publisher(Twist, panic_cmd_topic, 10)
        self.timer = self.create_timer(1.0 / rate, self._tick)

        self.get_logger().info(
            f"Supervisor panic armed — any /joy button → {self.panic_duration_sec:.1f}s override. "
            f"Topic trigger: {trigger_topic}"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _arm_panic(self, source: str) -> None:
        new_until = self._now_sec() + self.panic_duration_sec
        extending = new_until > self.panic_until_sec + 0.01
        self.panic_until_sec = new_until
        if extending:
            self.get_logger().warn(
                f"PANIC armed ({source}) — autonomy blocked for {self.panic_duration_sec:.1f}s"
            )

    def _joy_cb(self, msg: Joy) -> None:
        # autonomy_enabler publishes synthetic /joy with all-zero buttons; we
        # ignore those. Only human button presses arm the latch.
        if msg.buttons and any(bool(b) for b in msg.buttons):
            self._arm_panic("joystick")

    def _trigger_cb(self, _msg: Empty) -> None:
        self._arm_panic("topic")

    def _tick(self) -> None:
        in_panic = self._now_sec() < self.panic_until_sec
        state = "panic" if in_panic else "nominal"

        if state != self.last_published_state:
            self.last_published_state = state
            if state == "nominal" and self.panic_until_sec > 0.0:
                self.get_logger().info("Panic window expired → nominal")

        msg = String()
        msg.data = state
        self.state_pub.publish(msg)

        # Always publish zero twist on the panic channel — the mux is free to
        # ignore it in nominal mode, but during panic this guarantees a known-
        # safe fallback cmd if no manual twist is flowing.
        self.panic_cmd_pub.publish(Twist())


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SupervisorPanicNode()
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
