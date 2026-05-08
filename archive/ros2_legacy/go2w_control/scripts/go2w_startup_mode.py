#!/usr/bin/env python3
"""Set Go2W joystick mode and startup posture at launch time."""

from __future__ import annotations

import rclpy
from go2_interfaces.srv import Mode, SwitchJoystick
from rclpy.node import Node


class Go2WStartupMode(Node):
    def __init__(self) -> None:
        super().__init__("go2w_startup_mode")

        self.declare_parameter("switch_joystick_service", "/switch_joystick")
        self.declare_parameter("mode_service", "/mode")
        self.declare_parameter("startup_mode", "stand_up")
        self.declare_parameter("call_switch_joystick", True)
        self.declare_parameter("switch_joystick_flag", False)
        self.declare_parameter("wait_timeout_sec", 30.0)
        self.declare_parameter("retry_period_sec", 0.5)

        switch_service = str(self.get_parameter("switch_joystick_service").value)
        mode_service = str(self.get_parameter("mode_service").value)
        self.startup_mode = str(self.get_parameter("startup_mode").value).strip()
        self.call_switch_joystick = bool(self.get_parameter("call_switch_joystick").value)
        self.switch_joystick_flag = bool(self.get_parameter("switch_joystick_flag").value)
        self.wait_timeout_sec = max(1.0, float(self.get_parameter("wait_timeout_sec").value))
        retry_period_sec = max(0.1, float(self.get_parameter("retry_period_sec").value))

        self.switch_client = self.create_client(SwitchJoystick, switch_service)
        self.mode_client = self.create_client(Mode, mode_service)

        self._start_sec = self._now_sec()
        self._last_wait_log_sec = 0.0
        self._future = None
        self._stage = "switch" if self.call_switch_joystick else "mode"
        self.finished = False
        self.exit_code = 0

        self.timer = self.create_timer(retry_period_sec, self._tick)
        self.get_logger().info(
            f"Go2W startup helper armed: mode={self.startup_mode} "
            f"switch_joystick={self.call_switch_joystick} flag={int(self.switch_joystick_flag)}"
        )

    def _now_sec(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _complete(self, exit_code: int) -> None:
        self.finished = True
        self.exit_code = exit_code

    def _log_waiting(self, text: str) -> None:
        now_sec = self._now_sec()
        if (now_sec - self._last_wait_log_sec) >= 2.0:
            self.get_logger().info(text)
            self._last_wait_log_sec = now_sec

    def _tick(self) -> None:
        if self.finished:
            return

        now_sec = self._now_sec()
        if (now_sec - self._start_sec) > self.wait_timeout_sec:
            self.get_logger().error("Timed out waiting for Go2W startup services.")
            self._complete(1)
            return

        if self._future is not None:
            if not self._future.done():
                return
            try:
                response = self._future.result()
            except Exception as exc:  # pragma: no cover - ROS future exceptions are runtime-only.
                self.get_logger().error(f"Startup service call failed: {exc}")
                self._future = None
                return

            if self._stage == "switch_pending":
                if not response.success:
                    self.get_logger().error("Go2W /switch_joystick returned success=false.")
                    self._complete(1)
                    return
                self.get_logger().info(
                    f"Go2W joystick mode set to ROS-control flag={int(self.switch_joystick_flag)}"
                )
                self._stage = "mode"
                self._future = None
                return

            if self._stage == "mode_pending":
                if not response.success:
                    self.get_logger().error(
                        f"Go2W /mode rejected '{self.startup_mode}': {response.message}"
                    )
                    self._complete(1)
                    return
                self.get_logger().info(f"Go2W startup mode set to '{self.startup_mode}'")
                self._complete(0)
                self._future = None
                return

        if self._stage == "switch":
            if not self.switch_client.wait_for_service(timeout_sec=0.0):
                self._log_waiting("Waiting for /switch_joystick service...")
                return
            request = SwitchJoystick.Request()
            request.flag = self.switch_joystick_flag
            self._future = self.switch_client.call_async(request)
            self._stage = "switch_pending"
            return

        if self._stage == "mode":
            if not self.mode_client.wait_for_service(timeout_sec=0.0):
                self._log_waiting("Waiting for /mode service...")
                return
            request = Mode.Request()
            request.mode = self.startup_mode
            self._future = self.mode_client.call_async(request)
            self._stage = "mode_pending"
            return


def main(args=None) -> None:
    rclpy.init(args=args)
    node = Go2WStartupMode()
    try:
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        node.exit_code = 130
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    raise SystemExit(node.exit_code)


if __name__ == "__main__":
    main()
