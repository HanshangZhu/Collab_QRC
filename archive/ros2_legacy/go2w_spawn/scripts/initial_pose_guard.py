#!/usr/bin/env python3
"""
Hold a robot model at a deterministic initial pose for a short window after spawn.

This reduces startup drift / yaw deviation caused by physics transients before
stand-up and autonomy are active.
"""
import math
import time

import rclpy
from gazebo_msgs.msg import EntityState
from gazebo_msgs.srv import SetEntityState
from rcl_interfaces.msg import ParameterDescriptor
from rclpy.node import Node


class InitialPoseGuard(Node):
    def __init__(self) -> None:
        super().__init__("initial_pose_guard")
        dyn = ParameterDescriptor(dynamic_typing=True)

        self.declare_parameter("entity_name", "robot_a", dyn)
        self.declare_parameter("spawn_x", 0.0, dyn)
        self.declare_parameter("spawn_y", 0.0, dyn)
        self.declare_parameter("spawn_z", 0.38, dyn)
        self.declare_parameter("spawn_yaw", 0.0, dyn)
        self.declare_parameter("hold_sec", 14.0, dyn)
        self.declare_parameter("rate", 15.0, dyn)
        self.declare_parameter("request_timeout_sec", 0.8, dyn)
        self.declare_parameter("max_failures", 30, dyn)
        self.declare_parameter("retry_backoff_initial_sec", 0.1, dyn)
        self.declare_parameter("retry_backoff_max_sec", 1.5, dyn)

        self.entity_name = str(self.get_parameter("entity_name").value)
        self.spawn_x = self._as_float("spawn_x", 0.0)
        self.spawn_y = self._as_float("spawn_y", 0.0)
        self.spawn_z = self._as_float("spawn_z", 0.38)
        self.spawn_yaw = self._as_float("spawn_yaw", 0.0)
        self.hold_sec = max(0.5, self._as_float("hold_sec", 14.0))
        self.rate = max(2.0, self._as_float("rate", 15.0))
        self.request_timeout_sec = max(0.2, self._as_float("request_timeout_sec", 0.8))
        self.max_failures = max(5, self._as_int("max_failures", 30))
        self.retry_backoff_initial_sec = max(0.02, self._as_float("retry_backoff_initial_sec", 0.1))
        self.retry_backoff_max_sec = max(
            self.retry_backoff_initial_sec,
            self._as_float("retry_backoff_max_sec", 1.5),
        )

        self.client_primary = self.create_client(SetEntityState, "/gazebo/set_entity_state")
        self.client_fallback = self.create_client(SetEntityState, "/set_entity_state")
        self.hold_start_time = None
        self._last_warn_wall = 0.0
        self._in_flight = False
        self._pending_sent_wall = 0.0
        self._pending_client_name = ""
        self._try_primary_first = True
        self._next_try_wall = 0.0
        self._backoff_sec = self.retry_backoff_initial_sec
        self._last_probe_call_wall = 0.0
        self._consecutive_failures = 0
        self._success_samples = 0
        self._failed_samples = 0
        self.done = False

        self.timer = self.create_timer(1.0 / self.rate, self.on_timer)
        self.get_logger().info(
            f"InitialPoseGuard active for '{self.entity_name}' at "
            f"({self.spawn_x:.2f}, {self.spawn_y:.2f}, yaw={self.spawn_yaw:.2f}) "
            f"for {self.hold_sec:.1f}s"
        )

    def _as_float(self, name: str, default: float) -> float:
        value = self.get_parameter(name).value
        try:
            return float(value)
        except (TypeError, ValueError):
            self.get_logger().warn(
                f"Invalid float parameter '{name}'={value!r}; using default {default}."
            )
            return float(default)

    def _as_int(self, name: str, default: int) -> int:
        value = self.get_parameter(name).value
        try:
            return int(value)
        except (TypeError, ValueError):
            self.get_logger().warn(
                f"Invalid int parameter '{name}'={value!r}; using default {default}."
            )
            return int(default)

    def on_timer(self) -> None:
        if self.done:
            return

        wall_now = time.monotonic()
        if wall_now < self._next_try_wall:
            return

        if self._in_flight:
            if wall_now - self._pending_sent_wall > self.request_timeout_sec:
                self._in_flight = False
                self._failed_samples += 1
                self._consecutive_failures += 1
                self._try_primary_first = not self._try_primary_first
                self._next_try_wall = wall_now + self._backoff_sec
                self._backoff_sec = min(self._backoff_sec * 1.8, self.retry_backoff_max_sec)
                self.get_logger().warn(
                    f"SetEntityState request timeout via {self._pending_client_name}; retrying in "
                    f"{self._next_try_wall - wall_now:.2f}s."
                )
                if self._success_samples == 0 and self._failed_samples >= self.max_failures:
                    self.done = True
                    self.timer.cancel()
                    self.get_logger().warn(
                        "InitialPoseGuard disabled after repeated SetEntityState timeouts "
                        f"({self._failed_samples} failures)."
                    )
                    return
            else:
                return

        req = SetEntityState.Request()
        req.state = EntityState()
        req.state.name = self.entity_name
        req.state.reference_frame = "world"
        req.state.pose.position.x = self.spawn_x
        req.state.pose.position.y = self.spawn_y
        req.state.pose.position.z = self.spawn_z
        req.state.pose.orientation.w = math.cos(self.spawn_yaw / 2.0)
        req.state.pose.orientation.z = math.sin(self.spawn_yaw / 2.0)
        req.state.twist.linear.x = 0.0
        req.state.twist.linear.y = 0.0
        req.state.twist.linear.z = 0.0
        req.state.twist.angular.x = 0.0
        req.state.twist.angular.y = 0.0
        req.state.twist.angular.z = 0.0

        clients = [
            (self.client_primary, "/gazebo/set_entity_state"),
            (self.client_fallback, "/set_entity_state"),
        ]
        if not self._try_primary_first:
            clients.reverse()

        ready_clients = []
        for client, label in clients:
            if client.wait_for_service(timeout_sec=0.0):
                ready_clients.append((client, label))

        for client, label in ready_clients:
            try:
                self._in_flight = True
                self._pending_sent_wall = time.monotonic()
                self._pending_client_name = label
                future = client.call_async(req)
                future.add_done_callback(self._on_set_state_done)
                return
            except RuntimeError:
                self._failed_samples += 1
                self._consecutive_failures += 1
                self._next_try_wall = wall_now + self._backoff_sec
                self._backoff_sec = min(self._backoff_sec * 1.8, self.retry_backoff_max_sec)
                continue

        # Probe-call fallback: some graph states report service not-ready while
        # the server can still answer; attempt periodically to break deadlock.
        if wall_now - self._last_probe_call_wall >= 2.0:
            self._last_probe_call_wall = wall_now
            for client, label in clients:
                try:
                    self._in_flight = True
                    self._pending_sent_wall = time.monotonic()
                    self._pending_client_name = label
                    future = client.call_async(req)
                    future.add_done_callback(self._on_set_state_done)
                    return
                except RuntimeError:
                    self._failed_samples += 1
                    self._consecutive_failures += 1
                    self._next_try_wall = wall_now + self._backoff_sec
                    self._backoff_sec = min(self._backoff_sec * 1.8, self.retry_backoff_max_sec)
                    continue

        if ready_clients:
            return

        if wall_now - self._last_warn_wall > 2.0:
            self.get_logger().warn("Waiting for SetEntityState service readiness...")
            self._last_warn_wall = wall_now
        if self._success_samples == 0 and self._failed_samples >= self.max_failures:
            self.done = True
            self.timer.cancel()
            self.get_logger().warn(
                "InitialPoseGuard disabled after repeated SetEntityState failures "
                f"({self._failed_samples})."
            )

    def _on_set_state_done(self, future) -> None:
        self._in_flight = False
        if self.done:
            return

        try:
            result = future.result()
        except Exception as exc:  # noqa: BLE001
            self._failed_samples += 1
            self._consecutive_failures += 1
            self._next_try_wall = time.monotonic() + self._backoff_sec
            self._backoff_sec = min(self._backoff_sec * 1.8, self.retry_backoff_max_sec)
            self.get_logger().warn(
                f"SetEntityState call failed: {exc}; retrying in {self._backoff_sec:.2f}s"
            )
            return

        if not bool(getattr(result, "success", False)):
            self._failed_samples += 1
            self._consecutive_failures += 1
            self._next_try_wall = time.monotonic() + self._backoff_sec
            self._backoff_sec = min(self._backoff_sec * 1.8, self.retry_backoff_max_sec)
            status_message = str(getattr(result, "status_message", "")).strip()
            if not status_message:
                status_message = str(getattr(result, "message", "")).strip()
            if not status_message:
                status_message = "service returned success=false"
            self.get_logger().warn(f"SetEntityState success=false: {status_message}")
            return

        self._success_samples += 1
        self._consecutive_failures = 0
        self._backoff_sec = self.retry_backoff_initial_sec
        self._next_try_wall = 0.0
        now = self.get_clock().now()
        if self.hold_start_time is None:
            self.hold_start_time = now
            self.get_logger().info("SetEntityState confirmed; enforcing initial pose.")

        elapsed = (now - self.hold_start_time).nanoseconds / 1e9
        if elapsed < self.hold_sec:
            return

        # Require at least a few successful writes so we don't declare success
        # after a single accepted request.
        if self._success_samples < 5:
            return

        if self._failed_samples > 0:
            self.get_logger().warn(
                f"InitialPoseGuard saw {self._failed_samples} failed SetEntityState calls "
                f"before completion."
            )

        if elapsed >= self.hold_sec:
            self.done = True
            self.timer.cancel()
            self.get_logger().info("InitialPoseGuard complete.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = InitialPoseGuard()
    try:
        # Exit as soon as hold/enforcement is complete so launch OnProcessExit
        # handlers can gate downstream planner startup.
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.2)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
