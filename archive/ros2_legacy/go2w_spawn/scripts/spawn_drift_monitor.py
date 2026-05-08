#!/usr/bin/env python3
"""Monitor Gazebo entity drift from a target spawn pose during startup."""

import math
import time

import rclpy
from gazebo_msgs.msg import ModelStates
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data


class SpawnDriftMonitor(Node):
    def __init__(self) -> None:
        super().__init__("spawn_drift_monitor")

        self.declare_parameter("entity_name", "robot_a")
        self.declare_parameter("spawn_x", 0.0)
        self.declare_parameter("spawn_y", 0.0)
        self.declare_parameter("spawn_z", 0.40)
        self.declare_parameter("spawn_yaw", 0.0)
        self.declare_parameter("sample_rate", 20.0)
        self.declare_parameter("settle_sec", 1.0)
        self.declare_parameter("analysis_duration_sec", 12.0)
        self.declare_parameter("log_rate_hz", 1.0)
        self.declare_parameter("model_states_topic", "/gazebo/model_states")
        self.declare_parameter("model_states_topic_fallback", "/model_states")

        self.entity_name = str(self.get_parameter("entity_name").value)
        self.spawn_x = float(self.get_parameter("spawn_x").value)
        self.spawn_y = float(self.get_parameter("spawn_y").value)
        self.spawn_z = float(self.get_parameter("spawn_z").value)
        self.spawn_yaw = float(self.get_parameter("spawn_yaw").value)
        self.sample_rate = max(1.0, float(self.get_parameter("sample_rate").value))
        self.settle_sec = max(0.0, float(self.get_parameter("settle_sec").value))
        self.analysis_duration_sec = max(2.0, float(self.get_parameter("analysis_duration_sec").value))
        self.log_rate_hz = max(0.2, float(self.get_parameter("log_rate_hz").value))
        self.model_states_topic = str(self.get_parameter("model_states_topic").value)
        self.model_states_topic_fallback = str(self.get_parameter("model_states_topic_fallback").value)

        self._latest_pose = None
        self._first_good_time = None
        self._last_log_wall = 0.0
        self._last_wait_log_wall = 0.0
        self._done = False
        self._source_topic = None
        self._seen_model_states = False
        self._names_logged = False

        self.total_samples = 0
        self.settled_samples = 0

        self.max_xy = 0.0
        self.max_z = 0.0
        self.max_yaw_deg = 0.0

        self.max_xy_settled = 0.0
        self.max_z_settled = 0.0
        self.max_yaw_deg_settled = 0.0

        self.model_states_sub = self.create_subscription(
            ModelStates,
            self.model_states_topic,
            self._on_model_states_primary,
            qos_profile_sensor_data,
        )
        self.model_states_sub_fallback = self.create_subscription(
            ModelStates,
            self.model_states_topic_fallback,
            self._on_model_states_fallback,
            qos_profile_sensor_data,
        )
        self.timer = self.create_timer(1.0 / self.sample_rate, self._tick)
        self.get_logger().info(
            "SpawnDriftMonitor started for '%s' target=(%.3f, %.3f, %.3f, yaw=%.3f) "
            "duration=%.1fs settle=%.1fs"
            % (
                self.entity_name,
                self.spawn_x,
                self.spawn_y,
                self.spawn_z,
                self.spawn_yaw,
                self.analysis_duration_sec,
                self.settle_sec,
            )
        )

    @staticmethod
    def _wrap_angle(a: float) -> float:
        return math.atan2(math.sin(a), math.cos(a))

    @staticmethod
    def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
        siny_cosp = 2.0 * (w * z + x * y)
        cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny_cosp, cosy_cosp)

    def _on_model_states_primary(self, msg: ModelStates) -> None:
        self._on_model_states(msg, self.model_states_topic)

    def _on_model_states_fallback(self, msg: ModelStates) -> None:
        self._on_model_states(msg, self.model_states_topic_fallback)

    def _on_model_states(self, msg: ModelStates, source_topic: str) -> None:
        if self._done:
            return
        self._seen_model_states = True
        try:
            model_idx = msg.name.index(self.entity_name)
        except ValueError:
            if not self._names_logged and msg.name:
                preview = ", ".join(msg.name[:8])
                self.get_logger().info(f"ModelStates seen on {source_topic}; sample names: {preview}")
                self._names_logged = True
            return
        self._latest_pose = msg.pose[model_idx]
        if self._source_topic != source_topic:
            self._source_topic = source_topic
            self.get_logger().info(f"Using model states topic: {self._source_topic}")

    def _tick(self) -> None:
        if self._done:
            return

        if self._latest_pose is None:
            now_wall = time.monotonic()
            if now_wall - self._last_wait_log_wall > 2.0:
                if self._seen_model_states:
                    self.get_logger().warn(
                        f"ModelStates active, waiting for entity '{self.entity_name}' to appear..."
                    )
                else:
                    self.get_logger().warn(
                        f"Waiting for ModelStates on {self.model_states_topic} or "
                        f"{self.model_states_topic_fallback}..."
                    )
                self._last_wait_log_wall = now_wall
            return

        now = self.get_clock().now()
        if self._first_good_time is None:
            self._first_good_time = now
            self.get_logger().info("First valid entity pose received from Gazebo. Starting drift analysis window.")

        elapsed = (now - self._first_good_time).nanoseconds / 1e9

        pose = self._latest_pose
        dx = pose.position.x - self.spawn_x
        dy = pose.position.y - self.spawn_y
        dz = pose.position.z - self.spawn_z
        xy = math.hypot(dx, dy)

        yaw = self._yaw_from_quat(
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        )
        yaw_err = self._wrap_angle(yaw - self.spawn_yaw)
        yaw_err_deg = abs(math.degrees(yaw_err))

        self.total_samples += 1
        self.max_xy = max(self.max_xy, xy)
        self.max_z = max(self.max_z, abs(dz))
        self.max_yaw_deg = max(self.max_yaw_deg, yaw_err_deg)

        if elapsed >= self.settle_sec:
            self.settled_samples += 1
            self.max_xy_settled = max(self.max_xy_settled, xy)
            self.max_z_settled = max(self.max_z_settled, abs(dz))
            self.max_yaw_deg_settled = max(self.max_yaw_deg_settled, yaw_err_deg)

        now_wall = time.monotonic()
        if now_wall - self._last_log_wall >= 1.0 / self.log_rate_hz:
            self.get_logger().info(
                "t=%.2fs drift: xy=%.3fm z=%.3fm yaw=%.2fdeg"
                % (elapsed, xy, abs(dz), yaw_err_deg)
            )
            self._last_log_wall = now_wall

        if elapsed >= self.analysis_duration_sec:
            self._done = True
            self.timer.cancel()
            self._log_summary()

    def _log_summary(self) -> None:
        self.get_logger().info("=== Spawn Drift Summary (%s) ===" % self.entity_name)
        self.get_logger().info(
            "all samples=%d max_xy=%.3fm max_|z|=%.3fm max_|yaw|=%.2fdeg"
            % (self.total_samples, self.max_xy, self.max_z, self.max_yaw_deg)
        )
        if self.settled_samples > 0:
            self.get_logger().info(
                "post-settle samples=%d max_xy=%.3fm max_|z|=%.3fm max_|yaw|=%.2fdeg"
                % (
                    self.settled_samples,
                    self.max_xy_settled,
                    self.max_z_settled,
                    self.max_yaw_deg_settled,
                )
            )
        else:
            self.get_logger().warn("No post-settle samples collected; increase analysis_duration_sec.")


def main(args=None) -> None:
    rclpy.init(args=args)
    node = SpawnDriftMonitor()
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
