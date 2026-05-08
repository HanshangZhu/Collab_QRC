#!/usr/bin/env python3
"""
Spawn an entity directly via gazebo_msgs/SpawnEntity.

This bypasses gazebo_ros/spawn_entity.py so robot_namespace and request content
are fully explicit per robot instance.
"""

import argparse
import math
import time

import rclpy
from gazebo_msgs.srv import SpawnEntity
from geometry_msgs.msg import Pose
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from std_msgs.msg import String


def _quat_from_rpy(roll: float, pitch: float, yaw: float) -> tuple[float, float, float, float]:
    cy = math.cos(yaw * 0.5)
    sy = math.sin(yaw * 0.5)
    cp = math.cos(pitch * 0.5)
    sp = math.sin(pitch * 0.5)
    cr = math.cos(roll * 0.5)
    sr = math.sin(roll * 0.5)
    qw = cr * cp * cy + sr * sp * sy
    qx = sr * cp * cy - cr * sp * sy
    qy = cr * sp * cy + sr * cp * sy
    qz = cr * cp * sy - sr * sp * cy
    return qw, qx, qy, qz


class DirectSpawner(Node):
    def __init__(self, args: argparse.Namespace):
        super().__init__(f"spawn_entity_direct_{args.entity}")
        self.args = args
        self._entity_xml: str | None = None
        qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(String, self.args.topic, self._xml_cb, qos)
        self._spawn_client = self.create_client(SpawnEntity, self.args.spawn_service)

    def _xml_cb(self, msg: String) -> None:
        if self._entity_xml is None:
            self._entity_xml = msg.data

    def wait_for_xml(self, timeout_sec: float) -> bool:
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline:
            if self._entity_xml:
                return True
            rclpy.spin_once(self, timeout_sec=0.2)
        return False

    def spawn(self, timeout_sec: float) -> bool:
        if not self._entity_xml:
            self.get_logger().error("No entity XML received; cannot spawn.")
            return False

        if not self._spawn_client.wait_for_service(timeout_sec=timeout_sec):
            self.get_logger().error(
                f"Service {self.args.spawn_service} unavailable after {timeout_sec:.1f}s."
            )
            return False

        pose = Pose()
        pose.position.x = self.args.x
        pose.position.y = self.args.y
        pose.position.z = self.args.z
        qw, qx, qy, qz = _quat_from_rpy(self.args.roll, self.args.pitch, self.args.yaw)
        pose.orientation.w = qw
        pose.orientation.x = qx
        pose.orientation.y = qy
        pose.orientation.z = qz

        req = SpawnEntity.Request()
        req.name = self.args.entity
        req.xml = self._entity_xml
        req.robot_namespace = self.args.robot_namespace
        req.initial_pose = pose
        req.reference_frame = self.args.reference_frame

        self.get_logger().info(
            f"Spawning '{req.name}' via {self.args.spawn_service} with "
            f"robot_namespace='{req.robot_namespace}' topic='{self.args.topic}'"
        )
        future = self._spawn_client.call_async(req)
        deadline = time.monotonic() + timeout_sec
        while rclpy.ok() and time.monotonic() < deadline and not future.done():
            rclpy.spin_once(self, timeout_sec=0.2)

        if not future.done():
            self.get_logger().error("SpawnEntity call timed out.")
            return False

        result = future.result()
        if result is None:
            self.get_logger().error("SpawnEntity call failed with no result.")
            return False

        if not result.success:
            self.get_logger().error(f"SpawnEntity failed: {result.status_message}")
            return False

        self.get_logger().info(result.status_message)
        return True


def main() -> int:
    parser = argparse.ArgumentParser(description="Direct SpawnEntity client for Gazebo")
    parser.add_argument("--entity", required=True, type=str)
    parser.add_argument("--topic", required=True, type=str)
    parser.add_argument("--robot-namespace", default="", type=str)
    parser.add_argument("--spawn-service", default="/spawn_entity", type=str)
    parser.add_argument("--reference-frame", default="world", type=str)
    parser.add_argument("--timeout", default=30.0, type=float)
    parser.add_argument("--x", default=0.0, type=float)
    parser.add_argument("--y", default=0.0, type=float)
    parser.add_argument("--z", default=0.0, type=float)
    parser.add_argument("--roll", default=0.0, type=float)
    parser.add_argument("--pitch", default=0.0, type=float)
    parser.add_argument("--yaw", default=0.0, type=float)
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = DirectSpawner(args)
    ok = False
    try:
        ok = node.wait_for_xml(timeout_sec=args.timeout)
        if not ok:
            node.get_logger().error(
                f"Timed out waiting for XML on topic '{args.topic}' after {args.timeout:.1f}s."
            )
        else:
            ok = node.spawn(timeout_sec=args.timeout)
    finally:
        node.destroy_node()
        rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    raise SystemExit(main())
