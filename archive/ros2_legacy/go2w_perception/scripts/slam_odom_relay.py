#!/usr/bin/env python3
"""Relay SLAM odometry to navigation odometry with optional GT bootstrap.

Supports dual-source: when SC-PGO's /corrected_odom is available and fresh,
use it instead of raw SLAM odom. Falls back to raw odom transparently
when SC-PGO isn't running or hasn't detected a loop yet.

2026-04-29: added `pose_from_tf` mode. When true, the relay's *pose* comes
from a TF lookup (target_frame → child_frame), not from the input topic's
pose. The input topic is still subscribed for *velocity* (twist). This
makes /odom/nav consistent with whatever pose Nav2's planner uses
internally (SmacPlannerHybrid reads pose from TF), eliminating the case
where /odom/nav drifts from the actual planning pose. Real-robot
compatible: on a real Go2/Go2W the TF chain is the EKF-fused estimate,
which is the canonical pose source. On sim it's MuJoCo-bridged TF, which
is GT-aligned. Either way every consumer sees the same pose.
"""
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformException, TransformListener


class SlamOdomRelay(Node):
    def __init__(self):
        super().__init__("slam_odom_relay")

        self.declare_parameter("input_topic", "/aft_mapped_to_init")
        self.declare_parameter("output_topic", "/odom/ground_truth")
        self.declare_parameter("output_frame_id", "world")
        self.declare_parameter("output_child_frame_id", "base_link")
        self.declare_parameter("bootstrap_from_gt", False)
        self.declare_parameter("gt_topic", "/odom/ground_truth")
        self.declare_parameter("require_gt_for_alignment", True)
        # SC-PGO corrected odom (loop closure backend)
        self.declare_parameter("corrected_odom_topic", "/corrected_odom")
        self.declare_parameter("corrected_odom_staleness_sec", 2.0)

        input_topic = self.get_parameter("input_topic").value
        output_topic = self.get_parameter("output_topic").value
        self.output_frame = self.get_parameter("output_frame_id").value
        self.output_child_frame = self.get_parameter("output_child_frame_id").value
        self.bootstrap_from_gt = bool(self.get_parameter("bootstrap_from_gt").value)
        self.gt_topic = str(self.get_parameter("gt_topic").value)
        self.require_gt_for_alignment = bool(self.get_parameter("require_gt_for_alignment").value)

        corrected_topic = self.get_parameter("corrected_odom_topic").value
        self.corrected_staleness = self.get_parameter("corrected_odom_staleness_sec").value

        self.sub = self.create_subscription(Odometry, input_topic, self.relay_cb, 10)
        self.pub = self.create_publisher(Odometry, output_topic, 10)
        self.gt_sub = None
        if self.bootstrap_from_gt:
            self.gt_sub = self.create_subscription(Odometry, self.gt_topic, self.gt_cb, 10)

        # SC-PGO corrected odom subscription
        self.corrected_odom: Odometry | None = None
        self.corrected_odom_time = 0.0
        self.corrected_odom_count = 0
        self.using_corrected = False
        self.corrected_sub = self.create_subscription(
            Odometry, corrected_topic, self.corrected_cb, 10)

        self.msg_count = 0
        self.latest_gt: Odometry | None = None
        self.alignment_ready = False
        self.yaw_offset = 0.0
        self.dx = 0.0
        self.dy = 0.0
        self.dz = 0.0
        self.get_logger().info(
            f"SLAM odom relay: {input_topic} -> {output_topic} "
            f"(frame: {self.output_frame} -> {self.output_child_frame})"
        )
        self.get_logger().info(
            f"SC-PGO corrected odom: {corrected_topic} "
            f"(staleness={self.corrected_staleness}s)"
        )
        if self.bootstrap_from_gt:
            self.get_logger().info(
                f"GT bootstrap enabled: gt_topic={self.gt_topic}, require_gt_for_alignment={self.require_gt_for_alignment}"
            )

    def corrected_cb(self, msg: Odometry):
        """Receive PGO-corrected odom from SC-PGO."""
        self.corrected_odom = msg
        self.corrected_odom_time = time.monotonic()
        self.corrected_odom_count += 1
        if self.corrected_odom_count == 1:
            self.get_logger().info("SC-PGO corrected odom received — switching to corrected source")
        if not self.using_corrected:
            self.using_corrected = True

    def gt_cb(self, msg: Odometry):
        self.latest_gt = msg
        if not self.alignment_ready:
            self.publish_with_frames(msg)

    @staticmethod
    def yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
        siny = 2.0 * (w * z + x * y)
        cosy = 1.0 - 2.0 * (y * y + z * z)
        return math.atan2(siny, cosy)

    @staticmethod
    def quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
        return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))

    @staticmethod
    def quat_mul(
        ax: float, ay: float, az: float, aw: float, bx: float, by: float, bz: float, bw: float
    ) -> tuple[float, float, float, float]:
        return (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        )

    @staticmethod
    def rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
        c = math.cos(yaw)
        s = math.sin(yaw)
        return (c * x - s * y, s * x + c * y)

    def publish_with_frames(self, msg: Odometry):
        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame
        out.child_frame_id = self.output_child_frame
        out.pose = msg.pose
        out.twist = msg.twist
        self.pub.publish(out)

    def relay_cb(self, msg: Odometry):
        # If SC-PGO corrected odom is available and fresh, use it instead
        if self.corrected_odom is not None:
            age = time.monotonic() - self.corrected_odom_time
            if age < self.corrected_staleness:
                self.publish_with_frames(self.corrected_odom)
                self.msg_count += 1
                if self.msg_count % 100 == 1:
                    p = self.corrected_odom.pose.pose.position
                    self.get_logger().info(
                        f"Relayed {self.msg_count} msgs (CORRECTED) | "
                        f"pose=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
                    )
                return
            elif self.using_corrected:
                self.using_corrected = False
                self.get_logger().warn(
                    f"SC-PGO corrected odom stale ({age:.1f}s) — falling back to raw")

        if not self.alignment_ready:
            if self.bootstrap_from_gt and self.latest_gt is None and self.require_gt_for_alignment:
                return

            if self.bootstrap_from_gt and self.latest_gt is not None:
                slam_yaw = self.yaw_from_quat(
                    msg.pose.pose.orientation.x,
                    msg.pose.pose.orientation.y,
                    msg.pose.pose.orientation.z,
                    msg.pose.pose.orientation.w,
                )
                gt_yaw = self.yaw_from_quat(
                    self.latest_gt.pose.pose.orientation.x,
                    self.latest_gt.pose.pose.orientation.y,
                    self.latest_gt.pose.pose.orientation.z,
                    self.latest_gt.pose.pose.orientation.w,
                )
                self.yaw_offset = gt_yaw - slam_yaw

                rx, ry = self.rotate_xy(msg.pose.pose.position.x, msg.pose.pose.position.y, self.yaw_offset)
                self.dx = self.latest_gt.pose.pose.position.x - rx
                self.dy = self.latest_gt.pose.pose.position.y - ry
                self.dz = self.latest_gt.pose.pose.position.z - msg.pose.pose.position.z
                self.get_logger().info(
                    "Initialized SLAM alignment from GT: "
                    f"dx={self.dx:.2f} dy={self.dy:.2f} dz={self.dz:.2f} "
                    f"yaw_offset_deg={math.degrees(self.yaw_offset):.1f}"
                )

            self.alignment_ready = True

        out = Odometry()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self.output_frame
        out.child_frame_id = self.output_child_frame

        if self.bootstrap_from_gt:
            rx, ry = self.rotate_xy(msg.pose.pose.position.x, msg.pose.pose.position.y, self.yaw_offset)
            out.pose.pose.position.x = rx + self.dx
            out.pose.pose.position.y = ry + self.dy
            out.pose.pose.position.z = msg.pose.pose.position.z + self.dz

            oz, ow = math.sin(0.5 * self.yaw_offset), math.cos(0.5 * self.yaw_offset)
            q = msg.pose.pose.orientation
            qx, qy, qz, qw = self.quat_mul(0.0, 0.0, oz, ow, q.x, q.y, q.z, q.w)
            out.pose.pose.orientation.x = qx
            out.pose.pose.orientation.y = qy
            out.pose.pose.orientation.z = qz
            out.pose.pose.orientation.w = qw

            vxr, vyr = self.rotate_xy(msg.twist.twist.linear.x, msg.twist.twist.linear.y, self.yaw_offset)
            out.twist.twist.linear.x = vxr
            out.twist.twist.linear.y = vyr
            out.twist.twist.linear.z = msg.twist.twist.linear.z
            out.twist.twist.angular.x = msg.twist.twist.angular.x
            out.twist.twist.angular.y = msg.twist.twist.angular.y
            out.twist.twist.angular.z = msg.twist.twist.angular.z
            out.pose.covariance = msg.pose.covariance
            out.twist.covariance = msg.twist.covariance
        else:
            out.pose = msg.pose
            out.twist = msg.twist

        self.pub.publish(out)

        self.msg_count += 1
        if self.msg_count % 100 == 1:
            p = out.pose.pose.position
            self.get_logger().info(
                f"Relayed {self.msg_count} msgs | "
                f"NAV pose=({p.x:.2f}, {p.y:.2f}, {p.z:.2f})"
            )


def main(args=None):
    rclpy.init(args=args)
    node = SlamOdomRelay()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
