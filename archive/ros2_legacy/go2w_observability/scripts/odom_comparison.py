#!/usr/bin/env python3
"""
Compares ground-truth odometry with SLAM odometry.
Logs drift metrics to CSV for quantitative analysis.

Subscribes:
    /odom/ground_truth  (nav_msgs/Odometry) - Gazebo p3d ground truth
    /aft_mapped_to_init (nav_msgs/Odometry) - Point-LIO SLAM output

Logs to: /tmp/odom_comparison_<timestamp>.csv
"""
import math
import time

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry


class OdomComparison(Node):
    def __init__(self):
        super().__init__("odom_comparison")

        self.declare_parameter("gt_topic", "/odom/ground_truth")
        self.declare_parameter("slam_topic", "/aft_mapped_to_init")
        self.declare_parameter("report_rate", 1.0)

        gt_topic = self.get_parameter("gt_topic").value
        slam_topic = self.get_parameter("slam_topic").value
        report_rate = float(self.get_parameter("report_rate").value)

        self.gt_pose = None   # (x, y, z, yaw)
        self.slam_pose = None

        self.create_subscription(Odometry, gt_topic, self.gt_cb, 10)
        self.create_subscription(Odometry, slam_topic, self.slam_cb, 10)

        self.csv_path = f"/tmp/odom_comparison_{int(time.time())}.csv"
        with open(self.csv_path, "w") as f:
            f.write(
                "t,gt_x,gt_y,gt_z,gt_yaw,"
                "slam_x,slam_y,slam_z,slam_yaw,"
                "err_x,err_y,err_dist,err_yaw_deg\n"
            )
        self.get_logger().info(f"Logging odom comparison to {self.csv_path}")
        self.get_logger().info(f"  GT topic:   {gt_topic}")
        self.get_logger().info(f"  SLAM topic: {slam_topic}")

        self.timer = self.create_timer(1.0 / report_rate, self.compare)

    @staticmethod
    def _yaw(q) -> float:
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                          1.0 - 2.0 * (q.y ** 2 + q.z ** 2))

    def gt_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.gt_pose = (p.x, p.y, p.z, self._yaw(msg.pose.pose.orientation))

    def slam_cb(self, msg: Odometry):
        p = msg.pose.pose.position
        self.slam_pose = (p.x, p.y, p.z, self._yaw(msg.pose.pose.orientation))

    def compare(self):
        if self.gt_pose is None:
            self.get_logger().info("Waiting for ground-truth odom...")
            return
        if self.slam_pose is None:
            self.get_logger().info("Waiting for SLAM odom...")
            return

        gt = self.gt_pose
        sl = self.slam_pose
        err_x = sl[0] - gt[0]
        err_y = sl[1] - gt[1]
        err_dist = math.hypot(err_x, err_y)
        err_yaw = math.degrees(sl[3] - gt[3])
        # Wrap yaw error to [-180, 180]
        while err_yaw > 180.0:
            err_yaw -= 360.0
        while err_yaw < -180.0:
            err_yaw += 360.0

        self.get_logger().info(
            f"GT=({gt[0]:.2f},{gt[1]:.2f}) "
            f"SLAM=({sl[0]:.2f},{sl[1]:.2f}) "
            f"Err={err_dist:.3f}m yaw_err={err_yaw:.1f}°"
        )

        now = self.get_clock().now().nanoseconds / 1e9
        with open(self.csv_path, "a") as f:
            f.write(
                f"{now:.3f},"
                f"{gt[0]:.4f},{gt[1]:.4f},{gt[2]:.4f},{gt[3]:.4f},"
                f"{sl[0]:.4f},{sl[1]:.4f},{sl[2]:.4f},{sl[3]:.4f},"
                f"{err_x:.4f},{err_y:.4f},{err_dist:.4f},{err_yaw:.2f}\n"
            )


def main(args=None):
    rclpy.init(args=args)
    node = OdomComparison()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
