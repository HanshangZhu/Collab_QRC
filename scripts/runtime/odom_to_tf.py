#!/usr/bin/env python3
"""odom_to_tf.py — republish /robot/Odometry as TF (camera_init→body).

The NX's hil_relay_tx only sends Odometry; it doesn't forward the ROS 1 /tf
stream. This node reconstructs the TF from the Odometry message so RViz2 can
show the robot's position. Reads frame_id / child_frame_id from the message
so it works for both Point-LIO (camera_init→body) and any other SLAM.

Usage:
  ros2 run ... odom_to_tf  # or python3 scripts/runtime/odom_to_tf.py
"""
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class OdomToTf(Node):
    def __init__(self):
        super().__init__('odom_to_tf')
        self.br = TransformBroadcaster(self)
        self.sub = self.create_subscription(
            Odometry, '/robot/Odometry', self._cb, 10)
        self.get_logger().info('odom_to_tf: /robot/Odometry → TF')

    def _cb(self, msg: Odometry):
        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = msg.header.frame_id          # camera_init
        t.child_frame_id = msg.child_frame_id             # body
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation
        self.br.sendTransform(t)


def main():
    rclpy.init()
    node = OdomToTf()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    rclpy.shutdown()


if __name__ == '__main__':
    main()
