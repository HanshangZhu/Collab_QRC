#!/usr/bin/env python3
"""Publish nav_msgs/Odometry from Cartographer's TF (map → body).

Cartographer publishes TF: map → odom → body.
This node looks up the full map → body transform and publishes Odometry,
computing velocity from position deltas.  Designed to be lightweight (~50 Hz).
"""

import math
import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from tf2_ros import Buffer, TransformListener, LookupException, ConnectivityException, ExtrapolationException


class CartoOdomBridge(Node):
    def __init__(self):
        super().__init__('carto_odom_bridge')

        self.declare_parameter('parent_frame', 'map')
        self.declare_parameter('child_frame', 'body')
        self.declare_parameter('output_topic', '/robot/odom/nav')
        self.declare_parameter('output_frame_id', 'map')
        self.declare_parameter('output_child_frame_id', 'base_link')
        self.declare_parameter('rate', 50.0)

        parent = self.get_parameter('parent_frame').value
        child = self.get_parameter('child_frame').value
        output_topic = self.get_parameter('output_topic').value
        self.output_frame_id = self.get_parameter('output_frame_id').value
        self.output_child_frame_id = self.get_parameter('output_child_frame_id').value
        rate = max(1.0, float(self.get_parameter('rate').value))

        self.parent_frame = str(parent)
        self.child_frame = str(child)

        self.tf_buffer = Buffer(node=self)
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.odom_pub = self.create_publisher(Odometry, str(output_topic), 10)

        self.prev_x = None
        self.prev_y = None
        self.prev_yaw = None
        self.prev_time_sec = None

        self.timer = self.create_timer(1.0 / rate, self._tick)
        self.get_logger().info(
            f'Carto odom bridge: TF({self.parent_frame}→{self.child_frame}) '
            f'→ {output_topic} at {rate:.0f} Hz')

    def _tick(self):
        try:
            t = self.tf_buffer.lookup_transform(
                self.parent_frame, self.child_frame, rclpy.time.Time())
        except (LookupException, ConnectivityException, ExtrapolationException):
            return

        now_sec = self.get_clock().now().nanoseconds / 1e9

        x = t.transform.translation.x
        y = t.transform.translation.y
        z = t.transform.translation.z
        qx = t.transform.rotation.x
        qy = t.transform.rotation.y
        qz = t.transform.rotation.z
        qw = t.transform.rotation.w

        yaw = math.atan2(2.0 * (qw * qz + qx * qy),
                         1.0 - 2.0 * (qy * qy + qz * qz))

        # Compute velocity from deltas
        vx = 0.0
        wz = 0.0
        if self.prev_x is not None and self.prev_time_sec is not None:
            dt = now_sec - self.prev_time_sec
            if dt > 1e-6:
                dx = x - self.prev_x
                dy = y - self.prev_y
                # Velocity in body frame
                cos_y = math.cos(yaw)
                sin_y = math.sin(yaw)
                vx = cos_y * dx / dt + sin_y * dy / dt
                dyaw = math.atan2(math.sin(yaw - self.prev_yaw),
                                  math.cos(yaw - self.prev_yaw))
                wz = dyaw / dt

        self.prev_x = x
        self.prev_y = y
        self.prev_yaw = yaw
        self.prev_time_sec = now_sec

        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.output_frame_id
        msg.child_frame_id = self.output_child_frame_id
        msg.pose.pose.position.x = x
        msg.pose.pose.position.y = y
        msg.pose.pose.position.z = z
        msg.pose.pose.orientation.x = qx
        msg.pose.pose.orientation.y = qy
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        msg.twist.twist.linear.x = vx
        msg.twist.twist.angular.z = wz

        self.odom_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = CartoOdomBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
