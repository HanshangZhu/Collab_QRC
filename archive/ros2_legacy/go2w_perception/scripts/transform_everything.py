#!/usr/bin/env python3
"""Transform Go2W Utlidar L1 cloud + IMU into the body frame.

Pitches the LiDAR cloud 15.1° to compensate for L1 mounting angle, flips
IMU axes into ROS convention, applies bias + yaw-to-roll/pitch cross-axis
correction (from imu_calib), and optionally low-passes the accelerometer.

Ported from autonomy_stack_go2/utilities/transform_sensors. Calib path is
now a parameter instead of hardcoded ~/Desktop/imu_calib_data.yaml.
"""
import os

import rclpy
import tf_transformations
import yaml
from geometry_msgs.msg import TransformStamped, Vector3
from rclpy.node import Node
from rclpy.time import Time
import sensor_msgs_py.point_cloud2 as pc2
from sensor_msgs.msg import Imu, PointCloud2
from transforms3d.quaternions import quat2mat

import numpy as np


DEFAULT_CALIB = {
    'acc_bias_x': 0.0,
    'acc_bias_y': 0.0,
    'acc_bias_z': 0.0,
    'ang_bias_x': 0.0,
    'ang_bias_y': 0.0,
    'ang_bias_z': 0.0,
    'ang_z2x_proj': 0.15,
    'ang_z2y_proj': -0.28,
}


class TransformEverything(Node):
    def __init__(self):
        super().__init__('transform_everything')

        self.declare_parameter('accel_lpf_enabled', True)
        self.declare_parameter('accel_lpf_alpha', 0.15)
        self.declare_parameter('output_frame_id', 'body')
        self.declare_parameter('imu_calib_yaml', '')

        self.accel_lpf_enabled = bool(self.get_parameter('accel_lpf_enabled').value)
        self.accel_lpf_alpha = float(self.get_parameter('accel_lpf_alpha').value)
        self.output_frame_id = str(self.get_parameter('output_frame_id').value).strip() or 'body'
        calib_path = str(self.get_parameter('imu_calib_yaml').value).strip()

        self.accel_lpf_state = None
        if self.accel_lpf_enabled:
            self.get_logger().info(f'Accel LPF ON alpha={self.accel_lpf_alpha}')

        calib = self._load_calib(calib_path)
        self.acc_bias_x = calib['acc_bias_x']
        self.acc_bias_y = calib['acc_bias_y']
        self.acc_bias_z = calib['acc_bias_z']
        self.ang_bias_x = calib['ang_bias_x']
        self.ang_bias_y = calib['ang_bias_y']
        self.ang_bias_z = calib['ang_bias_z']
        self.ang_z2x_proj = calib['ang_z2x_proj']
        self.ang_z2y_proj = calib['ang_z2y_proj']

        self.cam_offset = 0.046825
        self.time_stamp_offset = 0
        self.time_stamp_offset_set = False

        self.body2cloud_trans = self._make_static_tf('utlidar_lidar_1', pitch=2.87820258505555555556)
        self.body2imu_trans = self._make_static_tf('utlidar_imu_1', pitch=2.87820258505555555556, yaw=3.14159265358)

        # Self-hit filter box (legs / body under LiDAR)
        self.x_filter_min, self.x_filter_max = -0.7, -0.1
        self.y_filter_min, self.y_filter_max = -0.3, 0.3
        self.z_filter_min = -0.6 - self.cam_offset
        self.z_filter_max = 0.0 - self.cam_offset

        self.imu_sub = self.create_subscription(Imu, '/utlidar/imu', self.imu_callback, 50)
        self.cloud_sub = self.create_subscription(PointCloud2, '/utlidar/cloud', self.cloud_callback, 50)
        self.imu_raw_pub = self.create_publisher(Imu, '/utlidar/transformed_raw_imu', 50)
        self.imu_pub = self.create_publisher(Imu, '/utlidar/transformed_imu', 50)
        self.cloud_pub = self.create_publisher(PointCloud2, '/utlidar/transformed_cloud', 50)

    def _load_calib(self, path: str) -> dict:
        if not path:
            path = os.path.expanduser('~/Desktop/imu_calib_data.yaml')
        try:
            with open(path, 'r') as f:
                data = yaml.safe_load(f)
            self.get_logger().info(f'Loaded IMU calib from {path}')
            return data
        except Exception as exc:
            self.get_logger().warn(f'IMU calib load failed ({path}): {exc}. Using defaults.')
            return DEFAULT_CALIB.copy()

    def _make_static_tf(self, child_frame: str, pitch: float = 0.0, yaw: float = 0.0) -> TransformStamped:
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'body'
        tf.child_frame_id = child_frame
        q = tf_transformations.quaternion_from_euler(0.0, pitch, yaw)
        tf.transform.rotation.x = q[0]
        tf.transform.rotation.y = q[1]
        tf.transform.rotation.z = q[2]
        tf.transform.rotation.w = q[3]
        return tf

    def _is_self_hit(self, point) -> bool:
        return (
            self.x_filter_min < point[0] < self.x_filter_max
            and self.y_filter_min < point[1] < self.y_filter_max
            and self.z_filter_min < point[2] < self.z_filter_max
        )

    def cloud_callback(self, data: PointCloud2) -> None:
        if not self.time_stamp_offset_set:
            self.time_stamp_offset = (
                self.get_clock().now().nanoseconds - Time.from_msg(data.header.stamp).nanoseconds
            )
            self.time_stamp_offset_set = True

        points = np.array(pc2.read_points_list(data))
        t = self.body2cloud_trans.transform
        mat = quat2mat(np.array([t.rotation.w, t.rotation.x, t.rotation.y, t.rotation.z]))
        translation = np.array([t.translation.x, t.translation.y, t.translation.z])
        points[:, 0:3] = points[:, 0:3] @ mat.T + translation
        points[:, 2] -= self.cam_offset

        kept = []
        for row in points.tolist():
            row[4] = int(row[4])
            if not self._is_self_hit(row):
                kept.append(row)

        out = pc2.create_cloud(data.header, data.fields, kept)
        out.header.stamp = Time(
            nanoseconds=Time.from_msg(out.header.stamp).nanoseconds + self.time_stamp_offset
        ).to_msg()
        out.header.frame_id = self.output_frame_id
        out.is_dense = data.is_dense
        self.cloud_pub.publish(out)

    def imu_callback(self, data: Imu) -> None:
        rot = np.array([
            self.body2imu_trans.transform.rotation.x,
            self.body2imu_trans.transform.rotation.y,
            self.body2imu_trans.transform.rotation.z,
            self.body2imu_trans.transform.rotation.w,
        ])
        orient = tf_transformations.quaternion_multiply(
            rot,
            [data.orientation.x, data.orientation.y, data.orientation.z, data.orientation.w],
        )

        theta = 15.1 / 180.0 * 3.1415926
        # Gyro: flip y/z, rotate by +theta around y, subtract bias, cross-axis compensate.
        gx, gy, gz = data.angular_velocity.x, -data.angular_velocity.y, -data.angular_velocity.z
        gx2 = np.cos(theta) * gx - np.sin(theta) * gz
        gy2 = gy
        gz2 = np.sin(theta) * gx + np.cos(theta) * gz
        gx2 -= self.ang_bias_x
        gy2 -= self.ang_bias_y
        gz2 -= self.ang_bias_z
        gx2 += self.ang_z2x_proj * gz2
        gy2 += self.ang_z2y_proj * gz2

        # Accel: same transform as gyro, no cross-axis compensation.
        ax, ay, az = data.linear_acceleration.x, -data.linear_acceleration.y, -data.linear_acceleration.z
        ax_out = np.cos(theta) * ax - np.sin(theta) * az - self.acc_bias_x
        ay_out = ay - self.acc_bias_y
        az_out = np.sin(theta) * ax + np.cos(theta) * az - self.acc_bias_z

        if self.accel_lpf_enabled:
            a = self.accel_lpf_alpha
            if self.accel_lpf_state is None:
                self.accel_lpf_state = (ax_out, ay_out, az_out)
            else:
                sx, sy, sz = self.accel_lpf_state
                ax_out = a * ax_out + (1.0 - a) * sx
                ay_out = a * ay_out + (1.0 - a) * sy
                az_out = a * az_out + (1.0 - a) * sz
                self.accel_lpf_state = (ax_out, ay_out, az_out)

        msg = Imu()
        msg.header.stamp = Time(
            nanoseconds=Time.from_msg(data.header.stamp).nanoseconds + self.time_stamp_offset
        ).to_msg()
        msg.header.frame_id = self.output_frame_id
        msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w = orient
        msg.angular_velocity = Vector3(x=gx2, y=gy2, z=gz2)
        msg.linear_acceleration = Vector3(x=ax_out, y=ay_out, z=az_out)

        self.imu_raw_pub.publish(msg)

        # Published "transformed_imu" (for Cartographer): zero orientation & accel,
        # so the SLAM uses gyro-only short-horizon prediction.
        msg.orientation.x = 0.0
        msg.orientation.y = 0.0
        msg.orientation.z = 0.0
        msg.orientation.w = 1.0
        msg.linear_acceleration = Vector3(x=0.0, y=0.0, z=0.0)
        self.imu_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = TransformEverything()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
