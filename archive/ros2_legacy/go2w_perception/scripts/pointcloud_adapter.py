#!/usr/bin/env python3
"""
Adapts Gazebo's gpu_ray PointCloud2 to include 'ring' and 'time' fields
expected by FAST-LIO's Velodyne handler.

Gazebo gpu_ray publishes: x, y, z, intensity  (PointXYZI)
FAST-LIO expects:        x, y, z, intensity, time (float32), ring (uint16)

This node:
  - Computes 'ring' from the vertical angle of each point (atan2 of z/xy_range)
  - Sets 'time' to 0 (all points are from the same scan instant in Gazebo)
  - Re-publishes on a new topic with the added fields

Subscribe: /registered_scan  (sensor_msgs/PointCloud2, BestEffort)
Publish:   /velodyne_points  (sensor_msgs/PointCloud2, Reliable)
"""

import math
import struct

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


class PointCloudAdapter(Node):
    def __init__(self):
        super().__init__('pointcloud_adapter')

        self.declare_parameter('input_topic', '/registered_scan')
        self.declare_parameter('output_topic', '/velodyne_points')
        self.declare_parameter('num_rings', 16)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self.num_rings = self.get_parameter('num_rings').value

        # Compute ring boundaries from URDF vertical FOV: -15° to +15°
        self.min_vert_angle = -15.0 * math.pi / 180.0
        self.max_vert_angle = 15.0 * math.pi / 180.0
        self.ring_step = (self.max_vert_angle - self.min_vert_angle) / self.num_rings

        # Subscribe with BestEffort (Gazebo default)
        qos_sub = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self.callback, qos_sub
        )

        # Publish with Reliable (what FAST-LIO expects)
        qos_pub = QoSProfile(
            depth=5,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )
        self.pub = self.create_publisher(PointCloud2, output_topic, qos_pub)

        self.msg_count = 0
        self.get_logger().info(
            f'PointCloud adapter: {input_topic} -> {output_topic} '
            f'(adding ring/time fields, {self.num_rings} rings)'
        )

    def callback(self, msg: PointCloud2):
        """Convert PointXYZI to Velodyne-compatible format with ring and time."""
        # Parse input fields to find offsets
        field_map = {f.name: f for f in msg.fields}

        if 'x' not in field_map:
            return

        # Read raw point data as numpy
        # Input: x(4) y(4) z(4) intensity(4) = 16 bytes per point typically
        point_step = msg.point_step
        n_points = msg.width * msg.height
        if n_points == 0:
            return

        raw = np.frombuffer(msg.data, dtype=np.uint8).reshape(n_points, point_step)

        x_off = field_map['x'].offset
        y_off = field_map['y'].offset
        z_off = field_map['z'].offset
        i_off = field_map['intensity'].offset if 'intensity' in field_map else None

        # Extract x, y, z as float32 arrays
        x = np.frombuffer(raw[:, x_off:x_off+4].tobytes(), dtype=np.float32)
        y = np.frombuffer(raw[:, y_off:y_off+4].tobytes(), dtype=np.float32)
        z = np.frombuffer(raw[:, z_off:z_off+4].tobytes(), dtype=np.float32)

        if i_off is not None:
            intensity = np.frombuffer(raw[:, i_off:i_off+4].tobytes(), dtype=np.float32)
        else:
            intensity = np.zeros(n_points, dtype=np.float32)

        # Compute ring from vertical angle
        xy_range = np.sqrt(x*x + y*y)
        vert_angle = np.arctan2(z, np.maximum(xy_range, 1e-6))
        ring = np.clip(
            ((vert_angle - self.min_vert_angle) / self.ring_step).astype(np.uint16),
            0, self.num_rings - 1
        )

        # Time field: Gazebo gpu_ray captures all points instantaneously,
        # but Fast-LIO needs varying per-point timestamps for scan sorting and
        # motion undistortion.  The span must be small to minimize distortion:
        # 10ms total → 0.5mm at 0.5m/s (vs original 100ms → 5cm → doubled walls).
        azimuth = np.arctan2(y, x)  # -pi to pi
        azimuth_normalized = (azimuth + math.pi) / (2.0 * math.pi)  # 0 to 1
        time_offset = (azimuth_normalized * 10000.0).astype(np.float32)  # 0-10ms in μs

        # Build output: x(4) y(4) z(4) intensity(4) time(4) ring(2) padding(2) = 24 bytes
        out_point_step = 24
        out_data = np.zeros((n_points, out_point_step), dtype=np.uint8)

        # Pack fields
        out_data[:, 0:4] = np.frombuffer(x.tobytes(), dtype=np.uint8).reshape(n_points, 4)
        out_data[:, 4:8] = np.frombuffer(y.tobytes(), dtype=np.uint8).reshape(n_points, 4)
        out_data[:, 8:12] = np.frombuffer(z.tobytes(), dtype=np.uint8).reshape(n_points, 4)
        out_data[:, 12:16] = np.frombuffer(intensity.tobytes(), dtype=np.uint8).reshape(n_points, 4)
        out_data[:, 16:20] = np.frombuffer(time_offset.tobytes(), dtype=np.uint8).reshape(n_points, 4)
        out_data[:, 20:22] = np.frombuffer(ring.tobytes(), dtype=np.uint8).reshape(n_points, 2)
        # bytes 22-23 are padding (zeros)

        # Build output message
        out_msg = PointCloud2()
        out_msg.header = msg.header
        out_msg.height = 1
        out_msg.width = n_points
        out_msg.fields = [
            PointField(name='x', offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name='y', offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name='z', offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name='intensity', offset=12, datatype=PointField.FLOAT32, count=1),
            PointField(name='time', offset=16, datatype=PointField.FLOAT32, count=1),
            PointField(name='ring', offset=20, datatype=PointField.UINT16, count=1),
        ]
        out_msg.is_bigendian = False
        out_msg.point_step = out_point_step
        out_msg.row_step = out_point_step * n_points
        out_msg.data = out_data.tobytes()
        out_msg.is_dense = True

        self.pub.publish(out_msg)

        self.msg_count += 1
        if self.msg_count % 50 == 1:
            self.get_logger().info(
                f'Adapted {self.msg_count} clouds ({n_points} pts each)'
            )


def main(args=None):
    rclpy.init(args=args)
    node = PointCloudAdapter()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
