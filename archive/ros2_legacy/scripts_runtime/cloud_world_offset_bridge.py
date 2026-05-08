#!/usr/bin/env python3
"""Camera-init → world point-cloud bridge for Fast-LIO in the sim stack.

Fast-LIO publishes ``/cloud_registered`` in its own ``camera_init`` frame
(spawn = origin). ``slam_odom_relay`` then applies a fixed (dx, dy, dz,
yaw_offset) to Fast-LIO's odometry so ``/robot/odom/nav`` lands in world
coordinates (spawn = actual demo-scene coords).

The old ``pointcloud_frame_bridge.py`` reproduced the same alignment by
walking the full TF chain per cloud, with a built-in 100 ms TF-wait and
50 ms timer pacing. Measured lag: ~0.6 s. terrain_analysis then filtered
0.6-s-stale points against a fresh odom → phantom voxels at the robot's
former position.

This bridge skips TF entirely:
  * Learn the bootstrap offset once from two odom streams (same logic as
    slam_odom_relay).
  * Apply it vectorized in NumPy to every cloud → publish at scan rate
    with zero artificial wait.

Subscribes:
  * ``cloud_input_topic``  (camera_init-frame PointCloud2)
  * ``raw_odom_topic``     (camera_init-frame Odometry — Fast-LIO's)
  * ``world_odom_topic``   (world-frame Odometry — slam_odom_relay's output)

Publishes:
  * ``cloud_output_topic`` (world-frame PointCloud2, frame_id=``output_frame``)
"""
from __future__ import annotations

import math
import struct

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import PointCloud2, PointField


def _yaw_from_quat(x: float, y: float, z: float, w: float) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


class CloudWorldOffsetBridge(Node):
    def __init__(self) -> None:
        super().__init__("cloud_world_offset_bridge")

        self.declare_parameter("cloud_input_topic", "cloud_registered_camera_init")
        self.declare_parameter("cloud_output_topic", "registered_scan_map")
        self.declare_parameter("raw_odom_topic", "Odometry")
        self.declare_parameter("world_odom_topic", "odom/nav")
        self.declare_parameter("output_frame", "map")
        self.declare_parameter("odom_match_timeout_sec", 0.05)

        p = self.get_parameter
        self._output_frame = str(p("output_frame").value)
        self._match_timeout = float(p("odom_match_timeout_sec").value)

        self._dx = self._dy = self._dz = 0.0
        self._yaw = 0.0
        self._cos = 1.0
        self._sin = 0.0
        self._aligned = False

        self._raw: Odometry | None = None
        self._world: Odometry | None = None
        self._n_pub = 0
        self._n_drop_no_align = 0

        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self._pub = self.create_publisher(
            PointCloud2, str(p("cloud_output_topic").value), pub_qos)
        self.create_subscription(
            PointCloud2, str(p("cloud_input_topic").value),
            self._cloud_cb, qos_profile_sensor_data)
        self.create_subscription(
            Odometry, str(p("raw_odom_topic").value),
            lambda m: self._odom_cb("raw", m), 10)
        self.create_subscription(
            Odometry, str(p("world_odom_topic").value),
            lambda m: self._odom_cb("world", m), 10)
        self.create_timer(5.0, self._heartbeat)

        self.get_logger().info(
            f"cloud bridge up: {p('cloud_input_topic').value} -> "
            f"{p('cloud_output_topic').value} (frame={self._output_frame}); "
            f"offset sourced from {p('raw_odom_topic').value} vs "
            f"{p('world_odom_topic').value}"
        )

    def _odom_cb(self, which: str, msg: Odometry) -> None:
        if which == "raw":
            self._raw = msg
        else:
            self._world = msg
        if self._aligned or self._raw is None or self._world is None:
            return
        # Close in time: avoid picking a stale pair.
        traw = self._raw.header.stamp.sec + self._raw.header.stamp.nanosec * 1e-9
        twrl = self._world.header.stamp.sec + self._world.header.stamp.nanosec * 1e-9
        if abs(traw - twrl) > self._match_timeout:
            return
        # Compute offset once: world = Rz(yaw) * raw + (dx, dy, dz)
        raw_p = self._raw.pose.pose.position
        wrl_p = self._world.pose.pose.position
        raw_yaw = _yaw_from_quat(
            self._raw.pose.pose.orientation.x, self._raw.pose.pose.orientation.y,
            self._raw.pose.pose.orientation.z, self._raw.pose.pose.orientation.w)
        wrl_yaw = _yaw_from_quat(
            self._world.pose.pose.orientation.x, self._world.pose.pose.orientation.y,
            self._world.pose.pose.orientation.z, self._world.pose.pose.orientation.w)
        self._yaw = wrl_yaw - raw_yaw
        self._cos = math.cos(self._yaw)
        self._sin = math.sin(self._yaw)
        rx = self._cos * raw_p.x - self._sin * raw_p.y
        ry = self._sin * raw_p.x + self._cos * raw_p.y
        self._dx = wrl_p.x - rx
        self._dy = wrl_p.y - ry
        self._dz = wrl_p.z - raw_p.z
        self._aligned = True
        self.get_logger().info(
            f"aligned: dx={self._dx:+.3f} dy={self._dy:+.3f} dz={self._dz:+.3f} "
            f"yaw={math.degrees(self._yaw):+.2f}°"
        )

    def _cloud_cb(self, msg: PointCloud2) -> None:
        if not self._aligned:
            self._n_drop_no_align += 1
            return

        # Find x, y, z field offsets and any remaining fields we pass through.
        fields = {f.name: f for f in msg.fields}
        if not {"x", "y", "z"}.issubset(fields):
            return
        fx, fy, fz = fields["x"], fields["y"], fields["z"]
        # Fast-LIO emits float32 xyz contiguous at offsets 0, 4, 8 by default,
        # but stay robust to layout.
        assert fx.datatype == PointField.FLOAT32 == fy.datatype == fz.datatype

        ps = msg.point_step
        n = msg.width * msg.height
        if n == 0:
            self._pub.publish(msg)
            return

        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(n, ps).copy()
        xs = np.frombuffer(buf[:, fx.offset:fx.offset + 4].tobytes(), dtype=np.float32)
        ys = np.frombuffer(buf[:, fy.offset:fy.offset + 4].tobytes(), dtype=np.float32)
        zs = np.frombuffer(buf[:, fz.offset:fz.offset + 4].tobytes(), dtype=np.float32)

        # Vectorized: rotate xy by yaw, add (dx, dy, dz).
        nx = (self._cos * xs - self._sin * ys + self._dx).astype(np.float32)
        ny = (self._sin * xs + self._cos * ys + self._dy).astype(np.float32)
        nz = (zs + self._dz).astype(np.float32)

        buf[:, fx.offset:fx.offset + 4] = np.frombuffer(nx.tobytes(), dtype=np.uint8).reshape(n, 4)
        buf[:, fy.offset:fy.offset + 4] = np.frombuffer(ny.tobytes(), dtype=np.uint8).reshape(n, 4)
        buf[:, fz.offset:fz.offset + 4] = np.frombuffer(nz.tobytes(), dtype=np.uint8).reshape(n, 4)

        out = PointCloud2()
        out.header.stamp = msg.header.stamp
        out.header.frame_id = self._output_frame
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = buf.tobytes()
        out.is_dense = msg.is_dense
        self._pub.publish(out)
        self._n_pub += 1

    def _heartbeat(self) -> None:
        self.get_logger().info(
            f"published={self._n_pub}, dropped_before_align={self._n_drop_no_align}, "
            f"aligned={self._aligned}"
        )


def main() -> None:
    rclpy.init()
    node = CloudWorldOffsetBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
