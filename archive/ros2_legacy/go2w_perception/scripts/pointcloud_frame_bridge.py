#!/usr/bin/env python3
"""Transform PointCloud2 messages into a target frame via TF."""

from __future__ import annotations

import struct
from collections import deque

import numpy as np
import rclpy
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.duration import Duration
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import PointCloud2, PointField
import tf2_ros
from tf2_ros import (
    Buffer,
    ConnectivityException,
    ExtrapolationException,
    LookupException,
    TransformListener,
)
from tf2_sensor_msgs.tf2_sensor_msgs import do_transform_cloud


class PointCloudFrameBridge(Node):
    def __init__(self) -> None:
        super().__init__("pointcloud_frame_bridge")

        self.declare_parameter("input_topic", "/registered_scan")
        self.declare_parameter("output_topic", "/registered_scan_map")
        self.declare_parameter("target_frame", "map")
        self.declare_parameter("tf_timeout_sec", 0.01)
        self.declare_parameter("transform_wait_sec", 0.25)
        self.declare_parameter("max_cloud_age_sec", 1.0)

        input_topic = str(self.get_parameter("input_topic").value).strip()
        output_topic = str(self.get_parameter("output_topic").value).strip()
        self.target_frame = str(self.get_parameter("target_frame").value).strip()
        self.tf_timeout = Duration(seconds=max(0.0, float(self.get_parameter("tf_timeout_sec").value)))
        self.transform_wait_sec = float(self.get_parameter("transform_wait_sec").value)
        self.max_cloud_age_sec = float(self.get_parameter("max_cloud_age_sec").value)
        self.last_warn_sec = 0.0
        self.pending = deque()
        self._dtype_fallback_logged = False

        self._sub_cb_group = MutuallyExclusiveCallbackGroup()
        self._timer_cb_group = MutuallyExclusiveCallbackGroup()

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        pub_qos = QoSProfile(reliability=ReliabilityPolicy.RELIABLE, depth=10)
        self.pub = self.create_publisher(PointCloud2, output_topic, pub_qos)
        self.sub = self.create_subscription(
            PointCloud2, input_topic, self._cloud_cb, qos_profile_sensor_data,
            callback_group=self._sub_cb_group,
        )
        self.timer = self.create_timer(0.05, self._flush_pending, callback_group=self._timer_cb_group)

        self.get_logger().info(
            f"PointCloud frame bridge: {input_topic} -> {output_topic} target_frame={self.target_frame}"
        )

    def _warn_throttle(self, message: str) -> None:
        now_sec = self.get_clock().now().nanoseconds / 1_000_000_000.0
        if now_sec - self.last_warn_sec >= 1.0:
            self.get_logger().warn(message)
            self.last_warn_sec = now_sec

    def _cloud_cb(self, msg: PointCloud2) -> None:
        self.pending.append(msg)
        if len(self.pending) > 30:
            self.pending.popleft()

    def _flush_pending(self) -> None:
        while self.pending:
            try:
                result = self._flush_pending_inner()
                if result == "waiting":
                    break  # Oldest message not ready yet; wait for next timer tick
            except Exception as e:
                self._warn_throttle(f"flush_pending error (dropping cloud): {e}")
                self.pending.popleft()

    def _flush_pending_inner(self) -> str:
        """Process the oldest pending cloud. Returns 'waiting' if not ready yet."""
        now = self.get_clock().now()
        msg = self.pending[0]

        try:
            stamp_time = rclpy.time.Time.from_msg(msg.header.stamp)
            stamp_sec = stamp_time.nanoseconds / 1_000_000_000.0
        except (OverflowError, ValueError):
            self._warn_throttle("Dropping cloud with invalid/future timestamp")
            self.pending.popleft()
            return "dropped"

        now_sec = now.nanoseconds / 1_000_000_000.0
        if stamp_sec > now_sec:
            self._warn_throttle("Dropping cloud with future timestamp")
            self.pending.popleft()
            return "dropped"

        age = now_sec - stamp_sec
        if self.transform_wait_sec > 0.0 and age < self.transform_wait_sec:
            return "waiting"  # Wait for TF to catch up — caller should break

        if self.max_cloud_age_sec > 0.0 and age > self.max_cloud_age_sec:
            self._warn_throttle(f"Dropping cloud older than {age:.2f}s while waiting for TF")
            self.pending.popleft()
            return "dropped"

        self.pending.popleft()
        self._transform_and_publish(msg)
        return "processed"

    def _transform_and_publish(self, msg: PointCloud2) -> bool:
        frame_id = msg.header.frame_id.strip()
        if not frame_id:
            self._warn_throttle("Dropping cloud with empty frame_id")
            return False

        if frame_id == self.target_frame:
            self.pub.publish(self._ensure_intensity(msg))
            return True

        try:
            transform = self.tf_buffer.lookup_transform(
                self.target_frame,
                frame_id,
                rclpy.time.Time.from_msg(msg.header.stamp),
                self.tf_timeout,
            )
        except (LookupException, ConnectivityException, ExtrapolationException):
            self._warn_throttle(f"Waiting for TF {frame_id}->{self.target_frame} at scan stamp ({msg.header.stamp})")
            return False

        try:
            transformed = do_transform_cloud(msg, transform)
        except (AssertionError, TypeError, ValueError, RuntimeError, Exception) as e:
            if not self._dtype_fallback_logged:
                self.get_logger().warn(
                    f"do_transform_cloud failed ({type(e).__name__}); using xyz-only fallback transform"
                )
                self._dtype_fallback_logged = True
            try:
                transformed = self._transform_preserve_fields(msg, transform)
            except Exception as e2:
                self._warn_throttle(f"Fallback transform also failed: {e2}")
                return False

        self.pub.publish(self._ensure_intensity(transformed))
        return True

    @staticmethod
    def _ensure_intensity(msg: PointCloud2) -> PointCloud2:
        """Inject a zero-filled intensity field if the cloud lacks one.

        CMU's FAR / terrain_analysis stack reads clouds as PointXYZI.
        MuJoCo's LiDAR plugin emits PointXYZ, producing spammy PCL
        "Failed to find match for field 'intensity'" warnings.
        """
        if any(f.name == "intensity" for f in msg.fields):
            return msg
        name_map = {f.name: f for f in msg.fields}
        if not {"x", "y", "z"}.issubset(name_map):
            return msg

        ps = msg.point_step
        npts = msg.width * msg.height
        if npts == 0 or ps == 0 or len(msg.data) < npts * ps:
            return msg

        buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(npts, ps)
        ox = name_map["x"].offset
        oy = name_map["y"].offset
        oz = name_map["z"].offset
        xs = np.frombuffer(buf[:, ox:ox + 4].tobytes(), dtype=np.float32)
        ys = np.frombuffer(buf[:, oy:oy + 4].tobytes(), dtype=np.float32)
        zs = np.frombuffer(buf[:, oz:oz + 4].tobytes(), dtype=np.float32)

        out = np.zeros((npts, 4), dtype=np.float32)
        out[:, 0] = xs
        out[:, 1] = ys
        out[:, 2] = zs
        # intensity column left at 0.0

        new_msg = PointCloud2()
        new_msg.header = msg.header
        new_msg.height = 1
        new_msg.width = npts
        new_msg.is_bigendian = msg.is_bigendian
        new_msg.point_step = 16
        new_msg.row_step = 16 * npts
        new_msg.is_dense = msg.is_dense
        new_msg.fields = [
            PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
            PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
            PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
            PointField(name="intensity", offset=12, datatype=PointField.FLOAT32, count=1),
        ]
        new_msg.data = out.tobytes()
        return new_msg

    @staticmethod
    def _transform_preserve_fields(msg: PointCloud2, transform) -> PointCloud2:
        """Manual xyz transform that preserves all original PointCloud2 fields."""
        tx = float(transform.transform.translation.x)
        ty = float(transform.transform.translation.y)
        tz = float(transform.transform.translation.z)
        qx = float(transform.transform.rotation.x)
        qy = float(transform.transform.rotation.y)
        qz = float(transform.transform.rotation.z)
        qw = float(transform.transform.rotation.w)

        # Rotation matrix from quaternion
        r00 = 1.0 - 2.0 * (qy * qy + qz * qz)
        r01 = 2.0 * (qx * qy - qz * qw)
        r02 = 2.0 * (qx * qz + qy * qw)
        r10 = 2.0 * (qx * qy + qz * qw)
        r11 = 1.0 - 2.0 * (qx * qx + qz * qz)
        r12 = 2.0 * (qy * qz - qx * qw)
        r20 = 2.0 * (qx * qz - qy * qw)
        r21 = 2.0 * (qy * qz + qx * qw)
        r22 = 1.0 - 2.0 * (qx * qx + qy * qy)

        field_map = {f.name: f for f in msg.fields}
        if "x" not in field_map or "y" not in field_map or "z" not in field_map:
            raise RuntimeError("PointCloud2 is missing x/y/z fields")

        ox = field_map["x"].offset
        oy = field_map["y"].offset
        oz = field_map["z"].offset
        ps = msg.point_step

        out_data = bytearray(msg.data)
        for i in range(0, len(msg.data), ps):
            x, = struct.unpack_from("<f", msg.data, i + ox)
            y, = struct.unpack_from("<f", msg.data, i + oy)
            z, = struct.unpack_from("<f", msg.data, i + oz)
            nx = r00 * x + r01 * y + r02 * z + tx
            ny = r10 * x + r11 * y + r12 * z + ty
            nz = r20 * x + r21 * y + r22 * z + tz
            struct.pack_into("<f", out_data, i + ox, nx)
            struct.pack_into("<f", out_data, i + oy, ny)
            struct.pack_into("<f", out_data, i + oz, nz)

        out = PointCloud2()
        out.header = msg.header
        out.header.frame_id = transform.header.frame_id
        out.height = msg.height
        out.width = msg.width
        out.fields = msg.fields
        out.is_bigendian = msg.is_bigendian
        out.point_step = msg.point_step
        out.row_step = msg.row_step
        out.data = bytes(out_data)
        out.is_dense = msg.is_dense
        return out


def main():
    rclpy.init()
    node = PointCloudFrameBridge()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
