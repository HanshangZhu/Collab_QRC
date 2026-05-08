#!/usr/bin/env python3
"""fast_lio_tf_adapter — turn Fast-LIO 2's `/<ns>/Odometry` (frame
camera_init → body) into a clean ROS-Navigation-compliant pose source.

What this node does
-------------------
Fast-LIO 2 publishes its mapped pose on `/<ns>/Odometry` with hardcoded
frame names (`camera_init` parent, `body` child). Those names conflict
with our nav stack which expects `map → base_link`. Upstream Fast-LIO
also broadcasts that TF directly to /tf, but our launch *sinks* it
(`/tf:=/<ns>/fastlio_tf_sink`) precisely because the names clash.

This adapter re-emits the same pose with the right frame names:
    - publishes `<ns>/odom/nav`        (Odometry, frame_id=`map`, child=`base_link`)
    - broadcasts TF                    (`map → base_link`)

It optionally bootstraps to GT (one-shot at startup) so the map frame's
origin coincides with the world origin, allowing CFPA2 frontiers (in
world coords) to be reachable. Without bootstrap, the map frame's
origin is wherever the robot was when Fast-LIO started.

It also optionally prefers SC-PGO's `/<ns>/corrected_odom` over the
raw Fast-LIO output, mirroring the slam_odom_relay's loop-closure
preference. When SC-PGO eventually ports to ROS 2 humble (see
`src/vendor/sc_pgo/PORT_TO_ROS2.md`), flipping the launch-arg
`loop_closure:=true` will start engaging this fallback.

Why this exists
---------------
Replaces three things at once:
  1. `slam_odom_relay`'s frame remapping (the relay was already doing
     the same thing but writing only the topic, not TF — leaving us
     with EKF-and-mujoco-odom-bridge as the actual TF sources).
  2. `mujoco_odom_bridge`'s `publish_tf=True` (sim-only privilege —
     real robots have no MuJoCo bridge).
  3. The two EKFs (`base_to_footprint_ekf` + `footprint_to_odom_ekf`)
     when their input `/<ns>/odom/raw` is broken (CHAMP state_estimation
     publishes 7.8e+34 nonsense — a real-robot blocker uncovered
     2026-04-29).

The cost: the `map → odom → base_link` REP-105 split collapses into
a single `map → base_link` link. Loop closures (when SC-PGO runs) will
cause discrete jumps in this single transform. For our 10 Hz lidar +
20 Hz controller setup with `transform_tolerance: 0.2 s`, the practical
impact is minor — MPPI's outer loop already tolerates per-tick pose
discontinuities. If higher-rate smoothness becomes a problem, add an
IMU-integrator publishing `odom → base_link` and have this adapter
publish `map → odom` instead.

CLI:
    python3 fast_lio_tf_adapter.py --ros-args \
        -p namespace:=robot_a \
        -p use_sim_time:=true \
        -p bootstrap_from_gt:=true
"""
from __future__ import annotations

import math
import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import (
    QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy,
)

from geometry_msgs.msg import TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


def _split_ros_argv(argv):
    if "--ros-args" in argv:
        i = argv.index("--ros-args")
        return argv[:i], argv[i:]
    return argv, []


def _yaw_from_quat(qx: float, qy: float, qz: float, qw: float) -> float:
    return math.atan2(2.0 * (qw * qz + qx * qy),
                      1.0 - 2.0 * (qy * qy + qz * qz))


def _quat_from_yaw(yaw: float) -> tuple[float, float, float, float]:
    return (0.0, 0.0, math.sin(0.5 * yaw), math.cos(0.5 * yaw))


def _quat_mul(a, b):
    ax, ay, az, aw = a
    bx, by, bz, bw = b
    return (
        aw * bx + ax * bw + ay * bz - az * by,
        aw * by - ax * bz + ay * bw + az * bx,
        aw * bz + ax * by - ay * bx + az * bw,
        aw * bw - ax * bx - ay * by - az * bz,
    )


def _rotate_xy(x: float, y: float, yaw: float) -> tuple[float, float]:
    c, s = math.cos(yaw), math.sin(yaw)
    return (c * x - s * y, s * x + c * y)


class FastLioTfAdapter(Node):
    def __init__(self) -> None:
        super().__init__("fast_lio_tf_adapter")
        self.declare_parameter("namespace", "robot_a")
        self.declare_parameter("input_topic", "Odometry")
        self.declare_parameter("output_topic", "odom/nav")
        # Frame names — Fast-LIO hardcodes camera_init→body in C++; we
        # translate to map→base_link here.
        self.declare_parameter("output_frame_id", "map")
        self.declare_parameter("output_child_frame_id", "base_link")
        # Whether to broadcast TF (map→base_link). When false, only
        # publishes the topic (legacy /odom/nav substitute).
        self.declare_parameter("publish_tf", True)
        # Optional one-shot GT bootstrap: aligns Fast-LIO's local
        # origin to the world origin. Useful in sim (uses MuJoCo GT)
        # and in real robots that come up at a known pose. Without
        # bootstrap, /odom/nav reflects displacement from Fast-LIO
        # start pose.
        self.declare_parameter("bootstrap_from_gt", True)
        self.declare_parameter("gt_topic", "odom/ground_truth")
        # SC-PGO: prefer corrected odom when fresh, otherwise raw.
        self.declare_parameter("corrected_topic", "corrected_odom")
        self.declare_parameter("corrected_staleness_sec", 2.0)

        ns = str(self.get_parameter("namespace").value)

        def _qualify(t: str) -> str:
            # Absolute topic (starts with `/`) → use as-is. This is the real-robot
            # case where Fast-LIO is launched un-namespaced and publishes /Odometry.
            # Relative topic → prepended with /<ns>/ (sim dual-robot case where
            # each Fast-LIO is wrapped in its own namespace).
            return t if t.startswith("/") else f"/{ns}/{t}"

        self.in_topic = _qualify(str(self.get_parameter("input_topic").value))
        self.out_topic = _qualify(str(self.get_parameter("output_topic").value))
        self.gt_topic = _qualify(str(self.get_parameter("gt_topic").value))
        self.corrected_topic = _qualify(str(self.get_parameter("corrected_topic").value))
        self.target_frame = str(self.get_parameter("output_frame_id").value)
        self.target_child = str(self.get_parameter("output_child_frame_id").value)
        self.publish_tf = bool(self.get_parameter("publish_tf").value)
        self.bootstrap_from_gt = bool(self.get_parameter("bootstrap_from_gt").value)
        self.corrected_staleness = float(
            self.get_parameter("corrected_staleness_sec").value
        )

        # Bootstrap state
        self._aligned: bool = not self.bootstrap_from_gt
        self._dx = 0.0
        self._dy = 0.0
        self._yaw_offset = 0.0
        self._latest_gt: Odometry | None = None
        # SC-PGO corrected odom cache
        self._corrected: Odometry | None = None
        self._corrected_t: float = 0.0
        self._using_corrected = False
        self._msg_count = 0

        self.create_subscription(Odometry, self.in_topic, self._on_raw, 10)
        if self.bootstrap_from_gt:
            self.create_subscription(Odometry, self.gt_topic, self._on_gt, 10)
        self.create_subscription(Odometry, self.corrected_topic, self._on_corrected, 10)
        self.pub = self.create_publisher(Odometry, self.out_topic, 10)
        self.tf_br = TransformBroadcaster(self) if self.publish_tf else None

        self.get_logger().info(
            f"fast_lio_tf_adapter up: ns={ns} {self.in_topic} → "
            f"{self.out_topic} (frame={self.target_frame}→{self.target_child}) "
            f"tf={self.publish_tf} bootstrap_gt={self.bootstrap_from_gt}"
        )

    @staticmethod
    def _now_mono() -> float:
        return time.monotonic()

    def _on_gt(self, msg: Odometry) -> None:
        self._latest_gt = msg

    def _on_corrected(self, msg: Odometry) -> None:
        self._corrected = msg
        self._corrected_t = self._now_mono()
        if not self._using_corrected:
            self.get_logger().info(
                f"SC-PGO corrected odom received on {self.corrected_topic}"
                f" — switching to corrected source"
            )
            self._using_corrected = True

    def _maybe_align(self, raw: Odometry) -> None:
        """One-shot GT-bootstrap. Computes the static offset (Δx, Δy,
        Δyaw) between Fast-LIO's local origin and the GT's world origin
        at the moment of first matching messages, so all subsequent
        re-emitted poses are in world coords.
        """
        if self._aligned or self._latest_gt is None:
            return
        gt = self._latest_gt
        gt_yaw = _yaw_from_quat(
            gt.pose.pose.orientation.x, gt.pose.pose.orientation.y,
            gt.pose.pose.orientation.z, gt.pose.pose.orientation.w,
        )
        raw_yaw = _yaw_from_quat(
            raw.pose.pose.orientation.x, raw.pose.pose.orientation.y,
            raw.pose.pose.orientation.z, raw.pose.pose.orientation.w,
        )
        self._yaw_offset = gt_yaw - raw_yaw
        # Rotate raw position by yaw_offset, then add gt - rotated_raw
        rx_aligned, ry_aligned = _rotate_xy(
            raw.pose.pose.position.x, raw.pose.pose.position.y, self._yaw_offset
        )
        self._dx = gt.pose.pose.position.x - rx_aligned
        self._dy = gt.pose.pose.position.y - ry_aligned
        self._aligned = True
        self.get_logger().info(
            f"Initialized SLAM alignment from GT: dx={self._dx:.2f} "
            f"dy={self._dy:.2f} yaw_offset_deg={math.degrees(self._yaw_offset):.1f}"
        )

    def _emit(self, src: Odometry) -> None:
        """Apply alignment offset and emit Odometry + TF."""
        # If no bootstrap requested, emit raw frames-only translated
        if self._aligned:
            x_in = src.pose.pose.position.x
            y_in = src.pose.pose.position.y
            x_rot, y_rot = _rotate_xy(x_in, y_in, self._yaw_offset)
            x_out = x_rot + self._dx
            y_out = y_rot + self._dy
            qx = src.pose.pose.orientation.x
            qy = src.pose.pose.orientation.y
            qz = src.pose.pose.orientation.z
            qw = src.pose.pose.orientation.w
            qoff = _quat_from_yaw(self._yaw_offset)
            qx, qy, qz, qw = _quat_mul(qoff, (qx, qy, qz, qw))
        else:
            # Pre-bootstrap: pass through raw pose (frames-only translation)
            x_out = src.pose.pose.position.x
            y_out = src.pose.pose.position.y
            qx = src.pose.pose.orientation.x
            qy = src.pose.pose.orientation.y
            qz = src.pose.pose.orientation.z
            qw = src.pose.pose.orientation.w

        out = Odometry()
        out.header.stamp = src.header.stamp
        out.header.frame_id = self.target_frame
        out.child_frame_id = self.target_child
        out.pose.pose.position.x = x_out
        out.pose.pose.position.y = y_out
        out.pose.pose.position.z = src.pose.pose.position.z
        out.pose.pose.orientation.x = qx
        out.pose.pose.orientation.y = qy
        out.pose.pose.orientation.z = qz
        out.pose.pose.orientation.w = qw
        out.pose.covariance = src.pose.covariance
        out.twist = src.twist
        self.pub.publish(out)

        if self.tf_br is not None:
            tf = TransformStamped()
            tf.header.stamp = src.header.stamp
            tf.header.frame_id = self.target_frame
            tf.child_frame_id = self.target_child
            tf.transform.translation.x = x_out
            tf.transform.translation.y = y_out
            tf.transform.translation.z = src.pose.pose.position.z
            tf.transform.rotation.x = qx
            tf.transform.rotation.y = qy
            tf.transform.rotation.z = qz
            tf.transform.rotation.w = qw
            self.tf_br.sendTransform(tf)

        self._msg_count += 1
        if self._msg_count % 100 == 1:
            tag = "CORRECTED" if (self._corrected is not None
                                  and (self._now_mono() - self._corrected_t)
                                  < self.corrected_staleness) else "RAW"
            self.get_logger().info(
                f"Relayed {self._msg_count} msgs ({tag}) | NAV pose=("
                f"{x_out:.2f}, {y_out:.2f})"
            )

    def _on_raw(self, msg: Odometry) -> None:
        self._maybe_align(msg)
        # Prefer SC-PGO's corrected odom when fresh
        if self._corrected is not None:
            age = self._now_mono() - self._corrected_t
            if age < self.corrected_staleness:
                self._emit(self._corrected)
                return
            elif self._using_corrected:
                self._using_corrected = False
                self.get_logger().warn(
                    f"SC-PGO corrected odom stale ({age:.1f}s) — falling back to raw"
                )
        self._emit(msg)


def main(argv=None) -> None:
    user_argv, ros_argv = _split_ros_argv(sys.argv if argv is None else argv)
    rclpy.init(args=ros_argv)
    node = FastLioTfAdapter()
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
