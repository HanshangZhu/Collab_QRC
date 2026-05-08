#!/usr/bin/env python3
"""Measure Livox Mid-360 mount tilt (roll + pitch) from /livox/imu.

The Mid-360 ships with an onboard IMU. When the robot is still on a flat
floor, the accelerometer reads gravity projected into the IMU's body frame.
The direction of that gravity vector — in body frame — gives us the IMU's
roll and pitch relative to the world vertical, which is exactly the mount
tilt of the Mid-360 on the Go2.

Usage:
  1. Put the robot on a level surface. Keep it COMPLETELY still (no teleop,
     no breathing against it).
  2. Make sure the ROS stack is NOT running (else Fast-LIO moves the body
     TF and this tool's assumption breaks). Either:
        ./scripts/real/real_autonomy.sh stop
     or just launch livox_ros_driver2 standalone:
        ros2 run livox_ros_driver2 livox_ros_driver2_node \\
          --ros-args -p user_config_path:=...MID360_config.json -p ...
  3. Run:
        python3 src/go2w/go2w_real_bringup/tools/measure_mid360_tilt.py --seconds 10
  4. Paste the printed static_transform_publisher line into
     slam.launch.py (fastlio_mid360 branch) replacing the identity
     body_to_base_link_fastlio TF. Rebuild + relaunch.

Limitations:
- Yaw is unobservable from gravity alone. If your Mid-360 is mounted with a
  yaw offset (e.g. rotated 15° around its own z-axis on the mount), this
  tool won't detect it. Either eyeball the mount yaw, or drive the robot in
  a straight line and compare Fast-LIO's heading drift to the robot's
  actual heading.
- Translation (where the IMU sits relative to base_link) isn't measurable
  from IMU data. Measure it with a tape measure.

Outputs:
  Approximate accel magnitude (should be ~9.81 m/s² — reality check)
  Roll, pitch in degrees (about the Mid-360's intrinsic x, y axes)
  The static_transform_publisher command you should use in slam.launch.py
"""
from __future__ import annotations

import argparse
import math
import sys
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu


class TiltMeasurer(Node):
    def __init__(self, duration_sec: float, imu_topic: str):
        super().__init__("mid360_tilt_measurer")
        self.duration = duration_sec
        self.imu_topic = imu_topic

        self.sum_x = 0.0
        self.sum_y = 0.0
        self.sum_z = 0.0
        self.sum_x2 = 0.0
        self.sum_y2 = 0.0
        self.sum_z2 = 0.0
        self.n = 0
        self.start_sec: float | None = None

        self.sub = self.create_subscription(Imu, imu_topic, self._cb, 200)
        self.get_logger().info(
            f"Recording {imu_topic} for {duration_sec:.1f} s. Keep the robot absolutely still."
        )
        # Preflight: give it 2s to see ANY message, else help the user debug.
        self.preflight_timer = self.create_timer(2.0, self._preflight_check)
        self._preflight_fired = False

    def _preflight_check(self) -> None:
        if self._preflight_fired:
            return
        self._preflight_fired = True
        self.preflight_timer.cancel()
        if self.n == 0:
            self.get_logger().error(
                f"No {self.imu_topic} messages in the first 2 s. Likely causes:\n"
                f"    (a) livox_ros_driver2_node isn't running — start slam.launch.py\n"
                f"        slam:=fastlio_mid360 in another terminal.\n"
                f"    (b) CycloneDDS env not exported in this shell — source\n"
                f"        scripts/real/connect_ethernet.sh && setup_cyclonedds_ethernet.\n"
                f"    (c) Wrong topic name — use --topic /your/topic.\n"
                f"    Continuing to wait, but if you want to abort: Ctrl+C."
            )
        else:
            self.get_logger().info(
                f"Receiving ({self.n} samples in 2 s). Continue to hold still."
            )

    def _now(self) -> float:
        return self.get_clock().now().nanoseconds / 1e9

    def _cb(self, msg: Imu) -> None:
        if self.start_sec is None:
            self.start_sec = self._now()
        a = msg.linear_acceleration
        self.sum_x += a.x; self.sum_y += a.y; self.sum_z += a.z
        self.sum_x2 += a.x * a.x
        self.sum_y2 += a.y * a.y
        self.sum_z2 += a.z * a.z
        self.n += 1

    def done(self) -> bool:
        return self.start_sec is not None and (self._now() - self.start_sec) >= self.duration

    def report(self) -> None:
        if self.n == 0:
            self.get_logger().error(f"No messages received on {self.imu_topic}.")
            sys.exit(2)

        mx = self.sum_x / self.n
        my = self.sum_y / self.n
        mz = self.sum_z / self.n

        # Sample variance for motion sanity check.
        var_x = self.sum_x2 / self.n - mx * mx
        var_y = self.sum_y2 / self.n - my * my
        var_z = self.sum_z2 / self.n - mz * mz
        sd_total = math.sqrt(max(0.0, var_x + var_y + var_z))

        g_mag = math.sqrt(mx * mx + my * my + mz * mz)
        pitch_rad = math.atan2(-mx, math.sqrt(my * my + mz * mz))
        roll_rad = math.atan2(my, mz)

        # Livox Mid-360 publishes accel in UNITS OF g (not m/s²) — a known
        # non-ROS-compliant quirk of livox_ros_driver2. Either reading is
        # fine for tilt (atan2 of ratios is unit-free). Range-check both.
        if 0.90 <= g_mag <= 1.10:
            mag_note = "(≈1 g — Livox's non-ROS-standard units; tilt calc still valid)"
            mag_ok = True
        elif 9.30 <= g_mag <= 10.30:
            mag_note = "(≈9.81 m/s² — ROS-standard units)"
            mag_ok = True
        else:
            mag_note = "⚠ UNEXPECTED magnitude — IMU miscalibrated or in motion"
            mag_ok = False

        # Convert to the rotation that maps body → base_link so the base_link's
        # +z is gravity-up. Because body accel reads ~(+) in the direction
        # opposing gravity (i.e., +z_body roughly points up when level), the
        # signs here assume standard REP-145 IMU convention. If your IMU uses
        # the "accel points along -g" convention you'll need to negate
        # pitch/roll — visually sanity check against the printed axes.
        print("")
        print("=" * 58)
        print(f"Samples collected : {self.n}")
        print(f"|a|  mean         : {g_mag:.4f}        {mag_note}")
        print(f"Accel std (total) : {sd_total:.5f}       "
              f"(<0.05 = still; "
              f"{'OK' if sd_total < 0.05 else 'TOO NOISY — redo'})")
        print(f"Mean accel body   : ({mx:+.4f}, {my:+.4f}, {mz:+.4f})")
        print("")
        print(f"Mount tilt (body relative to level base_link):")
        print(f"  roll   = {math.degrees(roll_rad):+8.3f} deg  ({roll_rad:+.5f} rad)")
        print(f"  pitch  = {math.degrees(pitch_rad):+8.3f} deg  ({pitch_rad:+.5f} rad)")
        print(f"  yaw    = unobservable from IMU alone — measure separately")
        print("")
        print("Patch for slam.launch.py (fastlio_mid360 branch):")
        print("  replace the identity body_to_base_link_fastlio with:")
        print("")
        print(f'    arguments=[')
        print(f'      "--frame-id", "body", "--child-frame-id", "base_link",')
        print(f'      "--x", "0", "--y", "0", "--z", "0",           # measure translation separately')
        print(f'      "--roll",  "{-roll_rad:+.6f}",')
        print(f'      "--pitch", "{-pitch_rad:+.6f}",')
        print(f'      "--yaw",   "0",                               # fix yaw separately if tilted about z')
        print(f'    ],')
        print("")
        print("(Signs flipped because the static TF publishes body → base_link;")
        print(" body is tilted by (roll, pitch) relative to base_link, so the TF")
        print(" that reverses that tilt has negated angles.)")
        print("=" * 58)


def main():
    parser = argparse.ArgumentParser(description="Measure Mid-360 mount tilt from /livox/imu.")
    parser.add_argument("--seconds", type=float, default=10.0,
                        help="Recording duration (default: 10 s)")
    parser.add_argument("--topic", type=str, default="/livox/imu",
                        help="IMU topic (default: /livox/imu)")
    args = parser.parse_args()

    rclpy.init()
    node = TiltMeasurer(args.seconds, args.topic)
    try:
        while rclpy.ok() and not node.done():
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        pass
    node.report()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
