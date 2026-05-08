#!/usr/bin/env python3
"""
calibrate_imu.py — Two-phase IMU calibration for UTLidar on Unitree Go2.

Computes biases and cross-axis coupling terms used by transform_everything.py.
All calculations happen AFTER the negate-Y,Z + pitch rotation (matching
transform_everything's pipeline), so biases subtract correctly at runtime.

Usage:
  Phase 1 (static — robot standing still on flat ground):
    python3 calibrate_imu.py --phase static --duration 30

  Phase 2 (spin — slowly spin robot in place via teleop):
    python3 calibrate_imu.py --phase spin --duration 20

  Both phases in sequence:
    python3 calibrate_imu.py --phase both --duration 30

Output: overwrites imu_calib_data.yaml
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from sensor_msgs.msg import Imu
import numpy as np
import yaml
import os
import sys
import time


CALIB_FILE = os.path.expanduser("~/COMP0225_LRC_stack/imu_calib_data.yaml")


def quat_rotate_vec(q, v):
    """Rotate vector v by quaternion q = [x, y, z, w]."""
    qx, qy, qz, qw = q
    vx, vy, vz = v
    # q * v * q_conj  (Hamilton product)
    tx = 2.0 * (qy * vz - qz * vy)
    ty = 2.0 * (qz * vx - qx * vz)
    tz = 2.0 * (qx * vy - qy * vx)
    return (
        vx + qw * tx + (qy * tz - qz * ty),
        vy + qw * ty + (qz * tx - qx * tz),
        vz + qw * tz + (qx * ty - qy * tx),
    )


def quat_from_two_vectors(v_from, v_to):
    """Quaternion [x, y, z, w] that rotates unit vector v_from to v_to."""
    v_from = np.array(v_from, dtype=float)
    v_to = np.array(v_to, dtype=float)
    v_from /= np.linalg.norm(v_from)
    v_to /= np.linalg.norm(v_to)
    cross = np.cross(v_from, v_to)
    dot = float(np.dot(v_from, v_to))
    if dot < -0.9999:
        # ~180° rotation — pick arbitrary perpendicular axis
        perp = np.array([1, 0, 0]) if abs(v_from[0]) < 0.9 else np.array([0, 1, 0])
        axis = np.cross(v_from, perp)
        axis /= np.linalg.norm(axis)
        return [axis[0], axis[1], axis[2], 0.0]
    w = 1.0 + dot
    q = np.array([cross[0], cross[1], cross[2], w])
    q /= np.linalg.norm(q)
    return q.tolist()


def negate_raw(ax, ay, az, gx, gy, gz):
    """Apply sensor axis convention: negate Y, Z."""
    return ax, -ay, -az, gx, -gy, -gz


def transform_raw_imu(ax, ay, az, gx, gy, gz, gravity_q=None):
    """Apply negate + gravity-alignment rotation.
    
    If gravity_q is provided, uses quaternion rotation (proper 3D).
    Otherwise falls back to legacy pitch-only rotation for backward compat.
    """
    ax, ay, az, gx, gy, gz = negate_raw(ax, ay, az, gx, gy, gz)
    
    if gravity_q is not None:
        ax, ay, az = quat_rotate_vec(gravity_q, (ax, ay, az))
        gx, gy, gz = quat_rotate_vec(gravity_q, (gx, gy, gz))
    else:
        # Legacy pitch-only fallback
        theta = 15.1 / 180.0 * np.pi
        ax2 = np.cos(theta) * ax - np.sin(theta) * az
        az2 = np.sin(theta) * ax + np.cos(theta) * az
        ax, az = ax2, az2
        gx2 = np.cos(theta) * gx - np.sin(theta) * gz
        gz2 = np.sin(theta) * gx + np.cos(theta) * gz
        gx, gz = gx2, gz2
    
    return ax, ay, az, gx, gy, gz


class IMUCalibrator(Node):
    def __init__(self, duration, use_raw=True):
        super().__init__("imu_calibrator")
        self.duration = duration
        self.samples = []
        self.start_time = None
        self.done = False
        
        # Subscribe to RAW imu (we apply the transform ourselves)
        if use_raw:
            topic = "/utlidar/imu"
            self.get_logger().info(f"Subscribing to RAW topic: {topic}")
            self.get_logger().info("Will apply negate+pitch transform internally")
        else:
            topic = "/utlidar/transformed_raw_imu"
            self.get_logger().info(f"Subscribing to transformed topic: {topic}")
        
        self.use_raw = use_raw
        # UTLidar publishes with BEST_EFFORT QoS — must match
        qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=50)
        self.sub = self.create_subscription(Imu, topic, self.imu_cb, qos)
        self.get_logger().info(f"Recording for {duration}s... keep robot STILL on flat ground")
    
    def imu_cb(self, msg):
        if self.done:
            return
        
        now = time.time()
        if self.start_time is None:
            self.start_time = now
        
        elapsed = now - self.start_time
        if elapsed > self.duration:
            self.done = True
            return
        
        ax = msg.linear_acceleration.x
        ay = msg.linear_acceleration.y
        az = msg.linear_acceleration.z
        gx = msg.angular_velocity.x
        gy = msg.angular_velocity.y
        gz = msg.angular_velocity.z
        
        if self.use_raw:
            ax, ay, az, gx, gy, gz = transform_raw_imu(
                ax, ay, az, gx, gy, gz, gravity_q=getattr(self, 'gravity_q', None))
        
        self.samples.append([ax, ay, az, gx, gy, gz])
        
        if len(self.samples) % 500 == 0:
            self.get_logger().info(
                f"  {len(self.samples)} samples, {elapsed:.0f}/{self.duration}s")


def run_static_calibration(duration=30, use_raw=True):
    """Phase 1: Static calibration — compute gyro biases in RAW sensor frame.
    
    Robot must be standing still on flat ground.
    Textbook approach: only compute gyro biases.
    Accel is published raw — Cartographer determines gravity internally.
    """
    print("=" * 50)
    print("  Phase 1: STATIC calibration (textbook mode)")
    print("  Robot must be STANDING STILL on FLAT GROUND")
    print("  Computing GYRO biases only (raw sensor frame)")
    print("=" * 50)
    
    rclpy.init()
    node = IMUCalibrator(duration, use_raw=True)  # subscribes to /utlidar/imu with BEST_EFFORT QoS
    node.use_raw = False  # don't apply any transform in callbacks
    
    while not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    node.destroy_node()
    rclpy.shutdown()
    
    if len(node.samples) < 100:
        print(f"ERROR: Only {len(node.samples)} samples collected. Is the IMU topic publishing?")
        return None
    
    data = np.array(node.samples)
    print(f"\n  Collected {len(data)} samples")
    
    acc_mean = data[:, :3].mean(axis=0)
    gyr_mean = data[:, 3:].mean(axis=0)
    acc_std = data[:, :3].std(axis=0)
    gyr_std = data[:, 3:].std(axis=0)
    
    print(f"\n  Raw accelerometer (mean +/- std):")
    print(f"    X: {acc_mean[0]:+.4f} +/- {acc_std[0]:.4f}")
    print(f"    Y: {acc_mean[1]:+.4f} +/- {acc_std[1]:.4f}")
    print(f"    Z: {acc_mean[2]:+.4f} +/- {acc_std[2]:.4f}")
    print(f"    |acc| = {np.linalg.norm(acc_mean):.3f} (expect ~9.81)")
    print(f"    (Accel NOT calibrated — Cartographer handles gravity)")
    
    print(f"\n  Raw gyroscope (mean +/- std):")
    print(f"    X: {gyr_mean[0]:+.6f} +/- {gyr_std[0]:.6f}")
    print(f"    Y: {gyr_mean[1]:+.6f} +/- {gyr_std[1]:.6f}")
    print(f"    Z: {gyr_mean[2]:+.6f} +/- {gyr_std[2]:.6f}")
    
    print(f"\n  -- Gyro biases (raw sensor frame) --")
    print(f"  ang_bias_x: {gyr_mean[0]:+.6f}")
    print(f"  ang_bias_y: {gyr_mean[1]:+.6f}")
    print(f"  ang_bias_z: {gyr_mean[2]:+.6f}")
    
    return {
        'ang_bias_x': float(round(gyr_mean[0], 6)),
        'ang_bias_y': float(round(gyr_mean[1], 6)),
        'ang_bias_z': float(round(gyr_mean[2], 6)),
    }


def run_spin_calibration(duration=15, use_raw=True, auto_spin=True):
    """Phase 2: Spin calibration — compute cross-axis gyro coupling.
    
    Slowly spin the robot in place (yaw only). The cross-axis terms
    ang_z2x_proj and ang_z2y_proj compensate for gyro X and Y readings
    that appear proportional to Z rotation rate.
    
    In transform_everything.py (lines 204-205):
      x2 += ang_z2x_proj * z2
      y2 += ang_z2y_proj * z2
    
    So we fit: gx_residual = slope * gz  ->  proj = -slope
    """
    print("\n" + "=" * 50)
    print("  Phase 2: SPIN calibration")
    if auto_spin:
        print("  Robot will AUTO-SPIN at 0.5 rad/s")
        print("  CLEAR AREA around robot!")
    else:
        print("  Slowly SPIN the robot in place (yaw rotation)")
        print("  Use teleop or manually rotate the robot")
    print("=" * 50)
    
    # Load phase 1 biases
    if os.path.exists(CALIB_FILE):
        with open(CALIB_FILE, 'r') as f:
            calib = yaml.safe_load(f)
        print(f"  Loaded existing biases from {CALIB_FILE}")
    else:
        print("  WARNING: No existing calibration file. Run --phase static first.")
        calib = {k: 0.0 for k in ['acc_bias_x', 'acc_bias_y', 'acc_bias_z',
                                    'ang_bias_x', 'ang_bias_y', 'ang_bias_z']}
    
    rclpy.init()
    node = IMUCalibrator(duration, use_raw=use_raw)
    
    # Auto-spin: publish rotation commands in a background thread
    spin_thread = None
    if auto_spin:
        import threading
        import json
        
        try:
            from unitree_api.msg import Request
            has_unitree_api = True
        except ImportError:
            has_unitree_api = False
            node.get_logger().warn("unitree_api not found — falling back to subprocess spin")
        
        def spin_robot():
            """Send rotation commands at 10 Hz via sport API."""
            if has_unitree_api:
                # Use ROS2 publisher directly
                pub = node.create_publisher(Request, '/api/sport/request', 10)
                rate_hz = 10
                interval = 1.0 / rate_hz
                start = time.time()
                while not node.done and (time.time() - start) < duration + 2:
                    msg = Request()
                    msg.header.identity.api_id = 1008
                    msg.parameter = json.dumps({"x": 0.0, "y": 0.0, "z": 0.5})
                    pub.publish(msg)
                    time.sleep(interval)
                # Send stop
                msg = Request()
                msg.header.identity.api_id = 1008
                msg.parameter = json.dumps({"x": 0.0, "y": 0.0, "z": 0.0})
                pub.publish(msg)
                node.get_logger().info("Auto-spin stopped")
            else:
                # Fallback: use subprocess
                import subprocess
                cmd = [
                    'ros2', 'topic', 'pub', '--rate', '10',
                    '/api/sport/request', 'unitree_api/msg/Request',
                    '{header: {identity: {api_id: 1008}}, '
                    'parameter: \'{"x":0.0,"y":0.0,"z":0.5}\'}'
                ]
                proc = subprocess.Popen(cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                while not node.done:
                    time.sleep(0.5)
                proc.terminate()
                proc.wait()
                # Send stop
                stop_cmd = [
                    'ros2', 'topic', 'pub', '--once',
                    '/api/sport/request', 'unitree_api/msg/Request',
                    '{header: {identity: {api_id: 1008}}, '
                    'parameter: \'{"x":0.0,"y":0.0,"z":0.0}\'}'
                ]
                subprocess.run(stop_cmd, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
                node.get_logger().info("Auto-spin stopped (subprocess)")
        
        node.get_logger().info("Starting auto-spin in 3 seconds...")
        time.sleep(3)
        spin_thread = threading.Thread(target=spin_robot, daemon=True)
        spin_thread.start()
    
    while not node.done:
        rclpy.spin_once(node, timeout_sec=0.1)
    
    if spin_thread:
        spin_thread.join(timeout=5)
    
    node.destroy_node()
    rclpy.shutdown()
    
    if len(node.samples) < 100:
        print(f"ERROR: Only {len(node.samples)} samples.")
        return None
    
    data = np.array(node.samples)
    print(f"\n  Collected {len(data)} samples")
    
    # Subtract biases
    gx = data[:, 3] - calib.get('ang_bias_x', 0)
    gy = data[:, 4] - calib.get('ang_bias_y', 0)
    gz = data[:, 5] - calib.get('ang_bias_z', 0)
    
    # Only use samples where robot is actually spinning (gz > threshold)
    spinning = np.abs(gz) > 0.05
    if spinning.sum() < 50:
        print("  WARNING: Not enough spinning data detected.")
        print(f"  Max |gz| = {np.max(np.abs(gz)):.4f} rad/s")
        print("  Make sure to spin the robot during recording.")
        return None
    
    gx_spin = gx[spinning]
    gy_spin = gy[spinning]
    gz_spin = gz[spinning]
    
    print(f"  {spinning.sum()} spinning samples (|gz| > 0.05)")
    print(f"  gz range: [{gz_spin.min():.3f}, {gz_spin.max():.3f}] rad/s")
    
    # Fit: gx_residual = ang_z2x_proj * gz  ->  proj = -cov(gx,gz)/var(gz)
    # The sign convention in transform_everything is: x2 += proj * z2
    # So we want proj such that (gx + proj * gz) has minimal variance
    ang_z2x = -np.dot(gx_spin, gz_spin) / np.dot(gz_spin, gz_spin)
    ang_z2y = -np.dot(gy_spin, gz_spin) / np.dot(gz_spin, gz_spin)
    
    # Check improvement
    gx_before = np.std(gx_spin)
    gx_after = np.std(gx_spin + ang_z2x * gz_spin)
    gy_before = np.std(gy_spin)
    gy_after = np.std(gy_spin + ang_z2y * gz_spin)
    
    print(f"\n  -- Cross-axis coupling --")
    print(f"  ang_z2x_proj: {ang_z2x:+.4f}  (gx std: {gx_before:.4f} -> {gx_after:.4f})")
    print(f"  ang_z2y_proj: {ang_z2y:+.4f}  (gy std: {gy_before:.4f} -> {gy_after:.4f})")
    
    return {
        'ang_z2x_proj': float(round(ang_z2x, 4)),
        'ang_z2y_proj': float(round(ang_z2y, 4)),
    }


def main():
    import argparse
    parser = argparse.ArgumentParser(description='UTLidar IMU Calibration')
    parser.add_argument('--phase', choices=['static', 'spin', 'both'], default='both',
                        help='Calibration phase to run')
    parser.add_argument('--duration', type=int, default=30,
                        help='Recording duration per phase (seconds)')
    parser.add_argument('--output', default=CALIB_FILE,
                        help='Output calibration file')
    parser.add_argument('--no-auto-spin', action='store_true',
                        help='Disable auto-spin in Phase 2 (spin robot manually instead)')
    args = parser.parse_args()
    
    use_raw = True  # Always subscribe to raw /utlidar/imu and transform internally
    result = {}
    
    # Load existing file as defaults
    if os.path.exists(args.output):
        with open(args.output, 'r') as f:
            result = yaml.safe_load(f) or {}
        print(f"Loaded existing: {args.output}")
    
    if args.phase in ('static', 'both'):
        static_result = run_static_calibration(args.duration, use_raw=use_raw)
        if static_result:
            result.update(static_result)
            # Save intermediate so spin phase can load biases
            with open(args.output, 'w') as f:
                yaml.dump(result, f, default_flow_style=False)
            print(f"\n  Saved biases to {args.output}")
    
    if args.phase in ('spin', 'both'):
        if args.phase == 'both':
            print("\n" + "!" * 50)
            print("  Now SPIN the robot slowly in place!")
            print("  Recording starts in 5 seconds...")
            print("!" * 50)
            time.sleep(5)
        
        spin_result = run_spin_calibration(args.duration, use_raw=use_raw,
                                            auto_spin=not args.no_auto_spin)
        if spin_result:
            result.update(spin_result)
    
    # Save final
    with open(args.output, 'w') as f:
        yaml.dump(result, f, default_flow_style=False)
    
    print(f"\n{'='*50}")
    print(f"  Calibration saved: {args.output}")
    print(f"{'='*50}")
    for k, v in sorted(result.items()):
        print(f"  {k}: {v}")
    print()
    print("  Restart transform_everything to apply new calibration.")


if __name__ == '__main__':
    main()
