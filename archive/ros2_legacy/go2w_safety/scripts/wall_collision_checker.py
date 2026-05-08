#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Int8
from std_msgs.msg import String
import numpy as np

class WallCollisionChecker(Node):
    def __init__(self):
        super().__init__('wall_collision_checker')
        
        # Parameters
        self.declare_parameter('safety_dist', 0.6)
        self.declare_parameter('scan_topic', '/scan')
        self.declare_parameter('stop_topic', '/stop')
        self.declare_parameter('check_angle_deg', 60.0) # Check +/- 30 degrees front
        self.declare_parameter('min_valid_range', 0.12)  # Filter body/self and near-noise
        self.declare_parameter('min_close_points', 5)    # Require a wall patch, not a single ray
        self.declare_parameter('mode_topic', '')
        self.declare_parameter('wheel_mode_label', 'wheel')
        self.declare_parameter('wheel_safety_dist', 0.0)
        self.declare_parameter('wheel_check_angle_deg', 0.0)
        self.declare_parameter('wheel_min_close_points', 0)
        
        self.safety_dist = float(self.get_parameter('safety_dist').value)
        self.scan_topic = str(self.get_parameter('scan_topic').value)
        self.stop_topic = str(self.get_parameter('stop_topic').value)
        self.check_angle_rad = np.deg2rad(float(self.get_parameter('check_angle_deg').value)) / 2.0
        self.min_valid_range = float(self.get_parameter('min_valid_range').value)
        self.min_close_points = int(self.get_parameter('min_close_points').value)
        self.mode_topic = str(self.get_parameter('mode_topic').value)
        self.wheel_mode_label = str(self.get_parameter('wheel_mode_label').value).strip() or 'wheel'
        wheel_safety_dist = float(self.get_parameter('wheel_safety_dist').value)
        wheel_check_angle_deg = float(self.get_parameter('wheel_check_angle_deg').value)
        wheel_min_close_points = int(self.get_parameter('wheel_min_close_points').value)
        self.wheel_safety_dist = wheel_safety_dist if wheel_safety_dist > 0.0 else self.safety_dist
        self.wheel_check_angle_rad = (
            np.deg2rad(wheel_check_angle_deg) / 2.0
            if wheel_check_angle_deg > 0.0
            else self.check_angle_rad
        )
        self.wheel_min_close_points = (
            wheel_min_close_points if wheel_min_close_points > 0 else self.min_close_points
        )
        self.last_stop_state = 0
        self.current_mode = ''

        self.sub = self.create_subscription(
            LaserScan,
            self.scan_topic,
            self.scan_callback,
            qos_profile_sensor_data
        )
        self.pub = self.create_publisher(Int8, self.stop_topic, 10)
        if self.mode_topic:
            self.create_subscription(
                String,
                self.mode_topic,
                self.mode_callback,
                10,
            )
        
        self.get_logger().info(
            f"Start detecting walls within {self.safety_dist}m on {self.scan_topic} "
            f"(wheel safety={self.wheel_safety_dist:.2f}m mode_topic={self.mode_topic or 'disabled'})"
        )

    def mode_callback(self, msg):
        self.current_mode = str(msg.data).strip().lower()

    def _active_limits(self):
        if self.current_mode == self.wheel_mode_label.lower():
            return (
                self.wheel_safety_dist,
                self.wheel_check_angle_rad,
                self.wheel_min_close_points,
            )
        return (
            self.safety_dist,
            self.check_angle_rad,
            self.min_close_points,
        )

    def scan_callback(self, msg):
        # Convert ranges to numpy array
        ranges = np.array(msg.ranges)
        active_safety_dist, active_check_angle_rad, active_min_close_points = self._active_limits()
        
        # Calculate angles
        angle_min = msg.angle_min
        angle_increment = msg.angle_increment
        angles = angle_min + np.arange(len(ranges)) * angle_increment
        
        # Filter strictly front sector
        # Assuming 0 is front. If LIDAR is rotated, adjust here.
        # Usually 0 is front in standard ROSREP-103.
        
        # Handle wrap around if needed (usually angles are -PI to PI)
        # We want angles between -threshold and +threshold
        
        # Select indices where angle is within limits
        # Using simple boolean indexing since angles are monotonic usually
        front_indices = np.abs(angles) < active_check_angle_rad
        
        front_ranges = ranges[front_indices]
        
        # Filter out inf/nan
        front_ranges = front_ranges[np.isfinite(front_ranges)]
        front_ranges = front_ranges[front_ranges > self.min_valid_range]

        stop_msg = Int8()
        
        if len(front_ranges) > 0:
            min_dist = np.min(front_ranges)
            close_count = int(np.sum(front_ranges < active_safety_dist))
            if min_dist < active_safety_dist and close_count >= active_min_close_points:
                stop_msg.data = 1
                if self.last_stop_state == 0:
                    self.get_logger().warn(
                        f"Wall detected at {min_dist:.2f}m (close rays={close_count}, mode={self.current_mode or 'default'})! Stopping."
                    )
            else:
                stop_msg.data = 0
        else:
            stop_msg.data = 0

        self.last_stop_state = int(stop_msg.data)
        self.pub.publish(stop_msg)

def main(args=None):
    rclpy.init(args=args)
    node = WallCollisionChecker()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
