"""MuJoCo ground-truth odometry bridge.

Subscribes to the PoseStamped output from mujoco_ros2_control's pose_sensor
(topic: ~/pose on the pose_sensor node) and the IMU for angular velocity,
then publishes nav_msgs/Odometry on /{ns}/odom/ground_truth at 50 Hz.
Also broadcasts the TF: odom -> base_link.

The DFKI mujoco_ros2_control pose_sensor publishes:
  - PoseStamped on <body_name>_pose_sensor/pose
  - TF: frame_id -> body_name

For ground truth odometry we also need linear/angular velocity. The MJCF model
includes framelinvel and frameangvel sensors, but those are not published by
the DFKI sensor system. So this node subscribes to the pose and IMU, and
computes velocity by finite differencing the pose (or uses IMU angular vel).

Alternative simpler approach used here: subscribe to the TF broadcast by
pose_sensor and the IMU topic, assemble into Odometry.
"""

import math
import threading

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from tf2_ros import TransformBroadcaster


class MujocoOdomBridge(Node):

    def __init__(self):
        super().__init__('mujoco_odom_bridge',
                         parameter_overrides=[rclpy.Parameter('use_sim_time', value=True)])

        # --- Parameters ---
        self.declare_parameter('publish_rate', 50.0)
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        # Topic where mujoco_ros2_control pose_sensor publishes PoseStamped
        self.declare_parameter('pose_topic', 'base_link_site_pose_sensor/pose')
        # Topic where mujoco_ros2_control imu_sensor publishes Imu
        self.declare_parameter('imu_topic', 'base_link_imu_sensor/imu')

        rate = self.get_parameter('publish_rate').get_parameter_value().double_value
        self.odom_frame = self.get_parameter('odom_frame').get_parameter_value().string_value
        self.base_frame = self.get_parameter('base_frame').get_parameter_value().string_value
        pose_topic = self.get_parameter('pose_topic').get_parameter_value().string_value
        imu_topic = self.get_parameter('imu_topic').get_parameter_value().string_value

        # --- State ---
        self._lock = threading.Lock()
        self._latest_pose = None
        self._prev_pose = None
        self._prev_stamp = None
        self._latest_imu = None
        self._last_odom_ns = 0   # monotonicity guard for sim-time stamps
        self._last_imu_ns = 0

        # Optional: republish IMU on standard topic for rest of pipeline
        self.declare_parameter('republish_imu_topic', '')
        self.declare_parameter('publish_tf', True)
        republish_imu = self.get_parameter('republish_imu_topic').get_parameter_value().string_value
        self._publish_tf = self.get_parameter('publish_tf').get_parameter_value().bool_value

        # --- ROS pub/sub ---
        self.odom_pub = self.create_publisher(Odometry, 'odom/ground_truth', 10)
        self.tf_broadcaster = TransformBroadcaster(self)

        self.imu_republish_pub = None
        if republish_imu:
            self.imu_republish_pub = self.create_publisher(Imu, republish_imu, 10)

        self.pose_sub = self.create_subscription(
            PoseStamped, pose_topic, self._pose_cb, 10)
        self.imu_sub = self.create_subscription(
            Imu, imu_topic, self._imu_cb, 10)

        self.timer = self.create_timer(1.0 / rate, self._timer_cb)
        self.get_logger().info(
            f'MuJoCo odom bridge started: pose={pose_topic}, imu={imu_topic}, '
            f'rate={rate} Hz, TF: {self.odom_frame} -> {self.base_frame}'
            f' (publish_tf={self._publish_tf})'
            + (f', IMU republish -> {republish_imu}' if republish_imu else ''))

    # ------------------------------------------------------------------
    def _pose_cb(self, msg: PoseStamped):
        with self._lock:
            self._prev_pose = self._latest_pose
            self._prev_stamp = self._latest_stamp if hasattr(self, '_latest_stamp') else None
            self._latest_pose = msg
            self._latest_stamp = msg.header.stamp

    def _imu_cb(self, msg: Imu):
        with self._lock:
            self._latest_imu = msg
        if self.imu_republish_pub is not None:
            # Re-stamp with sim time — the DFKI IMU sensor uses wall-clock
            # timestamps (create_wall_timer + nh_->now()), but downstream
            # nodes (Cartographer, CHAMP EKF) run on sim time.
            # Drop if sim time hasn't advanced (duplicate timestamps crash
            # Cartographer's ordered_multi_queue).
            now_ns = self.get_clock().now().nanoseconds
            if now_ns <= self._last_imu_ns:
                return
            self._last_imu_ns = now_ns
            msg.header.stamp = self.get_clock().now().to_msg()
            self.imu_republish_pub.publish(msg)

    # ------------------------------------------------------------------
    def _timer_cb(self):
        with self._lock:
            pose = self._latest_pose
            prev_pose = self._prev_pose
            prev_stamp = self._prev_stamp
            imu = self._latest_imu

        if pose is None:
            return

        # Drop if sim time hasn't advanced (duplicate timestamps crash
        # Cartographer's ordered_multi_queue CHECK).
        now_ns = self.get_clock().now().nanoseconds
        if now_ns <= self._last_odom_ns:
            return
        self._last_odom_ns = now_ns
        now = self.get_clock().now().to_msg()
        p = pose.pose

        # --- Compute velocity by finite difference ---
        vx = vy = vz = 0.0
        wx = wy = wz = 0.0

        if prev_pose is not None and prev_stamp is not None:
            dt = (self._stamp_to_sec(pose.header.stamp)
                  - self._stamp_to_sec(prev_stamp))
            if dt > 1e-6:
                pp = prev_pose.pose
                vx = (p.position.x - pp.position.x) / dt
                vy = (p.position.y - pp.position.y) / dt
                vz = (p.position.z - pp.position.z) / dt

        # Use IMU angular velocity if available (more accurate than diff)
        if imu is not None:
            wx = imu.angular_velocity.x
            wy = imu.angular_velocity.y
            wz = imu.angular_velocity.z

        # --- Publish Odometry ---
        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = p.position.x
        odom.pose.pose.position.y = p.position.y
        odom.pose.pose.position.z = p.position.z
        odom.pose.pose.orientation = p.orientation

        # Velocity in child (base_link) frame
        # Transform world-frame velocity to body frame using inverse rotation
        # For now, publish in odom frame and set covariance to indicate that
        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.linear.z = vz
        odom.twist.twist.angular.x = wx
        odom.twist.twist.angular.y = wy
        odom.twist.twist.angular.z = wz

        self.odom_pub.publish(odom)

        # --- Broadcast TF: odom -> base_link ---
        # Disabled when Cartographer provides odom frame (provide_odom_frame=true)
        # to avoid two publishers for the same transform.
        if self._publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame

            t.transform.translation.x = p.position.x
            t.transform.translation.y = p.position.y
            t.transform.translation.z = p.position.z
            t.transform.rotation = p.orientation

            self.tf_broadcaster.sendTransform(t)

    # ------------------------------------------------------------------
    @staticmethod
    def _stamp_to_sec(stamp) -> float:
        return stamp.sec + stamp.nanosec * 1e-9


def main(args=None):
    rclpy.init(args=args)
    node = MujocoOdomBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
