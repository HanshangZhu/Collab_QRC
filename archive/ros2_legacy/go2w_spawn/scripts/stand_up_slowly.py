#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration


JOINT_PRESETS = {
    # Unitree naming — used by both Go2 (LRC URDF, Menagerie MJCF) and Go2W.
    # The legacy lowercase CHAMP naming (lf_hip_joint ...) is no longer
    # carried by any URDF in the stack, so both presets point at the same
    # 12 joints. Kept as two keys for backwards compatibility with callers.
    'go2': [
        'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
        'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
        'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    ],
    'go2w': [
        'FL_hip_joint', 'FL_thigh_joint', 'FL_calf_joint',
        'FR_hip_joint', 'FR_thigh_joint', 'FR_calf_joint',
        'RL_hip_joint', 'RL_thigh_joint', 'RL_calf_joint',
        'RR_hip_joint', 'RR_thigh_joint', 'RR_calf_joint',
    ],
}


class StandUpSlowly(Node):
    def __init__(self):
        super().__init__('stand_up_slowly')
        self.declare_parameter('controller_wait_sec', 5.0)
        self.declare_parameter('phase1_sec', 4.0)
        self.declare_parameter('phase2_sec', 9.0)
        self.declare_parameter('phase3_sec', 14.0)
        self.declare_parameter('knee_bend_ratio', 0.85)
        self.declare_parameter('check_period_sec', 0.5)
        self.declare_parameter('joint_controller_topic', '/joint_group_effort_controller/joint_trajectory')
        self.declare_parameter('joint_name_preset', 'go2')
        # Prefix prepended to every preset joint name. Used by
        # heterogeneous dual-robot launches where robot_b's joints are
        # `b_FL_hip_joint`, `b_FL_thigh_joint`, etc. (different URDF
        # roots under one MuJoCo plugin). Default '' keeps single-robot
        # behaviour untouched.
        self.declare_parameter('joint_name_prefix', '')

        # Publisher to the controller
        self.controller_topic = str(self.get_parameter('joint_controller_topic').value)
        self.publisher_ = self.create_publisher(
            JointTrajectory,
            self.controller_topic,
            10
        )

        preset = str(self.get_parameter('joint_name_preset').value).strip().lower() or 'go2'
        if preset not in JOINT_PRESETS:
            self.get_logger().warn(
                f"Unknown joint_name_preset '{preset}', falling back to 'go2'."
            )
            preset = 'go2'
        joint_prefix = str(self.get_parameter('joint_name_prefix').value)
        self.joint_names = [joint_prefix + n for n in JOINT_PRESETS[preset]]
        if joint_prefix:
            self.get_logger().info(
                f"Applying joint_name_prefix='{joint_prefix}' to {preset} preset"
            )
        
        # Target standing positions (hip, thigh, calf)
        self.target_positions = [
            0.0, 0.9, -1.8,  # LF
            0.0, 0.9, -1.8,  # RF
            0.0, 0.9, -1.8,  # LH
            0.0, 0.9, -1.8   # RH
        ]

        bend_ratio = float(self.get_parameter('knee_bend_ratio').value)
        bend_ratio = min(1.0, max(0.60, bend_ratio))
        self.mid_positions = []
        for i, p in enumerate(self.target_positions):
            if i % 3 == 0:
                # keep hips neutral to avoid lateral throw
                self.mid_positions.append(0.0)
            else:
                self.mid_positions.append(p * bend_ratio)

        # Slightly crouched initial pose before full extension.
        self.start_positions = []
        for i, p in enumerate(self.target_positions):
            if i % 3 == 0:
                self.start_positions.append(0.0)
            else:
                self.start_positions.append(p * 0.65)
        
        self.get_logger().info(f"Waiting for controller on '{self.controller_topic}' to come up...")
        # Wait for controller startup and for at least one subscriber on the command topic.
        self.wait_sec = max(2.0, float(self.get_parameter('controller_wait_sec').value))
        self.check_period_sec = max(0.2, float(self.get_parameter('check_period_sec').value))
        self.start_time = self.get_clock().now()
        self.timer = self.create_timer(self.check_period_sec, self.try_publish_trajectory)
        self.published = False
        self._last_wait_log_sec = -1.0

    def try_publish_trajectory(self):
        if self.published:
            return

        elapsed = (self.get_clock().now() - self.start_time).nanoseconds / 1e9
        if elapsed < self.wait_sec:
            return

        sub_count = self.publisher_.get_subscription_count()
        if sub_count < 1:
            # Log at ~1 Hz to avoid spam.
            if self._last_wait_log_sec < 0.0 or elapsed - self._last_wait_log_sec >= 1.0:
                self.get_logger().info(
                    f"Still waiting: no controller subscriber on '{self.controller_topic}'"
                )
                self._last_wait_log_sec = elapsed
            return
            
        msg = JointTrajectory()
        msg.joint_names = self.joint_names
        
        phase_times = [
            max(2.0, float(self.get_parameter('phase1_sec').value)),
            max(4.0, float(self.get_parameter('phase2_sec').value)),
            max(7.0, float(self.get_parameter('phase3_sec').value)),
        ]
        if phase_times[1] <= phase_times[0]:
            phase_times[1] = phase_times[0] + 2.0
        if phase_times[2] <= phase_times[1]:
            phase_times[2] = phase_times[1] + 3.0

        waypoints = [
            self.start_positions,
            self.mid_positions,
            self.target_positions,
        ]
        for idx, positions in enumerate(waypoints):
            point = JointTrajectoryPoint()
            point.positions = positions
            point.velocities = [0.0] * 12
            point.accelerations = [0.0] * 12
            sec = int(phase_times[idx])
            nanosec = int((phase_times[idx] - sec) * 1e9)
            point.time_from_start = Duration(sec=sec, nanosec=nanosec)
            msg.points.append(point)
        
        self.publisher_.publish(msg)
        self.get_logger().info(
            f"Published stand-up trajectory with 3 phases "
            f"({phase_times[0]:.1f}s, {phase_times[1]:.1f}s, {phase_times[2]:.1f}s)"
        )
        self.published = True
        self.timer.cancel()
        
        # Exit after publishing
        # self.destroy_node()
        # rclpy.shutdown()

def main(args=None):
    rclpy.init(args=args)
    node = StandUpSlowly()
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
