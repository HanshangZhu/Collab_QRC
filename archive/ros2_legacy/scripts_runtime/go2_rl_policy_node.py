#!/usr/bin/env python3
"""Go2 RL policy inference node for the MuJoCo stack (direct-torque mode).

Loads the pre-trained Isaac-Lab ONNX policy from eppl-erau-db/go2_rl_ws
(``src/vendor/go2_rl_ws/src/unitree_ros2_python/share/models/flat_policy_v5.onnx``)
and drives the robot via **direct torque** on the 12 leg joints, replicating
what Unitree's LowCmd does on real hardware:

    τ_i = kp · (q_desired_i − q_current_i) − kd · dq_current_i

This bypasses ``joint_trajectory_controller``'s internal PID, which otherwise
applies CHAMP's kp=100/kd=1.0 and amplifies the policy's actions 5× past their
training distribution. By doing the PD here with kp=20 / kd=0.5 we stay on
policy's training gains exactly.

Pipeline
--------
  /robot/joint_states       sensor_msgs/JointState   → joint_pos, joint_vel
  /robot/imu/data           sensor_msgs/Imu          → base_ang_vel, proj_gravity
  /robot/cmd_vel_legged     geometry_msgs/Twist      → cmd_vel (vx, vy, vyaw)
  ─────────────────────────────────────────────────────────────────────
  build 45-dim obs → ONNX → 12-dim raw action
  target_q = raw * 0.25 + IL_DEFAULTS  (clipped to 0.95 × motor limits)
  τ = kp_rl · (target_q − joint_pos) − kd_rl · joint_vel
  reorder IL → YAML joint order (leg-grouped: FL/FR/RL/RR × hip/thigh/calf)
  publish Float64MultiArray →
    /robot/robot_joint_group_effort_controller/commands

Stand-up phase
--------------
For the first ``stand_up_sec`` seconds (default 3.0), ``target_q`` is forced
to ``IL_DEFAULTS`` regardless of policy output. This lets a soft PD settle
the robot to stance before the policy takes over. Needed because the policy
is trained assuming roughly-standing initial poses and is NOT robust to
"fallen from drop" starts.
"""
from __future__ import annotations

import os
import sys
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float64MultiArray

try:
    import onnxruntime as ort
except ImportError:
    print("onnxruntime not installed — `pip install onnxruntime` in cmu_env", file=sys.stderr)
    raise

# --------------------------------------------------------------------------
# Joint layouts

# Isaac-Lab observation / action layout (type-grouped: hips, thighs, calves).
IL_JOINT_NAMES = [
    "FL_hip_joint",  "FR_hip_joint",  "RL_hip_joint",  "RR_hip_joint",
    "FL_thigh_joint","FR_thigh_joint","RL_thigh_joint","RR_thigh_joint",
    "FL_calf_joint", "FR_calf_joint", "RL_calf_joint", "RR_calf_joint",
]

# Effort-controller joint layout (leg-grouped: FL/FR/RL/RR × hip/thigh/calf).
# Must match the ``joints:`` list in ros_control_go2_robot_rl_effort.yaml.
YAML_JOINT_NAMES = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]
# Precompute IL → YAML permutation (so Python code stays in IL order).
IL_TO_YAML = np.array([IL_JOINT_NAMES.index(n) for n in YAML_JOINT_NAMES])

# Isaac-Lab training defaults (used to centre the obs and the action output).
IL_DEFAULTS = np.array(
    [0.0, 0.0, 0.0, 0.0,   # hips
     1.1, 1.1, 1.1, 1.1,   # thighs
    -1.8,-1.8,-1.8,-1.8],  # calves
    dtype=np.float32,
)

IL_ACTION_SCALE = 0.25

# Menagerie Go2 home-keyframe joint positions. Used as the PD target during
# STANDUP / HOLD so we don't yank the robot away from its natural settled
# stance. The thigh value differs from IL_DEFAULTS (1.1 vs 0.9) — keeping
# these distinct lets the policy still receive (joint_pos − IL_DEFAULTS)
# in its observation, matching training distribution, while the PD target
# uses whatever actually balances under MuJoCo dynamics.
MENAGERIE_HOME = np.array(
    [0.0, 0.0, 0.0, 0.0,   # hips
     0.9, 0.9, 0.9, 0.9,   # thighs — Menagerie home, NOT IL's 1.1
    -1.8,-1.8,-1.8,-1.8],  # calves
    dtype=np.float32,
)

# 95% of motor range, per IL joint. Tight clip prevents policy spikes from
# driving joints past hard stops.
IL_LIMITS = np.array([
    [-0.837, 0.837], [-0.837, 0.837], [-0.837, 0.837], [-0.837, 0.837],
    [-3.490, 1.570], [-3.490, 1.570], [-4.530, 1.570], [-4.530, 1.570],
    [-2.720,-0.837], [-2.720,-0.837], [-2.720,-0.837], [-2.720,-0.837],
], dtype=np.float32) * 0.95

# Torque safety cap — matches Go2 motor spec (~40 N·m peak for calves, 30 for hips/thighs).
IL_TORQUE_LIMITS = np.array(
    [25.0, 25.0, 25.0, 25.0,   # hips
     25.0, 25.0, 25.0, 25.0,   # thighs
     40.0, 40.0, 40.0, 40.0],  # calves (knee motor is strongest)
    dtype=np.float32,
)


def project_gravity(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Gravity vector expressed in the body frame.

    Given the body→world rotation quaternion (x, y, z, w), the gravity-in-body
    observation that Isaac Lab uses is ``R_bw⁻¹ · g_world`` where
    ``g_world = [0, 0, -1]``.

    The upstream eppl-erau-db deploy node had a quaternion-ordering bug
    (passed (w, x, y, z) to scipy which expects (x, y, z, w)), which happens
    to produce the correct result at identity but diverges under yaw —
    returns ``[0, -1, 0]`` at yaw=90°, making the policy think the robot is
    on its side. That caused immediate faceplants under cmd_vel.
    """
    from scipy.spatial.transform import Rotation as R
    rot = R.from_quat([qx, qy, qz, qw])  # scipy convention (scalar-last)
    return rot.inv().apply(np.array([0.0, 0.0, -1.0]))


class Go2RLPolicy(Node):

    def __init__(self) -> None:
        super().__init__("go2_rl_policy")

        # ---- parameters ---------------------------------------------------
        self.declare_parameter("model_path", "")
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("imu_topic", "imu/data")
        self.declare_parameter("cmd_vel_topic", "cmd_vel_legged")
        self.declare_parameter("effort_topic",
                               "robot_joint_group_effort_controller/commands")
        self.declare_parameter("debug_actions_topic", "rl_actions")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("publish_efforts", False)   # dry-run by default
        self.declare_parameter("cmd_vel_deadband", 0.05)
        self.declare_parameter("stand_up_sec", 3.0)        # pre-policy settle window
        self.declare_parameter("cmd_hold_sec", 5.0)        # force cmd_vel=0 for this long after POLICY phase starts
        self.declare_parameter("kp", 20.0)                 # IL training-time stiffness
        self.declare_parameter("kd", 0.5)                  # IL training-time damping
        self.declare_parameter("stand_up_kp", 80.0)        # stiffer PD for stand-up recovery
        self.declare_parameter("stand_up_kd", 2.0)         # heavier damping during stand-up

        model_path = str(self.get_parameter("model_path").value).strip()
        if not model_path:
            # v3 + v5 are pronking-gait policies (per upstream comment in the
            # deploy node). v6 is the proper trot. v7 has a 49-dim obs we
            # don't build yet — stick with v6.
            workspace = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            model_path = os.path.join(
                workspace, "src", "vendor", "go2_rl_ws", "src", "unitree_ros2_python",
                "share", "models", "flat_policy_v6.onnx",
            )
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"ONNX model not found: {model_path}")
        self.ort_session = ort.InferenceSession(model_path)

        # ---- state buffers ------------------------------------------------
        self.joint_pos = np.zeros(12, dtype=np.float32)    # absolute positions (not centred)
        self.joint_vel = np.zeros(12, dtype=np.float32)
        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.projected_gravity = np.zeros(3, dtype=np.float32)
        self.cmd_vel = np.zeros(3, dtype=np.float32)
        self.last_raw_action = np.zeros(12, dtype=np.float32)
        self.have_joint_state = False
        self.have_imu = False

        # ---- subscriptions ------------------------------------------------
        self.create_subscription(JointState,
            str(self.get_parameter("joint_states_topic").value),
            self.joint_states_cb, 20)
        self.create_subscription(Imu,
            str(self.get_parameter("imu_topic").value),
            self.imu_cb, 50)
        self.create_subscription(Twist,
            str(self.get_parameter("cmd_vel_topic").value),
            self.cmd_vel_cb, 10)

        # ---- publishers ---------------------------------------------------
        self.debug_pub = self.create_publisher(Float32MultiArray,
            str(self.get_parameter("debug_actions_topic").value), 10)
        self.publish_efforts = bool(self.get_parameter("publish_efforts").value)
        self.effort_pub = None
        if self.publish_efforts:
            self.effort_pub = self.create_publisher(Float64MultiArray,
                str(self.get_parameter("effort_topic").value), 10)

        # ---- tunables read once ------------------------------------------
        self.deadband = float(self.get_parameter("cmd_vel_deadband").value)
        self.stand_up_sec = float(self.get_parameter("stand_up_sec").value)
        self.cmd_hold_sec = float(self.get_parameter("cmd_hold_sec").value)
        self.kp = float(self.get_parameter("kp").value)
        self.kd = float(self.get_parameter("kd").value)
        self.stand_up_kp = float(self.get_parameter("stand_up_kp").value)
        self.stand_up_kd = float(self.get_parameter("stand_up_kd").value)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.step)
        self._node_start_t = self.get_clock().now()
        self._last_log_t = self.get_clock().now()

        mode = "LIVE" if self.publish_efforts else "DRY"
        self.get_logger().info(
            f"Go2 RL policy [{mode}]: model={os.path.basename(model_path)}, "
            f"kp_rl={self.kp:.1f}/{self.kd:.2f}, kp_stand={self.stand_up_kp:.0f}/"
            f"{self.stand_up_kd:.1f}, stand_up_sec={self.stand_up_sec:.1f}"
        )

    # ---- subscribers ------------------------------------------------------
    def joint_states_cb(self, msg: JointState) -> None:
        name_to_idx = {n: i for i, n in enumerate(msg.name)}
        if any(j not in name_to_idx for j in IL_JOINT_NAMES):
            return
        pos = np.array(msg.position, dtype=np.float32)
        vel = np.array(msg.velocity, dtype=np.float32) if msg.velocity else np.zeros_like(pos)
        idx = np.array([name_to_idx[j] for j in IL_JOINT_NAMES])
        self.joint_pos = pos[idx]
        self.joint_vel = vel[idx]
        self.have_joint_state = True

    def imu_cb(self, msg: Imu) -> None:
        self.base_ang_vel = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=np.float32,
        )
        self.projected_gravity = project_gravity(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
        ).astype(np.float32)
        self.have_imu = True

    def cmd_vel_cb(self, msg: Twist) -> None:
        v = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)
        if np.all(np.abs(v) < self.deadband):
            v[:] = 0.0
        self.cmd_vel = v

    # ---- main tick --------------------------------------------------------
    def step(self) -> None:
        if not (self.have_joint_state and self.have_imu):
            return

        # (Re)anchor the stand-up timer the first time obs are valid. Warm-up
        # can take multiple wall seconds (MuJoCo init, joint_state broadcaster
        # spawn), which would otherwise pre-consume the stand-up budget and
        # skip straight into POLICY phase at launch — causing faceplants.
        if not getattr(self, "_standup_anchored", False):
            self._node_start_t = self.get_clock().now()
            self._standup_anchored = True
            self.get_logger().info(
                f"Observations live; anchoring stand_up_sec={self.stand_up_sec:.1f} now"
            )

        elapsed = (self.get_clock().now() - self._node_start_t).nanoseconds / 1e9

        # During STANDUP and the initial POLICY hold window, feed cmd_vel=0
        # to the policy so it produces a stance hold, not a trot. Without
        # this, FAR's residual waypoint keeps the policy in locomotion mode
        # from the very first POLICY tick, which tips the robot.
        elapsed_into_policy = elapsed - self.stand_up_sec
        feed_cmd = self.cmd_vel.copy()
        if elapsed < self.stand_up_sec or elapsed_into_policy < self.cmd_hold_sec:
            feed_cmd[:] = 0.0

        obs = np.concatenate([
            self.base_ang_vel,
            self.projected_gravity,
            feed_cmd,
            self.joint_pos - IL_DEFAULTS,   # IL convention: obs joint_pos is centred
            self.joint_vel,
            self.last_raw_action,
        ]).astype(np.float32).reshape(1, -1)
        assert obs.shape == (1, 45), f"obs shape {obs.shape} ≠ (1, 45)"

        raw = self.ort_session.run(
            None, {self.ort_session.get_inputs()[0].name: obs}
        )[0].astype(np.float32).flatten()
        self.last_raw_action = raw

        # One-shot dump of the first policy-phase obs for debugging.
        if elapsed >= self.stand_up_sec and not getattr(self, "_dumped_obs", False):
            self._dumped_obs = True
            self.get_logger().warn(
                f"FIRST-POLICY-OBS: ang_vel={obs[0, :3].tolist()} "
                f"grav={obs[0, 3:6].tolist()} cmd={obs[0, 6:9].tolist()} "
                f"pos-def={obs[0, 9:21].tolist()} vel={obs[0, 21:33].tolist()} "
                f"last_act={obs[0, 33:45].tolist()}"
            )
            self.get_logger().warn(f"FIRST-POLICY-RAW: {raw.tolist()}")

        in_cmd_hold = (
            elapsed >= self.stand_up_sec
            and (elapsed - self.stand_up_sec) < self.cmd_hold_sec
        )
        if elapsed < self.stand_up_sec:
            # Stiff PD to Menagerie home pose while the robot settles from
            # any initial contact impulses. Ignores policy output entirely.
            target_q = MENAGERIE_HOME.copy()
            kp, kd = self.stand_up_kp, self.stand_up_kd
            phase = "STANDUP"
        elif in_cmd_hold:
            # Hold Menagerie home under policy PD gains. Policy output (raw)
            # is still computed for the obs feedback loop, but doesn't affect
            # target_q — robot quietly holds stance before cmd_vel engages.
            target_q = MENAGERIE_HOME.copy()
            kp, kd = self.kp, self.kd
            phase = "HOLD"
        else:
            target_q = raw * IL_ACTION_SCALE + IL_DEFAULTS
            target_q = np.clip(target_q, IL_LIMITS[:, 0], IL_LIMITS[:, 1])
            kp, kd = self.kp, self.kd
            phase = "POLICY"

        # --- Direct-torque PD ---
        tau_il = kp * (target_q - self.joint_pos) - kd * self.joint_vel
        tau_il = np.clip(tau_il, -IL_TORQUE_LIMITS, IL_TORQUE_LIMITS)

        # Reorder IL → YAML (leg-grouped) order for the effort controller.
        tau_yaml = tau_il[IL_TO_YAML]

        # --- Publish ---
        dbg = Float32MultiArray()
        dbg.data = target_q.tolist()
        self.debug_pub.publish(dbg)

        if self.publish_efforts and self.effort_pub is not None:
            cmd = Float64MultiArray()
            cmd.data = tau_yaml.astype(np.float64).tolist()
            self.effort_pub.publish(cmd)

        # 1 Hz heartbeat
        now = self.get_clock().now()
        if (now - self._last_log_t).nanoseconds >= 1_000_000_000:
            self._last_log_t = now
            self.get_logger().info(
                f"[{phase}] cmd=({self.cmd_vel[0]:+.2f},{self.cmd_vel[1]:+.2f},"
                f"{self.cmd_vel[2]:+.2f}) |raw|={float(np.max(np.abs(raw))):4.2f} "
                f"|τ|={float(np.max(np.abs(tau_il))):5.1f}Nm "
                f"|q-def|={float(np.max(np.abs(self.joint_pos - IL_DEFAULTS))):4.2f}rad "
                f"{'[LIVE]' if self.publish_efforts else '[DRY]'}"
            )


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = Go2RLPolicy()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
