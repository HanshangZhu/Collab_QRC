#!/usr/bin/env python3
"""Config-driven Go2 RL locomotion node — runs rl_sar policies in MuJoCo.

This is a faithful Python port of fan-ziqi/rl_sar's inference path
(`library/core/rl_sdk/rl_sdk.cpp` + `observation_buffer.cpp`), so the vendored
rl_sar Go2 policies run unchanged in our MuJoCo + cmd_vel pipeline:

  src/vendor/rl_sar/policy/go2/robot_lab/policy.pt   45-dim, single-frame, rough
  src/vendor/rl_sar/policy/go2/himloco/himloco.pt    270-dim, 6-frame history (HIM)

Everything that defines a policy — observation terms + order, per-term scales,
default joint angles, action scale, PD gains, torque limits, joint mapping,
history length — is read from rl_sar's own YAML (`<config_dir>/config.yaml` +
`<base_config>` for joint_names). Point `config_dir` at a different policy dir
and it just works; no code change.

Pipeline (mirrors rl_sar)
-------------------------
  /<ns>/joint_states     sensor_msgs/JointState  → dof_pos, dof_vel (training order)
  /<ns>/imu/data         sensor_msgs/Imu         → base_ang_vel, projected_gravity
  /<ns>/cmd_vel_legged   geometry_msgs/Twist     → commands (vx, vy, vyaw)
  ───────────────────────────────────────────────────────────────────────
  ComputeObservation (scaled, term order, clip) → [history] → policy.forward
  → clip_actions → ComputeOutput (tau = rl_kp·(a·scale + default − q) − rl_kd·dq)
  → clamp torque_limits → remap training→effort-controller order
  → Float64MultiArray on /<ns>/robot_joint_group_effort_controller/commands

Startup
-------
In rl_policy mode nothing holds the robot between spawn and first command, so it
sags. STANDUP (first `stand_up_sec`) runs a stiff PD (fixed_kp/fixed_kd from the
config) to the default pose — rl_sar's "getup" state — actively lifting the
sagged base to standing before the policy engages. cmd_vel is forced to 0 during
STANDUP + the first `cmd_hold_sec` of POLICY so the robot settles before walking.
"""
from __future__ import annotations

import os
import sys

import numpy as np
import yaml

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import JointState, Imu
from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray, Float64MultiArray

try:
    import torch
except ImportError:
    print("torch not installed — needed to load rl_sar .pt policies (cmu_env has it)",
          file=sys.stderr)
    raise

# The MuJoCo effort (ForwardCommandController) joints, in the exact order the
# controller's `commands` array expects (go2_rl_mujoco_controllers.yaml).
EFFORT_ORDER = [
    "FL_hip_joint", "FL_thigh_joint", "FL_calf_joint",
    "FR_hip_joint", "FR_thigh_joint", "FR_calf_joint",
    "RL_hip_joint", "RL_thigh_joint", "RL_calf_joint",
    "RR_hip_joint", "RR_thigh_joint", "RR_calf_joint",
]


def project_gravity(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """Gravity [0,0,-1] expressed in the body frame — matches rl_sar's
    QuatRotateInverse(base_quat, gravity_vec)."""
    from scipy.spatial.transform import Rotation as R
    rot = R.from_quat([qx, qy, qz, qw])  # scalar-last
    return rot.inv().apply(np.array([0.0, 0.0, -1.0])).astype(np.float32)


def _first_section(doc: dict) -> dict:
    """rl_sar YAMLs nest everything under a single top-level key
    (e.g. 'go2/robot_lab:' or 'go2:'). Return that inner dict."""
    if not isinstance(doc, dict) or not doc:
        raise ValueError("empty/invalid rl_sar YAML")
    return next(iter(doc.values()))


class ObservationBuffer:
    """Port of rl_sar ObservationBuffer (time-priority). index 0 = newest."""

    def __init__(self, obs_dim: int, history_length: int):
        self.obs_dim = obs_dim
        self.history_length = history_length
        self.buf = np.zeros((history_length, obs_dim), dtype=np.float32)
        self._primed = False

    def insert(self, obs: np.ndarray) -> None:
        if not self._primed:
            # rl_sar resets all frames to the first real obs (avoids a step of
            # zero-history transients).
            self.buf[:] = obs
            self._primed = True
        else:
            self.buf[1:] = self.buf[:-1]
            self.buf[0] = obs

    def get(self, obs_ids: list[int]) -> np.ndarray:
        # time priority: concat full frames for each id in order ([newest..oldest]).
        return np.concatenate([self.buf[i] for i in obs_ids]).astype(np.float32)


class Go2RLSar(Node):

    def __init__(self) -> None:
        super().__init__("go2_rl_sar")

        # ---- params -------------------------------------------------------
        self.declare_parameter("config_dir", "")     # dir with config.yaml + .pt
        self.declare_parameter("base_config", "")     # base.yaml (joint_names)
        self.declare_parameter("joint_states_topic", "joint_states")
        self.declare_parameter("imu_topic", "imu/data")
        self.declare_parameter("cmd_vel_topic", "cmd_vel_legged")
        self.declare_parameter("effort_topic",
                               "robot_joint_group_effort_controller/commands")
        self.declare_parameter("debug_actions_topic", "rl_actions")
        self.declare_parameter("publish_rate_hz", 50.0)
        self.declare_parameter("publish_efforts", False)
        self.declare_parameter("cmd_vel_deadband", 0.05)
        self.declare_parameter("stand_up_sec", 5.0)   # fixed-PD getup window
        self.declare_parameter("cmd_hold_sec", 3.0)   # zero-cmd hold after getup

        config_dir = str(self.get_parameter("config_dir").value).strip()
        base_config = str(self.get_parameter("base_config").value).strip()
        if not config_dir:
            raise ValueError("config_dir param required (rl_sar policy dir)")
        cfg_path = os.path.join(config_dir, "config.yaml")
        with open(cfg_path) as f:
            cfg = _first_section(yaml.safe_load(f))
        base = {}
        if base_config and os.path.exists(base_config):
            with open(base_config) as f:
                base = _first_section(yaml.safe_load(f))

        def cv(key, default=None):
            if key in cfg:
                return cfg[key]
            if key in base:
                return base[key]
            if default is not None:
                return default
            raise KeyError(f"{key} not in config.yaml or base.yaml")

        # ---- static config ------------------------------------------------
        self.num_dofs = int(cv("num_of_dofs"))
        self.observations = list(cv("observations"))
        self.obs_history = list(cv("observations_history", []))
        self.clip_obs = float(cv("clip_obs", 100.0))
        self.ang_vel_scale = float(cv("ang_vel_scale", 1.0))
        self.dof_pos_scale = float(cv("dof_pos_scale", 1.0))
        self.dof_vel_scale = float(cv("dof_vel_scale", 1.0))
        self.lin_vel_scale = float(cv("lin_vel_scale", 1.0))
        self.commands_scale = np.array(cv("commands_scale", [1.0, 1.0, 1.0]), dtype=np.float32)
        self.action_scale = np.array(cv("action_scale"), dtype=np.float32)
        self.default_dof_pos = np.array(cv("default_dof_pos"), dtype=np.float32)
        self.rl_kp = np.array(cv("rl_kp"), dtype=np.float32)
        self.rl_kd = np.array(cv("rl_kd"), dtype=np.float32)
        self.fixed_kp = np.array(cv("fixed_kp"), dtype=np.float32)
        self.fixed_kd = np.array(cv("fixed_kd"), dtype=np.float32)
        self.torque_limits = np.array(cv("torque_limits"), dtype=np.float32)
        self.clip_a_lo = np.array(cv("clip_actions_lower"), dtype=np.float32)
        self.clip_a_hi = np.array(cv("clip_actions_upper"), dtype=np.float32)
        self.wheel_indices = list(cv("wheel_indices", []))
        joint_mapping = list(cv("joint_mapping"))
        sdk_names = list(cv("joint_names"))   # SDK order (FR,FL,RR,RL)

        # training_names[i] = sdk_names[joint_mapping[i]]  (rl_sar GetState)
        self.training_names = [sdk_names[joint_mapping[i]] for i in range(len(joint_mapping))]
        # training-index → effort-controller-index, by joint name. Only the
        # leg joints (first 12) map to the effort controller; wheel joints (if
        # any) have no effort-controller slot and are dropped.
        train_idx = {n: i for i, n in enumerate(self.training_names)}
        self.effort_from_train = [train_idx.get(n, -1) for n in EFFORT_ORDER]

        # ---- model --------------------------------------------------------
        model_path = os.path.join(config_dir, str(cv("model_name")))
        if not os.path.exists(model_path):
            raise FileNotFoundError(f"policy not found: {model_path}")
        torch.set_num_threads(1)
        self.model = torch.jit.load(model_path, map_location="cpu")
        self.model.eval()

        # per-frame obs dim (sum of term dims, computed from observation spec)
        self.frame_dim = self._frame_dim()
        self.use_history = len(self.obs_history) > 0
        if self.use_history:
            hist_len = max(self.obs_history) + 1
            self.obs_buf = ObservationBuffer(self.frame_dim, hist_len)
            self.total_obs_dim = self.frame_dim * len(self.obs_history)
        else:
            self.obs_buf = None
            self.total_obs_dim = self.frame_dim

        # ---- state --------------------------------------------------------
        self.dof_pos = self.default_dof_pos.copy()  # training order
        self.dof_vel = np.zeros(self.num_dofs, dtype=np.float32)
        self.base_ang_vel = np.zeros(3, dtype=np.float32)
        self.projected_gravity = np.array([0.0, 0.0, -1.0], dtype=np.float32)
        self.commands = np.zeros(3, dtype=np.float32)
        self.last_action = np.zeros(self.num_dofs, dtype=np.float32)
        self.have_joints = False
        self.have_imu = False

        # ---- subs / pubs --------------------------------------------------
        self.create_subscription(JointState,
            str(self.get_parameter("joint_states_topic").value), self._joints_cb, 20)
        self.create_subscription(Imu,
            str(self.get_parameter("imu_topic").value), self._imu_cb, 50)
        self.create_subscription(Twist,
            str(self.get_parameter("cmd_vel_topic").value), self._cmd_cb, 10)
        self.debug_pub = self.create_publisher(Float32MultiArray,
            str(self.get_parameter("debug_actions_topic").value), 10)
        self.publish_efforts = bool(self.get_parameter("publish_efforts").value)
        self.effort_pub = None
        if self.publish_efforts:
            self.effort_pub = self.create_publisher(Float64MultiArray,
                str(self.get_parameter("effort_topic").value), 10)

        self.deadband = float(self.get_parameter("cmd_vel_deadband").value)
        self.stand_up_sec = float(self.get_parameter("stand_up_sec").value)
        self.cmd_hold_sec = float(self.get_parameter("cmd_hold_sec").value)

        rate = float(self.get_parameter("publish_rate_hz").value)
        self.timer = self.create_timer(1.0 / rate, self.step)
        self._start_t = self.get_clock().now()
        self._last_log_t = self.get_clock().now()
        self._anchored = False

        mode = "LIVE" if self.publish_efforts else "DRY"
        self.get_logger().info(
            f"Go2 RL-sar [{mode}]: cfg={os.path.basename(config_dir)} "
            f"model={os.path.basename(model_path)} obs={self.observations} "
            f"frame={self.frame_dim} total={self.total_obs_dim} "
            f"history={self.obs_history} rl_kp={self.rl_kp[0]:.0f}/{self.rl_kd[0]:.1f} "
            f"fixed_kp={self.fixed_kp[0]:.0f}/{self.fixed_kd[0]:.1f}"
        )

    # ---- helpers ----------------------------------------------------------
    def _frame_dim(self) -> int:
        d = 0
        for term in self.observations:
            if term in ("ang_vel", "gravity_vec", "commands"):
                d += 3
            elif term in ("dof_pos", "dof_vel", "actions"):
                d += self.num_dofs
            elif term == "lin_vel":
                d += 3
            else:
                raise ValueError(f"unsupported observation term '{term}'")
        return d

    # ---- subscribers ------------------------------------------------------
    def _joints_cb(self, msg: JointState) -> None:
        idx = {n: i for i, n in enumerate(msg.name)}
        if any(n not in idx for n in self.training_names):
            return
        pos = np.asarray(msg.position, dtype=np.float32)
        vel = (np.asarray(msg.velocity, dtype=np.float32)
               if msg.velocity else np.zeros_like(pos))
        sel = np.array([idx[n] for n in self.training_names])
        self.dof_pos = pos[sel]
        self.dof_vel = vel[sel]
        self.have_joints = True

    def _imu_cb(self, msg: Imu) -> None:
        self.base_ang_vel = np.array(
            [msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z],
            dtype=np.float32)
        self.projected_gravity = project_gravity(
            msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w)
        self.have_imu = True

    def _cmd_cb(self, msg: Twist) -> None:
        v = np.array([msg.linear.x, msg.linear.y, msg.angular.z], dtype=np.float32)
        if np.all(np.abs(v) < self.deadband):
            v[:] = 0.0
        self.commands = v

    # ---- obs assembly (mirror rl_sar ComputeObservation) ------------------
    def _compute_obs(self, feed_cmd: np.ndarray) -> np.ndarray:
        parts = []
        for term in self.observations:
            if term == "ang_vel":
                parts.append(self.base_ang_vel * self.ang_vel_scale)
            elif term == "gravity_vec":
                parts.append(self.projected_gravity)
            elif term == "commands":
                parts.append(feed_cmd * self.commands_scale)
            elif term == "dof_pos":
                rel = self.dof_pos - self.default_dof_pos
                for w in self.wheel_indices:
                    rel[w] = 0.0
                parts.append(rel * self.dof_pos_scale)
            elif term == "dof_vel":
                parts.append(self.dof_vel * self.dof_vel_scale)
            elif term == "actions":
                parts.append(self.last_action)
            elif term == "lin_vel":
                parts.append(np.zeros(3, dtype=np.float32))
        obs = np.concatenate(parts).astype(np.float32)
        return np.clip(obs, -self.clip_obs, self.clip_obs)

    # ---- main tick --------------------------------------------------------
    def step(self) -> None:
        if not (self.have_joints and self.have_imu):
            return
        if not self._anchored:
            self._start_t = self.get_clock().now()
            self._anchored = True
            self.get_logger().info(
                f"Observations live; STANDUP={self.stand_up_sec:.1f}s")

        elapsed = (self.get_clock().now() - self._start_t).nanoseconds / 1e9
        into_policy = elapsed - self.stand_up_sec
        feed_cmd = self.commands.copy()
        if elapsed < self.stand_up_sec or into_policy < self.cmd_hold_sec:
            feed_cmd[:] = 0.0

        # --- forward (always; keeps last_action / history fresh) ---
        frame = self._compute_obs(feed_cmd)
        if self.use_history:
            self.obs_buf.insert(frame)
            obs_in = self.obs_buf.get(self.obs_history)
        else:
            obs_in = frame
        with torch.no_grad():
            raw = self.model(torch.from_numpy(obs_in).unsqueeze(0)).squeeze(0).numpy()
        raw = np.clip(raw.astype(np.float32), self.clip_a_lo, self.clip_a_hi)
        self.last_action = raw

        if elapsed < self.stand_up_sec:
            # getup: stiff PD to default, ignore policy
            tau = self.fixed_kp * (self.default_dof_pos - self.dof_pos) \
                - self.fixed_kd * self.dof_vel
            phase = "STANDUP"
        else:
            # rl_sar ComputeOutput (position actions; wheels handled if present)
            a_scaled = raw * self.action_scale
            pos_a = a_scaled.copy()
            vel_a = np.zeros_like(a_scaled)
            for w in self.wheel_indices:
                pos_a[w] = 0.0
                vel_a[w] = a_scaled[w]
            all_a = pos_a + vel_a
            tau = self.rl_kp * (all_a + self.default_dof_pos - self.dof_pos) \
                - self.rl_kd * self.dof_vel
            phase = "POLICY"
        tau = np.clip(tau, -self.torque_limits, self.torque_limits)

        # --- remap training-order tau → effort-controller order ---
        effort = np.zeros(len(EFFORT_ORDER), dtype=np.float64)
        for k, ti in enumerate(self.effort_from_train):
            if ti >= 0:
                effort[k] = tau[ti]

        dbg = Float32MultiArray(); dbg.data = raw.tolist()
        self.debug_pub.publish(dbg)
        if self.publish_efforts and self.effort_pub is not None:
            cmd = Float64MultiArray(); cmd.data = effort.tolist()
            self.effort_pub.publish(cmd)

        now = self.get_clock().now()
        if (now - self._last_log_t).nanoseconds >= 1_000_000_000:
            self._last_log_t = now
            self.get_logger().info(
                f"[{phase}] cmd=({self.commands[0]:+.2f},{self.commands[1]:+.2f},"
                f"{self.commands[2]:+.2f}) |raw|={float(np.max(np.abs(raw))):4.2f} "
                f"|tau|={float(np.max(np.abs(tau))):5.1f}Nm "
                f"|q-def|={float(np.max(np.abs(self.dof_pos - self.default_dof_pos))):4.2f}rad "
                f"{'[LIVE]' if self.publish_efforts else '[DRY]'}")


def main(argv=None) -> None:
    rclpy.init(args=argv)
    node = Go2RLSar()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
