#!/usr/bin/env python3
"""Single-Go2W MuJoCo CFPA2 launch aligned with the real-robot single-stack.

MuJoCo variant of single_go2w_gazebo_cfpa2.launch.py:
- Replaces Gazebo (gzserver/gzclient) with DFKI mujoco_ros2_control
- Replaces champ_gazebo contact_sensor with mujoco_sensor_bridge contact node
- Adds mujoco_sensor_bridge nodes for LiDAR raycasting and ground-truth odometry
- Keeps all CHAMP controllers, navigation, perception, safety, observability

Platform-specific (MuJoCo, SLAM) logic is inline.
Shared pipeline layers (navigation, safety, observability) are sub-launches from go2w_config.
Startup uses a readiness gate instead of hardcoded TimerAction delays.
"""

from __future__ import annotations

import os
import shlex
import sys

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

sys.path.append(os.path.dirname(__file__))
from modules import _find_mujoco_plugin_dir
from modules.assets import build_dual_robot_stack, build_namespaced_robot_description
from modules.orchestration import build_rviz_node
from modules.slam import build_slam_odom_relay_node
from go2_nav_algorithms.pipeline_components import build_pointcloud_to_laserscan_node


_ws_root = os.path.abspath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", ".."
))


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, key: str) -> str:
    return LaunchConfiguration(key).perform(context)


def _build_cleanup_stale_command() -> str:
    patterns = [
        "ros2 launch go2_gazebo_sim single_go2w_mujoco_cfpa2.launch.py",
        "ros2 launch go2_gazebo_sim dual_go2_modular.launch.py",
        "ros2 launch go2_gazebo_sim dual_go2w_modular.launch.py",
        "mujoco_ros2_control",
        "/mujoco_sensor_bridge/",
        "/go2_nav_algorithms/lib/go2_nav_algorithms/simple_scan_mapper_cpp",
        "/go2w_perception/lib/go2w_perception/qos_bridge.py",
        "/go2w_safety/lib/go2w_safety/autonomy_enabler.py",
        "/go2w_perception/lib/go2w_perception/twist_bridge.py",
        "/go2w_control/lib/go2w_control/go2w_hybrid_cmd_router.py",
        "/go2w_spawn/lib/go2w_spawn/initial_pose_guard.py",
        "/go2w_spawn/lib/go2w_spawn/spawn_entity_direct.py",
        "/go2w_perception/lib/go2w_perception/pointcloud_adapter.py",
        "/go2w_perception/lib/go2w_perception/slam_odom_relay.py",
        "fast_lio_tf_adapter",
        "cloud_world_offset_bridge",
        "/fast_lio/lib/fast_lio/fastlio_mapping",
        "fastlio_mapping",
        "__node:=slam_node",
        "laserMapping",
        "/cfpa2_collaborative_autonomy/lib/cfpa2_collaborative_autonomy/cfpa2_single_robot_node",
        "cfpa2_to_nav2_bridge",
        "exploration_metrics_logger",
        "path_relay.py",
        "stuck_watchdog.py",
        "/champ_base/lib/champ_base/quadruped_controller_node",
        "/champ_base/lib/champ_base/state_estimation_node",
        "/robot_localization/ekf_node",
        "/robot_state_publisher",
        "/opt/ros/.*/lib/controller_manager/spawner",
        "/pointcloud_to_laserscan_node",
        "octomap_server_node",
        "mapper_node",
        "nvblox_frontend_mapper",
        "planner_server",
        "controller_server",
        "bt_navigator",
        "behavior_server",
        "lifecycle_manager_navigation",
        "wall_collision_checker.py",
        "supervisor_panic_node.py",
        "autonomy_enabler.py",
    ]

    command = [
        "SELF=$$; PARENT=$PPID; ",
        "kill_pattern(){ ",
        "  PATTERN=\"$1\"; SIGNAL=\"$2\"; ",
        "  for PID in $(pgrep -f \"$PATTERN\" 2>/dev/null || true); do ",
        "    [ \"$PID\" = \"$SELF\" ] && continue; ",
        "    [ \"$PID\" = \"$PARENT\" ] && continue; ",
        "    kill -\"$SIGNAL\" \"$PID\" 2>/dev/null || true; ",
        "  done; ",
        "}; ",
    ]
    for pattern in patterns:
        command.append(f"kill_pattern {shlex.quote(pattern)} TERM; ")
    command.append("sleep 1; ")
    for pattern in patterns:
        command.append(f"kill_pattern {shlex.quote(pattern)} KILL; ")
    command.append("sleep 0.5")
    return "".join(command)


def _launch_setup(context):
    use_sim_time = _as_bool(_get(context, "use_sim_time"))
    gui = _as_bool(_get(context, "gui"))
    rviz = _as_bool(_get(context, "rviz"))
    cleanup_stale = _as_bool(_get(context, "cleanup_stale"))
    enable_assets = _as_bool(_get(context, "enable_assets"))
    enable_perception = _as_bool(_get(context, "enable_perception"))
    enable_slam = _as_bool(_get(context, "enable_slam"))
    enable_control = _as_bool(_get(context, "enable_control"))
    enable_navigation = _as_bool(_get(context, "enable_navigation"))
    # has_wheels=False spawns no wheel_velocity_controller — used by the
    # pure Go2 (non-W) variant which has passive spherical feet rather
    # than actively-driven wheels.
    has_wheels = _as_bool(_get(context, "has_wheels"))
    # rl_policy: swap CHAMP out for the Isaac-Lab-trained ONNX flat policy
    # (from scripts/go2_rl_policy_node.py). Pure Go2 only — wheel platform
    # isn't covered by the community checkpoint.
    rl_policy = _as_bool(_get(context, "rl_policy"))
    if rl_policy and has_wheels:
        raise ValueError("rl_policy:=true requires has_wheels:=false "
                         "(policy is Go2 leg-only).")
    use_fast_lio = _as_bool(_get(context, "use_fast_lio"))
    pointcloud_noise_enabled = _as_bool(_get(context, "pointcloud_noise_enabled"))
    pointcloud_noise_mean = _get(context, "pointcloud_noise_mean").strip() or "0.0"
    pointcloud_noise_stddev = _get(context, "pointcloud_noise_stddev").strip() or "0.015"

    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"
    spawn_x = _get(context, "spawn_x")
    spawn_y = _get(context, "spawn_y")
    spawn_yaw = _get(context, "spawn_yaw")
    cfpa2_w_ig = _get(context, "cfpa2_w_ig")
    cfpa2_w_c = _get(context, "cfpa2_w_c")
    cfpa2_w_momentum = _get(context, "cfpa2_w_momentum")
    cfpa2_min_utility = _get(context, "cfpa2_min_utility")

    # Pass-through args for navigation sub-launch
    map_frame = _get(context, "map_frame").strip() or "world"
    waypoint_input_suffix = _get(context, "waypoint_input_suffix").strip() or "/way_point_coord"
    cfpa2_goal_topic_suffix = _get(context, "cfpa2_goal_topic_suffix").strip() or "/way_point_coord"
    cfpa2_switch_hysteresis = _get(context, "cfpa2_switch_hysteresis").strip()
    max_linear_speed = _get(context, "max_linear_speed").strip()
    require_settle_before_motion = _get(context, "require_settle_before_motion").strip()
    nav_map_topic = _get(context, "nav_map_topic").strip()
    nav_config = _get(context, "nav_config").strip()
    nav_backend = _get(context, "nav_backend").strip()
    registered_scan_topic = _get(context, "registered_scan_topic").strip()
    far_max_speed = _get(context, "far_max_speed").strip()
    far_robot_id = _get(context, "far_robot_id").strip()
    ramp_force_legged_enabled = _as_bool(_get(context, "ramp_force_legged_enabled"))
    ramp_force_wheel_enabled = _as_bool(_get(context, "ramp_force_wheel_enabled"))
    ramp_goal_mode_topic = _get(context, "ramp_goal_mode_topic").strip()
    ramp_goal_stale_sec = float(_get(context, "ramp_goal_stale_sec"))
    ramp_force_max_vx_mps = float(_get(context, "ramp_force_max_vx_mps"))
    ramp_force_max_yaw_rate_rps = float(_get(context, "ramp_force_max_yaw_rate_rps"))
    pitch_force_wheel_enabled = _as_bool(_get(context, "pitch_force_wheel_enabled"))
    pitch_force_wheel_threshold_rad = float(_get(context, "pitch_force_wheel_threshold_rad"))
    pitch_climb_vx_mps = float(_get(context, "pitch_climb_vx_mps"))

    # MuJoCo MJCF model path
    mujoco_model_path = _get(context, "mujoco_model_path").strip()
    odom_bridge_publish_tf = _as_bool(_get(context, "odom_bridge_publish_tf"))

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    champ_base_pkg = get_package_share_directory("champ_base")

    # Override the default with rviz_config:=<absolute path> launch arg.
    # nav_test_hil_desktop.sh sets it to .../rviz/nav_test.rviz which has
    # GridMap displays for elevation_map_raw + filtered + the SetGoal tool
    # bound to /robot/goal_pose (2D Goal Pose button → Nav2 directly).
    rviz_config_arg = _get(context, "rviz_config").strip()
    rviz_config = rviz_config_arg or os.path.join(
        go2_gazebo_pkg, "rviz", "single_go2w_gazebo_cfpa2.rviz")
    go2w_config_pkg = get_package_share_directory("go2w_config")
    hybrid_motion_config = os.path.join(go2w_config_pkg, "config", "control", "go2w_hybrid_motion.yaml")
    slam_config = os.path.join(go2w_config_pkg, "config", "slam", "pointlio_gazebo.yaml")

    # Sub-launch paths
    nav_launch = os.path.join(go2w_config_pkg, "launch", "navigation.launch.py")
    safety_launch = os.path.join(go2w_config_pkg, "launch", "safety.launch.py")
    obs_launch = os.path.join(go2w_config_pkg, "launch", "observability.launch.py")

    # has_wheels=True → Go2W xacro (continuous wheel foot joints + wheel actuators).
    # has_wheels=False → pure Go2 xacro (fixed foot joints, 12-joint leg only;
    # pair with demo1_go2_real.xml / demo3_go2_real.xml built from Menagerie
    # body — calf-tip-centered feet, not the broken side-offset stripped-Go2W).
    xacro_rel = (
        ("go2w", "go2w_description_3d_lidar.xacro") if has_wheels
        else ("go2", "go2_description_3d_lidar.xacro")
    )
    base_robot_description = xacro.process_file(
        os.path.join(go2_gazebo_pkg, "urdf", *xacro_rel),
        mappings={
            "pointcloud_noise_enabled": "true" if pointcloud_noise_enabled else "false",
            "pointcloud_noise_mean": pointcloud_noise_mean,
            "pointcloud_noise_stddev": pointcloud_noise_stddev,
        },
    ).documentElement.toxml()
    # Select ros_control configuration by mode.
    # - Go2W:  ros_control_go2w_robot.yaml          joint_trajectory_controller + wheels
    # - Go2:   ros_control_go2_robot.yaml           joint_trajectory_controller (CHAMP PD)
    # - Go2+RL: ros_control_go2_robot_rl_effort.yaml
    #          ForwardCommandController on the effort interface — the RL node
    #          computes τ = kp·(q_des − q) − kd·dq in Python with the exact
    #          training-time gains (kp=20/kd=0.5), matching Unitree LowCmd
    #          semantics. Bypasses joint_trajectory_controller's internal PID.
    if has_wheels:
        ros_control_yaml = "ros_control_go2w_robot.yaml"
    elif rl_policy:
        ros_control_yaml = "ros_control_go2_robot_rl_effort.yaml"
    else:
        ros_control_yaml = "ros_control_go2_robot.yaml"
    robot_description = build_namespaced_robot_description(
        base_robot_description,
        robot_ns,
        os.path.join(go2_gazebo_pkg, "config", "ros_control", ros_control_yaml),
    ).replace(
        "gazebo_ros2_control/GazeboSystem",
        "mujoco_ros2_control/MujocoSystem",
    )

    # MuJoCo plugin dir (needed for STL/OBJ mesh decoders etc.)
    mujoco_plugin_dir = _find_mujoco_plugin_dir()

    joints_config = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "joints.yaml")
    links_config = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "links.yaml")
    gait_config = os.path.join(go2_gazebo_pkg, "config", "champ", "go2w", "gait.yaml")
    ekf_base_to_footprint = os.path.join(champ_base_pkg, "config", "ekf", "base_to_footprint.yaml")
    ekf_footprint_to_odom = os.path.join(champ_base_pkg, "config", "ekf", "footprint_to_odom.yaml")

    # ros2_control config for MuJoCo
    # DFKI plugin reads controller types from this manifest (distinct from the
    # Gazebo-side YAML loaded via the xacro plugin block). RL mode swaps in
    # a Forward­Command­Controller variant so the RL node can publish raw
    # torques on the effort interface directly.
    ros2_control_config = os.path.join(
        go2_gazebo_pkg, "mujoco",
        "go2_rl_mujoco_controllers.yaml" if rl_policy else "go2w_mujoco_controllers.yaml"
    )

    actions = [
        LogInfo(
            msg=(
                "[single_go2w_mujoco_cfpa2] "
                f"ns={robot_ns} assets={enable_assets} perception={enable_perception} "
                f"slam={enable_slam} control={enable_control} navigation={enable_navigation} "
                f"use_fast_lio={use_fast_lio} pointcloud_noise={pointcloud_noise_enabled} "
                f"pointcloud_noise_stddev={pointcloud_noise_stddev}"
            )
        )
    ]

    # ── Platform: MuJoCo (replaces Gazebo gzserver/gzclient) ──
    if cleanup_stale:
        actions.append(ExecuteProcess(cmd=["bash", "-lc", _build_cleanup_stale_command()], output="screen"))

    # MuJoCo simulation node
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        namespace=robot_ns,
        parameters=[
            {"robot_description": robot_description},
            ros2_control_config,
            {"robot_model_path": mujoco_model_path},
            {"simulation_frequency": 500.0},
            {"real_time_factor": 1.0},
            {"clock_publisher_frequency": 100.0},
            {"show_gui": gui},
        ],
        remappings=[
            (f"/{robot_ns}/controller_manager/robot_description", f"/{robot_ns}/robot_description"),
        ],
        additional_env={
            "MUJOCO_PLUGIN_DIR": mujoco_plugin_dir,
            # Apply the "home" keyframe in the MJCF on startup when running
            # the RL policy, so leg joints spawn at the Menagerie stance
            # rather than 0 (which would extend calves through the ground).
            **({"MUJOCO_INIT_KEYFRAME": "home"} if rl_policy else {}),
        },
        output="screen",
    )
    actions.append(
        TimerAction(period=3.0, actions=[mujoco_node])
    )

    # LiDAR raycasting is now handled inside mujoco_ros2_control via the
    # built-in LidarSensor plugin (lidar_sensor.cpp).  It auto-discovers
    # sites named "unitree_l1" or "livox_mid360" in the MJCF and publishes
    # /{ns}/mujoco_lidar_sensor/registered_scan using the live mjData —
    # zero sync lag with the physics sim.
    # The external mujoco_sensor_bridge lidar node is no longer needed.

    # MuJoCo sensor bridge: ground-truth odometry
    actions.append(
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="mujoco_sensor_bridge",
                    executable="mujoco_odom_bridge",
                    namespace=robot_ns,
                    name="mujoco_odom_bridge",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"mjcf_path": mujoco_model_path},
                        {"publish_rate": 50.0},
                        {"base_body_name": "base_link"},
                        {"odom_frame": "odom"},
                        {"base_frame": "base_link"},
                        {"publish_tf": odom_bridge_publish_tf},
                        # DFKI sub-nodes now inherit parent ns; IMU site name is imu_site
                        {"pose_topic": "base_link_site_pose_sensor/pose"},
                        {"imu_topic": "imu_imu_sensor/imu"},
                        {"republish_imu_topic": "imu/data"},
                        # Sim IMU low-pass (emulates real Mid-360 hardware filter).
                        # The raw MuJoCo accel + CHAMP standing controller spike to
                        # 4-23 m/s^2 on a still robot; fast_lio integrates that to
                        # divergence. Set at LAUNCH (not live) so fast_lio inits
                        # against a consistent IMU. Tune via SIM_IMU_LPF_HZ env.
                        {"imu_lpf_cutoff_hz": float(os.environ.get("SIM_IMU_LPF_HZ", "5.0"))},
                    ],
                    remappings=[
                        ("/tf", f"/{robot_ns}/tf"),
                        ("/tf_static", f"/{robot_ns}/tf_static"),
                    ],
                    output="screen",
                ),
            ],
        )
    )

    # MuJoCo sensor bridge: foot contact sensing (replaces champ_gazebo contact_sensor)
    actions.append(
        TimerAction(
            period=5.0,
            actions=[
                Node(
                    package="mujoco_sensor_bridge",
                    executable="mujoco_contact_node",
                    namespace=robot_ns,
                    name="mujoco_contact_bridge",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"mjcf_path": mujoco_model_path},
                        {"publish_rate": 50.0},
                        links_config,
                    ],
                    output="screen",
                ),
            ],
        )
    )

    if rviz:
        rviz_tf_remaps = [
            ("/tf", f"/{robot_ns}/tf"),
            ("/tf_static", f"/{robot_ns}/tf_static"),
        ]
        actions.append(
            TimerAction(
                period=7.0,
                actions=[build_rviz_node(rviz_config, use_sim_time,
                                          name="rviz2_single_go2w",
                                          remappings=rviz_tf_remaps)],
            )
        )

    # ── Platform: Robot controllers (CHAMP, EKF, RSP, ros2_control spawners) ──
    # In MuJoCo the robot is embedded in the MJCF scene. use_mujoco=True skips
    # Gazebo-only nodes (spawn_entity, initial_pose_guard, champ contact_sensor).
    # Contact sensing is handled by mujoco_contact_node instead.
    # Predefine the wait_for_settle node so the if-enable_assets block can
    # reference it in its OnProcessExit handler (which fires when
    # stack_handles["stand_up_node"] exits).
    _wait_for_settle_imu_topic = f"/{robot_ns}/imu/data"
    _wait_for_settle_signal = f"/{robot_ns}/platform_ready"
    wait_for_settle = Node(
        package="go2w_spawn",
        executable="wait_for_ready.py",
        name="wait_for_settle",
        parameters=[
            {
                "mode": "imu_stable",
                "imu_topic": _wait_for_settle_imu_topic,
                "angular_velocity_threshold": 0.05,
                "stable_count": 10,
                "check_interval_sec": 0.5,
                "timeout_sec": 60.0,
                "gate_name": "settle",
                "signal_topic": _wait_for_settle_signal,
                "post_signal_linger_sec": 10.0,
                "use_sim_time": False,
            }
        ],
        output="screen",
    )

    if enable_assets:
        stack_actions, stack_handles = build_dual_robot_stack(
            ns=robot_ns,
            spawn_x=spawn_x,
            spawn_y=spawn_y,
            spawn_yaw=spawn_yaw,
            use_sim_time=use_sim_time,
            robot_description=robot_description,
            joints_config=joints_config,
            links_config=links_config,
            gait_config=gait_config,
            ekf_base_to_footprint=ekf_base_to_footprint,
            ekf_footprint_to_odom=ekf_footprint_to_odom,
            joint_state_spawner_delay_sec=1.0,
            effort_spawner_delay_sec=1.2,
            standup_delay_sec=4.0,
            pose_guard_hold_sec=12.0,
            activate_controllers_on_spawn=True,
            stand_up_joint_preset="go2w" if has_wheels else "go2",
            cmd_vel_input_topic="cmd_vel_legged",
            wheel_controller_name=(
                f"{robot_ns}_wheel_velocity_controller" if has_wheels else ""
            ),
            rsp_publish_frequency=100.0,
            use_mujoco=True,
            # rl_policy mode owns the leg loop, so skip CHAMP's
            # quadruped_controller + state_estimator + EKFs. Keep RSP,
            # controller spawners, and one-shot stand-up trajectory.
            skip_champ=rl_policy,
            return_handles=True,
        )
        actions.append(TimerAction(period=5.0, actions=stack_actions))
        # Cross-host lifecycle handshake (2026-05-20):
        # stand_up_node (patched to block 20s until trajectory complete) exits
        # → start wait_for_settle (tighter |ω|<0.05 for 5s) → publish latched
        # /robot/platform_ready → Jetson run_jetson_hil.sh exec's autonomy
        # launch only AFTER receiving this signal. Solves the Fast-LIO
        # IMU-init-during-stand-up failure (z drifted to +6m).
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=stack_handles["stand_up_node"],
                    on_exit=[wait_for_settle],
                )
            )
        )

    # ── Readiness gate: wait for robot platform to be alive ──
    nav_odom_topic = f"/{robot_ns}/odom/nav"
    planning_scan_topic = f"/{robot_ns}/scan_3d"
    perception_cloud_topic = f"/{robot_ns}/registered_scan_reliable"

    wait_for_platform = Node(
        package="go2w_spawn",
        executable="wait_for_ready.py",
        name="wait_for_platform",
        parameters=[
            {
                "mode": "imu_stable",
                "imu_topic": f"/{robot_ns}/imu/data",
                "angular_velocity_threshold": 0.15,
                "stable_count": 5,
                "check_interval_sec": 1.0,
                "timeout_sec": 60.0,
                "gate_name": "platform",
                "use_sim_time": False,
            }
        ],
        output="screen",
    )

    # (Second gate `wait_for_settle` is defined earlier — moved above the
    # build_dual_robot_stack block so its name is in scope when we wire the
    # OnProcessExit handler against stack_handles["stand_up_node"].)

    # ── Robot actions: started once platform is ready ──
    robot_actions = []
    # rl_policy only: leg-loop actions (twist_bridge + RL node) that must engage
    # EARLY (as soon as the effort controller is up) to hold the robot at home
    # before it sags — fast_lio + the rest of robot_actions start later, after
    # the RL HOLD/STANDUP has the base steady (fast_lio must init on a settled
    # robot). Empty in non-rl_policy modes.
    rl_actions = []

    # -- Sim-specific control routing --
    if enable_control and rl_policy:
        # RL policy node: subscribes to joint_states/imu/cmd_vel_legged, runs
        # the ONNX flat policy at 50 Hz, publishes joint torques directly to
        # /robot/robot_joint_group_effort_controller/commands. We still need
        # twist_bridge to convert FAR pathFollower's TwistStamped output on
        # /cmd_vel_stamped into Twist on /cmd_vel_legged that the RL node
        # consumes.
        rl_actions.append(
            Node(
                package="go2w_perception",
                executable="twist_bridge.py",
                namespace=robot_ns,
                remappings=[("cmd_vel", "cmd_vel_legged")],
                output="screen",
            )
        )
        # The RL node needs cmu_env's python (onnxruntime/torch/numpy live there).
        # sys.executable is NOT it — the `ros2` CLI shebang is /usr/bin/python3,
        # so the OpaqueFunction runs under system python. CONDA_PREFIX is also
        # unreliable (not always exported into the launch env). Resolve the
        # cmu_env interpreter robustly + portably: CONDA_PREFIX first, then the
        # standard conda-distro env locations under $HOME, then sys.executable.
        def _resolve_cmu_python():
            cands = []
            cp = os.environ.get("CONDA_PREFIX", "")
            if cp:
                cands.append(os.path.join(cp, "bin", "python3"))
            home = os.path.expanduser("~")
            for distro in ("miniforge3", "mambaforge", "miniconda3", "anaconda3"):
                cands.append(os.path.join(home, distro, "envs", "cmu_env", "bin", "python3"))
            for c in cands:
                if os.path.exists(c):
                    return c
            return sys.executable
        rl_python = _resolve_cmu_python()
        rl_sar_config = _get(context, "rl_sar_config").strip()
        if rl_sar_config:
            # Config-driven path: run a vendored rl_sar policy (go2/robot_lab,
            # go2/himloco, …) via go2_rl_sar_node.py. Everything — obs terms,
            # scales, defaults, gains, joint mapping, history — comes from the
            # rl_sar config.yaml in this dir; base.yaml (joint_names) sits one
            # level up. This is the recommended path for rough-terrain cmd_vel
            # locomotion; the IL flat-policy path below is the legacy fallback.
            rl_node_path = os.path.join(_ws_root, "scripts/runtime/go2_rl_sar_node.py")
            rl_base_config = os.path.join(os.path.dirname(rl_sar_config.rstrip("/")),
                                          "base.yaml")
            rl_cmd = [
                rl_python, "-u", rl_node_path,
                "--ros-args", "-r", f"__ns:=/{robot_ns}",
                "-p", "publish_efforts:=true",
                "-p", f"config_dir:={rl_sar_config}",
                "-p", f"base_config:={rl_base_config}",
                # getup window: nothing holds the robot before the node engages
                # (~8 s), so it sags. STANDUP runs a stiff fixed-PD (from the
                # config) to the default pose to lift it before the policy + cmd
                # take over.
                "-p", "stand_up_sec:=5.0",
                "-p", "cmd_hold_sec:=3.0",
            ]
        else:
            # Legacy IL flat/rough policy path (scripts/runtime/go2_rl_policy_node.py).
            rl_node_path = os.path.join(_ws_root, "scripts/runtime/go2_rl_policy_node.py")
            rl_model_path = _get(context, "rl_model_path").strip()
            rl_cmd = [
                rl_python, "-u", rl_node_path,
                "--ros-args", "-r", f"__ns:=/{robot_ns}",
                "-p", "publish_efforts:=true",
                "-p", "stand_up_sec:=5.0",
                "-p", "cmd_hold_sec:=3.0",
            ]
            # Optional explicit policy. Empty → node default (flat_policy_v6).
            if rl_model_path:
                rl_cmd += ["-p", f"model_path:={rl_model_path}"]
        # The RL node + twist_bridge are gated EARLY (see the rl_policy branch of
        # the readiness gate below) so they engage right after the effort
        # controller is up, minimising sag time. No internal TimerAction here.
        rl_actions.append(
            ExecuteProcess(
                cmd=rl_cmd,
                name="go2_rl_policy",
                output="screen",
            )
        )
    elif enable_control:
        robot_actions.append(
            Node(
                package="go2w_perception",
                executable="twist_bridge.py",
                namespace=robot_ns,
                remappings=[
                    # Use RELATIVE keys — the twist_bridge script declares
                    # `cmd_vel_stamped` / `cmd_vel` as relative topics that
                    # resolve to `/{ns}/cmd_vel_stamped` / `/{ns}/cmd_vel`
                    # after namespacing. Absolute `/cmd_vel` keys never match
                    # the resolved name and the remap is a silent no-op.
                    #
                    # Go2W: keep the pub on `/{ns}/cmd_vel` so the hybrid
                    # router can split leg vs wheel commands.
                    # Go2 (no router): publish straight to `cmd_vel_legged`
                    # where CHAMP quadruped_controller listens.
                    *(
                        []
                        if has_wheels
                        else [("cmd_vel", "cmd_vel_legged")]
                    ),
                ],
                output="screen",
            )
        )
        if has_wheels:
            robot_actions.append(
                Node(
                    package="go2w_control",
                    executable="go2w_hybrid_cmd_router.py",
                    namespace=robot_ns,
                    name="go2w_hybrid_cmd_router",
                    parameters=[
                        hybrid_motion_config,
                        {
                            "wheel_command_topic": f"{robot_ns}_wheel_velocity_controller/commands",
                            "ramp_force_legged_enabled": ramp_force_legged_enabled,
                            "ramp_force_wheel_enabled": ramp_force_wheel_enabled,
                            "ramp_goal_mode_topic": ramp_goal_mode_topic,
                            "ramp_goal_stale_sec": ramp_goal_stale_sec,
                            "ramp_force_max_vx_mps": ramp_force_max_vx_mps,
                            "ramp_force_max_yaw_rate_rps": ramp_force_max_yaw_rate_rps,
                            # Pitch-triggered force-wheel (goal-independent ramp
                            # climb): when the body pitches past the threshold,
                            # hold sustained wheel mode + floor the climb speed.
                            "pitch_force_wheel_enabled": pitch_force_wheel_enabled,
                            "pitch_force_wheel_threshold_rad": pitch_force_wheel_threshold_rad,
                            "pitch_climb_vx_mps": pitch_climb_vx_mps,
                            "pitch_odom_topic": "odom/nav",
                        },
                    ],
                    output="screen",
                )
            )
        # Pure Go2 (has_wheels=false): no wheel routing — cmd_vel → twist_bridge
        # → cmd_vel_legged goes straight to CHAMP.

    # -- Sim-specific perception --
    if enable_perception:
        robot_actions.append(
            Node(
                package="go2w_perception",
                executable="qos_bridge.py",
                namespace=robot_ns,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": f"/{robot_ns}/mujoco_lidar_sensor/registered_scan"},
                    {"output_topic": f"/{robot_ns}/registered_scan_reliable"},
                ],
                output="screen",
            )
        )

    # pointcloud_adapter is needed for HIL (desktop-side sensor publishing)
    # even when enable_slam=false (fast_lio runs on Jetson, subscribes to
    # the /robot/velodyne_points this adapter publishes). Gate it on
    # perception+use_fast_lio only; the fast_lio Node below stays gated on
    # all three flags.
    if enable_perception and use_fast_lio:
        robot_actions.append(
            Node(
                package="go2w_perception",
                executable="pointcloud_adapter.py",
                namespace=robot_ns,
                name="pointcloud_adapter",
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": perception_cloud_topic},
                    {"output_topic": f"/{robot_ns}/velodyne_points"},
                    {"num_rings": 16},
                ],
                output="screen",
            )
        )
    # fast_lio stays gated on enable_slam so HIL desktop (enable_slam=false)
    # skips local fast_lio (Jetson runs its own).
    if enable_perception and enable_slam and use_fast_lio:
        robot_actions.append(
            Node(
                package="fast_lio",
                executable="fastlio_mapping",
                namespace=robot_ns,
                name="slam_node",
                parameters=[slam_config, {"use_sim_time": use_sim_time}],
                remappings=[
                    ("/velodyne_points", f"/{robot_ns}/velodyne_points"),
                    ("/imu/data", f"/{robot_ns}/imu/data"),
                    ("/Odometry", f"/{robot_ns}/Odometry"),
                    ("/cloud_registered_body", f"/{robot_ns}/cloud_registered_body"),
                    # Fast-LIO also publishes /cloud_registered in camera_init
                    # frame (SLAM-internal world). We pull it into our namespace
                    # as an intermediate for the camera_init → world offset
                    # applied by pointcloud_frame_bridge_fast.
                    ("/cloud_registered", f"/{robot_ns}/cloud_registered_camera_init"),
                ],
                output="screen",
            )
        )

    # -- Sim-specific SLAM odom relay --
    # SIM_GT_ODOM=1 (sim test harness only): feed /odom/nav from MuJoCo
    # ground truth instead of Fast-LIO. Fast-LIO in the standalone desktop
    # sim is NOT lifecycle-gated, so it inits gravity DURING the stand-up leg
    # motion → z drifts upward (observed 0.28 → 1.3 m, climbing) which makes
    # /odom/nav garbage for CFPA2/Nav2 progress checks. The body-frame cloud
    # (cloud_registered_body) is independent of Fast-LIO's global pose, so the
    # trav grid (built via GT TF + body cloud) stays REAL perception. This
    # isolates exploration validation from sim-SLAM init flakiness — the
    # established 2026-05-18 GT-passthrough pattern. The real robot / HIL path
    # keeps gated Fast-LIO (validated to z=-0.14 m) and ignores this env.
    _sim_gt_odom = os.environ.get("SIM_GT_ODOM", "0") == "1"
    if enable_slam:
        if use_fast_lio and not _sim_gt_odom:
            robot_actions.append(
                build_slam_odom_relay_node(
                    ns=robot_ns,
                    use_sim_time=use_sim_time,
                    name="slam_odom_relay",
                    input_topic=f"/{robot_ns}/Odometry",
                    gt_topic=f"/{robot_ns}/odom/ground_truth",
                    output_topic=nav_odom_topic,
                    output_frame_id="world",
                    output_child_frame_id="base_link",
                    bootstrap_from_gt=True,
                    require_gt_for_alignment=True,
                )
            )
        else:
            robot_actions.append(
                build_slam_odom_relay_node(
                    ns=robot_ns,
                    use_sim_time=use_sim_time,
                    name="gt_odom_relay",
                    input_topic=f"/{robot_ns}/odom/ground_truth",
                    output_topic=nav_odom_topic,
                    output_frame_id="world",
                    output_child_frame_id="base_link",
                )
            )

    # -- Sim-specific scan pipeline --
    if enable_perception:
        if use_fast_lio:
            scan_cloud_topic = f"/{robot_ns}/cloud_registered_body"
            # Standalone-sim only: when fast_lio runs locally on desktop, this
            # static gives the scan pipeline a TF path body → base_link. In
            # HIL split (enable_slam=false → Jetson owns SLAM), the Jetson
            # publishes camera_init → body → base_link itself, so a desktop-
            # side imu→body static gives body TWO parents (imu locally +
            # camera_init from Jetson) → TF rejects with "two unconnected
            # trees" or one branch gets dropped. Gate on enable_slam.
            if enable_slam:
                robot_actions.append(
                    Node(
                        package="tf2_ros",
                        executable="static_transform_publisher",
                        namespace=robot_ns,
                        name="imu_to_body_tf",
                        arguments=[
                            "--frame-id", "imu", "--child-frame-id", "body",
                            "--x", "0", "--y", "0", "--z", "0",
                            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
                        ],
                        remappings=[("/tf_static", f"/{robot_ns}/tf_static")],
                        parameters=[{"use_sim_time": use_sim_time}],
                        output="log",
                    )
                )
        else:
            scan_cloud_topic = perception_cloud_topic

        robot_actions.append(
            build_pointcloud_to_laserscan_node(
                ns=robot_ns,
                use_sim_time=use_sim_time,
                extra_params={
                    "target_frame": "base_link",
                    # Height band filter in base_link frame. Tuned for
                    # Livox MID-360 on back-top mount (z=+0.12 body frame).
                    # If switching to Unitree L1 at chin (z=-0.04 body frame),
                    # raise min_height to ~0.15–0.30 to filter close-range
                    # ground hits (L1 is 16 cm lower → ground rays hit at
                    # 1.5 m instead of 5.9 m).
                    "min_height": 0.05,
                    "max_height": 0.60,
                    "range_min": 0.10,
                    "range_max": 8.0,
                },
                remappings=[
                    ("/tf", f"/{robot_ns}/tf"),
                    ("/tf_static", f"/{robot_ns}/tf_static"),
                    ("cloud_in", scan_cloud_topic),
                    ("scan", planning_scan_topic),
                ],
            )
        )

    # -- Shared navigation sub-launch (mapper + cfpa2 + default_nav) --
    if enable_navigation:
        nav_args = {
            "robot_namespace": robot_ns,
            "use_sim_time": str(use_sim_time),
            "map_frame": map_frame,
            "remap_tf": "true",
            "scan_topic": planning_scan_topic,
            "odom_topic": nav_odom_topic,
            "waypoint_input_suffix": waypoint_input_suffix,
            "cfpa2_goal_topic_suffix": cfpa2_goal_topic_suffix,
            "cfpa2_w_ig": cfpa2_w_ig,
            "cfpa2_w_c": cfpa2_w_c,
            "cfpa2_w_momentum": cfpa2_w_momentum,
            "cfpa2_min_utility": cfpa2_min_utility,
            "cfpa2_executable_suffix": _get(context, "cfpa2_executable_suffix"),
        }
        if cfpa2_switch_hysteresis:
            nav_args["cfpa2_switch_hysteresis"] = cfpa2_switch_hysteresis
        if max_linear_speed:
            nav_args["max_linear_speed"] = max_linear_speed
        if require_settle_before_motion:
            nav_args["require_settle_before_motion"] = require_settle_before_motion
        if nav_map_topic:
            nav_args["nav_map_topic"] = nav_map_topic
        if nav_config:
            nav_args["nav_config"] = nav_config
        if nav_backend:
            nav_args["nav_backend"] = nav_backend
        if registered_scan_topic:
            nav_args["registered_scan_topic"] = registered_scan_topic
        if far_max_speed:
            nav_args["far_max_speed"] = far_max_speed
        if far_robot_id:
            nav_args["far_robot_id"] = far_robot_id

        robot_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(nav_launch),
                launch_arguments=nav_args.items(),
            )
        )

    # -- Shared safety sub-launch (wall checker + autonomy enabler) --
    if enable_control:
        robot_actions.append(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(safety_launch),
                launch_arguments={
                    "robot_namespace": robot_ns,
                    "use_sim_time": str(use_sim_time),
                    "scan_topic": planning_scan_topic,
                    "autonomy_startup_delay": "8.0",
                }.items(),
            )
        )

    # -- Shared observability sub-launch --
    robot_actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(obs_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "use_sim_time": str(use_sim_time),
                "experiment_name": "single_go2w",
            }.items(),
        )
    )

    # ── Wire readiness gate -> robot actions ──
    if rl_policy:
        # rl_policy owns the leg loop. CHAMP's stand_up_slowly node publishes a
        # JointTrajectory to <effort_ctrl>/joint_trajectory, but the RL effort
        # YAML loads a forward-command (Float64MultiArray) controller with no
        # joint_trajectory interface — so stand_up_slowly waits for a subscriber
        # that never appears and NEVER exits. Gating on its exit (the CHAMP path
        # below) would hang the RL node forever. Two fixed timers instead:
        #   • rl_actions EARLY (8 s) — the effort controller is up by ~7 s, so
        #     the RL node engages and its STANDUP phase lifts the sagged base
        #     back to home before it sprawls further.
        #   • robot_actions LATER (16 s) — fast_lio + perception start once the
        #     RL HOLD phase has the base steady, so fast_lio inits on a settled
        #     robot (init during the standup lift would corrupt its gravity est).
        if rl_actions:
            actions.append(TimerAction(period=8.0, actions=rl_actions))
        if robot_actions:
            actions.append(TimerAction(period=16.0, actions=robot_actions))
    elif robot_actions:
        if enable_assets and "stand_up_node" in stack_handles:
            # Gate fast_lio (+ the rest of robot_actions) on stand-up COMPLETION
            # plus a settle delay — NOT the early IMU-stable wait_for_platform
            # gate. wait_for_platform starts at t~0 and fires during the
            # lying-still-at-spawn phase (|w|~0 while lying down) BEFORE stand-up
            # even begins, so fast_lio inits its gravity vector pre-stand-up and
            # the stand-up leg motion then corrupts the estimate -> divergence
            # (the "stand up FIRST, then fast_lio" requirement). stand_up_node is
            # patched to block until the stand-up trajectory completes, so its
            # exit == robot standing; +6 s covers the |w|<0.05 settle window.
            actions.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=stack_handles["stand_up_node"],
                        on_exit=[TimerAction(period=6.0, actions=robot_actions)],
                    )
                )
            )
        else:
            actions.append(wait_for_platform)
            actions.append(
                RegisterEventHandler(
                    OnProcessExit(
                        target_action=wait_for_platform,
                        on_exit=robot_actions,
                    )
                )
            )

    return actions


def generate_launch_description():
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    # Default MJCF: the standalone Go2W scene (robot + flat ground).
    # The VLM launch overrides this with the combined world+robot scene.
    default_mujoco_model = os.path.join(go2_gazebo_pkg, "mujoco", "go2w.xml")
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument(
                "rviz_config", default_value="",
                description="Absolute path to .rviz config; empty → default "
                            "single_go2w_gazebo_cfpa2.rviz"),
            DeclareLaunchArgument("cleanup_stale", default_value="true"),
            DeclareLaunchArgument("enable_assets", default_value="true"),
            DeclareLaunchArgument("enable_perception", default_value="true"),
            DeclareLaunchArgument("enable_slam", default_value="true"),
            DeclareLaunchArgument("use_fast_lio", default_value="true"),
            DeclareLaunchArgument("pointcloud_noise_enabled", default_value="false"),
            DeclareLaunchArgument("pointcloud_noise_mean", default_value="0.0"),
            DeclareLaunchArgument("pointcloud_noise_stddev", default_value="0.015"),
            DeclareLaunchArgument("enable_control", default_value="true"),
            DeclareLaunchArgument("enable_navigation", default_value="true"),
            DeclareLaunchArgument("has_wheels", default_value="true",
                                  description="Set to false for pure Go2 (non-W) — "
                                  "skips wheel_velocity_controller spawn and uses "
                                  "the 'go2' CHAMP stand-up preset instead of 'go2w'."),
            DeclareLaunchArgument("rl_policy", default_value="false",
                                  description="Swap CHAMP's quadruped_controller for the "
                                  "Isaac-Lab-trained ONNX flat policy "
                                  "(scripts/go2_rl_policy_node.py, model: flat_policy_v5). "
                                  "Requires has_wheels:=false. Uses "
                                  "ros_control_go2_robot_rl.yaml (kp=20/kd=0.5) instead "
                                  "of CHAMP's kp=100/kd=1.0 — the policy was trained "
                                  "under those exact gains. Starts ~24 s after launch "
                                  "so the one-shot stand-up finishes first."),
            DeclareLaunchArgument("rl_use_champ_gains", default_value="false",
                                  description="Only honoured when rl_policy:=true. "
                                  "Uses CHAMP's ros_control_go2_robot.yaml (kp=100/kd=1.0) "
                                  "instead of the RL-training gains. Trade-off: the RL "
                                  "policy drives 5× too much torque per step (may oscillate), "
                                  "but the pre-RL stand-up trajectory actually holds the "
                                  "robot up. Useful for smoke-testing before we wire a "
                                  "proper stand → RL handoff."),
            DeclareLaunchArgument("rl_model_path", default_value="",
                                  description="Only honoured when rl_policy:=true. "
                                  "Absolute path to the ONNX policy. Empty = node default "
                                  "(flat_policy_v6, 45-dim). Pass a 49-dim rough policy "
                                  "(e.g. .../share/models/rough_policy_v1.onnx) for ramp / "
                                  "rough-terrain climbing — the node auto-detects the obs "
                                  "dim and adds the 4 foot-contact inputs."),
            DeclareLaunchArgument("rl_sar_config", default_value="",
                                  description="Only honoured when rl_policy:=true. Path to "
                                  "an rl_sar policy dir (containing config.yaml + .pt, e.g. "
                                  "src/vendor/rl_sar/policy/go2/robot_lab). When set, runs "
                                  "the config-driven go2_rl_sar_node (rough-terrain cmd_vel "
                                  "locomotion) instead of the legacy IL flat-policy node. "
                                  "base.yaml is read from the parent dir. Takes precedence "
                                  "over rl_model_path."),
            DeclareLaunchArgument("spawn_x", default_value="1.0"),
            DeclareLaunchArgument("spawn_y", default_value="0.0"),
            DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("world", default_value=os.path.join(go2_gazebo_pkg, "worlds", "3.world")),
            DeclareLaunchArgument(
                "mujoco_model_path",
                default_value=default_mujoco_model,
                description="Path to the MuJoCo MJCF model file",
            ),
            DeclareLaunchArgument("cfpa2_w_ig", default_value="1.0", description="CFPA2 info-gain weight"),
            DeclareLaunchArgument("cfpa2_w_c", default_value="0.6", description="CFPA2 distance-cost weight"),
            DeclareLaunchArgument("cfpa2_w_momentum", default_value="0.8", description="CFPA2 momentum bonus weight"),
            DeclareLaunchArgument(
                "cfpa2_min_utility",
                default_value="-0.5",
                description="CFPA2 min utility to assign a frontier (below = stop)",
            ),
            # Navigation sub-launch pass-through
            DeclareLaunchArgument("map_frame", default_value="world"),
            DeclareLaunchArgument("waypoint_input_suffix", default_value="/way_point_coord"),
            DeclareLaunchArgument("cfpa2_goal_topic_suffix", default_value="/way_point_coord"),
            DeclareLaunchArgument(
                "cfpa2_executable_suffix",
                default_value="",
                description="Suffix appended to cfpa2_single_robot_node "
                            "(use '_cpp' for the pure-C++ rclcpp port).",
            ),
            DeclareLaunchArgument("cfpa2_switch_hysteresis", default_value=""),
            DeclareLaunchArgument("max_linear_speed", default_value=""),
            DeclareLaunchArgument("require_settle_before_motion", default_value=""),
            DeclareLaunchArgument("nav_map_topic", default_value=""),
            DeclareLaunchArgument("nav_config", default_value=""),
            DeclareLaunchArgument("nav_backend", default_value=""),
            DeclareLaunchArgument("registered_scan_topic", default_value=""),
            DeclareLaunchArgument("far_max_speed", default_value=""),
            DeclareLaunchArgument("far_robot_id", default_value=""),
            DeclareLaunchArgument("ramp_force_legged_enabled", default_value="false"),
            DeclareLaunchArgument("ramp_force_wheel_enabled", default_value="false"),
            DeclareLaunchArgument("ramp_goal_mode_topic", default_value=""),
            DeclareLaunchArgument("ramp_goal_stale_sec", default_value="1.5"),
            DeclareLaunchArgument("ramp_force_max_vx_mps", default_value="0.30"),
            DeclareLaunchArgument("ramp_force_max_yaw_rate_rps", default_value="0.20"),
            DeclareLaunchArgument("pitch_force_wheel_enabled", default_value="false",
                                  description="Go2W only: when the body pitches past "
                                  "pitch_force_wheel_threshold_rad (on a slope), hold "
                                  "sustained wheel mode + floor forward speed to "
                                  "pitch_climb_vx_mps so the wheels roll up the incline "
                                  "instead of stalling. Goal-independent ramp-climb assist."),
            DeclareLaunchArgument("pitch_force_wheel_threshold_rad", default_value="0.070"),
            DeclareLaunchArgument("pitch_climb_vx_mps", default_value="0.45"),
            DeclareLaunchArgument(
                "odom_bridge_publish_tf",
                default_value="true",
                description="Disable when Cartographer provides odom frame to avoid TF conflict",
            ),
            OpaqueFunction(function=_launch_setup),
        ]
    )
