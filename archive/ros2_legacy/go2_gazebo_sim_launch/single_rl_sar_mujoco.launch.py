#!/usr/bin/env python3
"""Single-robot launch using rl_sar's Go2W RL policy as the locomotion stack
(replaces CHAMP), with optional SLAM + nav2_hybrid_astar on top.

Stack:

    MuJoCo plugin  ──▶  /robot/joint_states    (JointStateBroadcaster)
                   └──▶ /robot/imu/data        (mujoco_odom_bridge)
                                 │
                                 ▼
                       rl_locomotion_node     (loads policy.pt via libtorch)
                          │             │
                          ▼             ▼
            robot_leg_effort_      robot_wheel_velocity_
            controller (12 ×      controller (4 × velocity)
            effort)
                                 ▲
                                 │ /robot/cmd_vel  (Twist)
                                 │
                       — manual ros2 topic pub for smoke test
                       — nav2_hybrid_astar_nav_node  if nav:=true
                       — Fast-LIO + octomap          if slam:=true

Args:
    scene:=demo1 | demo3                 (default demo3)
    gui:=true|false                       (default true)
    rviz:=true|false                      (default false)
    slam:=true|false                      (default false — extend after locomotion proven)
    nav:=true|false                       (default false — needs slam:=true)
    auto_stand_up:=true|false             (default true)
    cleanup_stale:=true|false             (default true)
    spawn_x, spawn_y, spawn_yaw           (per-scene defaults)

Examples:
    ros2 launch go2_gazebo_sim single_rl_sar_mujoco.launch.py
    ros2 launch go2_gazebo_sim single_rl_sar_mujoco.launch.py scene:=demo3 gui:=true rviz:=true
    ros2 launch go2_gazebo_sim single_rl_sar_mujoco.launch.py slam:=true nav:=true rviz:=true
"""
from __future__ import annotations

import os
import sys

_here = os.path.dirname(os.path.abspath(__file__))
if _here not in sys.path:
    sys.path.insert(0, _here)

import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, LogInfo,
                            OpaqueFunction, TimerAction)
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterValue

# Reuse helpers from the A* launch (same MuJoCo plugin, sensor bridges, SLAM + nav stacks).
# .launch.py extensions can't be imported as ordinary modules, so we load by path.
import importlib.util as _ilu
_astar_path = os.path.join(_here, "single_astar_mujoco.launch.py")
_spec = _ilu.spec_from_file_location("single_astar_mujoco_launch_helpers", _astar_path)
_astar_mod = _ilu.module_from_spec(_spec)
_spec.loader.exec_module(_astar_mod)
_as_bool = _astar_mod._as_bool
_build_astar_stack = _astar_mod._build_astar_stack
_build_cleanup_stale_cmd = _astar_mod._build_cleanup_stale_cmd
_build_sensor_bridges = _astar_mod._build_sensor_bridges
_build_slam_stack = _astar_mod._build_slam_stack
_find_mujoco_plugin_dir = _astar_mod._find_mujoco_plugin_dir
_get = _astar_mod._get
_patch_urdf_for_mujoco = _astar_mod._patch_urdf_for_mujoco
_scene_defaults = _astar_mod._scene_defaults

NS = "robot"


def launch_setup(context, *args, **kwargs):
    scene = _get(context, "scene")
    gui = _as_bool(_get(context, "gui"))
    rviz = _as_bool(_get(context, "rviz"))
    enable_slam = _as_bool(_get(context, "slam"))
    enable_nav = _as_bool(_get(context, "nav"))
    auto_stand_up = _as_bool(_get(context, "auto_stand_up"))
    cleanup_stale = _as_bool(_get(context, "cleanup_stale"))
    spawn_x_arg = _get(context, "spawn_x").strip()
    spawn_y_arg = _get(context, "spawn_y").strip()
    spawn_yaw = _get(context, "spawn_yaw").strip() or "0.0"
    use_sim_time = True

    if enable_nav and not enable_slam:
        raise ValueError("nav:=true requires slam:=true (nav stack consumes /robot/map)")

    robot = "go2w"
    if scene not in ("demo1", "demo3"):
        raise ValueError(f"scene must be 'demo1' or 'demo3', got {scene!r}")

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2w_config_pkg = get_package_share_directory("go2w_config")
    rl_sar_root = os.path.expanduser("~/Research/Collab_QRC/src/vendor/rl_sar")

    mjcf_path, default_x, default_y = _scene_defaults(robot, scene, go2_gazebo_pkg)
    spawn_x = spawn_x_arg or str(default_x)
    spawn_y = spawn_y_arg or str(default_y)

    urdf_xacro = os.path.join(
        go2_gazebo_pkg, "urdf", "go2w", "go2w_description_3d_lidar.xacro"
    )
    # New controllers config: effort forward_command for legs (so the
    # rl_locomotion node can publish raw torques) + velocity for wheels.
    ros_control_yaml = os.path.join(
        go2_gazebo_pkg, "mujoco", "go2w_rl_mujoco_controllers.yaml"
    )

    robot_description = xacro.process_file(urdf_xacro).documentElement.toxml()
    robot_description = _patch_urdf_for_mujoco(robot_description)

    links_config = os.path.join(
        go2_gazebo_pkg, "config", "champ", "go2w", "links.yaml"
    )
    mujoco_plugin_dir = _find_mujoco_plugin_dir()

    actions = [LogInfo(msg=(
        f"[single_rl_sar] scene={scene}  spawn=({spawn_x},{spawn_y},{spawn_yaw})  "
        f"slam={enable_slam}  nav={enable_nav}"
    ))]

    if cleanup_stale:
        actions.append(ExecuteProcess(
            cmd=["bash", "-lc", _build_cleanup_stale_cmd()], output="screen",
        ))

    # ── T=3: MuJoCo + rl_sar controllers ─────────────────────────────────
    mujoco_node = Node(
        package="mujoco_ros2_control",
        executable="mujoco_ros2_control",
        namespace=NS,
        parameters=[
            {"robot_description": robot_description},
            ros_control_yaml,
            {"robot_model_path": mjcf_path},
            {"simulation_frequency": 500.0},
            {"real_time_factor": 1.0},
            {"clock_publisher_frequency": 100.0},
            {"show_gui": gui},
        ],
        remappings=[
            (f"/{NS}/controller_manager/robot_description", f"/{NS}/robot_description"),
        ],
        additional_env={"MUJOCO_PLUGIN_DIR": mujoco_plugin_dir},
        output="screen",
    )
    actions.append(TimerAction(period=3.0, actions=[mujoco_node]))

    # ── T=5: sensor bridges (odom + imu + foot contacts) ────────────────
    base_body = "base_link"
    actions.append(TimerAction(
        period=5.0,
        actions=_build_sensor_bridges(
            mjcf_path=mjcf_path,
            use_sim_time=use_sim_time,
            base_body=base_body,
            links_config=links_config,
            pose_sensor="base_link_site_pose_sensor",
            imu_sensor="imu_imu_sensor",
        ),
    ))

    # ── T=7: RSP + controllers + rl_locomotion ──────────────────────────
    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        namespace=NS,
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            {"use_tf_static": False},
            {"publish_frequency": 200.0},
            {"ignore_timestamp": True},
            {"use_sim_time": use_sim_time},
        ],
        remappings=[("/tf", f"/{NS}/tf"), ("/tf_static", f"/{NS}/tf_static")],
        output="screen",
    )

    # Spawn each controller from the controller_manager. The CM lives at
    # /{NS}/controller_manager (the MuJoCo plugin runs under /robot ns).
    cm_path = f"/{NS}/controller_manager"
    spawner = lambda name: Node(
        package="controller_manager",
        executable="spawner",
        arguments=[name, "--controller-manager", cm_path],
        output="screen",
    )
    spawn_joint_states = spawner("robot_joint_states_controller")
    spawn_leg_effort = spawner("robot_leg_effort_controller")
    spawn_wheel_velocity = spawner("robot_wheel_velocity_controller")

    rl_locomotion_node = Node(
        package="go2w_nav",
        executable="rl_locomotion_node.py",
        namespace=NS,
        name="rl_locomotion",
        parameters=[{
            "use_sim_time": use_sim_time,
            "rl_sar_root": rl_sar_root,
            "policy_path": os.path.join(rl_sar_root, "policy/go2w/robot_lab/policy.pt"),
            "policy_rate_hz": 50.0,
            "stand_up_seconds": 2.0,
            "stand_up_kp": 70.0,
            "stand_up_kd": 5.0,
            "auto_stand_up": auto_stand_up,
            "auto_locomotion_after_stand": True,
            "imu_topic": "imu/data",
            "joint_states_topic": "joint_states",
            "cmd_vel_topic": "cmd_vel",
            "leg_command_topic": "robot_leg_effort_controller/commands",
            "wheel_command_topic": "robot_wheel_velocity_controller/commands",
        }],
        output="screen",
    )

    actions.append(TimerAction(
        period=7.0,
        actions=[rsp_node, spawn_joint_states, spawn_leg_effort, spawn_wheel_velocity],
    ))
    # Give the controllers a beat to come up, then start the policy.
    actions.append(TimerAction(period=10.0, actions=[rl_locomotion_node]))

    # ── Optional SLAM + nav stack on top of the rl_sar locomotion ──
    if enable_slam:
        slam_delay = 20.0
        actions.extend(_build_slam_stack(
            use_sim_time=use_sim_time,
            go2w_config_pkg=go2w_config_pkg,
            slam_delay=slam_delay,
            base_frame=base_body,
        ))

        if enable_nav:
            nav_delay = slam_delay + 5.0
            astar_config_path = os.path.join(
                go2w_config_pkg, "config", "nav", "nav2_hybrid_astar_nav_go2w.yaml"
            )
            # has_wheels=False — skip hybrid_cmd_router. The rl_sar policy
            # already handles wheel↔leg dispatch internally; the router would
            # double-publish to the wheel velocity topic and intercept
            # /cmd_vel, starving the rl_locomotion node.
            actions.extend(_build_astar_stack(
                use_sim_time=use_sim_time,
                astar_config_path=astar_config_path,
                nav_delay=nav_delay,
                has_wheels=False,
                go2w_config_pkg=go2w_config_pkg,
                nav_backend="nav2_hybrid_astar",
            ))
            # RViz '2D Goal Pose' (PoseStamped on /goal_pose) → planner's
            # PointStamped on /robot/way_point_coord. Without this no goal
            # ever reaches nav and /robot/cmd_vel stays at zero.
            actions.append(TimerAction(
                period=nav_delay,
                actions=[Node(
                    package="go2w_nav",
                    executable="rviz_goal_relay.py",
                    namespace=NS,
                    name="rviz_goal_relay",
                    parameters=[{
                        "use_sim_time": use_sim_time,
                        "output_topic": "way_point_coord",
                    }],
                    output="screen",
                )],
            ))

    if rviz:
        actions.append(TimerAction(
            period=12.0,
            actions=[
                Node(
                    package="go2w_perception",
                    executable="multi_tf_relay",
                    name="multi_tf_relay",
                    parameters=[
                        {"use_sim_time": use_sim_time},
                        {"sources": [NS]},
                    ],
                    output="screen",
                ),
                ExecuteProcess(
                    cmd=[
                        "bash", "-c",
                        "unset XDG_DATA_HOME GSETTINGS_SCHEMA_DIR GTK_PATH LOCPATH "
                        "SNAP SNAP_NAME SNAP_INSTANCE_NAME SNAP_REVISION; "
                        "exec rviz2 -d \"$1\" --ros-args -p use_sim_time:=true "
                        "--log-level rviz2:=WARN",
                        "--",
                        os.path.join(go2_gazebo_pkg, "rviz", "nav_test.rviz"),
                    ],
                    name="rviz2_rl_sar",
                    output="log",
                ),
            ],
        ))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("scene", default_value="demo3"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="false"),
        DeclareLaunchArgument("slam", default_value="false"),
        DeclareLaunchArgument("nav", default_value="false"),
        DeclareLaunchArgument("auto_stand_up", default_value="true"),
        DeclareLaunchArgument("cleanup_stale", default_value="true"),
        DeclareLaunchArgument("spawn_x", default_value=""),
        DeclareLaunchArgument("spawn_y", default_value=""),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        OpaqueFunction(function=launch_setup),
    ])
