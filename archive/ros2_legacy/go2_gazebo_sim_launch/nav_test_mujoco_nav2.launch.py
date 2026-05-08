#!/usr/bin/env python3
"""Minimal MuJoCo nav test with nav2 + slam_toolbox.

Swaps out the CMU autonomy stack (cartographer + far_planner + CFPA2 +
local_planner + pathFollower) for:

  - slam_toolbox (online async mapping, symmetric-room-robust)
  - nav2 (planner_server Smac Hybrid A* + controller_server Regulated Pure
    Pursuit + bt_navigator recovery behaviors)

Everything else stays the same as nav_test_mujoco.launch.py:
MuJoCo + mujoco_ros2_control + CHAMP control chain + go2w_perception +
twist_bridge (passthrough, becomes a no-op because nav2 publishes the
Twist output directly on the post-twist_bridge cmd_vel topic).

Modes:
  - Default: RViz manual 2D Goal Pose drives the robot.
  - explore:=true (future): CFPA2 frontier allocator emits frontier goals
    that a small relay node converts to NavigateToPose actions. Not wired
    yet in this first slice; use a manual click for now.

Usage:
  ros2 launch go2_gazebo_sim nav_test_mujoco_nav2.launch.py
  ros2 launch go2_gazebo_sim nav_test_mujoco_nav2.launch.py gui:=false
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    LogInfo,
    OpaqueFunction,
    RegisterEventHandler,
    Shutdown,
    TimerAction,
)
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


_ws_root = os.path.abspath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", ".."
))


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, key: str) -> str:
    return LaunchConfiguration(key).perform(context)


def _launch_setup(context):
    use_sim_time = True
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"
    gui = _get(context, "gui")
    rviz = _as_bool(_get(context, "rviz"))
    mujoco_model_path = _get(context, "mujoco_model_path").strip()

    session_duration_sec = float(_get(context, "session_duration_sec"))
    session_output_path = _get(context, "session_output_path").strip()
    scene_area_m2 = float(_get(context, "scene_area_m2"))

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2w_config_pkg = get_package_share_directory("go2w_config")

    if not mujoco_model_path:
        mujoco_model_path = os.path.join(
            go2_gazebo_pkg, "mujoco", "vlm_exploration_scene_no_artifacts.xml"
        )

    nav2_params = os.path.join(go2w_config_pkg, "config", "nav2", "nav2_params_sim.yaml")
    # RewrittenYaml prepends the robot namespace to each YAML top-level key
    # so that `controller_server:` becomes `robot/controller_server:` which
    # matches the namespaced node's fully-qualified name. Without this, the
    # node doesn't find its params and falls back to DWB defaults.
    configured_nav2_params = ParameterFile(
        RewrittenYaml(
            source_file=nav2_params,
            root_key=robot_ns,
            param_rewrites={"use_sim_time": str(use_sim_time).lower()},
            convert_types=True,
        ),
        allow_substs=True,
    )
    slam_params = os.path.join(
        go2w_config_pkg, "config", "nav2", "slam_toolbox_params.yaml"
    )

    tf_remaps = [("/tf", f"/{robot_ns}/tf"), ("/tf_static", f"/{robot_ns}/tf_static")]

    actions = []

    # ── 1. Base platform: MuJoCo + CHAMP + sensors + perception ──
    base_launch = os.path.join(
        go2_gazebo_pkg, "launch", "single_go2w_mujoco_cfpa2.launch.py"
    )
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "use_sim_time": "true",
                "gui": gui,
                "rviz": "false",               # we launch our own
                "cleanup_stale": "true",
                "enable_assets": "true",
                "enable_perception": "true",
                "enable_slam": "false",        # slam_toolbox provides SLAM
                "enable_control": "true",
                "enable_navigation": "false",  # nav2 provides navigation
                "use_fast_lio": "false",
                # slam_toolbox owns map→odom TF; keep the mujoco_odom_bridge
                # from also broadcasting so the TF tree stays single-sourced.
                "odom_bridge_publish_tf": "false",
                "mujoco_model_path": mujoco_model_path,
                "spawn_x": _get(context, "spawn_x"),
                "spawn_y": _get(context, "spawn_y"),
                "spawn_yaw": _get(context, "spawn_yaw"),
            }.items(),
        )
    )

    # ── 2. slam_toolbox (online async mapping) ──
    slam_delay = 10.0  # give CHAMP standup + perception time to settle

    slam_nodes = [
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            namespace=robot_ns,
            output="screen",
            parameters=[slam_params, {"use_sim_time": use_sim_time}],
            remappings=tf_remaps + [
                # slam_toolbox's scan_topic param is already `/robot/scan_3d`
                # so no remap needed for scan. It publishes /robot/map.
            ],
        ),
    ]
    actions.append(TimerAction(period=slam_delay, actions=slam_nodes))

    # ── 3. nav2 stack (direct nodes — no nav2_bringup wrapping) ──
    # Wiring the nav2 nodes directly gives full control over params
    # without going through nav2_bringup's RewrittenYaml which has
    # namespace-key-resolution issues when used in a sub-included launch.
    nav_delay = slam_delay + 3.0

    nav2_lifecycle_nodes = [
        "controller_server",
        "planner_server",
        "behavior_server",
        "bt_navigator",
    ]

    nav2_nodes = [
        Node(
            package="nav2_controller",
            executable="controller_server",
            name="controller_server",
            namespace=robot_ns,
            output="screen",
            parameters=[configured_nav2_params],
            remappings=tf_remaps,
        ),
        Node(
            package="nav2_planner",
            executable="planner_server",
            name="planner_server",
            namespace=robot_ns,
            output="screen",
            parameters=[configured_nav2_params],
            remappings=tf_remaps,
        ),
        Node(
            package="nav2_behaviors",
            executable="behavior_server",
            name="behavior_server",
            namespace=robot_ns,
            output="screen",
            parameters=[configured_nav2_params],
            remappings=tf_remaps,
        ),
        Node(
            package="nav2_bt_navigator",
            executable="bt_navigator",
            name="bt_navigator",
            namespace=robot_ns,
            output="screen",
            parameters=[configured_nav2_params],
            remappings=tf_remaps,
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            namespace=robot_ns,
            output="screen",
            parameters=[{
                "use_sim_time": use_sim_time,
                "autostart": True,
                "node_names": nav2_lifecycle_nodes,
            }],
        ),
    ]
    actions.append(TimerAction(period=nav_delay, actions=nav2_nodes))

    # ── 4. Bounded session reporter (optional) ──
    if session_duration_sec > 0.0:
        if not session_output_path:
            session_output_path = "/tmp/session_reports/latest.json"
        session_script = os.path.join(_ws_root, "scripts/bench/session_reporter.py")
        session_proc = ExecuteProcess(
            cmd=[
                "python3", "-u", session_script,
                "--duration", str(session_duration_sec),
                "--namespace", robot_ns,
                "--output", session_output_path,
                "--scene-area-m2", str(scene_area_m2),
            ],
            name="session_reporter",
            output="screen",
        )
        actions.append(
            TimerAction(period=nav_delay + 3.0, actions=[session_proc])
        )
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=session_proc,
                    on_exit=[
                        LogInfo(
                            msg="session_reporter exited — shutting down launch"
                        ),
                        Shutdown(reason="session_reporter session complete"),
                    ],
                )
            )
        )

    # ── 5. RViz2 ──
    if rviz:
        nav2_bringup_pkg = get_package_share_directory("nav2_bringup")
        rviz_config = os.path.join(
            nav2_bringup_pkg, "rviz", "nav2_default_view.rviz"
        )
        actions.append(
            TimerAction(
                period=nav_delay + 2.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2_nav2",
                        arguments=["-d", rviz_config],
                        parameters=[{"use_sim_time": use_sim_time}],
                        remappings=tf_remaps,
                        output="screen",
                    ),
                ],
            )
        )

    return actions


def generate_launch_description():
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    default_scene = os.path.join(
        go2_gazebo_pkg, "mujoco", "vlm_exploration_scene_no_artifacts.xml"
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="robot"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("mujoco_model_path", default_value=default_scene),
        DeclareLaunchArgument("spawn_x", default_value="4.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("session_duration_sec", default_value="0",
                              description="If > 0, run bounded session reporter"),
        DeclareLaunchArgument("session_output_path",
                              default_value="/tmp/session_reports/latest.json"),
        DeclareLaunchArgument("scene_area_m2", default_value="96.0"),
        OpaqueFunction(function=_launch_setup),
    ])
