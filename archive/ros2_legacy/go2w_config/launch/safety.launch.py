#!/usr/bin/env python3
"""Shared safety sub-launch: wall collision checker + autonomy enabler.

Included by both sim and real top-level launch files.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, key: str) -> str:
    return LaunchConfiguration(key).perform(context)


def _setup(context):
    robot_ns = _get(context, "robot_namespace")
    use_sim_time = _as_bool(_get(context, "use_sim_time"))
    scan_topic = _get(context, "scan_topic") or f"/{robot_ns}/scan_3d"
    autonomy_startup_delay = float(_get(context, "autonomy_startup_delay"))
    panic_duration_sec = float(_get(context, "panic_duration_sec"))
    supervisor_state_topic = f"/{robot_ns}/supervisor_state"

    go2w_config_pkg = get_package_share_directory("go2w_config")

    actions = []

    # ── Wall Collision Checker ──
    actions.append(
        Node(
            package="go2w_safety",
            executable="wall_collision_checker.py",
            namespace=robot_ns,
            name="wheel_wall_collision_checker",
            parameters=[
                os.path.join(go2w_config_pkg, "config", "safety", "wall_checker.yaml"),
                {
                    "use_sim_time": use_sim_time,
                    "scan_topic": scan_topic,
                    "stop_topic": f"/{robot_ns}/stop",
                    "mode_topic": "mobility_mode",
                    # Legged mode defaults
                    "safety_dist": 0.32,
                    "check_angle_deg": 40.0,
                    "min_close_points": 4,
                    # Wheel mode: wider cone and farther detection
                    "wheel_safety_dist": 0.55,
                    "wheel_check_angle_deg": 80.0,
                    "wheel_min_close_points": 4,
                },
            ],
            output="screen",
        )
    )

    # ── Autonomy Enabler ──
    actions.append(
        Node(
            package="go2w_safety",
            executable="autonomy_enabler.py",
            namespace=robot_ns,
            name="autonomy_enabler",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "startup_delay": autonomy_startup_delay,
                    "rate": 10.0,
                    "supervisor_state_topic": supervisor_state_topic,
                },
            ],
            remappings=[
                ("/way_point", f"/{robot_ns}/way_point_coord"),
                ("/joy", f"/{robot_ns}/joy"),
            ],
            output="screen",
        )
    )

    # ── Supervisor panic latch (any-button joystick → 5s override) ──
    actions.append(
        Node(
            package="go2w_safety",
            executable="supervisor_panic_node.py",
            namespace=robot_ns,
            name="supervisor_panic",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "panic_duration_sec": panic_duration_sec,
                    "publish_rate_hz": 20.0,
                    "joy_topic": f"/{robot_ns}/joy",
                    "trigger_topic": f"/{robot_ns}/supervisor/panic_trigger",
                    "state_topic": supervisor_state_topic,
                    "panic_cmd_topic": f"/{robot_ns}/panic_cmd_vel",
                },
            ],
            output="screen",
        )
    )

    return actions


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("scan_topic", default_value=""),
            DeclareLaunchArgument("autonomy_startup_delay", default_value="8.0"),
            DeclareLaunchArgument("panic_duration_sec", default_value="5.0"),
            OpaqueFunction(function=_setup),
        ]
    )
