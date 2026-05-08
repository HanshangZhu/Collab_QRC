#!/usr/bin/env python3
"""Shared observability sub-launch: exploration metrics logger.

Included by both sim and real top-level launch files.
"""

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
    experiment_name = _get(context, "experiment_name") or "exploration"
    log_rate = float(_get(context, "log_rate"))

    return [
        Node(
            package="go2w_observability",
            executable="exploration_metrics_logger.py",
            namespace=robot_ns,
            name="exploration_metrics_logger",
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "namespaces": [robot_ns],
                    "experiment_name": experiment_name,
                    "log_rate": log_rate,
                    "output_dir": "/tmp",
                },
            ],
            output="screen",
        )
    ]


def generate_launch_description():
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("experiment_name", default_value="exploration"),
            DeclareLaunchArgument("log_rate", default_value="1.0"),
            OpaqueFunction(function=_setup),
        ]
    )
