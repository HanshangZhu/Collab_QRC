#!/usr/bin/env python3
"""Single-robot real Go2W runtime with TARE waypoint wiring.

Wraps real_single.launch.py and inserts:
  - tare_planner_node: /<ns>/way_point_seed → /<ns>/way_point_tare
  - waypoint_mux: prefer /<ns>/way_point_tare, fall back to /<ns>/way_point_coord,
                  publish /<ns>/way_point_coord_nav (consumed by the local planner)

CFPA2 keeps publishing /way_point_coord as its frontier goal; TARE overlays
smooth waypoints on top. If TARE goes silent for >primary_timeout_sec the mux
falls back to the raw CFPA2 goal.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _launch_setup(context):
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"

    bringup_share = get_package_share_directory("go2w_real_bringup")
    base_launch = os.path.join(bringup_share, "launch", "real_single.launch.py")

    def _norm(suffix: str, default: str) -> str:
        s = suffix.strip() or default
        return s if s.startswith("/") else "/" + s

    seed_suffix = _norm(_get(context, "tare_seed_input_suffix"), "/way_point_coord")
    tare_out_suffix = _norm(_get(context, "tare_output_suffix"), "/way_point_tare")
    mux_out_suffix = _norm(_get(context, "tare_mux_output_suffix"), "/way_point_coord_nav")

    include_base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            "robot_namespace": robot_ns,
            "robot_model": _get(context, "robot_model"),
            "slam": _get(context, "slam"),
            "carto_mode": _get(context, "carto_mode"),
            "nav_backend": _get(context, "nav_backend"),
            "map_backend": _get(context, "map_backend"),
            "obstacle_avoidance": _get(context, "obstacle_avoidance"),
            "execute_controller": _get(context, "execute_controller"),
            "enable_manual_fallback": _get(context, "enable_manual_fallback"),
            "joy_dev": _get(context, "joy_dev"),
            "manual_timeout_sec": _get(context, "manual_timeout_sec"),
            "auto_timeout_sec": _get(context, "auto_timeout_sec"),
            "manual_linear_threshold": _get(context, "manual_linear_threshold"),
            "manual_angular_threshold": _get(context, "manual_angular_threshold"),
            # Route the local planner to the muxed output instead of raw CFPA2 goal.
            "waypoint_input_suffix": mux_out_suffix,
        }.items(),
    )

    tare_node = Node(
        package="go2_tare_planner_ros2",
        executable="tare_planner_node",
        name="tare_planner_node",
        parameters=[
            {"use_sim_time": False},
            {"namespaces": [robot_ns]},
            {"input_topic_suffix": seed_suffix},
            {"output_topic_suffix": tare_out_suffix},
            {"output_rate_hz": float(_get(context, "tare_output_rate_hz"))},
        ],
        output="screen",
    )

    tare_mux = Node(
        package="go2_gazebo_sim",
        executable="waypoint_mux.py",
        name="tare_waypoint_mux",
        parameters=[
            {"use_sim_time": False},
            {"namespaces": [robot_ns]},
            {"primary_input_suffix": tare_out_suffix},
            {"fallback_input_suffix": "/way_point_coord"},
            {"output_suffix": mux_out_suffix},
            {"primary_timeout_sec": float(_get(context, "tare_primary_timeout_sec"))},
            {"output_rate": float(_get(context, "tare_mux_output_rate_hz"))},
            {"hold_last_output": True},
            {"stamp_now": True},
        ],
        output="screen",
    )

    return [include_base, tare_node, tare_mux]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument("robot_model", default_value="go2w",
                                   description="go2w or go2"),
            DeclareLaunchArgument("slam", default_value="carto_l1"),
            DeclareLaunchArgument("carto_mode", default_value="2d"),
            DeclareLaunchArgument("nav_backend", default_value="reactive"),
            DeclareLaunchArgument("map_backend", default_value="carto_2d"),
            DeclareLaunchArgument("obstacle_avoidance", default_value="true"),
            DeclareLaunchArgument("execute_controller", default_value="true",
                                   description="false = dry-run; sport API disconnected"),
            DeclareLaunchArgument("enable_manual_fallback", default_value="true"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("manual_timeout_sec", default_value="0.35"),
            DeclareLaunchArgument("auto_timeout_sec", default_value="0.60"),
            DeclareLaunchArgument("manual_linear_threshold", default_value="0.02"),
            DeclareLaunchArgument("manual_angular_threshold", default_value="0.05"),
            DeclareLaunchArgument("tare_seed_input_suffix", default_value="/way_point_coord"),
            DeclareLaunchArgument("tare_output_suffix", default_value="/way_point_tare"),
            DeclareLaunchArgument("tare_mux_output_suffix", default_value="/way_point_coord_nav"),
            DeclareLaunchArgument("tare_primary_timeout_sec", default_value="1.0"),
            DeclareLaunchArgument("tare_output_rate_hz", default_value="5.0"),
            DeclareLaunchArgument("tare_mux_output_rate_hz", default_value="8.0"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
