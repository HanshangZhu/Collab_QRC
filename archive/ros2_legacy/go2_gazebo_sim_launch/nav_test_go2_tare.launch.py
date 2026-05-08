#!/usr/bin/env python3
"""Go2 (non-W) nav test with TARE + FAR layered goal chain.

Wraps nav_test_mujoco_fastlio.launch.py and adds:
  - tare_planner_node : /{ns}/way_point_coord → /{ns}/way_point_tare
                        (takes CFPA2's frontier goal as a "seed", plans a
                         keypose-graph-reachable intermediate goal)
  - waypoint_mux      : primary /{ns}/way_point_tare, fallback
                        /{ns}/way_point_coord → /{ns}/way_point_coord_nav
                        (falls back to CFPA2 if TARE goes silent >1 s)
  - FAR is pointed at /{ns}/way_point_coord_nav via ``far_goal_topic``

Why exists: on demo3, CFPA2 can assign frontier goals that sit inside/behind
unobserved space. FAR has no V-graph vertices there, so it silently stops
publishing waypoints and the robot sits at `STUCK(N)` forever. TARE does its
own viewpoint optimization on known-traversable terrain and emits goals
that are reachable by construction — FAR always has a plannable target.

Defaults assume pure Go2 (has_wheels:=false). Pass `gui:=true rviz:=true`
for interactive work. `session_duration_sec:=300 session_output_path:=...`
for headless benchmarking.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _get(ctx, name: str) -> str:
    return LaunchConfiguration(name).perform(ctx)


def _launch_setup(ctx):
    robot_ns = _get(ctx, "robot_namespace").strip().strip("/") or "robot"

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    base_launch = os.path.join(
        go2_gazebo_pkg, "launch", "nav_test_mujoco_fastlio.launch.py"
    )

    seed_suffix   = "/way_point_coord"         # CFPA2 output, TARE input
    tare_suffix   = "/way_point_tare"          # TARE output
    mux_suffix    = "/way_point_coord_nav"     # mux output, FAR input
    primary_timeout_sec = float(_get(ctx, "tare_primary_timeout_sec"))
    tare_rate_hz        = float(_get(ctx, "tare_output_rate_hz"))
    mux_rate_hz         = float(_get(ctx, "tare_mux_output_rate_hz"))

    # --- Base platform (MuJoCo + CHAMP + Fast-LIO + octomap + FAR + CFPA2 + pathFollower).
    #     Only override: point FAR's goal subscription at the mux output.
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            "robot_namespace": robot_ns,
            "gui":            _get(ctx, "gui"),
            "rviz":           _get(ctx, "rviz"),
            "explore":        _get(ctx, "explore"),
            "nav_backend":    "far",
            "mujoco_model_path": _get(ctx, "mujoco_model_path"),
            "scene_area_m2":  _get(ctx, "scene_area_m2"),
            "has_wheels":     _get(ctx, "has_wheels"),
            "two_way_drive":  _get(ctx, "two_way_drive"),
            "session_duration_sec": _get(ctx, "session_duration_sec"),
            "session_output_path":  _get(ctx, "session_output_path"),
            "enable_wall_checker":  _get(ctx, "enable_wall_checker"),
            "far_max_speed":  _get(ctx, "far_max_speed"),
            "spawn_x":  _get(ctx, "spawn_x"),
            "spawn_y":  _get(ctx, "spawn_y"),
            "spawn_yaw": _get(ctx, "spawn_yaw"),
            # Route FAR through the mux.
            "far_goal_topic": f"/{robot_ns}{mux_suffix}",
        }.items(),
    )

    tare_node = Node(
        package="go2_tare_planner_ros2",
        executable="tare_planner_node",
        name="tare_planner_node",
        parameters=[
            {"use_sim_time": True},
            {"namespaces": [robot_ns]},
            {"input_topic_suffix": seed_suffix},
            {"output_topic_suffix": tare_suffix},
            {"output_rate_hz": tare_rate_hz},
        ],
        output="screen",
    )

    tare_mux = Node(
        package="go2_gazebo_sim",
        executable="waypoint_mux.py",
        name="tare_waypoint_mux",
        parameters=[
            {"use_sim_time": True},
            {"namespaces": [robot_ns]},
            {"primary_input_suffix": tare_suffix},
            {"fallback_input_suffix": seed_suffix},
            {"output_suffix": mux_suffix},
            {"primary_timeout_sec": primary_timeout_sec},
            {"output_rate": mux_rate_hz},
            {"hold_last_output": True},
            {"stamp_now": True},
        ],
        output="screen",
    )

    return [base, tare_node, tare_mux]


def generate_launch_description() -> LaunchDescription:
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    default_scene = os.path.join(
        go2_gazebo_pkg, "mujoco", "demo3_go2_real.xml"
    )
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="robot"),
        DeclareLaunchArgument("gui",  default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("explore", default_value="true",
                              description="Leave CFPA2 on as a frontier seed "
                              "for TARE. TARE does its own reachable-keypose "
                              "selection; CFPA2 just provides a global target "
                              "direction."),
        DeclareLaunchArgument("mujoco_model_path", default_value=default_scene),
        DeclareLaunchArgument("scene_area_m2", default_value="384.0"),
        DeclareLaunchArgument("has_wheels",    default_value="false",
                              description="Pure Go2 by default."),
        DeclareLaunchArgument("two_way_drive", default_value="false",
                              description="CHAMP has no validated reverse "
                              "gait for walking; keep false on Go2."),
        DeclareLaunchArgument("spawn_x",   default_value="4.0"),
        DeclareLaunchArgument("spawn_y",   default_value="2.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("session_duration_sec", default_value="0"),
        DeclareLaunchArgument("session_output_path",  default_value=""),
        DeclareLaunchArgument("enable_wall_checker",  default_value="false"),
        DeclareLaunchArgument("far_max_speed",        default_value=""),
        DeclareLaunchArgument("tare_primary_timeout_sec", default_value="1.0",
                              description="Mux fallback kicks in if TARE "
                              "goes silent for this many seconds."),
        DeclareLaunchArgument("tare_output_rate_hz",     default_value="5.0"),
        DeclareLaunchArgument("tare_mux_output_rate_hz", default_value="8.0"),
        OpaqueFunction(function=_launch_setup),
    ])
