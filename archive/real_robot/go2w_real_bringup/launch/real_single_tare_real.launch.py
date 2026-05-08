#!/usr/bin/env python3
"""Real Go2/Go2W runtime with the *real CMU TARE planner* driving exploration.

Unlike ``real_single_tare.launch.py`` — which overlays the 117-line
``go2_tare_planner_ros2`` stub on top of CFPA2 via a waypoint_mux — this
launch puts the full CMU ``tare_planner`` package in charge of exploration,
with FAR unwired and CFPA2 disabled. Matches the sim's
``nav_test_go2_tare_real.launch.py`` architecture:

Pipeline:

    state_estimation_at_scan  ┐
    /{ns}/registered_scan_map ├─► tare_planner_node ─► /{ns}/way_point
    /{ns}/terrain_map[_ext]   ┘                              │
                                                             ▼
                                                       localPlanner
                                                             │
                                                             ▼
                                                       pathFollower
                                                             │
                                                             ▼
                                                     /{ns}/cmd_vel_stamped

Differences from the sim launch:
  * No ``cloud_world_offset_bridge`` — on real there's no GT bootstrap, so
    Fast-LIO's camera_init frame is numerically aligned with the odom/nav
    output (both start at spawn=origin). Cloud can be consumed directly.
  * No ``robust_controller_spawner`` — controller startup is the Unitree
    SDK's responsibility, not mujoco_ros2_control.
  * Real-robot topic conventions: odom on ``/{ns}/odom/nav``,
    registered cloud on whatever ``real_single`` wires as
    ``registered_scan_topic`` (caller-decided — Mid-360 is
    ``/cloud_registered`` when slam=fastlio_mid360).

Usage::

    ros2 launch go2w_real_bringup real_single_tare_real.launch.py \\
        robot_model:=go2 slam:=fastlio_mid360
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Resolve repo root from this file's location: launch/ → ../../../../..
# (go2w_real_bringup/launch is 4 levels deep from repo root, same as
# go2_gazebo_sim/launch).
_ws_root = os.path.abspath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", ".."
))


def _get(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _launch_setup(context):
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"

    bringup_share = get_package_share_directory("go2w_real_bringup")
    base_launch = os.path.join(bringup_share, "launch", "real_single.launch.py")
    scenario = _get(context, "tare_scenario").strip() or "indoor"
    tare_config = os.path.join(bringup_share, "config", "tare", f"{scenario}.yaml")

    # --- Base platform: SLAM + core bringup + nav + safety + obs.
    # We keep nav_backend=far so terrain_analysis + FAR + localPlanner +
    # pathFollower all launch, then unwire FAR via the topic overrides
    # (empty goal, dead-sink output). CFPA2 is disabled with explore=false.
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            "robot_namespace": robot_ns,
            "robot_model": _get(context, "robot_model"),
            "slam": _get(context, "slam"),
            "carto_mode": _get(context, "carto_mode"),
            "nav_backend": "far",
            "map_backend": _get(context, "map_backend"),
            "obstacle_avoidance": _get(context, "obstacle_avoidance"),
            "execute_controller": _get(context, "execute_controller"),
            "enable_manual_fallback": _get(context, "enable_manual_fallback"),
            "joy_dev": _get(context, "joy_dev"),
            "manual_timeout_sec": _get(context, "manual_timeout_sec"),
            "auto_timeout_sec": _get(context, "auto_timeout_sec"),
            "manual_linear_threshold": _get(context, "manual_linear_threshold"),
            "manual_angular_threshold": _get(context, "manual_angular_threshold"),
            "rviz": _get(context, "rviz"),
            "rviz_3d": _get(context, "rviz_3d"),
            # localPlanner's input is /{ns}/way_point — leave as default.
            "waypoint_input_suffix": "/way_point",
            # TARE replaces CFPA2 as the goal source.
            "explore": "false",
            # FAR unwired: input has no publisher, output dumped to a sink.
            "far_goal_topic": "",
            "far_way_point_out": f"/{robot_ns}/_far_way_point_unused",
            # Route terrain_analysis + FAR + TARE all to the same
            # namespaced cloud topic that our relay (below) is filling
            # from Fast-LIO's /cloud_registered.
            "registered_scan_topic": f"/{robot_ns}/registered_scan_map",
        }.items(),
    )

    # Real-robot bringup publishes TFs in the ROOT namespace (/tf, /tf_static),
    # not /{ns}/tf. Sim remaps them; real doesn't. If we forward-copied the
    # sim remap here, TARE would subscribe to the empty /{ns}/tf topic and
    # its internal TF buffer would stay empty — no pose, no keypose graph.
    tf_remaps: list = []

    # --- Cloud topic relay: Fast-LIO publishes to the un-namespaced
    # /cloud_registered (world frame = camera_init, which on real robot is
    # numerically identical to the odom/nav frame since there's no GT
    # bootstrap). Our CMU autonomy pipeline (sensor_scan_generation,
    # terrain_analysis, TARE) expects the namespaced /{ns}/registered_scan_map.
    # topic_tools/relay is a plain topic republisher — 1:1 forwarding, no
    # frame transform (we don't need one; frames align numerically).
    cloud_relay = Node(
        package="topic_tools",
        executable="relay",
        name="registered_scan_map_relay",
        arguments=["/cloud_registered", f"/{robot_ns}/registered_scan_map"],
        output="screen",
    )

    # --- Real CMU TARE planner. Same wiring as sim, minus the parts that
    #     only make sense in MuJoCo. Topic params are namespace-aware
    #     (the /**-keyed YAML loads regardless of node namespace).
    tare_node = Node(
        package="tare_planner",
        executable="tare_planner_node",
        name="tare_planner_node",
        namespace=robot_ns,
        parameters=[
            tare_config,
            {"use_sim_time": False},
            # Inputs — real-robot topic conventions.
            {"sub_state_estimation_topic_":  f"/{robot_ns}/state_estimation_at_scan"},
            {"sub_registered_scan_topic_":   f"/{robot_ns}/registered_scan_map"},
            {"sub_terrain_map_topic_":       f"/{robot_ns}/terrain_map"},
            {"sub_terrain_map_ext_topic_":   f"/{robot_ns}/terrain_map_ext"},
            {"sub_start_exploration_topic_": f"/{robot_ns}/start_exploration"},
            {"sub_joystick_topic_":          f"/{robot_ns}/joy"},
            {"sub_reset_waypoint_topic_":    f"/{robot_ns}/reset_waypoint"},
            {"sub_coverage_boundary_topic_": f"/{robot_ns}/coverage_boundary"},
            {"sub_viewpoint_boundary_topic_": f"/{robot_ns}/navigation_boundary"},
            {"sub_nogo_boundary_topic_":     f"/{robot_ns}/nogo_boundary"},
            # Output — publish DIRECTLY to localPlanner's input, FAR idle.
            {"pub_waypoint_topic_":          f"/{robot_ns}/way_point"},
            {"pub_runtime_topic_":           f"/{robot_ns}/tare_runtime"},
        ],
        remappings=tf_remaps,
        output="screen",
    )

    # --- Sensor-derived waypoint watchdog: terrain + occgrid + OOB + stall
    # + persistent nogo blacklist. Same node as sim.
    watchdog_script = os.path.join(_ws_root, "scripts/runtime/tare_waypoint_watchdog.py")
    watchdog_proc = ExecuteProcess(
        cmd=[
            "python3", "-u", watchdog_script,
            "--ros-args",
            "-r", f"__ns:=/{robot_ns}",
            "-p", "use_sim_time:=false",
            "-p", f"terrain_map_topic:=/{robot_ns}/terrain_map",
            "-p", f"waypoint_topic:=/{robot_ns}/way_point",
            "-p", f"reset_topic:=/{robot_ns}/reset_waypoint",
            "-p", f"occgrid_topic:=/{robot_ns}/map",
            "-p", f"odom_topic:=/{robot_ns}/odom/nav",
            "-p", f"marker_topic:=/{robot_ns}/way_point_marker",
            "-p", f"robot_marker_topic:=/{robot_ns}/robot_pose_marker",
            "-p", "marker_frame:=map",
            "-p", f"nogo_topic:=/{robot_ns}/nogo_boundary",
            "-p", "obstacle_radius:=0.4",
            "-p", "obstacle_height_thre:=0.2",
            "-p", "min_obstacle_points:=2",
            "-p", "occgrid_occupied_thre:=50",
            "-p", "occgrid_inflate_m:=0.25",
            "-p", "reset_cooldown_sec:=2.0",
            "-p", "stall_timeout_sec:=10.0",
            "-p", "stall_improve_epsilon_m:=0.05",
            "-p", "waypoint_change_epsilon_m:=0.20",
            "-p", "nogo_square_half_m:=0.6",
            "-p", "nogo_max_regions:=40",
            "-p", "nogo_min_dist_from_robot_m:=0.8",
            "-p", "stall_already_there_m:=0.4",
        ],
        name="tare_waypoint_watchdog",
        output="screen",
    )

    # TARE autoStarts via kAutoStart=true in config/tare/indoor.yaml.
    # 12 s delay gives the real stack time to get sensors flowing
    # (Fast-LIO bootstrap, octomap first publish, terrain_analysis).
    return [base, cloud_relay, TimerAction(period=12.0, actions=[tare_node, watchdog_proc])]


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="robot"),
        DeclareLaunchArgument("robot_model", default_value="go2w"),
        DeclareLaunchArgument("slam", default_value="fastlio_mid360"),
        DeclareLaunchArgument("carto_mode", default_value="2d"),
        DeclareLaunchArgument("map_backend", default_value="carto_2d"),
        DeclareLaunchArgument("obstacle_avoidance", default_value="true"),
        DeclareLaunchArgument("execute_controller", default_value="true"),
        DeclareLaunchArgument("enable_manual_fallback", default_value="true"),
        DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
        DeclareLaunchArgument("manual_timeout_sec", default_value="0.35"),
        DeclareLaunchArgument("auto_timeout_sec", default_value="0.60"),
        DeclareLaunchArgument("manual_linear_threshold", default_value="0.02"),
        DeclareLaunchArgument("manual_angular_threshold", default_value="0.05"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("rviz_3d", default_value="true"),
        DeclareLaunchArgument(
            "tare_scenario", default_value="indoor",
            description="TARE config profile from config/tare/ — indoor is "
                        "the vendored default. Switch if you port garage/"
                        "campus/forest/tunnel profiles into the real tree."),
        OpaqueFunction(function=_launch_setup),
    ])
