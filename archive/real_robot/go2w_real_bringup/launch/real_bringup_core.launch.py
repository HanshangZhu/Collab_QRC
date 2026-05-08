#!/usr/bin/env python3
"""Real-robot core bringup — the nodes that are identical for any nav backend.

Provides:
  - transform_everything: /utlidar/{cloud,imu} → /utlidar/transformed_{cloud,raw_imu,imu}
  - static TF body → base_link (identity; body is the frame Cartographer tracks)
  - carto_odom_bridge: cartographer-only TF → /<ns>/odom/nav Odometry
  - pointcloud_to_laserscan + scan_rear_filter: cloud → /<ns>/scan_3d
  - probability_grid_binarizer (only when map_backend ∈ {carto_binary, carto_2d})
  - twist_bridge: cmd_vel_stamped → cmd_vel_auto
  - cmd_vel_activity_mux: auto/manual arbitration → /cmd_vel
  - cmd_vel_to_sport_bridge: /cmd_vel → Unitree sport API
  - (optional) joy + teleop for manual fallback

All nav/exploration launches include THIS file. Nav-specific nodes live in
real_single.launch.py and real_single_tare.launch.py.
"""
from __future__ import annotations

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(text: str) -> bool:
    return str(text).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, name: str) -> str:
    return LaunchConfiguration(name).perform(context)


def _launch_setup(context):
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"
    enable_manual_fallback = _as_bool(_get(context, "enable_manual_fallback"))
    map_backend = _get(context, "map_backend").strip().lower() or "carto_2d"
    slam = _get(context, "slam").strip().lower() or "carto_l1"
    use_obstacle_avoidance = _as_bool(_get(context, "obstacle_avoidance"))
    run_transform_everything = _as_bool(_get(context, "run_transform_everything"))
    execute_controller = _as_bool(_get(context, "execute_controller"))
    imu_calib_yaml = _get(context, "imu_calib_yaml").strip()
    lidar_range = float(_get(context, "lidar_range"))

    bringup_share = get_package_share_directory("go2w_real_bringup")
    go2w_config_pkg = get_package_share_directory("go2w_config")
    teleop_config = os.path.join(go2w_config_pkg, "config", "teleop", "teleop_twist_joy_go2w.yaml")

    if not imu_calib_yaml:
        imu_calib_yaml = os.path.join(bringup_share, "config", "imu_calib.yaml")

    if map_backend not in {"scan", "carto_binary", "carto_2d"}:
        raise ValueError(f"Unsupported map_backend '{map_backend}'")

    actions = []

    # ── IMU + LiDAR preprocessing (was autonomy_stack_go2 transform_sensors) ──
    if run_transform_everything:
        actions.append(
            Node(
                package="go2w_perception",
                executable="transform_everything.py",
                name="transform_everything",
                parameters=[
                    {
                        "output_frame_id": "body",
                        "imu_calib_yaml": imu_calib_yaml,
                        "accel_lpf_enabled": True,
                        "accel_lpf_alpha": 0.15,
                    }
                ],
                output="screen",
            )
        )

    # NOTE: the base_link ↔ body static TF is now spawned inside slam.launch.py
    # per-backend, because the direction depends on the SLAM source frame.
    #
    #   carto_l1       → Cartographer publishes map → base_link, so we add
    #                     base_link → body (body hangs off base_link).
    #   fastlio_mid360 → Fast-LIO publishes camera_init → body, so we add
    #                     body → base_link (base_link hangs off body).
    #
    # If we added it unconditionally here, Fast-LIO runs would give `body`
    # two parents (camera_init and base_link), breaking the TF tree.

    # ── Cartographer TF → Odometry ──
    # Fast-LIO + nav2_mppi already publishes /<ns>/odom/nav via
    # fast_lio_tf_adapter. Leaving carto_odom_bridge alive in fastlio mode
    # creates a dual-publisher race on the same topic, with mismatched
    # frame_ids (`map` from carto_odom_bridge vs `odom` from the adapter).
    # That pollutes CFPA2 / bt_navigator / stuck_watchdog even when TF and RViz
    # still look sane. Keep this bridge strictly cartographer-only.
    if slam == "carto_l1":
        actions.append(
            Node(
                package="go2w_perception",
                executable="carto_odom_bridge.py",
                namespace=robot_ns,
                name="carto_odom_bridge",
                parameters=[
                    {
                        "parent_frame": "map",
                        "child_frame": "body",
                        "output_topic": f"/{robot_ns}/odom/nav",
                        "output_frame_id": "map",
                        "output_child_frame_id": "base_link",
                        "rate": 50.0,
                    }
                ],
                output="screen",
            )
        )

    # ── PointCloud → LaserScan ──
    # Source cloud depends on SLAM backend:
    #   carto_l1       → /utlidar/transformed_cloud (from transform_everything)
    #   fastlio_mid360 → /cloud_registered_body     (from Fast-LIO, body frame)
    # Without this branching, Fast-LIO mode had no scan → reactive_nav's
    # has_scan stayed false forever → zero cmd_vel → robot never moved.
    if slam == "fastlio_mid360":
        cloud_in_topic = "/cloud_registered_body"
    else:
        cloud_in_topic = "/utlidar/transformed_cloud"
    actions.append(
        Node(
            package="pointcloud_to_laserscan",
            executable="pointcloud_to_laserscan_node",
            namespace=robot_ns,
            name="pointcloud_to_laserscan",
            parameters=[
                {
                    "use_sim_time": False,
                    "target_frame": "base_link",
                    "transform_tolerance": 0.3,
                    "min_height": -0.25,
                    "max_height": 0.60,
                    "angle_min": -3.14159,
                    "angle_max": 3.14159,
                    "angle_increment": 0.006135923151543,
                    "scan_time": 0.1,
                    "range_min": 0.10,
                    "range_max": lidar_range,
                    "use_inf": True,
                }
            ],
            remappings=[
                ("cloud_in", cloud_in_topic),
                ("scan", f"/{robot_ns}/scan_3d_raw"),
            ],
            output="screen",
        )
    )

    # ── Rear self-hit filter (masks rays that catch the robot's own rear) ──
    actions.append(
        Node(
            package="go2_nav_algorithms",
            executable="scan_rear_filter",
            namespace=robot_ns,
            name="scan_rear_filter",
            parameters=[{"rear_blank_radius": 0.45}],
            remappings=[
                ("scan_in", f"/{robot_ns}/scan_3d_raw"),
                ("scan_out", f"/{robot_ns}/scan_3d"),
            ],
            output="screen",
        )
    )

    # ── Cartographer probability-grid → binary occupancy (optional) ──
    # ONLY when Cartographer is the SLAM backend — otherwise there's no
    # /robot/map_prob input, and the binarizer publishes an empty grid on
    # the same topic octomap writes to, clobbering octomap's output with
    # TRANSIENT_LOCAL-latched emptiness (RViz then shows a fixed, tiny map).
    if map_backend in {"carto_binary", "carto_2d"} and slam == "carto_l1":
        actions.append(
            Node(
                package="go2w_perception",
                executable="probability_grid_binarizer.py",
                namespace=robot_ns,
                name="probability_grid_binarizer",
                parameters=[
                    {
                        "input_topic": f"/{robot_ns}/map_prob",
                        "output_topic": f"/{robot_ns}/map",
                        "free_threshold": 25,
                        "occupied_threshold": 65,
                        "min_occupied_component_cells": 3,
                        "fill_holes": True,
                        "hole_neighbor_threshold": 7,
                    }
                ],
                output="screen",
            )
        )

    # ── cmd_vel pipeline: stamped → auto_mux → sport API ──
    actions.append(
        Node(
            package="go2w_perception",
            executable="twist_bridge.py",
            namespace=robot_ns,
            name="twist_bridge",
            remappings=[
                ("cmd_vel_stamped", f"/{robot_ns}/cmd_vel_stamped"),
                ("cmd_vel", f"/{robot_ns}/cmd_vel_auto"),
            ],
            output="screen",
        )
    )
    actions.append(
        Node(
            package="go2w_control",
            executable="cmd_vel_activity_mux.py",
            namespace=robot_ns,
            name="cmd_vel_activity_mux",
            parameters=[
                {
                    "auto_topic": f"/{robot_ns}/cmd_vel_auto",
                    "manual_topic": f"/{robot_ns}/cmd_vel_manual",
                    "output_topic": "/cmd_vel",
                    "status_topic": f"/{robot_ns}/control_source",
                    "supervisor_state_topic": f"/{robot_ns}/supervisor_state",
                    "publish_rate": 20.0,
                    "manual_timeout_sec": float(_get(context, "manual_timeout_sec")),
                    "auto_timeout_sec": float(_get(context, "auto_timeout_sec")),
                    "linear_activity_threshold": float(_get(context, "manual_linear_threshold")),
                    "angular_activity_threshold": float(_get(context, "manual_angular_threshold")),
                }
            ],
            output="screen",
        )
    )
    # ── /cmd_vel → Unitree sport API ──
    # Gated by execute_controller: setting it false leaves the full nav +
    # exploration + mux stack running and /cmd_vel published, but nothing
    # reaches the robot. Used by dry_run shim for pre-flight sanity checks.
    if execute_controller:
        actions.append(
            Node(
                package="go2w_control",
                executable="cmd_vel_to_sport_bridge.py",
                name="cmd_vel_to_sport_bridge",
                parameters=[
                    {"cmd_vel_topic": "/cmd_vel"},
                    {"sport_topic": "/api/sport/request"},
                    {"obstacle_avoidance": use_obstacle_avoidance},
                ],
                output="screen",
            )
        )

    # ── Manual fallback (PS3 joystick) ──
    if enable_manual_fallback:
        actions.extend(
            [
                Node(
                    package="joy",
                    executable="joy_node",
                    name="ps3_joy",
                    parameters=[
                        {
                            "dev": _get(context, "joy_dev"),
                            "deadzone": float(_get(context, "joy_deadzone")),
                            "autorepeat_rate": float(_get(context, "joy_autorepeat_rate")),
                        }
                    ],
                    output="screen",
                ),
                Node(
                    package="teleop_twist_joy",
                    executable="teleop_node",
                    namespace=robot_ns,
                    name="teleop_twist_joy_node",
                    parameters=[teleop_config, {"publish_stamped_twist": False}],
                    remappings=[
                        ("/joy", "/joy"),
                        ("/cmd_vel", f"/{robot_ns}/cmd_vel_manual"),
                    ],
                    output="screen",
                ),
            ]
        )

    return actions


def generate_launch_description() -> LaunchDescription:
    return LaunchDescription(
        [
            DeclareLaunchArgument("robot_namespace", default_value="robot"),
            DeclareLaunchArgument("map_backend", default_value="carto_2d"),
            DeclareLaunchArgument("slam", default_value="carto_l1",
                                   description="carto_l1 or fastlio_mid360 — used to gate the Cartographer-specific binarizer so it doesn't clobber octomap's /robot/map in Fast-LIO mode"),
            DeclareLaunchArgument("obstacle_avoidance", default_value="true"),
            DeclareLaunchArgument("run_transform_everything", default_value="true"),
            DeclareLaunchArgument("execute_controller", default_value="true",
                                   description="false = dry-run; publish /cmd_vel but DON'T forward to sport API"),
            DeclareLaunchArgument("imu_calib_yaml", default_value=""),
            DeclareLaunchArgument(
                "lidar_range", default_value="8.0",
                description="Perception range cap (m) for pointcloud_to_laserscan."),
            DeclareLaunchArgument("enable_manual_fallback", default_value="true"),
            DeclareLaunchArgument("joy_dev", default_value="/dev/input/js0"),
            DeclareLaunchArgument("joy_deadzone", default_value="0.12"),
            DeclareLaunchArgument("joy_autorepeat_rate", default_value="20.0"),
            DeclareLaunchArgument("manual_timeout_sec", default_value="0.35"),
            DeclareLaunchArgument("auto_timeout_sec", default_value="0.60"),
            DeclareLaunchArgument("manual_linear_threshold", default_value="0.02"),
            DeclareLaunchArgument("manual_angular_threshold", default_value="0.05"),
            OpaqueFunction(function=_launch_setup),
        ]
    )
