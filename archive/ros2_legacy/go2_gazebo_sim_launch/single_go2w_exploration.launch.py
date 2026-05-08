#!/usr/bin/env python3
"""Single Go2W autonomous exploration with switchable nav backend.

MuJoCo sim + Cartographer 2D SLAM + CFPA2 frontier exploration.
No VLM stack — pure autonomous frontier-driven exploration.

Nav backends (nav_backend:=):
  astar     — C++ A* + pure-pursuit + oriented footprint check (default)
  far       — CMU autonomy stack: terrain_analysis + far_planner + localPlanner + pathFollower

Usage:
  ros2 launch go2_gazebo_sim single_go2w_exploration.launch.py
  ros2 launch go2_gazebo_sim single_go2w_exploration.launch.py nav_backend:=far
  ros2 launch go2_gazebo_sim single_go2w_exploration.launch.py explore:=false   # manual RViz goals only
"""

from __future__ import annotations

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    OpaqueFunction,
    TimerAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, key: str) -> str:
    return LaunchConfiguration(key).perform(context)


def _load_yaml_params(yaml_path: str) -> dict:
    """Load a ROS2 YAML param file and return the ros__parameters dict.

    CMU autonomy stack YAML files are keyed by unqualified node name
    (e.g. ``far_planner:``), which doesn't match when the node is
    launched in a namespace.  This strips the outer key and returns
    just the parameter dict.
    """
    with open(yaml_path, "r") as f:
        data = yaml.safe_load(f) or {}
    for _node_name, inner in data.items():
        if isinstance(inner, dict) and "ros__parameters" in inner:
            return dict(inner["ros__parameters"])
    return data


def _launch_setup(context):
    use_sim_time = True
    robot_ns = _get(context, "robot_namespace").strip().strip("/") or "robot"
    gui = _get(context, "gui")
    rviz = _as_bool(_get(context, "rviz"))
    explore = _as_bool(_get(context, "explore"))
    nav_backend = _get(context, "nav_backend").strip().lower() or "astar"
    # Back-compat: reactive_nav_node and mppi_nav_node were deleted 2026-04-24.
    # Silently upgrade old invocations.
    if nav_backend in ("rrt_star", "reactive", "far_rrt_star", "mppi"):
        nav_backend = "astar"
    mujoco_model_path = _get(context, "mujoco_model_path").strip()

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    vlm_pkg = get_package_share_directory("vlm_explorer")
    go2w_config_pkg = get_package_share_directory("go2w_config")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")

    if not mujoco_model_path:
        mujoco_model_path = os.path.join(go2_gazebo_pkg, "mujoco", "vlm_exploration_scene.xml")

    tf_remaps = [("/tf", f"/{robot_ns}/tf"), ("/tf_static", f"/{robot_ns}/tf_static")]
    carto_cfg_dir = os.path.join(vlm_pkg, "config")
    carto_cfg_basename = "cartographer_sim_2d.lua"
    cfpa2_config_path = os.path.join(cfpa2_pkg, "config", "cfpa2_single_robot.yaml")

    actions = []

    # ── 1. Base platform: MuJoCo + CHAMP + sensors + perception ──────
    base_launch = os.path.join(go2_gazebo_pkg, "launch", "single_go2w_mujoco_cfpa2.launch.py")
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "use_sim_time": "true",
                "gui": gui,
                "rviz": "false",
                "cleanup_stale": "true",
                "enable_assets": "true",
                "enable_perception": "true",
                "enable_slam": "false",
                "enable_control": "true",
                "enable_navigation": "false",
                "use_fast_lio": "false",
                "odom_bridge_publish_tf": "false",
                "mujoco_model_path": mujoco_model_path,
                "spawn_x": _get(context, "spawn_x"),
                "spawn_y": _get(context, "spawn_y"),
                "spawn_yaw": _get(context, "spawn_yaw"),
            }.items(),
        )
    )

    # ── 2. Cartographer 2D SLAM ──────────────────────────────────────
    slam_delay = 10.0

    carto_nodes = [
        Node(
            package="cartographer_ros",
            executable="cartographer_node",
            name="cartographer_node",
            namespace=robot_ns,
            parameters=[{"use_sim_time": use_sim_time}],
            arguments=[
                "-configuration_directory", carto_cfg_dir,
                "-configuration_basename", carto_cfg_basename,
            ],
            remappings=tf_remaps + [
                ("points2", f"/{robot_ns}/registered_scan_reliable"),
                ("imu", f"/{robot_ns}/imu/data"),
            ],
            output="screen",
        ),
        Node(
            package="cartographer_ros",
            executable="cartographer_occupancy_grid_node",
            name="cartographer_occupancy_grid_node",
            namespace=robot_ns,
            remappings=[("map", "map_prob")],
            arguments=[
                "-resolution=0.05",
                "-publish_period_sec=0.5",
            ],
            parameters=[{"use_sim_time": use_sim_time}],
            output="screen",
        ),
        Node(
            package="go2w_perception",
            executable="probability_grid_binarizer.py",
            name="probability_grid_binarizer",
            namespace=robot_ns,
            parameters=[
                {"use_sim_time": use_sim_time},
                {"input_topic": f"/{robot_ns}/map_prob"},
                {"output_topic": f"/{robot_ns}/map"},
                {"free_threshold": 49},
                {"occupied_threshold": 65},
                {"min_occupied_component_cells": 2},
                {"fill_holes": True},
                {"hole_neighbor_threshold": 7},
            ],
            output="screen",
        ),
        Node(
            package="go2w_perception",
            executable="carto_odom_bridge.py",
            name="carto_odom_bridge",
            namespace=robot_ns,
            parameters=[
                {"use_sim_time": use_sim_time},
                {"parent_frame": "map"},
                {"child_frame": "base_link"},
                {"output_topic": f"/{robot_ns}/odom/nav"},
                {"output_frame_id": "map"},
                {"output_child_frame_id": "base_link"},
                {"rate": 50.0},
            ],
            remappings=tf_remaps,
            output="screen",
        ),
    ]

    # FAR needs point cloud in map frame — add frame bridge alongside Cartographer
    if nav_backend == "far":
        carto_nodes.append(
            Node(
                package="go2w_perception",
                executable="pointcloud_frame_bridge.py",
                name="registered_scan_frame_bridge",
                namespace=robot_ns,
                parameters=[
                    {"use_sim_time": use_sim_time},
                    {"input_topic": f"/{robot_ns}/registered_scan_reliable"},
                    {"output_topic": f"/{robot_ns}/registered_scan_map"},
                    {"target_frame": "map"},
                    {"tf_timeout_sec": 0.15},
                    {"transform_wait_sec": 0.10},
                    {"max_cloud_age_sec": 0.80},
                ],
                remappings=tf_remaps,
                output="screen",
            )
        )

    actions.append(TimerAction(period=slam_delay, actions=carto_nodes))

    # ── 3. CFPA2 frontier exploration (optional) ─────────────────────
    nav_delay = slam_delay + 3.0

    if explore:
        actions.append(
            TimerAction(
                period=nav_delay,
                actions=[
                    Node(
                        package="cfpa2_collaborative_autonomy",
                        executable="cfpa2_single_robot_node",
                        name="cfpa2_single_robot",
                        parameters=[
                            cfpa2_config_path,
                            {
                                "use_sim_time": use_sim_time,
                                "robot_namespace": robot_ns,
                                "namespaces": [robot_ns],
                                "goal_topic_suffix": "/way_point_coord",
                                "marker_frame_override": "map",
                            },
                        ],
                        output="screen",
                    ),
                ],
            )
        )

    # ── 4. Navigation backend ────────────────────────────────────────
    if nav_backend == "far":
        far_scan_topic = f"/{robot_ns}/registered_scan_map"
        far_odom_topic = f"/{robot_ns}/odom/nav"
        far_max_speed = 0.5
        local_planner_pkg = get_package_share_directory("local_planner")

        far_nodes = [
            # sensor_scan_generation: sync odom+cloud, broadcast sensor_at_scan TF
            Node(
                package="sensor_scan_generation",
                executable="sensorScanGeneration",
                namespace=robot_ns,
                name="sensor_scan_generation",
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    ("/state_estimation", far_odom_topic),
                    ("/registered_scan", far_scan_topic),
                    ("/state_estimation_at_scan", f"/{robot_ns}/state_estimation_at_scan"),
                    ("/sensor_scan", f"/{robot_ns}/sensor_scan"),
                ] + tf_remaps,
                output="screen",
            ),
            # terrain_analysis: local terrain voxel map
            Node(
                package="terrain_analysis",
                executable="terrainAnalysis",
                namespace=robot_ns,
                name="terrain_analysis",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "maxRelZ": 0.8,
                }],
                remappings=[
                    ("/state_estimation", far_odom_topic),
                    ("/registered_scan", far_scan_topic),
                    ("/joy", f"/{robot_ns}/joy"),
                    ("/map_clearing", f"/{robot_ns}/map_clearing"),
                    ("/terrain_map", f"/{robot_ns}/terrain_map"),
                ],
                output="screen",
            ),
            # terrain_analysis_ext: extended range terrain
            Node(
                package="terrain_analysis_ext",
                executable="terrainAnalysisExt",
                namespace=robot_ns,
                name="terrain_analysis_ext",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "maxRelZ": 0.8,
                }],
                remappings=[
                    ("/state_estimation", far_odom_topic),
                    ("/registered_scan", far_scan_topic),
                    ("/joy", f"/{robot_ns}/joy"),
                    ("/cloud_clearing", f"/{robot_ns}/cloud_clearing"),
                    ("/terrain_map", f"/{robot_ns}/terrain_map"),
                    ("/terrain_map_ext", f"/{robot_ns}/terrain_map_ext"),
                ],
                output="screen",
            ),
            # far_planner: V-graph global route planner
            Node(
                package="far_planner",
                executable="far_planner",
                namespace=robot_ns,
                name="far_planner",
                parameters=[
                    _load_yaml_params(os.path.join(
                        get_package_share_directory("far_planner"), "config", "default.yaml"
                    )),
                    {
                        "use_sim_time": use_sim_time,
                        "world_frame": "map",
                        "graph_msger/robot_id": 0,
                        "g_planner/converge_distance": 0.5,
                        "util/terrain_free_Z": 0.45,
                        "util/obs_inflate_size": 1,
                    },
                ],
                remappings=[
                    ("/odom_world", far_odom_topic),
                    ("/terrain_cloud", f"/{robot_ns}/terrain_map_ext"),
                    ("/scan_cloud", f"/{robot_ns}/terrain_map"),
                    ("/terrain_local_cloud", far_scan_topic),
                    ("/goal_point", f"/{robot_ns}/way_point_coord"),
                    ("/way_point", f"/{robot_ns}/way_point"),
                    ("/joy", f"/{robot_ns}/joy"),
                    ("/navigation_boundary", f"/{robot_ns}/navigation_boundary"),
                    ("/runtime", f"/{robot_ns}/far_runtime"),
                    ("/planning_time", f"/{robot_ns}/far_planning_time"),
                    ("/robot_vgraph", f"/{robot_ns}/robot_vgraph"),
                    ("/decoded_vgraph", f"/{robot_ns}/decoded_vgraph"),
                ] + tf_remaps,
                output="screen",
            ),
            # localPlanner: kinematically-feasible path primitives
            Node(
                package="local_planner",
                executable="localPlanner",
                namespace=robot_ns,
                name="localPlanner",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "pathFolder": os.path.join(local_planner_pkg, "paths"),
                    # Real Go2W footprint (length 0.70m, width 0.31m + margin).
                    "vehicleLength": 0.70,
                    "vehicleWidth": 0.40,
                    "sensorOffsetX": 0.0,
                    "sensorOffsetY": 0.0,
                    "twoWayDrive": True,
                    "laserVoxelSize": 0.05,
                    "terrainVoxelSize": 0.2,
                    "useTerrainAnalysis": True,
                    "checkObstacle": True,
                    "checkRotObstacle": True,
                    "adjacentRange": 4.0,
                    "obstacleHeightThre": 0.15,
                    "groundHeightThre": 0.1,
                    "costHeightThre": 0.1,
                    "costScore": 0.02,
                    "useCost": False,
                    "pointPerPathThre": 2,
                    "minRelZ": -0.5,
                    "maxRelZ": 1.2,
                    "maxSpeed": far_max_speed,
                    "dirWeight": 0.02,
                    "dirThre": 90.0,
                    "dirToVehicle": False,
                    "pathScale": 1.0,
                    "minPathScale": 0.75,
                    "pathScaleStep": 0.25,
                    "pathScaleBySpeed": True,
                    "minPathRange": 1.0,
                    "pathRangeStep": 0.5,
                    "pathRangeBySpeed": True,
                    "pathCropByGoal": True,
                    "autonomyMode": True,
                    "autonomySpeed": far_max_speed,
                    "joyToSpeedDelay": 2.0,
                    "joyToCheckObstacleDelay": 5.0,
                    "goalClearRange": 0.5,
                    "goalX": 0.0,
                    "goalY": 0.0,
                }],
                remappings=[
                    ("/state_estimation", far_odom_topic),
                    ("/registered_scan", far_scan_topic),
                    ("/way_point", f"/{robot_ns}/way_point"),
                    ("/terrain_map", f"/{robot_ns}/terrain_map"),
                    ("/overall_map", f"/{robot_ns}/terrain_map"),
                    ("/joy", f"/{robot_ns}/joy"),
                    ("/path", f"/{robot_ns}/local_path"),
                    ("/freePaths", f"/{robot_ns}/free_paths"),
                ],
                output="screen",
            ),
            # pathFollower: pure-pursuit path tracking → cmd_vel
            Node(
                package="local_planner",
                executable="pathFollower",
                namespace=robot_ns,
                name="pathFollower",
                parameters=[{
                    "use_sim_time": use_sim_time,
                    "sensorOffsetX": 0.0,
                    "sensorOffsetY": 0.0,
                    "pubSkipNum": 1,
                    "twoWayDrive": True,
                    "lookAheadDis": 0.5,
                    "yawRateGain": 1.5,
                    "stopYawRateGain": 1.5,
                    "maxYawRate": 80.0,
                    "maxSpeed": far_max_speed,
                    "maxAccel": 2.0,
                    "switchTimeThre": 1.0,
                    "dirDiffThre": 0.4,
                    "omniDirDiffThre": 1.5,
                    "noRotSpeed": 10.0,
                    "stopDisThre": 0.15,
                    "slowDwnDisThre": 0.75,
                    "useInclRateToSlow": False,
                    "inclRateThre": 120.0,
                    "slowRate1": 0.25,
                    "slowRate2": 0.5,
                    "slowTime1": 2.0,
                    "slowTime2": 2.0,
                    "useInclToStop": False,
                    "inclThre": 45.0,
                    "stopTime": 5.0,
                    "noRotAtStop": False,
                    "noRotAtGoal": True,
                    "autonomyMode": True,
                    "autonomySpeed": far_max_speed,
                    "joyToSpeedDelay": 2.0,
                    "goalCloseDis": 0.4,
                    "is_real_robot": False,
                }],
                remappings=[
                    ("/state_estimation", far_odom_topic),
                    ("/path", f"/{robot_ns}/local_path"),
                    ("/cmd_vel", f"/{robot_ns}/cmd_vel_stamped"),
                    ("/joy", f"/{robot_ns}/joy"),
                    ("/speed", f"/{robot_ns}/speed"),
                    ("/stop", f"/{robot_ns}/stop"),
                ],
                output="screen",
            ),
            # Static TFs for CMU local planner convention
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=robot_ns,
                name="far_vehicle_tf",
                arguments=["0", "0", "0", "0", "0", "0", "sensor", "vehicle"],
                remappings=tf_remaps,
                output="screen",
            ),
            Node(
                package="tf2_ros",
                executable="static_transform_publisher",
                namespace=robot_ns,
                name="far_camera_tf",
                arguments=["0", "0", "0", "-1.5707963", "0", "-1.5707963", "sensor", "camera"],
                remappings=tf_remaps,
                output="screen",
            ),
        ]

        actions.append(TimerAction(period=nav_delay, actions=far_nodes))

    else:
        # A* local planner (only remaining non-FAR backend)
        nav_config_path = os.path.join(go2w_config_pkg, "config", "nav", "astar_nav_go2w.yaml")
        nav_executable = "astar_nav_node"
        nav_node_name = "astar_nav"

        nav_remappings = [
            ("/way_point", f"/{robot_ns}/way_point_coord"),
            ("/odom/ground_truth", f"/{robot_ns}/odom/nav"),
            ("/scan", f"/{robot_ns}/scan_3d"),
            ("/cmd_vel_stamped", f"/{robot_ns}/cmd_vel_stamped"),
            ("/nav_status", f"/{robot_ns}/nav_status"),
            ("/planned_path", f"/{robot_ns}/planned_path"),
            ("/robot_trajectory", f"/{robot_ns}/robot_trajectory"),
            ("/final_goal_marker", f"/{robot_ns}/final_goal_marker"),
            ("/robot_pose_marker", f"/{robot_ns}/robot_pose_marker"),
        ]
        actions.append(
            TimerAction(
                period=nav_delay,
                actions=[
                    Node(
                        package="go2w_nav",
                        executable=nav_executable,
                        namespace=robot_ns,
                        name=nav_node_name,
                        parameters=[
                            nav_config_path,
                            {"use_sim_time": use_sim_time},
                            {
                                "map_frame": "map",
                                "map_topic": f"/{robot_ns}/map",
                                "frontier_replan_topic": f"/{robot_ns}/frontier_replan",
                                "stop_topic": f"/{robot_ns}/stop",
                            },
                        ],
                        remappings=nav_remappings + tf_remaps,
                        output="screen",
                    ),
                ],
            )
        )

    # ── 5. RViz goal relay ───────────────────────────────────────────
    actions.append(
        TimerAction(
            period=nav_delay,
            actions=[
                Node(
                    package="go2w_nav",
                    executable="rviz_goal_relay.py",
                    namespace=robot_ns,
                    name="rviz_goal_relay",
                    parameters=[
                        {"output_topic": f"/{robot_ns}/way_point_coord"},
                    ],
                    output="screen",
                ),
            ],
        )
    )

    # ── 6. RViz2 ─────────────────────────────────────────────────────
    if rviz:
        rviz_config = os.path.join(go2_gazebo_pkg, "rviz", "nav_test.rviz")
        actions.append(
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2_exploration",
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
    default_scene = os.path.join(go2_gazebo_pkg, "mujoco", "vlm_exploration_scene.xml")

    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="robot"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("explore", default_value="true",
                              description="Enable CFPA2 autonomous frontier exploration"),
        DeclareLaunchArgument("nav_backend", default_value="astar",
                              description="Local planner: astar (default) or far (CMU autonomy stack)"),
        DeclareLaunchArgument("mujoco_model_path", default_value=default_scene),
        DeclareLaunchArgument("spawn_x", default_value="1.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        OpaqueFunction(function=_launch_setup),
    ])
