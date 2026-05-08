#!/usr/bin/env python3

_ws_root = os.path.abspath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", ".."
))

"""Minimal MuJoCo nav test: Cartographer SLAM + CFPA2 + A*/FAR + RViz waypoint.

No VLM stack — just sim + SLAM + navigation for debugging.

Nav backends (nav_backend:=):
  astar — go2w_nav astar_nav_node (default)
  far   — CMU autonomy stack: terrain_analysis + far_planner + localPlanner + pathFollower

Modes:
  - Default: CFPA2 frontier exploration drives the robot autonomously.
  - Manual:  Pass explore:=false, then use RViz "2D Goal Pose" to send goals.

Usage:
  ros2 launch go2_gazebo_sim nav_test_mujoco.launch.py
  ros2 launch go2_gazebo_sim nav_test_mujoco.launch.py nav_backend:=far
  ros2 launch go2_gazebo_sim nav_test_mujoco.launch.py explore:=false   # manual goals only
  ros2 launch go2_gazebo_sim nav_test_mujoco.launch.py gui:=false       # headless MuJoCo
"""

from __future__ import annotations

import os

import yaml
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


def _as_bool(value: str) -> bool:
    return str(value).strip().lower() in {"1", "true", "yes", "on"}


def _get(context, key: str) -> str:
    return LaunchConfiguration(key).perform(context)


def _load_yaml_params(yaml_path: str) -> dict:
    """Load a ROS2 YAML param file and return the ros__parameters dict.

    CMU autonomy stack YAML files are keyed by unqualified node name
    (e.g. ``far_planner:``), which doesn't match when the node is
    launched in a namespace. Strip the outer key and return just the
    parameter dict so it can be merged into the launch params list.
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
    mujoco_model_path = _get(context, "mujoco_model_path").strip()
    # Bounded session + wall-checker toggles (for headless benchmark runs).
    session_duration_sec = float(_get(context, "session_duration_sec"))
    session_output_path = _get(context, "session_output_path").strip()
    enable_wall_checker = _as_bool(_get(context, "enable_wall_checker"))
    # Ground-truth observable area for coverage_ratio_of_scene. 96 m² is
    # the 12 m × 8 m inner room of demo1.xml.
    scene_area_m2 = float(_get(context, "scene_area_m2"))
    # Velocity-aware safety supervisor (LiDAR-scan based velocity clamp).
    # When true, inserts scripts/velocity_safety_supervisor.py between
    # pathFollower and twist_bridge on the cmd_vel path. Real-robot safe.
    enable_velocity_supervisor = _as_bool(_get(context, "enable_velocity_supervisor"))

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    vlm_pkg = get_package_share_directory("vlm_explorer")
    go2w_config_pkg = get_package_share_directory("go2w_config")
    cfpa2_pkg = get_package_share_directory("cfpa2_collaborative_autonomy")

    if not mujoco_model_path:
        mujoco_model_path = os.path.join(go2_gazebo_pkg, "mujoco", "demo1.xml")

    tf_remaps = [("/tf", f"/{robot_ns}/tf"), ("/tf_static", f"/{robot_ns}/tf_static")]
    carto_cfg_dir = os.path.join(vlm_pkg, "config")
    carto_cfg_basename = "cartographer_sim_2d.lua"
    nav_backend = _get(context, "nav_backend").strip().lower() or "astar"
    # Back-compat alias — rrt_star points to astar now (reactive_nav deleted).
    if nav_backend == "rrt_star":
        nav_backend = "astar"
    if nav_backend not in {"astar", "far"}:
        raise ValueError(f"nav_backend must be 'astar' or 'far', got '{nav_backend}'")
    cfpa2_config_path = os.path.join(cfpa2_pkg, "config", "cfpa2_single_robot.yaml")

    actions = []

    # ── 1. Base platform: MuJoCo + CHAMP + sensors + perception ──
    base_launch = os.path.join(go2_gazebo_pkg, "launch", "single_go2w_mujoco_cfpa2.launch.py")
    actions.append(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(base_launch),
            launch_arguments={
                "robot_namespace": robot_ns,
                "use_sim_time": "true",
                "gui": gui,
                "rviz": "false",  # we launch our own RViz
                "cleanup_stale": "true",
                "enable_assets": "true",
                "enable_perception": "true",
                "enable_slam": "false",       # Cartographer provides SLAM
                "enable_control": "true",
                "enable_navigation": "false",  # we add our own below
                "use_fast_lio": "false",       # using Cartographer, not Fast-LIO
                "odom_bridge_publish_tf": "false",  # Cartographer owns map→odom→base_link
                "mujoco_model_path": mujoco_model_path,
                "spawn_x": _get(context, "spawn_x"),
                "spawn_y": _get(context, "spawn_y"),
                "spawn_yaw": _get(context, "spawn_yaw"),
            }.items(),
        )
    )

    # ── 2. Cartographer 2D SLAM ──
    slam_delay = 10.0

    # Cartographer uses Google glog for its info spam; silence it via
    # GLOG_minloglevel=2 (warning+). Also drop the ROS2 wrapper logger
    # to WARN so the "imu rate: ..." lines stop.
    carto_env = {"GLOG_minloglevel": "2"}
    carto_ros_args = ["--ros-args", "--log-level", "WARN"]
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
            ] + carto_ros_args,
            remappings=tf_remaps + [
                ("points2", f"/{robot_ns}/registered_scan_reliable"),
                ("imu", f"/{robot_ns}/imu/data"),
            ],
            additional_env=carto_env,
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
            ] + carto_ros_args,
            parameters=[{"use_sim_time": use_sim_time}],
            additional_env=carto_env,
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

    # ── 3. CFPA2 frontier exploration (optional) ──
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

    # ── 4. Navigation backend ──
    if nav_backend == "astar":
        nav_config_path = os.path.join(go2w_config_pkg, "config", "nav", "astar_nav_go2w.yaml")
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

        # octomap_server: 3D voxel grid from the reliable registered scan
        # projected to 2D, merged with Cartographer's /robot/map so astar
        # sees thin obstacles (cylinders, crates, dividers) as fast as
        # Cartographer sees walls.
        octomap_node = Node(
            package="octomap_server",
            executable="octomap_server_node",
            namespace=robot_ns,
            name="octomap_server",
            parameters=[{
                "use_sim_time": use_sim_time,
                "resolution": 0.05,
                "frame_id": "map",
                "base_frame_id": "base_link",
                "sensor_model.max_range": 6.0,
                "sensor_model.hit": 0.8,
                "sensor_model.miss": 0.35,
                "sensor_model.min": 0.12,
                "sensor_model.max": 0.97,
                "point_cloud_min_z": 0.05,
                "point_cloud_max_z": 1.10,
                "occupancy_min_z": 0.05,
                "occupancy_max_z": 1.00,
                "filter_ground_plane": False,
                "incremental_2D_projection": False,
                "filter_speckles": False,
                "compress_map": True,
                "latch": False,
                "publish_free_space": False,
            }],
            remappings=[
                ("cloud_in", f"/{robot_ns}/registered_scan_reliable"),
                ("projected_map", f"/{robot_ns}/octomap_projected_map"),
            ] + tf_remaps,
            output="screen",
        )

        # map_merger: union Cartographer's /robot/map with octomap's
        # projected map and publish /robot/map_merged for the A* planner.
        map_merger_node = Node(
            package="go2w_perception",
            executable="map_merger.py",
            namespace=robot_ns,
            name="map_merger",
            parameters=[{
                "use_sim_time": use_sim_time,
                "primary_topic": f"/{robot_ns}/map",
                "secondary_topic": f"/{robot_ns}/octomap_projected_map",
                "output_topic": f"/{robot_ns}/map_merged",
                "secondary_occupied_thresh": 50,
                "publish_rate_hz": 4.0,
            }],
            output="screen",
        )

        astar_nav_node = Node(
            package="go2w_nav",
            executable="astar_nav_node",
            namespace=robot_ns,
            name="astar_nav",
            parameters=[
                nav_config_path,
                {"use_sim_time": use_sim_time},
                {
                    "map_frame": "map",
                    # Union of Cartographer + octomap (see map_merger above).
                    "map_topic": f"/{robot_ns}/map_merged",
                    "frontier_replan_topic": f"/{robot_ns}/frontier_replan",
                    "stop_topic": f"/{robot_ns}/stop",
                },
            ],
            remappings=nav_remappings + tf_remaps,
            output="screen",
        )

        actions.append(
            TimerAction(
                period=nav_delay,
                actions=[octomap_node, map_merger_node, astar_nav_node],
            )
        )
    else:  # nav_backend == "far"
        far_scan_topic = f"/{robot_ns}/registered_scan_map"
        far_odom_topic = f"/{robot_ns}/odom/nav"
        # Iter 6 (2026-04-15): 0.2 m/s is the proven-safe default at the
        # position-based checks. A 0.4 m/s ceiling only makes sense when
        # the velocity-aware supervisor is enabled (enable_velocity_supervisor
        # launch arg), which caps cmd_vel to v_cap = sqrt(2·a·(d_nearest-
        # d_safe)) based on live scan clearance. Without the supervisor,
        # stay at 0.2 — position checks + twoWayDrive + checkRotObstacle
        # give 7/10 PASS at 120s with two failure modes (corner-wedge +
        # yaw-drift stall).
        far_max_speed = 0.4 if enable_velocity_supervisor else 0.2

        # pathFollower output topic: canonical cmd_vel_stamped when no
        # supervisor, or a "_raw" sibling that the supervisor consumes.
        if enable_velocity_supervisor:
            pf_cmd_out_topic = f"/{robot_ns}/cmd_vel_stamped_raw"
        else:
            pf_cmd_out_topic = f"/{robot_ns}/cmd_vel_stamped"
        local_planner_pkg = get_package_share_directory("local_planner")

        far_nodes = [
            # sensor_scan_generation: sync odom+cloud (WARN to reduce noise)
            Node(
                package="sensor_scan_generation",
                executable="sensorScanGeneration",
                namespace=robot_ns,
                name="sensor_scan_generation",
                arguments=["--ros-args", "--log-level", "WARN"],
                parameters=[{"use_sim_time": use_sim_time}],
                remappings=[
                    ("/state_estimation", far_odom_topic),
                    ("/registered_scan", far_scan_topic),
                    ("/state_estimation_at_scan", f"/{robot_ns}/state_estimation_at_scan"),
                    ("/sensor_scan", f"/{robot_ns}/sensor_scan"),
                ] + tf_remaps,
                output="screen",
            ),
            # terrain_analysis: local terrain voxel map (WARN to reduce noise)
            Node(
                package="terrain_analysis",
                executable="terrainAnalysis",
                namespace=robot_ns,
                name="terrain_analysis",
                arguments=["--ros-args", "--log-level", "WARN"],
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
                arguments=["--ros-args", "--log-level", "WARN"],
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
                        # Iter 2: tighten waypoint convergence so FAR doesn't
                        # declare a waypoint "reached" at 0.5 m and flip.
                        "g_planner/converge_distance": 0.3,
                        # Iter 7 (2026-04-15): 10 → 25. Higher momentum makes
                        # FAR commit harder to its current path instead of
                        # flip-flopping between routes. Reduces the upstream
                        # cause of "waypoint suddenly behind the robot" that
                        # triggers pathFollower's reverse mode.
                        "g_planner/path_momentum_thred": 25,
                        # Iter 3: back to default (5). Higher values kept stale
                        # edges around for 10 ticks, amplifying thrash.
                        "graph/connect_votes_size": 5,
                        "util/terrain_free_Z": 0.45,
                        # Kept at 2. Tried 3 on 2026-04-15 to stop V-graph
                        # edges passing through thin walls — caused a deadlock
                        # loop (robot couldn't find any valid path through
                        # narrow corridors → oscillated → collision). The
                        # wall-through-planning issue needs a different fix
                        # (costmap static layer injection, not more inflation).
                        "util/obs_inflate_size": 2,
                        # Iter 4: CAP THE GRAPH SIZE.
                        # The scene is 12x8m; with sensor_range=10 and
                        # map_grid_max_length=200, FAR accumulated thousands
                        # of stale nodes/edges from repeated passes, causing
                        # plan_ms to spike to 3700ms. Shrink both to limit
                        # how much of the scene contributes to the graph.
                        "sensor_range": 6.0,
                        "terrain_range": 4.5,
                        "local_planner_range": 2.0,
                        "map_handler/map_grid_max_length": 30.0,
                        # Iter 8: raised from 0.5 → 3.0 s. At 0.5 s, wall
                        # evidence vanished from the V-graph before FAR
                        # finished replanning, letting edges pass through
                        # walls the robot had already seen. At 3.0 s the
                        # evidence persists across several FAR planning
                        # cycles (~1 Hz), keeping walls solid in the graph.
                        # Trade-off: dynamic obstacles (other robots, doors)
                        # also persist longer — acceptable in static scenes.
                        "util/dynamic_obs_dacay_time": 3.0,
                        "util/new_points_decay_time": 3.0,
                        # Iter 4: quieter logs — debug output was adding
                        # overhead every tick.
                        "is_debug_output": False,
                        "is_opencv_visual": False,
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
                    "obstacleHeightThre": 0.20,
                    "groundHeightThre": 0.1,
                    "costHeightThre": 0.1,
                    "costScore": 0.02,
                    "useCost": False,
                    "pointPerPathThre": 2,
                    "minRelZ": -0.5,
                    "maxRelZ": 1.2,
                    "maxSpeed": far_max_speed,
                    # Iter 7: forward-bias — strongly prefer forward primitives.
                    "dirWeight": 0.5,
                    "dirThre": 90.0,
                    "dirToVehicle": False,
                    "pathScale": 1.0,
                    "minPathScale": 0.75,
                    "pathScaleStep": 0.25,
                    "pathScaleBySpeed": False,
                    "minPathRange": 1.0,
                    "pathRangeStep": 0.5,
                    "pathRangeBySpeed": False,
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
                    "lookAheadDis": 0.8,
                    "yawRateGain": 1.0,
                    "stopYawRateGain": 0.8,
                    "maxYawRate": 45.0,
                    "maxSpeed": far_max_speed,
                    "maxAccel": 2.0,
                    "switchTimeThre": 1.0,
                    # Iter 7: forward-bias — higher threshold before reverse.
                    "dirDiffThre": 1.2,
                    "omniDirDiffThre": 1.5,
                    "noRotSpeed": 10.0,
                    "stopDisThre": 0.10,
                    "slowDwnDisThre": 0.50,
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
                    ("/cmd_vel", pf_cmd_out_topic),
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

        # ── 4b. Velocity-aware safety supervisor (optional) ──
        # Caps commanded velocity by live scan-based nearest-obstacle
        # clearance. Only wired into the FAR path because pathFollower is
        # the cmd_vel source there; astar branch would need a separate
        # hookup on astar_nav_node's output.
        if enable_velocity_supervisor:
            supervisor_script = os.path.join(_ws_root, "scripts/runtime/velocity_safety_supervisor.py")
            supervisor_proc = ExecuteProcess(
                cmd=[
                    "python3", "-u", supervisor_script,
                    "--ros-args",
                    "-p", f"scan_topic:=/{robot_ns}/scan_3d",
                    "-p", f"cmd_in_topic:=/{robot_ns}/cmd_vel_stamped_raw",
                    "-p", f"cmd_out_topic:=/{robot_ns}/cmd_vel_stamped",
                    "-p", f"max_linear_speed_m_s:={far_max_speed}",
                    "-p", "max_decel_m_s2:=2.0",
                    "-p", "safety_margin_m:=0.10",
                    "-p", "forward_arc_half_rad:=1.047",
                    "-p", "publish_rate_hz:=50.0",
                ],
                name="velocity_safety_supervisor",
                output="screen",
            )
            actions.append(
                TimerAction(period=nav_delay, actions=[supervisor_proc])
            )

    # ── 5. RViz goal relay (always on — click "2D Goal Pose" to send manual goals) ──
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

    # ── 5b. FAR debug monitor ──
    # Integrated into launch — prints 1-line/sec summary of FAR I/O
    # with color-coded STUCK/REVERSE/OSCILLATE/CONTACT warnings.
    # Silences non-FAR nodes to keep the terminal readable.
    far_debug_script = os.path.join(_ws_root, "scripts/debug/far_debug_monitor.py")
    actions.append(
        TimerAction(
            period=nav_delay + 5.0,
            actions=[
                ExecuteProcess(
                    cmd=["python3", "-u", far_debug_script],
                    name="far_debug_monitor",
                    output="screen",
                ),
            ],
        )
    )

    # ── 6. Wall / tip-over fail checker (optional — terminal) ──
    # Runs scripts/far_wall_checker.py as a launch process. On robot-body
    # contact with wall_/divider_ geoms OR tip-over (|roll|, |pitch| > 45°),
    # the checker prints a FAIL banner and exits 1. The OnProcessExit event
    # handler then shuts down the entire launch so the failure is terminal.
    # Disabled via `enable_wall_checker:=false` for benchmark runs that
    # want to measure contact count without the launch dying on first hit.
    if enable_wall_checker:
        wall_checker_script = os.path.join(_ws_root, "scripts/runtime/far_wall_checker.py")
        wall_checker_proc = ExecuteProcess(
            cmd=["python3", "-u", wall_checker_script],
            name="far_wall_checker",
            output="screen",
        )
        actions.append(
            TimerAction(
                period=nav_delay + 3.0,  # wait until standup + nav are done
                actions=[wall_checker_proc],
            )
        )
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=wall_checker_proc,
                    on_exit=[
                        LogInfo(
                            msg="far_wall_checker exited — shutting down launch "
                            "(robot hit a wall or tipped over)"
                        ),
                        Shutdown(reason="far_wall_checker detected failure"),
                    ],
                )
            )
        )

    # ── 6b. Bounded session reporter (optional — graceful exit on timeout) ──
    # Runs scripts/session_reporter.py for `session_duration_sec` seconds.
    # Subscribes to /<ns>/map, /<ns>/odom/nav, /mujoco/contacts and emits a
    # JSON report to `session_output_path` on final tick (or SIGTERM). When
    # it exits cleanly, the OnProcessExit handler shuts the whole launch
    # down so headless benchmark runs reliably terminate at the set bound.
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
            TimerAction(
                period=nav_delay + 3.0,
                actions=[session_proc],
            )
        )
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=session_proc,
                    on_exit=[
                        LogInfo(
                            msg="session_reporter exited — shutting down launch "
                            "(bounded session complete)"
                        ),
                        Shutdown(reason="session_reporter session complete"),
                    ],
                )
            )
        )

    # ── 7. RViz2 ──
    if rviz:
        rviz_config = os.path.join(go2_gazebo_pkg, "rviz", "nav_test.rviz")
        actions.append(
            TimerAction(
                period=7.0,
                actions=[
                    Node(
                        package="rviz2",
                        executable="rviz2",
                        name="rviz2_nav_test",
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
    default_scene = os.path.join(go2_gazebo_pkg, "mujoco", "demo1.xml")

    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="robot"),
        DeclareLaunchArgument("gui", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("explore", default_value="true",
                              description="Enable CFPA2 autonomous frontier exploration"),
        DeclareLaunchArgument("nav_backend", default_value="astar",
                              description="Nav backend: astar (default) or far"),
        DeclareLaunchArgument("mujoco_model_path", default_value=default_scene),
        DeclareLaunchArgument("spawn_x", default_value="4.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("session_duration_sec", default_value="0",
                              description="If > 0, run bounded session reporter "
                              "and shut launch down after N seconds"),
        DeclareLaunchArgument("session_output_path",
                              default_value="/tmp/session_reports/latest.json",
                              description="JSON output path for the session reporter"),
        DeclareLaunchArgument("scene_area_m2", default_value="96.0",
                              description="Sim ground-truth observable area (m²) "
                              "used as denominator for coverage_ratio_of_scene. "
                              "Default 96 for vlm_exploration_scene_no_artifacts."),
        DeclareLaunchArgument("enable_wall_checker", default_value="false",
                              description="Test mode: crash-stop on first wall contact. "
                              "Default false (benchmark mode — session reporter "
                              "captures all contacts without killing the launch). "
                              "Use enable_wall_checker:=true when developing to "
                              "get instant feedback on wall hits."),
        DeclareLaunchArgument("enable_velocity_supervisor", default_value="false",
                              description="Enable LiDAR-scan velocity-aware safety "
                              "supervisor between pathFollower and twist_bridge; "
                              "allows far_max_speed=0.4 to be safe"),
        OpaqueFunction(function=_launch_setup),
    ])
