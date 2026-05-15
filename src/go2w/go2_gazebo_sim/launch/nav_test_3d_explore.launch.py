"""3D-frontier-exploration launch wrapper.

Composes the canonical nav_test_mujoco_fastlio.launch.py with:
  - demo_ramp.xml scene (override mujoco_model_path)
  - spawn at (2, 0) — west end of ramp scene
  - nvblox_frontend mapper_node (CUDA 3D mapping)

For CFPA2 ig_dimension=3d, edit
src/collaborative_exploration/cfpa2_collaborative_autonomy/config/cfpa2_single_robot.yaml
(or pass a yaml overlay via --ros-args -p) — the base launch hardcodes the
config path. The cfpa2_single_robot_3d.yaml overlay sitting next to it
contains the 3-line diff (planning_map_topic_suffix + ig_dimension +
voxels_3d_topic_suffix) ready to copy in.

A future tidy-up is to wire `cfpa2_config_path` as a LaunchArgument in the
base launch; deferred so this wrapper stays purely additive.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description() -> LaunchDescription:
    # Resolve the workspace root to point at the demo_ramp scene by default.
    # (This launch file lives in <ws>/src/go2w/go2_gazebo_sim/launch/.)
    here = os.path.dirname(os.path.realpath(__file__))
    ws_root = os.path.realpath(os.path.join(here, "..", "..", "..", ".."))
    default_scene = os.path.join(
        ws_root, "src", "go2w", "go2_gazebo_sim", "mujoco", "demo_ramp.xml")

    # Config paths for the new trav pipeline (Phase 4–5).
    elevation_cupy_share = get_package_share_directory("elevation_mapping_cupy")
    trav_share = get_package_share_directory("trav_cost_filters")
    emap_core_params = os.path.join(
        elevation_cupy_share, "config", "core", "core_param.yaml")
    emap_setup_params = os.path.join(
        trav_share, "config", "elevation_mapping.yaml")
    filter_chain_params = os.path.join(
        trav_share, "config", "grid_map_filters.yaml")

    args = [
        DeclareLaunchArgument("mujoco_model_path", default_value=default_scene),
        DeclareLaunchArgument("spawn_x", default_value="2.0"),
        DeclareLaunchArgument("spawn_y", default_value="0.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("robot_namespace", default_value="robot"),
        DeclareLaunchArgument("use_sim_time", default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("gui", default_value="true"),
        # nvblox_frontend knobs
        DeclareLaunchArgument("nvblox_voxel_size_m", default_value="0.10"),
        DeclareLaunchArgument(
            "enable_nvblox_mapper", default_value="false",
            description="Enable the optional nvblox_frontend mapper for "
                        "/<ns>/voxels_3d. Default false because the ETH "
                        "elevation_mapping_cupy traversability path does not "
                        "need nvblox and many dev machines do not have the "
                        "vendored nvblox CUDA library built."),
        # Costmap source: '3d' swaps both global+local StaticLayers to read
        # /robot/traversability_grid so planner and MPPI treat ramps as free.
        # Pass nav_costmap_mode:=2d to revert to the octomap-based baseline.
        DeclareLaunchArgument("nav_costmap_mode", default_value="3d",
            description="'3d': both costmaps use traversability_grid. "
                        "'2d': default octomap /robot/map (baseline)."),
        DeclareLaunchArgument(
            "enable_legacy_2d_proj", default_value="false",
            description="Re-enable mapper_node's legacy 2D traversability "
                        "projection. Default false; the planned "
                        "elevation_mapping_cupy + grid_map filter pipeline "
                        "(docs/claude/plans/2026-05-14-trav-grid-rewrite.md) "
                        "owns /<ns>/traversability_grid when this is false. "
                        "Set true for A/B comparison or fallback."),
    ]

    # Reuse the full fastlio launch — it handles MuJoCo, Point-LIO/Fast-LIO,
    # CFPA2, Nav2, RViz, and all the supporting plumbing.
    base_launch_path = os.path.join(
        get_package_share_directory("go2_gazebo_sim"),
        "launch",
        "nav_test_mujoco_fastlio.launch.py")
    base_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch_path),
        launch_arguments={
            "mujoco_model_path": LaunchConfiguration("mujoco_model_path"),
            "spawn_x":   LaunchConfiguration("spawn_x"),
            "spawn_y":   LaunchConfiguration("spawn_y"),
            "spawn_yaw": LaunchConfiguration("spawn_yaw"),
            "robot_namespace": LaunchConfiguration("robot_namespace"),
            "rviz": LaunchConfiguration("rviz"),
            "gui":  LaunchConfiguration("gui"),
            "nav_costmap_mode": LaunchConfiguration("nav_costmap_mode"),
        }.items(),
    )

    # nvblox_frontend mapper node — delayed 5 s so any residual preflight
    # activity from a previous session finishes before the mapper starts.
    # respawn=True guards against SIGTERM from a concurrent preflight.
    mapper = Node(
        package="nvblox_frontend",
        executable="mapper_node",
        name="nvblox_frontend_mapper",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        condition=IfCondition(LaunchConfiguration("enable_nvblox_mapper")),
        parameters=[{
            # cloud_topic and odom_topic intentionally not set here.
            # The node's namespace (robot) prefixes relative defaults
            # cloud_registered_body → /robot/cloud_registered_body and
            # odom/nav → /robot/odom/nav, matching Fast-LIO + slam_odom_relay.
            "use_sim_time":      LaunchConfiguration("use_sim_time"),
            "world_frame":       "map",
            "voxel_size_m":      LaunchConfiguration("nvblox_voxel_size_m"),
            "publish_period_s":  0.5,
            # 40m × 40m world-fixed grid: covers demo_ramp (24×16m) plus
            # margin, so historical observations persist as the robot moves.
            "trav_xy_extent_m":  40.0,
            "voxel_xy_extent_m": 20.0,
            "voxel_z_extent_m":  3.0,
            "voxel_z_origin_m":  -0.5,
            "slope_max_deg":     30.0,
            "step_max_m":        0.20,
            "robot_clearance_m": 0.50,
            "enable_legacy_2d_proj": LaunchConfiguration("enable_legacy_2d_proj"),
        }],
    )

    # frontier_3d_test_node — subscribes voxels_3d + traversability_grid + goal_pose;
    # publishes /robot/frontier_3d_markers (top-5 spheres, red = current goal).
    frontier_viz = Node(
        package="cfpa2_collaborative_autonomy",
        executable="frontier_3d_test_node",
        name="frontier_3d_test_node",
        namespace=LaunchConfiguration("robot_namespace"),
        parameters=[{
            "use_sim_time":    LaunchConfiguration("use_sim_time"),
            "robot_namespace": LaunchConfiguration("robot_namespace"),
            "top_n_clusters":  5,
            "publish_period_s": 1.0,
        }],
        output="screen",
        respawn=True,
        respawn_delay=3.0,
    )

    # ---- Traversability pipeline (nav_costmap_mode:=3d only) ---------------
    # elevation_mapping_cupy → filter_chain_runner → grid_map_to_occupancy_grid
    # All three are delayed 6 s (1 s after the nvblox mapper) so SLAM + MuJoCo
    # are ready and no concurrent preflight kills the mapper before it starts.
    is_3d = IfCondition(
        PythonExpression(["'", LaunchConfiguration("nav_costmap_mode"), "' == '3d'"])
    )

    # 1. elevation_mapping_cupy: Kalman-fused height map on GPU.
    #    Publishes /<node_name>/elevation_map_raw = /elevation_mapping/elevation_map_raw
    #    → remapped to /<ns>/elevation_map_raw so the filter chain picks it up.
    #    TF is read from /<ns>/tf + /<ns>/tf_static (namespaced per CLAUDE.md rule 4).
    elevation_mapping = Node(
        package="elevation_mapping_cupy",
        executable="elevation_mapping_node.py",
        name="elevation_mapping",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[emap_core_params, emap_setup_params,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        remappings=[
            # elevation_mapping_cupy hardcodes topic as f"/{self.get_name()}/{pub_key}".
            # With name="elevation_mapping" that is /elevation_mapping/elevation_map_raw.
            ("/elevation_mapping/elevation_map_raw",
             ["/", LaunchConfiguration("robot_namespace"), "/elevation_map_raw"]),
            # Namespace the TF streams (CLAUDE.md golden rule #4 + #10).
            ("/tf",        ["/", LaunchConfiguration("robot_namespace"), "/tf"]),
            ("/tf_static", ["/", LaunchConfiguration("robot_namespace"), "/tf_static"]),
        ],
        condition=is_3d,
    )

    # 2. filter_chain_runner: 10-stage grid_map_filters chain.
    #    elevation_map_raw (3 layers) → elevation_map_filtered (12 layers).
    filter_runner = Node(
        package="trav_cost_filters",
        executable="filter_chain_runner",
        name="filter_chain_runner",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[filter_chain_params,
                    {"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=is_3d,
    )

    # 3. grid_map_to_occupancy_grid: traversability layer → OccupancyGrid.
    #    Output /<ns>/traversability_grid consumed by Nav2 StaticLayer costmap.
    occ_adapter = Node(
        package="trav_cost_filters",
        executable="grid_map_to_occupancy_grid",
        name="grid_map_to_occupancy_grid",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            "use_sim_time":    LaunchConfiguration("use_sim_time"),
            "input_topic":     "elevation_map_filtered",
            "output_topic":    "traversability_grid",
            "free_threshold":  0.30,
            "lethal_threshold": 0.15,
            "seed_robot_footprint": True,
            "robot_frame": "base_link",
            "robot_seed_radius_m": 0.65,
            "seed_max_clear_cost": 50,
            "ramp_override_enabled": True,
            "slope_layer": "slope",
            "step_residual_layer": "step_residual",
            "ramp_min_slope_rad": 0.13962634015954636,
            "ramp_max_slope_rad": 0.5235987755982988,
            "ramp_max_step_residual_m": 0.06,
        }],
        remappings=[
            ("/tf",        ["/", LaunchConfiguration("robot_namespace"), "/tf"]),
            ("/tf_static", ["/", LaunchConfiguration("robot_namespace"), "/tf_static"]),
        ],
        condition=is_3d,
    )

    # 4. ramp_ascent_goal_node: ETH-style ramp equation → high-priority
    #    CFPA2 candidate. It does not publish directly to Nav2; CFPA2 still
    #    applies reachability, blacklist, and clearance checks before sending
    #    /<ns>/way_point_coord.
    ramp_goal = Node(
        package="trav_cost_filters",
        executable="ramp_ascent_goal_node",
        name="ramp_ascent_goal",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "input_topic": "elevation_map_filtered",
            "output_topic": "ramp_ascent_goal",
            "robot_frame": "base_link",
            "map_frame": "map",
            "pointcloud_topic": "registered_scan_reliable",
            "use_pointcloud_ramp_detection": True,
            "pointcloud_stride": 2,
            "verified_hold_sec": 1.5,
            "min_traversability": 0.30,
            "min_slope_deg": 8.0,
            "max_slope_deg": 30.0,
            "max_step_residual_m": 0.06,
            "min_candidate_cells": 8,
            "min_elevation_span_m": 0.25,
            "min_goal_distance_m": 0.45,
            "max_goal_distance_m": 1.60,
            "goal_lookahead_m": 1.20,
            "goal_center_y": 0.0,
            "monotonic_ascent_enabled": True,
            "monotonic_min_ahead_m": 1.20,
            "monotonic_hold_sec": 30.0,
            "monotonic_terminal_hold_enabled": True,
            "ascent_terminal_x": 10.80,
            "platform_min_elevation_gain_m": 0.45,
            "preferred_uphill_yaw_deg": 0.0,
            "preferred_uphill_tolerance_deg": 35.0,
            "min_x": 5.5,
            "max_x": 11.2,
            "min_y": -0.7,
            "max_y": 0.7,
            "approach_enabled": True,
            "approach_x": 5.6,
            "approach_y": 0.0,
            "approach_step_m": 1.3,
            "approach_stop_radius_m": 0.30,
        }],
        remappings=[
            ("/tf",        ["/", LaunchConfiguration("robot_namespace"), "/tf"]),
            ("/tf_static", ["/", LaunchConfiguration("robot_namespace"), "/tf_static"]),
        ],
        condition=is_3d,
    )

    ramp_cmd_assist = Node(
        package="trav_cost_filters",
        executable="ramp_cmd_vel_assist_node",
        name="ramp_cmd_vel_assist",
        namespace=LaunchConfiguration("robot_namespace"),
        output="screen",
        respawn=True,
        respawn_delay=3.0,
        parameters=[{
            "use_sim_time": LaunchConfiguration("use_sim_time"),
            "goal_topic": "ramp_ascent_goal",
            "odom_topic": "odom/nav",
            "cmd_vel_topic": "cmd_vel",
            "goal_stale_sec": 8.0,
            "min_x": 5.3,
            "max_x": 11.2,
            "max_abs_y": 0.9,
            "min_forward_error_m": 0.08,
            "max_goal_distance_m": 2.0,
            "min_vx_mps": 0.22,
            "max_vx_mps": 0.30,
            "forward_gain": 0.45,
            "yaw_gain": 1.2,
            "max_yaw_rate_rps": 0.45,
        }],
        condition=is_3d,
    )

    # Static identity: base_link → body
    # Fast-LIO hardcodes cloud_registered_body.header.frame_id = "body"
    # (laserMapping.cpp:564). elevation_mapping_cupy looks up map→body to
    # transform each cloud into the map frame. Our TF tree only has
    # map→odom→base_link; "body" is absent. fast_lio_tf_adapter already
    # treats body ≡ base_link (it republishes the odom→body pose as
    # odom→base_link) but never publishes the explicit link.
    # Adding base_link→body identity closes map→odom→base_link→body so
    # safe_lookup_transform(map, body) succeeds and terrain data is integrated.
    body_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="base_link_to_body_tf",
        namespace=LaunchConfiguration("robot_namespace"),
        arguments=[
            "--frame-id", "base_link",
            "--child-frame-id", "body",
            "--x", "0", "--y", "0", "--z", "0",
            "--qx", "0", "--qy", "0", "--qz", "0", "--qw", "1",
        ],
        remappings=[("/tf_static", ["/", LaunchConfiguration("robot_namespace"), "/tf_static"])],
        parameters=[{"use_sim_time": LaunchConfiguration("use_sim_time")}],
        condition=is_3d,
    )

    # Delay mapper + frontier viz by 5 s. The preflight kill script targets
    # "mapper_node" by name; if a second launch attempt runs ≤5 s into the
    # first, the mapper would be killed before MuJoCo even starts. A 5 s
    # delay means the mapper starts after MuJoCo+SLAM are up and any
    # concurrent preflight has already finished. respawn=True above gives a
    # second layer of protection if it's still killed.
    deferred = TimerAction(period=5.0, actions=[mapper, frontier_viz])
    # Trav pipeline nodes start 1 s after the nvblox mapper.
    deferred_trav = TimerAction(
        period=6.0,
        actions=[elevation_mapping, filter_runner, occ_adapter, ramp_goal, ramp_cmd_assist],
    )

    return LaunchDescription([*args, base_launch, body_tf, deferred, deferred_trav])
