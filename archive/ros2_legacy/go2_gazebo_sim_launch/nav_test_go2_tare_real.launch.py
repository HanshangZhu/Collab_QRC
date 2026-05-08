#!/usr/bin/env python3
"""Go2 (non-W) nav test with the *real* CMU TARE Planner replacing CFPA2.

CMU's TARE (Chao Cao, TRO 2023 — sub-module already in src/vendor/tare_planner
from the humble-jazzy branch of github.com/caochao39/tare_planner) is a full
hierarchical exploration planner: local dense sensor-coverage + global sparse
keypose graph with TSP, viewpoint manager, rolling occupancy grid. It replaces
CFPA2 as the goal source — pathFollower still tracks via FAR.

Pipeline:
  state_estimation_at_scan  ┐
  registered_scan           ├─► tare_planner_node ─► /way_point ─► FAR goal
  terrain_map(_ext)         ┘                                       │
                                                                    ▼
                                                            localPlanner
                                                                    │
                                                                    ▼
                                                            pathFollower ─► cmd_vel

Why this vs the previous `nav_test_go2_tare.launch.py`:
the earlier launch used the 117-line `go2_tare_planner_ros2` stub, which only
forwards CFPA2's frontier goal through a waypoint_mux (no actual planning).
Here we run the full TARE algorithm, and CFPA2 is *not* started.

Usage::

    ./scripts/launch/nav_test_go2_tare_real.sh gui:=true rviz:=true
    ./scripts/launch/nav_test_go2_tare_real.sh gui:=false session_duration_sec:=500 \\
        session_output_path:=/tmp/go2_tare_bench.json
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


_ws_root = os.path.abspath(os.path.join(
    os.path.dirname(os.path.realpath(__file__)), "..", "..", "..", ".."
))


def _get(ctx, name: str) -> str:
    return LaunchConfiguration(name).perform(ctx)


def _launch_setup(ctx):
    robot_ns = _get(ctx, "robot_namespace").strip().strip("/") or "robot"

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    base_launch    = os.path.join(go2_gazebo_pkg, "launch", "nav_test_mujoco_fastlio.launch.py")
    scenario       = _get(ctx, "tare_scenario").strip() or "indoor"
    # Use our namespace-wildcard-keyed copy, not the vendor file. See the
    # header of config/tare/indoor.yaml for the full rationale.
    tare_config    = os.path.join(go2_gazebo_pkg, "config", "tare", f"{scenario}.yaml")

    tf_remaps = [("/tf", f"/{robot_ns}/tf"), ("/tf_static", f"/{robot_ns}/tf_static")]

    # --- Base platform: MuJoCo + CHAMP + Fast-LIO + octomap + terrain_analysis
    #     + localPlanner + pathFollower. CFPA2 is disabled (explore:=false) and
    #     FAR is wired OUT of the exploration pipeline:
    #       * FAR's goal input (/way_point_coord) is remapped to an empty
    #         string — no publisher feeds it, so FAR sits idle.
    #       * FAR's output (/way_point) is redirected to a dead topic so it
    #         can't collide with TARE's direct publication to the same name.
    #     Pipeline: TARE → /way_point → localPlanner → pathFollower.
    #
    #     WHY FAR IS UNWIRED: FAR's V-graph is built exclusively over
    #     *observed traversable space*. TARE's frontier goals, by definition,
    #     sit at the boundary of observed space — so FAR repeatedly had no
    #     V-graph vertex near the goal and silently stopped publishing
    #     /way_point (confirmed live today: FAR at 43% CPU, 0 messages in 6
    #     s, cmd_vel decayed to zero, robot wedged in NW corner of demo3).
    #     This is a contract mismatch, not a tuning bug: FAR is CMU's
    #     destination-directed nav planner ("drive to known point X"), while
    #     TARE is their exploration planner that already handles global
    #     routing itself via keypose_graph + TSP. Running both in series
    #     stacks two global planners with conflicting scopes. CMU's own
    #     reference pairs TARE → localPlanner directly.
    base = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(base_launch),
        launch_arguments={
            "robot_namespace": robot_ns,
            "gui":              _get(ctx, "gui"),
            "rviz":             _get(ctx, "rviz"),
            "explore":          "false",                # TARE replaces CFPA2
            "nav_backend":      "far",
            "mujoco_model_path": _get(ctx, "mujoco_model_path"),
            "scene_area_m2":    _get(ctx, "scene_area_m2"),
            "has_wheels":       _get(ctx, "has_wheels"),
            "two_way_drive":    _get(ctx, "two_way_drive"),
            "session_duration_sec": _get(ctx, "session_duration_sec"),
            "session_output_path":  _get(ctx, "session_output_path"),
            "enable_wall_checker":  _get(ctx, "enable_wall_checker"),
            "far_max_speed":    _get(ctx, "far_max_speed"),
            "spawn_x":  _get(ctx, "spawn_x"),
            "spawn_y":  _get(ctx, "spawn_y"),
            "spawn_yaw": _get(ctx, "spawn_yaw"),
            # FAR unwired — input dead, output routed to a sink topic.
            "far_goal_topic": "",
            "far_way_point_out": f"/{robot_ns}/_far_way_point_unused",
        }.items(),
    )

    # TARE runs in the robot namespace and overrides its topic params via
    # the command line — these are the ones its sensor_coverage_planner_ground
    # node reads. Other internal topics (viewpoint boundary, momentum,
    # runtime breakdown, etc.) are left on default absolute names.
    tare_node = Node(
        package="tare_planner",
        executable="tare_planner_node",
        name="tare_planner_node",
        namespace=robot_ns,
        # tare_config is our /**-keyed copy, so every algo param actually
        # loads under the /robot namespace. Topic names below still need
        # explicit overrides because they must carry the robot prefix.
        parameters=[
            tare_config,
            {"use_sim_time": True},
            # Input topics — our /{ns}-prefixed stack.
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
            # TARE publishes DIRECTLY to localPlanner's input. FAR is
            # unwired above (see rationale in the base-launch block). The
            # waypoint watchdog below still guards against TARE picking a
            # viewpoint inside an obstacle cluster.
            {"pub_waypoint_topic_":          f"/{robot_ns}/way_point"},
            {"pub_runtime_topic_":           f"/{robot_ns}/tare_runtime"},
        ],
        remappings=tf_remaps,
        output="screen",
    )

    # Sensor-derived watchdog. Two independent in-wall checks:
    #   * terrain_map: cluster of obstacle-height points within 0.4 m of
    #     the waypoint (catches "waypoint ON the wall surface").
    #   * /map (octomap 2D projection): occupied cells within 0.25 m of
    #     the waypoint (catches "waypoint INSIDE the wall volume" — the
    #     LiDAR can't see through walls so terrain_map is empty there,
    #     but the projected 2D grid inflates the wall's full footprint).
    # Either triggers Empty() on /reset_waypoint → TARE skips that goal.
    watchdog_script = os.path.join(_ws_root, "scripts/runtime/tare_waypoint_watchdog.py")
    watchdog_proc = ExecuteProcess(
        cmd=[
            "python3", "-u", watchdog_script,
            "--ros-args",
            "-r", f"__ns:=/{robot_ns}",
            "-p", "use_sim_time:=true",
            "-p", f"terrain_map_topic:=/{robot_ns}/terrain_map",
            "-p", f"waypoint_topic:=/{robot_ns}/way_point",
            "-p", f"reset_topic:=/{robot_ns}/reset_waypoint",
            "-p", f"occgrid_topic:=/{robot_ns}/map",
            "-p", f"odom_topic:=/{robot_ns}/odom/nav",
            "-p", f"marker_topic:=/{robot_ns}/way_point_marker",
            "-p", f"robot_marker_topic:=/{robot_ns}/robot_pose_marker",
            "-p", "marker_frame:=map",
            "-p", f"nogo_topic:=/{robot_ns}/nogo_boundary",
            "-p", "nogo_square_half_m:=0.6",
            "-p", "nogo_max_regions:=40",
            "-p", "nogo_min_dist_from_robot_m:=0.8",
            "-p", "stall_already_there_m:=0.4",
            "-p", "obstacle_radius:=0.4",
            "-p", "obstacle_height_thre:=0.2",
            "-p", "min_obstacle_points:=2",
            "-p", "occgrid_occupied_thre:=50",
            "-p", "occgrid_inflate_m:=0.25",
            "-p", "reset_cooldown_sec:=2.0",
            "-p", "stall_timeout_sec:=10.0",
            "-p", "stall_improve_epsilon_m:=0.05",
            "-p", "waypoint_change_epsilon_m:=0.20",
        ],
        name="tare_waypoint_watchdog",
        output="screen",
    )

    # TARE self-starts when kAutoStart=true (set in config/tare/indoor.yaml).
    # Delay until perception is flowing — terrain_analysis needs ~5s.
    return [base, TimerAction(period=10.0, actions=[tare_node, watchdog_proc])]


def generate_launch_description() -> LaunchDescription:
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    default_scene = os.path.join(go2_gazebo_pkg, "mujoco", "demo3_go2_real.xml")
    return LaunchDescription([
        DeclareLaunchArgument("robot_namespace", default_value="robot"),
        DeclareLaunchArgument("gui",  default_value="true"),
        DeclareLaunchArgument("rviz", default_value="true"),
        DeclareLaunchArgument("mujoco_model_path", default_value=default_scene),
        DeclareLaunchArgument("scene_area_m2", default_value="384.0"),
        DeclareLaunchArgument("has_wheels",    default_value="false",
                              description="Pure Go2 by default."),
        DeclareLaunchArgument("two_way_drive", default_value="false",
                              description="CHAMP has no validated reverse gait."),
        DeclareLaunchArgument("spawn_x",   default_value="4.0"),
        DeclareLaunchArgument("spawn_y",   default_value="2.0"),
        DeclareLaunchArgument("spawn_yaw", default_value="0.0"),
        DeclareLaunchArgument("session_duration_sec", default_value="0"),
        DeclareLaunchArgument("session_output_path",  default_value=""),
        DeclareLaunchArgument("enable_wall_checker",  default_value="false"),
        DeclareLaunchArgument("far_max_speed",        default_value=""),
        DeclareLaunchArgument("tare_scenario",        default_value="indoor",
                              description="TARE config profile — indoor / garage / "
                              "campus / forest / tunnel / matterport. "
                              "Controls rolling-grid size, line-of-sight depth, "
                              "frontier thresholds. demo3 fits 'indoor'."),
        OpaqueFunction(function=_launch_setup),
    ])
