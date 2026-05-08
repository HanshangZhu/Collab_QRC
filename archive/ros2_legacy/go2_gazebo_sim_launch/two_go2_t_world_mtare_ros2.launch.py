import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


"""Compatibility wrapper.

Deprecated: use `dual_go2_modular.launch.py profile:=mtare_ros2 planner_backend:=<backend>`.
"""


def generate_launch_description():
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    canonical = os.path.join(go2_gazebo_pkg, "launch", "dual_go2_modular.launch.py")

    args = {
        "use_sim_time": LaunchConfiguration("use_sim_time"),
        "gui": LaunchConfiguration("gui"),
        "rviz": LaunchConfiguration("rviz"),
        "cleanup_stale": LaunchConfiguration("cleanup_stale"),
        "use_fast_lio": LaunchConfiguration("use_fast_lio"),
        "enable_frontier_aux": LaunchConfiguration("enable_frontier_aux"),
        "use_shared_map": LaunchConfiguration("use_shared_map"),
        "shared_map_topic": LaunchConfiguration("shared_map_topic"),
        "shared_map_wait_sec": LaunchConfiguration("shared_map_wait_sec"),
        "mtare_algorithm_mode": LaunchConfiguration("mtare_algorithm_mode"),
        "mtare_goal_publish_rate": LaunchConfiguration("mtare_goal_publish_rate"),
        "mtare_overlap_weight": LaunchConfiguration("mtare_overlap_weight"),
        "mtare_communication_timeout_sec": LaunchConfiguration("mtare_communication_timeout_sec"),
        "mtare_prediction_horizon_sec": LaunchConfiguration("mtare_prediction_horizon_sec"),
        "mtare_pursuit_weight": LaunchConfiguration("mtare_pursuit_weight"),
        "mtare_pursuit_switch_margin": LaunchConfiguration("mtare_pursuit_switch_margin"),
        "mtare_exploration_gain_radius_cells": LaunchConfiguration("mtare_exploration_gain_radius_cells"),
        "mtare_meeting_min_distance": LaunchConfiguration("mtare_meeting_min_distance"),
        "mtare_teammate_stale_ttl_sec": LaunchConfiguration("mtare_teammate_stale_ttl_sec"),
        "robot_a_spawn_x": LaunchConfiguration("robot_a_spawn_x"),
        "robot_a_spawn_y": LaunchConfiguration("robot_a_spawn_y"),
        "robot_a_spawn_yaw": LaunchConfiguration("robot_a_spawn_yaw"),
        "robot_b_spawn_x": LaunchConfiguration("robot_b_spawn_x"),
        "robot_b_spawn_y": LaunchConfiguration("robot_b_spawn_y"),
        "robot_b_spawn_yaw": LaunchConfiguration("robot_b_spawn_yaw"),
        "world": LaunchConfiguration("world"),
        "require_shared_graph": LaunchConfiguration("require_shared_graph"),
        "exact_far_world_frame": LaunchConfiguration("exact_far_world_frame"),
        "profile": "mtare_ros2",
        "planner_backend": LaunchConfiguration("planner_backend"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("cleanup_stale", default_value="true"),
            DeclareLaunchArgument("use_fast_lio", default_value="false"),
            DeclareLaunchArgument("enable_frontier_aux", default_value="false"),
            DeclareLaunchArgument("use_shared_map", default_value="false"),
            DeclareLaunchArgument("shared_map_topic", default_value="/disco_slam/global_map"),
            DeclareLaunchArgument("shared_map_wait_sec", default_value="8.0"),
            DeclareLaunchArgument("mtare_algorithm_mode", default_value="mtare"),
            DeclareLaunchArgument("mtare_goal_publish_rate", default_value="2.0"),
            DeclareLaunchArgument("mtare_overlap_weight", default_value="1.0"),
            DeclareLaunchArgument("mtare_communication_timeout_sec", default_value="6.0"),
            DeclareLaunchArgument("mtare_prediction_horizon_sec", default_value="4.0"),
            DeclareLaunchArgument("mtare_pursuit_weight", default_value="2.0"),
            DeclareLaunchArgument("mtare_pursuit_switch_margin", default_value="0.10"),
            DeclareLaunchArgument("mtare_exploration_gain_radius_cells", default_value="4"),
            DeclareLaunchArgument("mtare_meeting_min_distance", default_value="1.5"),
            DeclareLaunchArgument("mtare_teammate_stale_ttl_sec", default_value="120.0"),
            DeclareLaunchArgument("robot_a_spawn_x", default_value="1.0"),
            DeclareLaunchArgument("robot_a_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_a_spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_x", default_value="18.0"),
            DeclareLaunchArgument("robot_b_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_yaw", default_value="3.14159"),
            DeclareLaunchArgument("world", default_value=os.path.join(go2_gazebo_pkg, "worlds", "3.world")),
            DeclareLaunchArgument("planner_backend", default_value="mtare_ros2"),
            DeclareLaunchArgument("require_shared_graph", default_value="true"),
            DeclareLaunchArgument("exact_far_world_frame", default_value="world"),
            LogInfo(
                msg=(
                    "[DEPRECATED] two_go2_t_world_mtare_ros2.launch.py -> "
                    "dual_go2_modular.launch.py profile:=mtare_ros2 planner_backend:=<backend>"
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(canonical),
                launch_arguments={k: str(v) if isinstance(v, str) else v for k, v in args.items()}.items(),
            ),
        ]
    )
