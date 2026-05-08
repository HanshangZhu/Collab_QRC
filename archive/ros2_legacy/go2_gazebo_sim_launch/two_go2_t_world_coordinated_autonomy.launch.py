import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


"""Compatibility wrapper.

Deprecated: use `dual_go2_modular.launch.py profile:=coordinated`.
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
        "use_shared_map": LaunchConfiguration("use_shared_map"),
        "shared_map_topic": LaunchConfiguration("shared_map_topic"),
        "robot_a_spawn_x": LaunchConfiguration("robot_a_spawn_x"),
        "robot_a_spawn_y": LaunchConfiguration("robot_a_spawn_y"),
        "robot_a_spawn_yaw": LaunchConfiguration("robot_a_spawn_yaw"),
        "robot_b_spawn_x": LaunchConfiguration("robot_b_spawn_x"),
        "robot_b_spawn_y": LaunchConfiguration("robot_b_spawn_y"),
        "robot_b_spawn_yaw": LaunchConfiguration("robot_b_spawn_yaw"),
        "world": LaunchConfiguration("world"),
        "profile": "coordinated",
        "planner_backend": "coordinated",
        "coordinated_algorithm_mode": "committed",
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true"),
            DeclareLaunchArgument("gui", default_value="true"),
            DeclareLaunchArgument("rviz", default_value="true"),
            DeclareLaunchArgument("cleanup_stale", default_value="true"),
            DeclareLaunchArgument("use_fast_lio", default_value="false"),
            DeclareLaunchArgument("use_shared_map", default_value="false"),
            DeclareLaunchArgument("shared_map_topic", default_value="/disco_slam/global_map"),
            DeclareLaunchArgument("robot_a_spawn_x", default_value="1.0"),
            DeclareLaunchArgument("robot_a_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_a_spawn_yaw", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_x", default_value="18.0"),
            DeclareLaunchArgument("robot_b_spawn_y", default_value="0.0"),
            DeclareLaunchArgument("robot_b_spawn_yaw", default_value="3.14159"),
            DeclareLaunchArgument("world", default_value=os.path.join(go2_gazebo_pkg, "worlds", "3.world")),
            LogInfo(msg="[DEPRECATED] two_go2_t_world_coordinated_autonomy.launch.py -> dual_go2_modular.launch.py profile:=coordinated"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(canonical),
                launch_arguments={k: str(v) if isinstance(v, str) else v for k, v in args.items()}.items(),
            ),
        ]
    )
