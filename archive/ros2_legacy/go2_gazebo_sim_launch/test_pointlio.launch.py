import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, LogInfo
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


"""Compatibility wrapper.

Deprecated: use `dual_go2_modular.launch.py profile:=pointlio_debug`.
"""


def generate_launch_description():
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    canonical = os.path.join(go2_gazebo_pkg, "launch", "dual_go2_modular.launch.py")

    args = {
        "profile": "pointlio_debug",
        "gui": LaunchConfiguration("gui"),
        "pointlio_autonomous": LaunchConfiguration("autonomous"),
        "pointlio_spawn_x": LaunchConfiguration("spawn_x"),
        "pointlio_spawn_y": LaunchConfiguration("spawn_y"),
        "pointlio_spawn_z": LaunchConfiguration("spawn_z"),
        "pointlio_spawn_heading": LaunchConfiguration("spawn_heading"),
    }

    return LaunchDescription(
        [
            DeclareLaunchArgument("gui", default_value="true", description="Run Gazebo GUI"),
            DeclareLaunchArgument("autonomous", default_value="false", description="Enable autonomous frontier exploration"),
            DeclareLaunchArgument("spawn_x", default_value="2.5", description="Spawn X coordinate"),
            DeclareLaunchArgument("spawn_y", default_value="0.0", description="Spawn Y coordinate"),
            DeclareLaunchArgument("spawn_z", default_value="0.32", description="Spawn Z coordinate"),
            DeclareLaunchArgument("spawn_heading", default_value="0.0", description="Spawn heading (yaw)"),
            LogInfo(msg="[DEPRECATED] test_pointlio.launch.py -> dual_go2_modular.launch.py profile:=pointlio_debug"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(canonical),
                launch_arguments={k: str(v) if isinstance(v, str) else v for k, v in args.items()}.items(),
            ),
        ]
    )
