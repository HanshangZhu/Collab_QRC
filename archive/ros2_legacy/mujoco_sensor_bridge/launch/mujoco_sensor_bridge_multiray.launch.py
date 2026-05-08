"""Launch all MuJoCo sensor bridge nodes (mj_multiRay LiDAR backend)."""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_dir = get_package_share_directory('mujoco_sensor_bridge')
    default_config = os.path.join(pkg_dir, 'config', 'mujoco_sensor_bridge.yaml')

    ns_arg = DeclareLaunchArgument('ns', default_value='', description='Robot namespace')
    mjcf_arg = DeclareLaunchArgument('mjcf_path', default_value='',
                                     description='Path to MuJoCo XML model')
    config_arg = DeclareLaunchArgument('config', default_value=default_config,
                                       description='Path to parameter YAML')

    ns = LaunchConfiguration('ns')
    mjcf_path = LaunchConfiguration('mjcf_path')
    config = LaunchConfiguration('config')

    lidar_node = Node(
        package='mujoco_sensor_bridge',
        executable='mujoco_lidar_node_multiray',
        name='mujoco_lidar_node',
        namespace=ns,
        parameters=[config, {'mjcf_path': mjcf_path}],
        output='screen',
    )

    contact_node = Node(
        package='mujoco_sensor_bridge',
        executable='mujoco_contact_node',
        name='mujoco_contact_node',
        namespace=ns,
        parameters=[config, {'mjcf_path': mjcf_path}],
        output='screen',
    )

    odom_node = Node(
        package='mujoco_sensor_bridge',
        executable='mujoco_odom_bridge',
        name='mujoco_odom_bridge',
        namespace=ns,
        parameters=[config],
        output='screen',
    )

    return LaunchDescription([
        ns_arg,
        mjcf_arg,
        config_arg,
        lidar_node,
        contact_node,
        odom_node,
    ])
