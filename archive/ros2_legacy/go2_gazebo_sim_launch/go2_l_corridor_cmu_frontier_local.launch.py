import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, SetRemap

from go2_nav_algorithms.pipeline_components import build_pointcloud_to_laserscan_node, build_simple_scan_mapper_cpp_node


"""Gazebo single-Go2 stack with CMU local planning + frontier goal generation.

Pipeline:
- Gazebo + stand-up
- SLAM (FAST-LIO by default) -> /state_estimation
- Frontier planner (simple_scan_mapper + geometric_frontier) -> /way_point
- CMU local stack (terrain_analysis + localPlanner + pathFollower)
- pathFollower publishes sport API commands on /api/sport/request
"""


def generate_launch_description():
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2_nav_pkg = get_package_share_directory("go2_nav_algorithms")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    autonomous = LaunchConfiguration("autonomous")

    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_heading = LaunchConfiguration("spawn_heading")

    slam_package = LaunchConfiguration("slam_package")
    slam_executable = LaunchConfiguration("slam_executable")
    slam_odom_topic = LaunchConfiguration("slam_odom_topic")

    frontier_profile = os.path.join(go2_nav_pkg, "config", "nav", "geometric_frontier_single.yaml")
    slam_config = os.path.join(get_package_share_directory("go2w_config"), "config", "slam", "pointlio_gazebo.yaml")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_gazebo_pkg, "launch", "go2_l_corridor.launch.py")
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "gui": gui,
            "rviz": rviz,
            "world_init_x": spawn_x,
            "world_init_y": spawn_y,
            "world_init_z": spawn_z,
            "world_init_heading": spawn_heading,
        }.items(),
    )

    stand_up_node = Node(
        package="go2w_spawn",
        executable="stand_up_slowly.py",
        name="stand_up_slowly",
        output="screen",
    )

    qos_bridge_node = Node(
        package="go2w_perception",
        executable="qos_bridge.py",
        name="qos_bridge",
        parameters=[
            {
                "use_sim_time": use_sim_time,
                "input_topic": "/registered_scan",
                "output_topic": "/registered_scan_reliable",
            }
        ],
        output="screen",
    )

    pointcloud_adapter_node = Node(
        package="go2w_perception",
        executable="pointcloud_adapter.py",
        name="pointcloud_adapter",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": "/registered_scan_reliable"},
            {"output_topic": "/velodyne_points"},
            {"num_rings": 16},
        ],
        output="screen",
    )

    slam_node = Node(
        package=slam_package,
        executable=slam_executable,
        name="slam_node",
        parameters=[slam_config, {"use_sim_time": use_sim_time}],
        remappings=[
            ("/velodyne_points", "/velodyne_points"),
            ("/imu/data", "/imu/data"),
            ("/Odometry", "/Odometry"),
        ],
        output="screen",
        condition=IfCondition(autonomous),
    )

    slam_relay_node = Node(
        package="go2w_perception",
        executable="slam_odom_relay.py",
        name="slam_odom_relay",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"input_topic": slam_odom_topic},
            {"output_topic": "/state_estimation"},
            {"output_frame_id": "map"},
            {"output_child_frame_id": "base_link"},
            {"bootstrap_from_gt": True},
            {"gt_topic": "/odom/ground_truth"},
            {"require_gt_for_alignment": True},
        ],
        output="screen",
        condition=IfCondition(autonomous),
    )

    pointcloud_to_laserscan = build_pointcloud_to_laserscan_node(
        ns=None,
        use_sim_time=use_sim_time,
        extra_params={
            "target_frame": "base_link",
            "transform_tolerance": 0.3,
            "min_height": 0.05,
            "max_height": 0.60,
            "range_min": 0.2,
            "range_max": 12.0,
        },
        remappings=[("cloud_in", "/registered_scan_reliable"), ("scan", "/scan")],
        condition=IfCondition(autonomous),
    )

    simple_scan_mapper_node = build_simple_scan_mapper_cpp_node(
        ns=None,
        use_sim_time=use_sim_time,
        profile="geometric_frontier_single.yaml",
        extra_params={
            "scan_topic": "/scan",
            "odom_topic": "/state_estimation",
            "map_topic": "/map",
            "map_frame": "map",
            "startup_delay": 0.0,
            "max_scan_odom_dt": 0.1,
        },
        name="simple_scan_mapper_cpp",
        condition=IfCondition(autonomous),
    )

    geometric_frontier_node = Node(
        package="go2_nav_algorithms",
        executable="simple_frontier_explorer.py",
        name="geometric_frontier",
        parameters=[
            frontier_profile,
            {
                "use_sim_time": use_sim_time,
                "odom_topic": "/state_estimation",
                "map_topic": "/map",
                "prefer_costmap": False,
                "costmap_topic": "",
                "frontier_goal_topic": "/way_point",
                "frontier_marker_topic": "/frontier_goal_marker",
                "frontier_regions_topic": "/frontier_markers",
                "frontier_replan_topic": "/frontier_replan",
                "startup_delay": 0.0,
                "max_map_odom_dt": 0.5,
            },
        ],
        output="screen",
        condition=IfCondition(autonomous),
    )

    autonomy_enabler_node = Node(
        package="go2w_safety",
        executable="autonomy_enabler.py",
        name="autonomy_enabler",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"startup_delay": 8.0},
            {"rate": 10.0},
            {"wait_for_waypoint": True},
        ],
        output="screen",
        condition=IfCondition(autonomous),
    )

    # CMU local planner stack. Remap /registered_scan to reliable QoS topic.
    terrain_stack = GroupAction(
        [
            SetRemap(src="/registered_scan", dst="/registered_scan_reliable"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("terrain_analysis"),
                        "launch",
                        "terrain_analysis.launch",
                    )
                )
            ),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("terrain_analysis_ext"),
                        "launch",
                        "terrain_analysis_ext.launch",
                    )
                ),
                launch_arguments={"checkTerrainConn": "true"}.items(),
            ),
        ]
    )

    # Remap pathFollower's /cmd_vel (TwistStamped) so Gazebo can consume through twist_bridge.
    local_planner_stack = GroupAction(
        [
            SetRemap(src="/registered_scan", dst="/registered_scan_reliable"),
            SetRemap(src="/cmd_vel", dst="/cmd_vel_stamped"),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory("local_planner"),
                        "launch",
                        "local_planner.launch",
                    )
                ),
                launch_arguments={
                    "autonomyMode": "true",
                    "is_real_robot": "true",
                    "maxSpeed": "0.6",
                }.items(),
            ),
        ]
    )

    twist_bridge_node = Node(
        package="go2w_perception",
        executable="twist_bridge.py",
        name="twist_bridge",
        parameters=[{"use_sim_time": use_sim_time}],
        output="screen",
        condition=IfCondition(autonomous),
    )

    delayed_stand_up = TimerAction(period=8.0, actions=[stand_up_node])
    delayed_perception = TimerAction(period=10.0, actions=[qos_bridge_node, pointcloud_adapter_node])
    delayed_slam = TimerAction(period=14.0, actions=[slam_node])

    delayed_autonomy = TimerAction(
        period=22.0,
        actions=[
            slam_relay_node,
            pointcloud_to_laserscan,
            simple_scan_mapper_node,
            geometric_frontier_node,
            autonomy_enabler_node,
            terrain_stack,
            local_planner_stack,
            twist_bridge_node,
        ],
        condition=IfCondition(autonomous),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
            DeclareLaunchArgument("gui", default_value="true", description="Run Gazebo GUI"),
            DeclareLaunchArgument("rviz", default_value="true", description="Run RViz"),
            DeclareLaunchArgument("autonomous", default_value="true", description="Enable CMU-style autonomy stack"),
            DeclareLaunchArgument("spawn_x", default_value="2.5", description="Spawn X coordinate"),
            DeclareLaunchArgument("spawn_y", default_value="0.0", description="Spawn Y coordinate"),
            DeclareLaunchArgument("spawn_z", default_value="0.32", description="Spawn Z coordinate"),
            DeclareLaunchArgument("spawn_heading", default_value="0.0", description="Spawn Heading (yaw)"),
            DeclareLaunchArgument("slam_package", default_value="fast_lio", description="SLAM package (fast_lio or point_lio_unilidar)"),
            DeclareLaunchArgument("slam_executable", default_value="fastlio_mapping", description="SLAM executable"),
            DeclareLaunchArgument("slam_odom_topic", default_value="/Odometry", description="SLAM odometry topic to relay into /state_estimation"),
            gazebo_launch,
            delayed_stand_up,
            delayed_perception,
            delayed_slam,
            delayed_autonomy,
        ]
    )
