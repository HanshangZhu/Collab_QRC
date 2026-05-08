"""Test LiDAR-Inertial SLAM integration in Gazebo.

This launch file:
1. Starts the standard Gazebo sim (go2_l_corridor) — unchanged
2. Stands the robot up
3. Starts QoS bridge for reliable point cloud delivery
4. Launches FAST-LIO / Point-LIO SLAM node
5. Launches odom comparison tool (SLAM vs ground truth → CSV)
6. [OPTIONAL] Launches the autonomy stack (frontier + default_nav)

Modes:
  Manual (default):
    ros2 launch go2_gazebo_sim test_pointlio.launch.py
    # Then drive with: ros2 run teleop_twist_keyboard teleop_twist_keyboard

  Autonomous:
    ros2 launch go2_gazebo_sim test_pointlio.launch.py autonomous:=true
    # Robot explores on its own using simple_scan_mapper + simple_frontier_explorer + default_nav

  The 'autonomous' flag controls whether the frontier planner + default_nav
  are launched. SLAM runs in both modes. The autonomy nodes consume SLAM
  odometry via slam_odom_relay instead of ground truth.

SLAM Package Options:
  Option A (recommended): Ericsii/FAST_LIO_ROS2 (Humble-compatible)
    cd src && git clone --recursive https://github.com/Ericsii/FAST_LIO_ROS2.git fast_lio
    colcon build --packages-select fast_lio --symlink-install
    Set SLAM_PACKAGE="fast_lio", SLAM_EXECUTABLE="fastlio_mapping"

  Option B: Bundled point_lio_unilidar from CMU submodule
    May need C++ fixes for Humble compatibility.
    Set SLAM_PACKAGE="point_lio_unilidar", SLAM_EXECUTABLE="pointlio_mapping"

Monitor:
    ros2 topic echo /Odometry             # SLAM odom (FAST-LIO)
    ros2 topic echo /aft_mapped_to_init   # SLAM odom (Point-LIO)
    ros2 topic echo /odom/ground_truth    # Ground truth for comparison
    tail -f /tmp/odom_comparison_*.csv    # Drift log
"""

import os
import sys

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

from go2_nav_algorithms.pipeline_components import build_pointcloud_to_laserscan_node, build_simple_scan_mapper_cpp_node


# =====================================================================
# CONFIGURATION: Change these to match whichever SLAM package you built
# =====================================================================
SLAM_PACKAGE = "fast_lio"          # or "point_lio_unilidar"
SLAM_EXECUTABLE = "fastlio_mapping"  # or "pointlio_mapping"
# FAST-LIO publishes odom to /Odometry; Point-LIO to /aft_mapped_to_init
SLAM_ODOM_TOPIC = "/Odometry"        # or "/aft_mapped_to_init"

# The topic that autonomy nodes will consume for odometry
SLAM_RELAY_TOPIC = "/slam/odom"


def generate_launch_description():
    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")

    gui = LaunchConfiguration("gui")
    autonomous = LaunchConfiguration("autonomous")
    
    # Spawn arguments (defaults match go2_l_corridor)
    spawn_x = LaunchConfiguration("spawn_x")
    spawn_y = LaunchConfiguration("spawn_y")
    spawn_z = LaunchConfiguration("spawn_z")
    spawn_heading = LaunchConfiguration("spawn_heading")

    # ================================================================
    # 1. Standard Gazebo sim (go2 in l_corridor) — completely unchanged
    # ================================================================
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(go2_gazebo_pkg, "launch", "go2_l_corridor.launch.py")
        ),
        launch_arguments={
            "gui": gui,
            "rviz": "true",
            "world_init_x": spawn_x,
            "world_init_y": spawn_y,
            "world_init_z": spawn_z,
            "world_init_heading": spawn_heading,
        }.items(),
    )

    # ================================================================
    # 2. Stand Up (8s delay for robot to spawn)
    # ================================================================
    stand_up_node = Node(
        package="go2w_spawn",
        executable="stand_up_slowly.py",
        name="stand_up_slowly",
        output="screen",
    )
    delayed_stand_up = TimerAction(period=8.0, actions=[stand_up_node])

    # ================================================================
    # 3. QoS Bridge — converts BestEffort → Reliable for /registered_scan
    # ================================================================
    qos_bridge_node = Node(
        package="go2w_perception",
        executable="qos_bridge.py",
        name="qos_bridge",
        parameters=[{"use_sim_time": True}],
        output="screen",
    )

    # ================================================================
    # 3b. PointCloud Adapter — adds ring/time fields for FAST-LIO
    #     Gazebo gpu_ray only publishes XYZI; FAST-LIO Velodyne handler
    #     needs ring (uint16) and time (float32) fields.
    # ================================================================
    pc_adapter_node = Node(
        package="go2w_perception",
        executable="pointcloud_adapter.py",
        name="pointcloud_adapter",
        parameters=[
            {"use_sim_time": True},
            {"input_topic": "/registered_scan"},
            {"output_topic": "/velodyne_points"},
            {"num_rings": 16},
        ],
        output="screen",
    )

    # ================================================================
    # 4. FAST-LIO / Point-LIO SLAM Node
    # ================================================================
    slam_config = os.path.join(
        go2_gazebo_pkg, "config", "slam", "pointlio_gazebo.yaml"
    )

    slam_node = Node(
        package=SLAM_PACKAGE,
        executable=SLAM_EXECUTABLE,
        name="slam_node",
        parameters=[
            slam_config,
            {"use_sim_time": True},
        ],
        output="screen",
    )

    # ================================================================
    # 5. Odom Comparison (SLAM vs Ground Truth → CSV)
    # ================================================================
    odom_comparison_node = Node(
        package="go2w_observability",
        executable="odom_comparison.py",
        name="odom_comparison",
        parameters=[
            {"use_sim_time": True},
            {"gt_topic": "/odom/ground_truth"},
            {"slam_topic": SLAM_ODOM_TOPIC},
            {"report_rate": 1.0},
        ],
        output="screen",
    )

    # ================================================================
    # 6. AUTONOMY (only when autonomous:=true)
    # ================================================================

    # 6a. SLAM odom relay: /Odometry → /slam/odom (with correct frames)
    # Target frame 'odom' because we link world->odom via static TF below
    slam_relay_node = Node(
        package="go2w_perception",
        executable="slam_odom_relay.py",
        name="slam_odom_relay",
        parameters=[
            {"use_sim_time": True},
            {"input_topic": SLAM_ODOM_TOPIC},
            {"output_topic": SLAM_RELAY_TOPIC},
            {"output_frame_id": "odom"},
            {"output_child_frame_id": "base_link"},
        ],
        output="screen",
        condition=IfCondition(autonomous),
    )

    # 6g. Static TF world->odom to account for spawn offset
    # Robot spawns at (spawn_x, spawn_y), but odom estimation starts at (0,0).
    # We publish world->odom = (spawn_x, spawn_y, spawn_z) so the robot appears correctly in World frame.
    static_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="world_to_odom_static_broadcaster",
        arguments=[spawn_x, spawn_y, spawn_z, "0", "0", "0", "world", "odom"],
        output="screen",
        condition=IfCondition(autonomous),
    )

    # 6b. PointCloud → LaserScan (used by mapper and default_nav)
    pointcloud_to_laserscan = build_pointcloud_to_laserscan_node(
        ns=None,
        use_sim_time=True,
        extra_params={
            "target_frame": "odom",
            "transform_tolerance": 1.0,
            "min_height": 0.15,
            "max_height": 1.0,
            "range_min": 0.2,
            "range_max": 20.0,
        },
        remappings=[("cloud_in", "/registered_scan"), ("scan", "/scan")],
        condition=IfCondition(autonomous),
    )

    # 6c. Twist bridge (TwistStamped → Twist for Gazebo)
    twist_bridge_node = Node(
        package="go2w_perception",
        executable="twist_bridge.py",
        name="twist_bridge",
        parameters=[{"use_sim_time": True}],
        output="screen",
        condition=IfCondition(autonomous),
    )

    # 6d. Mapper + Isaac frontier planner — both use SLAM odom
    frontier_config = os.path.join(
        go2_gazebo_pkg, "config", "nav", "geometric_frontier_single.yaml"
    )
    simple_scan_mapper_node = build_simple_scan_mapper_cpp_node(
        ns=None,
        use_sim_time=True,
        profile="geometric_frontier_single.yaml",
        extra_params={
            "scan_topic": "/scan",
            "odom_topic": SLAM_RELAY_TOPIC,
            "map_topic": "/map",
            "map_frame": "odom",
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
            frontier_config,
            {
                "use_sim_time": True,
                "odom_topic": SLAM_RELAY_TOPIC,
                "map_topic": "/map",
                "prefer_costmap": False,
                "costmap_topic": "",
                "max_map_odom_dt": 0.5,
                "startup_delay": 0.0,
            },
        ],
        output="screen",
        condition=IfCondition(autonomous),
    )

    # 6e. Default Nav controller — uses SLAM odom
    default_nav_config = os.path.join(
        get_package_share_directory("go2w_config"), "config", "nav", "default_nav_single.yaml"
    )
    default_nav_node = Node(
        package="go2w_nav",
        executable="default_nav.py",
        name="default_nav",
        parameters=[
            default_nav_config,
            {"use_sim_time": True},
        ],
        remappings=[
            ("/odom/ground_truth", SLAM_RELAY_TOPIC),
        ],
        output="screen",
        condition=IfCondition(autonomous),
    )

    # 6f. Wall collision checker
    wall_checker_node = Node(
        package="go2w_safety",
        executable="wall_collision_checker.py",
        name="wall_collision_checker",
        parameters=[
            os.path.join(get_package_share_directory("go2w_config"), "config", "safety", "wall_checker.yaml"),
            {"use_sim_time": True},
        ],
        output="screen",
        condition=IfCondition(autonomous),
    )

    # ================================================================
    # Timing
    # ================================================================
    delayed_bridges = TimerAction(
        period=10.0,
        actions=[qos_bridge_node, pc_adapter_node],
    )

    delayed_slam = TimerAction(
        period=14.0,
        actions=[slam_node],
    )

    delayed_comparison = TimerAction(
        period=18.0,
        actions=[odom_comparison_node],
    )

    # Autonomy nodes launch later to give SLAM time to initialise
    delayed_autonomy = TimerAction(
        period=22.0,
        actions=[
            slam_relay_node,
            # static_tf_node,  <-- Moved to main launch list for immediate visibility
            pointcloud_to_laserscan,
            twist_bridge_node,
            wall_checker_node,
            simple_scan_mapper_node,
            geometric_frontier_node,
            default_nav_node,
        ],
    )

    return LaunchDescription([
        DeclareLaunchArgument("gui", default_value="true", description="Run Gazebo GUI"),
        DeclareLaunchArgument("autonomous", default_value="false", description="Enable autonomous frontier exploration"),
        DeclareLaunchArgument("spawn_x", default_value="2.5", description="Spawn X coordinate"),
        DeclareLaunchArgument("spawn_y", default_value="0.0", description="Spawn Y coordinate"),
        DeclareLaunchArgument("spawn_z", default_value="0.32", description="Spawn Z coordinate"),
        DeclareLaunchArgument("spawn_heading", default_value="0.0", description="Spawn Heading (yaw)"),

        # 1. Gazebo
        gazebo_launch,
        
        # 1b. Static TF (immediate)
        static_tf_node,

        # 2. Stand up
        delayed_stand_up,

        # 3. QoS bridge
        delayed_bridges,

        # 4. SLAM
        delayed_slam,

        # 5. Odom comparison
        delayed_comparison,

        # 6. Autonomy (conditional on autonomous:=true)
        delayed_autonomy,
    ])
