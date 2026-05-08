import os
import xacro

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def _strip_comments(node):
    for child in list(node.childNodes):
        if child.nodeType == child.COMMENT_NODE:
            node.removeChild(child)
        else:
            _strip_comments(child)


def _build_runtime_nodes(context):
    use_sim_time = LaunchConfiguration("use_sim_time").perform(context).lower() == "true"
    rviz_enabled = LaunchConfiguration("rviz").perform(context).lower() == "true"

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    go2_description_pkg = get_package_share_directory("go2_description")
    go2_config_pkg = get_package_share_directory("go2_config")
    champ_base_pkg = get_package_share_directory("champ_base")

    # Use the local lidar-enabled model to guarantee a live scan source.
    # Upstream go2_description/robot.xacro may not always include front_laser.
    description_path = os.path.join(go2_gazebo_pkg, "urdf", "go2_description_3d_lidar.xacro")
    joints_config = os.path.join(go2_config_pkg, "config", "joints", "joints.yaml")
    links_config = os.path.join(go2_config_pkg, "config", "links", "links.yaml")
    gait_config = os.path.join(go2_config_pkg, "config", "gait", "gait.yaml")
    rviz_path = os.path.join(go2_gazebo_pkg, "rviz", "autonomy.rviz")

    # Keep robot_description compact; gazebo_ros2_control can choke on XML comments.
    doc = xacro.process_file(description_path)
    _strip_comments(doc)
    robot_description = doc.documentElement.toxml()

    robot_state_publisher_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[
            {"robot_description": ParameterValue(robot_description, value_type=str)},
            {"use_tf_static": False},
            {"publish_frequency": 200.0},
            {"ignore_timestamp": True},
            {"use_sim_time": use_sim_time},
        ],
        output="screen",
    )

    quadruped_controller_node = Node(
        package="champ_base",
        executable="quadruped_controller_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"gazebo": True},
            {"publish_joint_states": True},
            {"publish_joint_control": True},
            {"publish_foot_contacts": False},
            {"joint_controller_topic": "joint_group_effort_controller/joint_trajectory"},
            {"urdf": ParameterValue(robot_description, value_type=str)},
            joints_config,
            links_config,
            gait_config,
        ],
        remappings=[("/cmd_vel/smooth", "/cmd_vel")],
    )

    state_estimator_node = Node(
        package="champ_base",
        executable="state_estimation_node",
        output="screen",
        parameters=[
            {"use_sim_time": use_sim_time},
            {"orientation_from_imu": False},
            {"urdf": ParameterValue(robot_description, value_type=str)},
            joints_config,
            links_config,
            gait_config,
        ],
    )

    base_to_footprint_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="base_to_footprint_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": "base_link"},
            {"use_sim_time": use_sim_time},
            os.path.join(champ_base_pkg, "config", "ekf", "base_to_footprint.yaml"),
        ],
        remappings=[("odometry/filtered", "odom/local")],
    )

    footprint_to_odom_ekf = Node(
        package="robot_localization",
        executable="ekf_node",
        name="footprint_to_odom_ekf",
        output="screen",
        parameters=[
            {"base_link_frame": "base_link"},
            {"use_sim_time": use_sim_time},
            os.path.join(champ_base_pkg, "config", "ekf", "footprint_to_odom.yaml"),
        ],
        remappings=[("odometry/filtered", "odom")],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        arguments=["-d", rviz_path],
        condition=IfCondition(str(rviz_enabled).lower()),
        output="screen",
    )

    return [
        robot_state_publisher_node,
        quadruped_controller_node,
        state_estimator_node,
        base_to_footprint_ekf,
        footprint_to_odom_ekf,
        rviz_node,
    ]


def generate_launch_description():
    """Launch Go2 robot in L-shaped corridor world (2D stack)."""

    go2_gazebo_pkg = get_package_share_directory("go2_gazebo_sim")
    l_corridor_world = os.path.join(go2_gazebo_pkg, "worlds", "l_corridor.world")

    use_sim_time = LaunchConfiguration("use_sim_time")
    gui = LaunchConfiguration("gui")
    rviz = LaunchConfiguration("rviz")
    world_init_x = LaunchConfiguration("world_init_x")
    world_init_y = LaunchConfiguration("world_init_y")
    world_init_z = LaunchConfiguration("world_init_z")
    world_init_heading = LaunchConfiguration("world_init_heading")

    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("champ_gazebo"),
                "launch",
                "gazebo.launch.py",
            )
        ),
        launch_arguments={
            "use_sim_time": use_sim_time,
            "robot_name": "go2",
            "world": l_corridor_world,
            "lite": "false",
            "world_init_x": world_init_x,
            "world_init_y": world_init_y,
            "world_init_z": world_init_z,
            "world_init_heading": world_init_heading,
            "gui": gui,
        }.items(),
    )

    return LaunchDescription(
        [
            DeclareLaunchArgument("use_sim_time", default_value="true", description="Use simulation time"),
            DeclareLaunchArgument("gui", default_value="true", description="Run Gazebo GUI"),
            DeclareLaunchArgument("rviz", default_value="true", description="Run RViz"),
            DeclareLaunchArgument("world_init_x", default_value="2.5"),
            DeclareLaunchArgument("world_init_y", default_value="0.0"),
            DeclareLaunchArgument("world_init_z", default_value="0.32"),
            DeclareLaunchArgument("world_init_heading", default_value="0.0"),
            OpaqueFunction(function=_build_runtime_nodes),
            gazebo_launch,
        ]
    )
