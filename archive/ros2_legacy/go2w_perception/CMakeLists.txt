cmake_minimum_required(VERSION 3.8)
project(go2w_perception)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(nav_msgs REQUIRED)

# C++ executables
add_executable(multi_tf_relay src/multi_tf_relay.cpp)
ament_target_dependencies(multi_tf_relay rclcpp tf2_msgs)
install(TARGETS multi_tf_relay
  DESTINATION lib/${PROJECT_NAME})

# robot_self_filter — strip LiDAR returns hitting peer-robot bodies before
# they reach octomap. Runs on the cloud hot path (10 Hz × 24 k pts), so
# C++ is a hard requirement; Python with numpy adds enough rclpy
# serialisation overhead to make octomap input rate stutter.
add_executable(robot_self_filter src/robot_self_filter.cpp)
ament_target_dependencies(robot_self_filter rclcpp sensor_msgs nav_msgs)
install(TARGETS robot_self_filter
  DESTINATION lib/${PROJECT_NAME})

# Install Python scripts
install(PROGRAMS
  scripts/qos_bridge.py
  scripts/pointcloud_adapter.py
  scripts/twist_bridge.py
  scripts/slam_odom_relay.py
  scripts/temporal_scan_filter.py
  scripts/carto_odom_bridge.py
  scripts/frontier_3d_markers.py
  scripts/probability_grid_binarizer.py
  scripts/pointcloud_frame_bridge.py
  scripts/map_merger.py
  scripts/elevation_to_occupancy.py
  scripts/transform_everything.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
