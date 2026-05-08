cmake_minimum_required(VERSION 3.8)
project(go2w_observability)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install Python scripts
install(PROGRAMS
  scripts/robot_status_monitor.py
  scripts/dual_map_coverage_visualizer.py
  scripts/odom_comparison.py
  scripts/exploration_metrics_logger.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
