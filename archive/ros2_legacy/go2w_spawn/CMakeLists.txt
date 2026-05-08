cmake_minimum_required(VERSION 3.8)
project(go2w_spawn)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# Install config (ros_control YAML)
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

# Install Python scripts
install(PROGRAMS
  scripts/spawn_entity_direct.py
  scripts/initial_pose_guard.py
  scripts/stand_up_slowly.py
  scripts/spawn_drift_monitor.py
  scripts/wait_for_ready.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
