cmake_minimum_required(VERSION 3.8)
project(go2w_safety)

find_package(ament_cmake REQUIRED)

install(PROGRAMS
  scripts/wall_collision_checker.py
  scripts/autonomy_enabler.py
  scripts/supervisor_panic_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
