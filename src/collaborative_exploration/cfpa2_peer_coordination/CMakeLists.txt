cmake_minimum_required(VERSION 3.10)
project(cfpa2_peer_coordination)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(cfpa2_peer_coordination_msgs REQUIRED)

# OR-Tools (optional). Modern OR-Tools (9.x+) ships an ortools-config.cmake
# that find_package picks up when the install prefix is on CMAKE_PREFIX_PATH.
# If not found, the MDVRP library compiles without OR-Tools and silently
# uses the load-balanced heuristic fallback — matches the Python
# _HAS_ORTOOLS try/except in mdvrp_solver.py.
find_package(ortools CONFIG QUIET)
if(ortools_FOUND)
  message(STATUS "cfpa2_peer_coordination: OR-Tools found, MDVRP will use routing solver")
else()
  message(STATUS "cfpa2_peer_coordination: OR-Tools not found, MDVRP will use heuristic fallback")
endif()

# ──────────────────────────────────────────────
# cfpa2_peer_coordination_mdvrp — pure-C++ algorithmic core.
#
# Direct port of cfpa2_peer_coordination/mdvrp_solver.py and
# mdvrp_adapter.py. Depends only on geometry_msgs (for the ROS msg
# conversion helpers in the adapter) and optionally ortools.
# No rclcpp dependency — unit-testable standalone.
# ────────────────────────────────────────────────
add_library(cfpa2_peer_coordination_mdvrp STATIC
  src/mdvrp_solver.cpp
  src/mdvrp_adapter.cpp)
target_include_directories(cfpa2_peer_coordination_mdvrp PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(cfpa2_peer_coordination_mdvrp
  geometry_msgs)
set_target_properties(cfpa2_peer_coordination_mdvrp PROPERTIES
  POSITION_INDEPENDENT_CODE ON)

if(ortools_FOUND)
  target_link_libraries(cfpa2_peer_coordination_mdvrp ortools::ortools)
  target_compile_definitions(cfpa2_peer_coordination_mdvrp
    PRIVATE CFPA2_PC_HAS_ORTOOLS=1)
endif()

install(DIRECTORY include/
  DESTINATION include)
install(TARGETS cfpa2_peer_coordination_mdvrp
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

# ───────────────────────────────────────────────────────────
# peer_coordinator_lib — rclcpp::Node implementation (Phase B).
#
# Decentralised peer coordinator node: PeerState heartbeats,
# NegotiationRequest/Response protocol, claim management, blocked
# frontier publishing. One instance per robot.
# ──────────────────────────────────────────────────────────
add_library(peer_coordinator_lib STATIC
  src/peer_coordinator.cpp)
target_include_directories(peer_coordinator_lib PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
target_link_libraries(peer_coordinator_lib cfpa2_peer_coordination_mdvrp)
ament_target_dependencies(peer_coordinator_lib
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  builtin_interfaces
  cfpa2_peer_coordination_msgs)
set_target_properties(peer_coordinator_lib PROPERTIES POSITION_INDEPENDENT_CODE ON)

install(TARGETS peer_coordinator_lib
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib)

# ──────────────────────────────────────────────────────────
# Pure-C++ executable. Suffixed `_cpp` during the transition so the
# existing Python entry-point (peer_coordinator_node) remains runnable
# as a fallback. Once verified on Jetson, rename + drop Python install.
# ──────────────────────────────────────────────────────────
add_executable(peer_coordinator_node_cpp
  src/peer_coordinator_node_main.cpp)
target_link_libraries(peer_coordinator_node_cpp peer_coordinator_lib)
ament_target_dependencies(peer_coordinator_node_cpp rclcpp)

install(TARGETS peer_coordinator_node_cpp
  RUNTIME DESTINATION lib/${PROJECT_NAME})

# ──────────────────────────────────────────────────────────
# Python sources (transitional fallback).
# ──────────────────────────────────────────────────────────
ament_python_install_package(${PROJECT_NAME})

install(
  FILES cfpa2_peer_coordination/peer_coordinator_node.py
  RENAME peer_coordinator_node
  DESTINATION lib/${PROJECT_NAME}
  PERMISSIONS
    OWNER_READ OWNER_WRITE OWNER_EXECUTE
    GROUP_READ GROUP_EXECUTE
    WORLD_READ WORLD_EXECUTE
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_export_include_directories(include)
ament_export_libraries(cfpa2_peer_coordination_mdvrp peer_coordinator_lib)
ament_export_dependencies(
  rclcpp
  geometry_msgs
  nav_msgs
  std_msgs
  visualization_msgs
  builtin_interfaces
  cfpa2_peer_coordination_msgs)

ament_package()