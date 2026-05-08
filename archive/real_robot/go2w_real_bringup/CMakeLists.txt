cmake_minimum_required(VERSION 3.8)
project(go2w_real_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  docker
  docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
