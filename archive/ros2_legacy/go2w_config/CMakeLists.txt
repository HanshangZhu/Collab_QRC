cmake_minimum_required(VERSION 3.8)
project(go2w_config)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY
  docs
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
