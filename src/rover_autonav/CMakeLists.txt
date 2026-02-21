cmake_minimum_required(VERSION 3.5)
project(rover_autonav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  description
  launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
