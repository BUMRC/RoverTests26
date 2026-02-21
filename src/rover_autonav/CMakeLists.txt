cmake_minimum_required(VERSION 3.5)
project(rover_autonav)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  description
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  src/motor_driver.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
