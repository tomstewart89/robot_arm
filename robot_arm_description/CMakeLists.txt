cmake_minimum_required(VERSION 3.5)
project(robot_arm_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY
    launch
    models
    urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  PROGRAMS
  launch/display.launch.py
  DESTINATION lib/${PROJECT_NAME}
)
ament_package()
