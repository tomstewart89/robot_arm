cmake_minimum_required(VERSION 3.10.2)
project(robot_arm_bringup)

find_package(ament_cmake REQUIRED)

ament_package()

install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})
