cmake_minimum_required(VERSION 3.8)
project(robot_arm_control)

set(CXX_STANDARD 17)
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(Boost REQUIRED COMPONENTS system)

add_executable(robot_arm_driver src/robot_arm_driver.cpp src/cds5500.cpp src/serial_port.cpp)
target_link_libraries(robot_arm_driver ${Boost_SYSTEM_LIBRARY})
ament_target_dependencies(robot_arm_driver rclcpp sensor_msgs)

## COMPILE
add_library(${PROJECT_NAME} SHARED src/serial_port.cpp src/cds5500.cpp src/robot_arm_hardware_interface.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE include)
target_link_libraries(${PROJECT_NAME} ${Boost_SYSTEM_LIBRARY})
ament_target_dependencies(${PROJECT_NAME} hardware_interface pluginlib rclcpp rclcpp_lifecycle)

# Export hardware plugins
pluginlib_export_plugin_description_file(hardware_interface robot_arm_control.xml)

# INSTALL
install(TARGETS robot_arm_driver DESTINATION lib/${PROJECT_NAME})
install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(DIRECTORY include/ DESTINATION include)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME})

## EXPORTS
ament_export_include_directories(include)
ament_export_libraries(${PROJECT_NAME})
ament_export_dependencies(hardware_interface pluginlib rclcpp rclcpp_lifecycle)

ament_package()
