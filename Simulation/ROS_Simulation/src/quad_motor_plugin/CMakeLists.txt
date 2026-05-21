cmake_minimum_required(VERSION 3.8)
project(quad_motor_plugin)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)

find_package(ignition-gazebo6 REQUIRED)
find_package(ignition-plugin1 REQUIRED)
find_package(ignition-math6 REQUIRED)

include_directories(include)

add_library(QuadMotorPlugin SHARED
  src/QuadMotorPlugin.cc
)

ament_target_dependencies(QuadMotorPlugin
  rclcpp
  std_msgs
)

target_link_libraries(QuadMotorPlugin
  ignition-gazebo6::ignition-gazebo6
  ignition-plugin1::ignition-plugin1
  ignition-math6::ignition-math6
)

install(TARGETS QuadMotorPlugin
  LIBRARY DESTINATION lib
)

ament_package()