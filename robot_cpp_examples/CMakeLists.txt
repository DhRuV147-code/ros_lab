cmake_minimum_required(VERSION 3.8)
project(robot_cpp_examples)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(rcl_interfaces REQUIRED)

# MoveIt
find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# Geometry messages
find_package(geometry_msgs REQUIRED)

# Your custom messages/actions
find_package(robot_msgs REQUIRED)

# Simple MoveIt interface (example)
add_executable(simple_moveit_interface
  src/simple_moveit_interface.cpp
)
ament_target_dependencies(simple_moveit_interface
  rclcpp
  moveit_ros_planning_interface
)

# Task Action Server
add_executable(task_action_server
  src/task_action_server.cpp
)
ament_target_dependencies(task_action_server
  rclcpp
  rclcpp_action
  moveit_core
  moveit_ros_planning_interface
  geometry_msgs
  robot_msgs
)

install(
  TARGETS
    simple_moveit_interface
    task_action_server
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()