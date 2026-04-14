cmake_minimum_required(VERSION 3.8)
project(my_robot_commander_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(example_interfaces REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(test_moveit src/test_moveit.cpp)

ament_target_dependencies(
  test_moveit
  PUBLIC
  rclcpp
  moveit_ros_planning_interface
  tf2_geometry_msgs
)

add_executable(commander src/commander_template.cpp)
ament_target_dependencies(
  commander
  PUBLIC
  rclcpp
  moveit_ros_planning_interface
  example_interfaces
  tf2_geometry_msgs
)


install(TARGETS
  test_moveit
  commander
  DESTINATION lib/${PROJECT_NAME}/
)

ament_package()
