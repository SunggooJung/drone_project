cmake_minimum_required(VERSION 3.8)
project(drone_mpc_control)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(px4_msgs REQUIRED)
find_package(Eigen3 REQUIRED)

# Include directories
include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
)

# MPC Controller executable
add_executable(mpc_controller_node src/mpc_controller_node.cpp)
ament_target_dependencies(mpc_controller_node
  rclcpp
  geometry_msgs
  nav_msgs
  sensor_msgs
  px4_msgs
)

# PX4 Bridge executable
add_executable(px4_bridge_node src/px4_bridge_node.cpp)
ament_target_dependencies(px4_bridge_node
  rclcpp
  px4_msgs
  sensor_msgs
)

# Trajectory Generator executable
add_executable(trajectory_generator_node src/trajectory_generator_node.cpp)
ament_target_dependencies(trajectory_generator_node
  rclcpp
  geometry_msgs
  nav_msgs
)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}/
)

install(TARGETS
  mpc_controller_node
  px4_bridge_node
  trajectory_generator_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_python_install_package(${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
