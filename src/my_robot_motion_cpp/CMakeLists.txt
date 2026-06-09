cmake_minimum_required(VERSION 3.8)
project(my_robot_motion_cpp)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# =====================================================
# Required packages
# =====================================================

find_package(ament_cmake REQUIRED)

find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(trajectory_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

find_package(moveit_core REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)

find_package(PCL REQUIRED)
find_package(pcl_conversions REQUIRED)

# =====================================================
# Include directories
# =====================================================

include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(
  ${PCL_LIBRARY_DIRS}
)

add_definitions(
  ${PCL_DEFINITIONS}
)

# =====================================================
# ARM MOTION NODE
# =====================================================

add_executable(
  arm_motion_node
  src/arm_motion_node.cpp
)

ament_target_dependencies(
  arm_motion_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

# =====================================================
# ARM MOTION TEST NODE
# =====================================================

add_executable(
  arm_motion_test_node
  src/arm_motion_test_node.cpp
)

ament_target_dependencies(
  arm_motion_test_node
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

# =====================================================
# SERIAL BRIDGE NODE (ESP32 Bridge)
# =====================================================

add_executable(
  serial_bridge_node
  src/serial_bridge_node.cpp
)

ament_target_dependencies(
  serial_bridge_node
  rclcpp
  geometry_msgs
  moveit_ros_planning_interface
)

# =====================================================
# WORKSPACE SAMPLER
# =====================================================

add_executable(
  workspace_sampler
  src/workspace_sampler.cpp
)

ament_target_dependencies(
  workspace_sampler
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

# =====================================================
# NOISE TEST NODE
# =====================================================

add_executable(
  noise_test_node
  src/noise_test_node.cpp
)

ament_target_dependencies(
  noise_test_node
  rclcpp
  geometry_msgs
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
)

# =====================================================
# REACHABILITY MAP NODE
# =====================================================

add_executable(
  reachability_map_node
  src/reachability_map_node.cpp
)

ament_target_dependencies(
  reachability_map_node
  rclcpp
  geometry_msgs
  sensor_msgs
  tf2_ros
  moveit_core
  moveit_ros_planning
  moveit_ros_planning_interface
  pcl_conversions
)

target_link_libraries(
  reachability_map_node
  ${PCL_LIBRARIES}
)

# =====================================================
# INSTALL
# =====================================================

install(
  TARGETS
  arm_motion_node
  arm_motion_test_node
  serial_bridge_node
  workspace_sampler
  noise_test_node
  reachability_map_node
  DESTINATION
  lib/${PROJECT_NAME}
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()