cmake_minimum_required(VERSION 3.8)
project(ur3_control)

# Find necessary packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_ros_planning REQUIRED)
find_package(moveit_ros_move_group REQUIRED)
find_package(moveit_core REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(nlohmann_json REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_cmake_gtest REQUIRED)
find_package(std_msgs REQUIRED)

# Ensure include directories
include_directories(
  include
  ${rclcpp_INCLUDE_DIRS}
  ${Eigen3_INCLUDE_DIRS}
  ${nlohmann_json_INCLUDE_DIRS}
  ${YAML_CPP_INCLUDE_DIRS}
)

# Add library for SplineFollower
add_library(spline_follower_lib
  src/spline_follower.cpp
)

# Link dependencies to the library
ament_target_dependencies(spline_follower_lib
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_core
  Eigen3
  nlohmann_json
  yaml-cpp
  tf2_ros
  std_msgs
)

# Ensure all required symbols are included by explicitly linking the library
target_link_libraries(spline_follower_lib
  ${rclcpp_LIBRARIES}
  ${Eigen3_LIBRARIES}
  ${YAML_CPP_LIBRARIES}
)

# Define executable for main
add_executable(spline_follower src/main.cpp)

# Link the main executable with the library
target_link_libraries(spline_follower spline_follower_lib Eigen3::Eigen)

# Link dependencies for the main executable
ament_target_dependencies(spline_follower
  rclcpp
  moveit_ros_planning_interface
  moveit_ros_planning
  moveit_core
  Eigen3
  nlohmann_json
  yaml-cpp
)

ament_add_gtest(test_tool_path_accuracy unit_testing/test_tool_path_accuracy.cpp)
target_link_libraries(test_tool_path_accuracy nlohmann_json::nlohmann_json)

ament_add_gtest(test_tool_path_z_consistency unit_testing/test_tool_path_z_consistency.cpp)
target_link_libraries(test_tool_path_z_consistency nlohmann_json::nlohmann_json yaml-cpp)

# Install the library and executable
install(
  TARGETS spline_follower_lib spline_follower
  DESTINATION lib/${PROJECT_NAME}
)

# Install the config directory (JSON files)
install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}/
)

install(
  DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}/
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()