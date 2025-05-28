cmake_minimum_required(VERSION 3.8)
project(ur3_localisation)

# Dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_srvs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(GTest REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(orocos_kdl REQUIRED)
find_package(kdl_parser REQUIRED)
find_package(urdf REQUIRED)
find_package(rosbag2_cpp REQUIRED)

# Executables
add_executable(localisation_node src/localisation_node.cpp)
ament_target_dependencies(localisation_node rclcpp std_srvs geometry_msgs yaml-cpp Eigen3 visualization_msgs tf2_ros)

add_executable(test1_four_locations_present unit_testing/test1_four_locations_present.cpp)
target_link_libraries(test1_four_locations_present GTest::gtest_main yaml-cpp)

add_executable(test2_location_accuracy unit_testing/test2_location_accuracy.cpp)
target_link_libraries(test2_location_accuracy GTest::gtest_main yaml-cpp)

add_executable(integration_test_localisation_file_present unit_testing/integration_test_localisation_file_present.cpp)
target_link_libraries(integration_test_localisation_file_present GTest::gtest_main yaml-cpp)

# Link YAML-CPP explicitly
target_link_libraries(localisation_node yaml-cpp Eigen3::Eigen)

install(TARGETS
  localisation_node
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
