cmake_minimum_required(VERSION 3.8)
project(hector_mapping)

set(CMAKE_CXX_STANDARD 14)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(laser_geometry REQUIRED)
find_package(hector_nav_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)

find_package(Boost REQUIRED COMPONENTS thread)

find_package(Eigen3 REQUIRED)

add_executable(hector_mapping_node src/main.cpp src/HectorMappingRos.cpp src/PoseInfoContainer.cpp)
include_directories("include/hector_slam_lib/")
ament_target_dependencies(hector_mapping_node rclcpp Boost tf2 tf2_ros sensor_msgs hector_nav_msgs std_srvs laser_geometry visualization_msgs pcl_conversions)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  hector_mapping_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
