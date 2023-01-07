cmake_minimum_required(VERSION 3.8)
project(world_info)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(world_info_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

add_executable(world_info src/world_info.cpp)
ament_target_dependencies(world_info rclcpp world_info_msgs tf2_ros tf2_geometry_msgs)

install(TARGETS
  world_info
  DESTINATION lib/${PROJECT_NAME})

ament_package()
