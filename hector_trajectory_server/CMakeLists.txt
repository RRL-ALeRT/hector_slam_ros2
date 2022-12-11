cmake_minimum_required(VERSION 3.8)
project(hector_trajectory_server)

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
find_package(hector_nav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hector_map_tools REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs)

###########
## Build ##
###########

add_executable(hector_trajectory_server src/hector_trajectory_server.cpp)
ament_target_dependencies(hector_trajectory_server
  rclcpp hector_nav_msgs nav_msgs hector_map_tools tf2 tf2_ros tf2_geometry_msgs
)

#############
## Install ##
#############

install(TARGETS
  hector_trajectory_server
  DESTINATION lib/${PROJECT_NAME})

ament_package()
