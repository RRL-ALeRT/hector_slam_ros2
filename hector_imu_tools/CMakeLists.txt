cmake_minimum_required(VERSION 3.8)
project(hector_imu_tools)
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
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(tf2_ros REQUIRED)

###########
## Build ##
###########

add_executable(pose_and_orientation_to_imu_node src/pose_and_orientation_to_imu_node.cpp)
ament_target_dependencies(pose_and_orientation_to_imu_node rclcpp sensor_msgs geometry_msgs nav_msgs tf2_ros)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  pose_and_orientation_to_imu_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()