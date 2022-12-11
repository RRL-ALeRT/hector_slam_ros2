cmake_minimum_required(VERSION 3.8)
project(hector_imu_attitude_to_tf)
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
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)

add_executable(imu_attitude_to_tf_node src/imu_attitude_to_tf_node.cpp)
ament_target_dependencies(imu_attitude_to_tf_node rclcpp sensor_msgs geometry_msgs tf2 tf2_ros)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  imu_attitude_to_tf_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()
