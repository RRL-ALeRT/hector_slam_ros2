cmake_minimum_required(VERSION 3.8)
project(hector_compressed_map_transport)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(hector_map_tools REQUIRED)
find_package(image_transport REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

###########
## Build ##
###########

include_directories(
  ${EIGEN3_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
)

add_executable(map_to_image_node src/map_to_image_node.cpp)
ament_target_dependencies(map_to_image_node
OpenCV Eigen3 rclcpp sensor_msgs cv_bridge nav_msgs geometry_msgs image_transport hector_map_tools)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(TARGETS
  map_to_image_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()