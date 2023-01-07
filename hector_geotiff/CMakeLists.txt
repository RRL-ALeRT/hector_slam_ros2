cmake_minimum_required(VERSION 3.8)
project(hector_geotiff)
set(CMAKE_CXX_STANDARD 14)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(hector_map_tools REQUIRED)
find_package(hector_nav_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(ament_index_cpp)
find_package(world_info_msgs)

find_package(Qt5 COMPONENTS Widgets REQUIRED)
find_package(Eigen3 REQUIRED)

###########
## Build ##
###########

include_directories(
  include
  ${EIGEN3_INCLUDE_DIRS}
  ${Qt5Widgets_INCLUDE_DIRS}
)

add_library(geotiff_writer src/geotiff_writer/geotiff_writer.cpp)
target_link_libraries(geotiff_writer Qt5::Widgets stdc++fs)
ament_target_dependencies(geotiff_writer rclcpp nav_msgs hector_map_tools ament_index_cpp)

add_executable(geotiff_saver src/geotiff_saver.cpp)
target_link_libraries(geotiff_saver geotiff_writer)
ament_target_dependencies(geotiff_saver rclcpp nav_msgs visualization_msgs)

add_executable(geotiff_node src/geotiff_node.cpp)
target_link_libraries(geotiff_node geotiff_writer)
ament_target_dependencies(geotiff_node rclcpp pluginlib hector_nav_msgs world_info_msgs)

add_executable(geotiff_node_cartographer src/geotiff_node_cartographer.cpp)
target_link_libraries(geotiff_node_cartographer geotiff_writer)
ament_target_dependencies(geotiff_node_cartographer rclcpp pluginlib hector_nav_msgs world_info_msgs)

#############
## Install ##
#############

# all install targets should use catkin DESTINATION variables
# See http://ros.org/doc/api/catkin/html/adv_user_guide/variables.html

install(DIRECTORY include/${PROJECT_NAME}/
  DESTINATION include/${PROJECT_NAME}
  FILES_MATCHING PATTERN "*.h"
  PATTERN ".svn" EXCLUDE
)

install(DIRECTORY fonts
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  geotiff_writer
  geotiff_saver
  geotiff_node_cartographer
  geotiff_node
  DESTINATION lib/${PROJECT_NAME})

ament_package()