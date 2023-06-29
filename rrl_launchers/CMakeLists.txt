cmake_minimum_required(VERSION 3.8)
project(rrl_launchers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(PCL REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(octomap_msgs REQUIRED)
find_package(octomap REQUIRED)
find_package(octomap_server REQUIRED)

add_executable(map_vision src/map_vision.cpp)
ament_target_dependencies(map_vision rclcpp tf2 tf2_ros geometry_msgs)

add_executable(pcl_to_octomap src/pcl_to_octomap.cpp)
ament_target_dependencies(pcl_to_octomap rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs PCL pcl_msgs pcl_conversions pcl_ros octomap_msgs octomap octomap_server)
target_link_libraries(pcl_to_octomap pcl_common pcl_filters pcl_io)

add_library(filter_pcl SHARED src/filter_pcl.cpp)
ament_target_dependencies(filter_pcl rclcpp rclcpp_components std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs PCL pcl_msgs pcl_conversions)
target_link_libraries(filter_pcl ${PCL_libs} pcl_filters)
rclcpp_components_register_node(filter_pcl
PLUGIN "rrl_launchers::FilteredPointCloud"
EXECUTABLE filter_pcl_node)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY config
  DESTINATION share/${PROJECT_NAME}
)

install(DIRECTORY params
  DESTINATION share/${PROJECT_NAME}
)

install(TARGETS
  filter_pcl
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  map_vision
  pcl_to_octomap
  DESTINATION lib/${PROJECT_NAME})

ament_package()
