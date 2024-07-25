cmake_minimum_required(VERSION 3.8)
project(world_info)

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
find_package(world_info_msgs REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(image_transport REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(std_srvs REQUIRED)

add_executable(world_info src/world_info.cpp)
ament_target_dependencies(world_info rclcpp world_info_msgs tf2_ros tf2_geometry_msgs std_srvs)

add_executable(tf2_object_detection_yolov5 src/tf2_object_detection_yolov5.cpp)
ament_target_dependencies(tf2_object_detection_yolov5 rclcpp sensor_msgs image_transport OpenCV cv_bridge world_info_msgs tf2_ros)

add_library(aruco SHARED src/aruco.cpp)
ament_target_dependencies(aruco rclcpp rclcpp_components world_info_msgs sensor_msgs image_transport Eigen3 OpenCV cv_bridge tf2_ros)
rclcpp_components_register_node(aruco
PLUGIN "world_info::DetectAruco"
EXECUTABLE aruco_node)

install(DIRECTORY launch
  DESTINATION share/${PROJECT_NAME})

install(TARGETS
  aruco
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
  DESTINATION lib/${PROJECT_NAME})

install(TARGETS
  world_info
  tf2_object_detection_yolov5
  DESTINATION lib/${PROJECT_NAME})

ament_package()
