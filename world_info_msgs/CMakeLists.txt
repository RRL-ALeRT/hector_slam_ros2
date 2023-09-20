cmake_minimum_required(VERSION 3.8)
project(world_info_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/WorldInfo.msg"
  "msg/WorldInfoArray.msg"
  "msg/BoundingBox.msg"
  "msg/BoundingBoxArray.msg"
  "msg/BoundingPolygon.msg"
  "msg/BoundingPolygonArray.msg"
  "srv/GetMedianDepthXYZ.srv"
  DEPENDENCIES builtin_interfaces geometry_msgs std_msgs visualization_msgs
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
