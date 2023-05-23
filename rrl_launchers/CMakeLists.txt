cmake_minimum_required(VERSION 3.8)
project(rrl_launchers)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)

add_executable(dad_vision src/dad_vision.cpp)
ament_target_dependencies(dad_vision rclcpp tf2 tf2_ros geometry_msgs)

add_executable(map_vision src/map_vision.cpp)
ament_target_dependencies(map_vision rclcpp tf2 tf2_ros geometry_msgs)

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
  dad_vision
  map_vision
  DESTINATION lib/${PROJECT_NAME})

ament_package()
