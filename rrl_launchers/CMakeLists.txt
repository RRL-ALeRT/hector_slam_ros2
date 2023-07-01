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
find_package(visualization_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(hector_nav_msgs REQUIRED)
find_package(image_transport REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(Qt5 COMPONENTS Widgets REQUIRED)

include_directories(include)

add_executable(map_vision src/map_vision.cpp)
ament_target_dependencies(map_vision rclcpp tf2 tf2_ros geometry_msgs visualization_msgs std_srvs hector_nav_msgs)

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
  map_vision
  DESTINATION lib/${PROJECT_NAME})

ament_package()
