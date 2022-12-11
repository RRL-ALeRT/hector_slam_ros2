cmake_minimum_required(VERSION 3.8)
project(hector_nav_msgs)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(builtin_interfaces REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)

#######################################
## Declare ROS messages and services ##
#######################################

set(srv_files
  "srv/GetDistanceToObstacle.srv"
  "srv/GetRecoveryInfo.srv"
  "srv/GetRobotTrajectory.srv"
  "srv/GetSearchPosition.srv"
  "srv/GetNormal.srv"
)

## Generate services in the 'srv' folder
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/HectorIterData.msg"
  "msg/HectorDebugInfo.msg"
  "srv/ResetMapping.srv"
  ${srv_files}
  DEPENDENCIES builtin_interfaces nav_msgs geometry_msgs std_msgs
  ADD_LINTER_TESTS
  )

ament_package()