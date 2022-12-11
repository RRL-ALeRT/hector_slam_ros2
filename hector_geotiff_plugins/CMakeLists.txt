cmake_minimum_required(VERSION 3.8)
project(hector_geotiff_plugins)

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
find_package(hector_geotiff REQUIRED)
find_package(hector_nav_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pluginlib REQUIRED)

add_library(hector_geotiff_plugins src/trajectory_geotiff_plugin.cpp)
target_include_directories(hector_geotiff_plugins PUBLIC
<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
<INSTALL_INTERFACE:include>)
ament_target_dependencies(hector_geotiff_plugins
  rclcpp
  pluginlib
  hector_geotiff
  hector_nav_msgs
)
include_directories(include "../hector_geotiff/include" ${EIGEN3_INCLUDE_DIR})

pluginlib_export_plugin_description_file(hector_geotiff_plugins hector_geotiff_plugins.xml)

install(
  TARGETS hector_geotiff_plugins
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(
  hector_geotiff_plugins
)
ament_export_targets(
  export_${PROJECT_NAME}
)
