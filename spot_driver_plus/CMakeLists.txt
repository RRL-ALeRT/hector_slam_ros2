cmake_minimum_required(VERSION 3.8)
project(spot_driver_plus)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(spot_msgs REQUIRED)
find_package(pcl_msgs REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(pcl_ros REQUIRED)
find_package(image_geometry REQUIRED)
find_package(OpenCV REQUIRED)
find_package(depth_image_proc REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(hector_nav_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(controller_manager_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)

add_executable(battery_screen cpp/battery_screen.cpp)
ament_target_dependencies(battery_screen rclcpp spot_msgs)

add_executable(depth_to_pcl cpp/depth_to_pcl.cpp)
ament_target_dependencies(depth_to_pcl rclcpp std_msgs sensor_msgs geometry_msgs tf2 tf2_ros tf2_geometry_msgs PCL pcl_ros pcl_conversions pcl_msgs cv_bridge image_geometry OpenCV depth_image_proc)
target_link_libraries(depth_to_pcl ${PCL_LIBRARIES} pcl_common pcl_filters pcl_io)

add_executable(spot_marker cpp/spot_marker.cpp)
ament_target_dependencies(spot_marker rclcpp tf2 tf2_ros geometry_msgs visualization_msgs interactive_markers)

add_executable(spot_kinova_controller cpp/spot_kinova_controller.cpp)
ament_target_dependencies(spot_kinova_controller rclcpp rclcpp_action sensor_msgs geometry_msgs std_srvs spot_msgs tf2 control_msgs controller_manager_msgs moveit_msgs)

add_executable(map_vision cpp/map_vision.cpp)
ament_target_dependencies(map_vision rclcpp geometry_msgs tf2 tf2_ros std_srvs hector_nav_msgs visualization_msgs nav_msgs tf2_geometry_msgs)

install(TARGETS
  battery_screen
  depth_to_pcl
  spot_marker
  spot_kinova_controller
  map_vision
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY launch config urdf
  DESTINATION share/${PROJECT_NAME}
)

# Python nodes (at least one cpp node should be available to see these)
set(PYTHON_NODES
  get_pcl.py
  goal_pose_to_trajectory.py
  spot_estop.py
  motion_detection.py
  plan_3d_path.py
  rrl_qreader.py
)
install(DIRECTORY python/${PROJECT_NAME}
  DESTINATION lib
)
foreach(PY_FILE ${PYTHON_NODES})
  execute_process(COMMAND chmod +x ${CMAKE_INSTALL_PREFIX}/lib/${PROJECT_NAME}/${PY_FILE})
endforeach()

ament_package()
