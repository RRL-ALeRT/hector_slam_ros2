import os
import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
from launch.actions import ExecuteProcess


def generate_launch_description():
    exp_sensor_combi = LaunchDescription()

    object_detection_left = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'object_detection_yolov5', 'hazmat', '/rs_left/color/image_raw', '/rs_left/aligned_depth_to_color/image_raw'],
                        output='screen'
                    )
    object_detection_right = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'object_detection_yolov5', 'hazmat', '/rs_right/color/image_raw', '/rs_right/aligned_depth_to_color/image_raw'],
                        output='screen'
                    )
    object_detection_front = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'object_detection_yolov5', 'hazmat', '/rs_front/color/image_raw', '/rs_front/aligned_depth_to_color/image_raw'],
                        output='screen'
                    )
    tf2_object_detection_left = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'tf2_object_detection_yolov5', '/rs_left/aligned_depth_to_color/image_raw', '/rs_left/aligned_depth_to_color/camera_info'],
                        output='screen'
                    )
    tf2_object_detection_right = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'tf2_object_detection_yolov5', '/rs_right/aligned_depth_to_color/image_raw', '/rs_right/aligned_depth_to_color/camera_info'],
                        output='screen'
                    )
    tf2_object_detection_front = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'tf2_object_detection_yolov5', '/rs_front/aligned_depth_to_color/image_raw', '/rs_front/aligned_depth_to_color/camera_info'],
                        output='screen'
                    )
    
    exp_sensor_combi.add_action(object_detection_left, object_detection_right, object_detection_front, tf2_object_detection_left, tf2_object_detection_right, tf2_object_detection_front )

    return exp_sensor_combi

