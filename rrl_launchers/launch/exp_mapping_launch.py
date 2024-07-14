from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    launch_list = []

    world_info = Node(
        package="world_info",
        executable="world_info",
        output="screen",
    )
    launch_list.append(world_info)

    qr_detector = Node(
        package="spot_driver_plus",
        executable="rrl_qreader.py",
        output="screen",
    )
    launch_list.append(qr_detector)

    hazmat_node = Node(
        package="spot_driver_plus",
        executable="rrl_yolov8_openvino.py",
        output="screen",
        parameters=[{"model", "hazmat"}],
    )
    launch_list.append(hazmat_node)

    # object_node = Node(
    #     package="spot_driver_plus",
    #     executable="rrl_yolov8_openvino.py",
    #     output="screen",
    #     parameters=[{"model", "object"}],
    # )
    # launch_list.append(object_node)


    tf2_rs_front = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'tf2_object_detection_yolov5', '/rs_front/aligned_depth_to_color/image_raw', '/rs_front/aligned_depth_to_color/camera_info'],
                        output='screen'
                    )
    launch_list.append(tf2_rs_front)
    tf2_rs_left = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'tf2_object_detection_yolov5', '/rs_left/aligned_depth_to_color/image_raw', '/rs_left/aligned_depth_to_color/camera_info'],
                        output='screen'
                    )
    launch_list.append(tf2_rs_left)
    tf2_rs_right = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'tf2_object_detection_yolov5', '/rs_right/aligned_depth_to_color/image_raw', '/rs_right/aligned_depth_to_color/camera_info'],
                        output='screen'
                    )
    launch_list.append(tf2_rs_right)

    return LaunchDescription(launch_list)
