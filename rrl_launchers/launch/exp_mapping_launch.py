from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess


def generate_launch_description():
    launch_list = []

    # frontier = Node(
    #     package="autonomous_exploration",
    #     executable="control",
    #     output="screen",
    # )
    # launch_list.append(frontier)

    world_info = Node(
        package="world_info",
        executable="world_info",
        output="screen",
    )
    launch_list.append(world_info)

    qr_front = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'qrcode', '/rs_front/color/image_raw', '/rs_front/aligned_depth_to_color/image_raw', 'rs_front_color_optical_frame'],
                        output='screen'
                    )
    launch_list.append(qr_front)
    qr_left = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'qrcode', '/rs_left/color/image_raw', '/rs_left/aligned_depth_to_color/image_raw', 'rs_left_color_optical_frame'],
                        output='screen'
                    )
    launch_list.append(qr_left)
    qr_right = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'qrcode', '/rs_right/color/image_raw', '/rs_right/aligned_depth_to_color/image_raw', 'rs_right_color_optical_frame'],
                        output='screen'
                    )
    launch_list.append(qr_right)

    # object_front = ExecuteProcess(
    #                     cmd=['ros2', 'run', 'world_info', 'object_detection_yolov5', 'object', '/rs_front/color/image_raw', '/rs_front/aligned_depth_to_color/image_raw', 'rs_front_color_optical_frame'],
    #                     output='screen'
    #                 )
    # launch_list.append(object_front)
    # object_left = ExecuteProcess(
    #                     cmd=['ros2', 'run', 'world_info', 'object_detection_yolov5', 'object', '/rs_left/color/image_raw', '/rs_left/aligned_depth_to_color/image_raw', 'rs_left_color_optical_frame'],
    #                     output='screen'
    #                 )
    # launch_list.append(object_left)
    # object_right = ExecuteProcess(
    #                     cmd=['ros2', 'run', 'world_info', 'object_detection_yolov5', 'object', '/rs_right/color/image_raw', '/rs_right/aligned_depth_to_color/image_raw', 'rs_right_color_optical_frame'],
    #                     output='screen'
    #                 )
    # launch_list.append(object_right)

    hazmat_front = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'object_detection_yolov5', 'hazmat', '/rs_front/color/image_raw', '/rs_front/aligned_depth_to_color/image_raw', 'rs_front_color_optical_frame'],
                        output='screen'
                    )
    launch_list.append(hazmat_front)
    hazmat_left = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'object_detection_yolov5', 'hazmat', '/rs_left/color/image_raw', '/rs_left/aligned_depth_to_color/image_raw', 'rs_left_color_optical_frame'],
                        output='screen'
                    )
    launch_list.append(hazmat_left)
    hazmat_right = ExecuteProcess(
                        cmd=['ros2', 'run', 'world_info', 'object_detection_yolov5', 'hazmat', '/rs_right/color/image_raw', '/rs_right/aligned_depth_to_color/image_raw', 'rs_right_color_optical_frame'],
                        output='screen'
                    )
    launch_list.append(hazmat_right)

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
