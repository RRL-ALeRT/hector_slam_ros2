#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node, ComposableNodeContainer
from launch_ros.descriptions import ComposableNode


def generate_launch_description():
    container = ComposableNodeContainer(
            name='tag_detectors_container',
            namespace='',
            package='rclcpp_components',
            executable='component_container',
            composable_node_descriptions=[
                ComposableNode(
                    package='world_info',
                    plugin='world_info::DetectAruco',
                    name='aruco_node',
                    parameters=[{'aruco_square_length': 0.3}],
                    remappings=[('/image_rect', '/Spot/kinect_color')],
                ),
                ComposableNode(
                    package='world_info',
                    plugin='world_info::DetectQR',
                    name='qr_node',
                    parameters=[{'qr_square_length': 0.85*0.3}],
                    remappings=[('/image_rect', '/Spot/kinect_color')],
                ),
                ComposableNode(
                    package='world_info',
                    plugin='world_info::DetectHazmat',
                    name='hazmat_node',
                    parameters=[{'hazmat_confidence_threshold': 0.9},
                                {'inference_mode': "GPU"}],
                    remappings=[('/image_rect', '/Spot/kinect_color')],
                ),
                ComposableNode(
                    package='world_info',
                    plugin='world_info::DetectBabyface',
                    name='babyface_node',
                    parameters=[{'babyface_confidence_threshold': 0.9},
                                {'inference_mode': "GPU"}],
                    remappings=[('/image_rect', '/Spot/kinect_color')],
                ),
                # Images from webots aren't synchronised, work with this later if required*. for other tags temporary hax
                # ComposableNode(
                #     package='apriltag_ros',
                #     plugin='AprilTagNode',
                #     name='apriltag_node',
                #     remappings=[('/image_rect', '/Spot/kinect_color'),
                #                 ('/camera_info', '/Spot/kinect_color/camera_info')],
                # ),
            ],
            output='both',
    )

    world_info_node = Node(
        package='world_info',
        executable='world_info',
        output='both',
    )

    return LaunchDescription([container, world_info_node])
