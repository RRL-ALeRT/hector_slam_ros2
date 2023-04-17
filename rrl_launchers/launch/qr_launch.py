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
                    plugin='world_info::DetectQR',
                    name='rs_back',
                    parameters=[{'qr_square_length': 0.066},
                                {'camera_frame': 'rs_back'},
                                {'K1': 618.9777221679688},
                                {'K2': 330.1564025878906},
                                {'K3': 619.1235961914062},
                                {'K4': 236.6548614501953}],
                    remappings=[('/image_rect', '/rs_back/color/image_raw'),
                                ('/qr_detected', '/rs_back/qr_detected')],
                ),
                ComposableNode(
                    package='world_info',
                    plugin='world_info::DetectQR',
                    name='rs_left',
                    parameters=[{'qr_square_length': 0.066},
                                {'camera_frame': 'rs_left'},
                                {'K1': 614.178955078125},
                                {'K2': 321.61285400390625},
                                {'K3': 614.341796875},
                                {'K4': 247.35923767089844}],
                    remappings=[('/image_rect', '/rs_left/color/image_raw'),
                                ('/qr_detected', '/rs_left/qr_detected')],
                ),
                ComposableNode(
                    package='world_info',
                    plugin='world_info::DetectQR',
                    name='rs_right',
                    parameters=[{'qr_square_length': 0.066},
                                {'camera_frame': 'rs_right'},
                                {'K1': 615.2658081054688},
                                {'K2': 323.0284118652344},
                                {'K3': 615.265869140625},
                                {'K4': 240.1706085205078}],
                    remappings=[('/image_rect', '/rs_right/color/image_raw'),
                                ('/qr_detected', '/rs_right/qr_detected')],
                ),
            ],
            output='both',
    )

    world_info_node = Node(
        package='world_info',
        executable='world_info',
        output='both',
    )

    return LaunchDescription([container])#, world_info_node])
