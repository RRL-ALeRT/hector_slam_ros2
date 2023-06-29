# Follow and build from source, compulsory!
# https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md#building-from-source-using-rsusb-backend
# Refer
# https://dev.intelrealsense.com/docs/multiple-depth-cameras-configuration?_ga=2.7010101.649757684.1676046093-271033209.1676046093

import launch
import launch_ros.actions
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    rs_front = launch_ros.actions.Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="rs_front",
        parameters=[{"serial_no": "_825312073924"}, #{"serial_no": "_826212070098"},
        {"depth_module.profile": "848x480x15"},
        {"rgb_camera.profile": "640x480x15"},
        {"enable_infra1": False},
        {"enable_infra2": False},
        {"infra_rgb": False},
        {"pointcloud.enable": True},
        {"align_depth.enable": True}]
    )
# {'name': 'pointcloud.stream_filter',     'default': '2', 'description': 'texture stream for pointcloud'},
# {'name': 'pointcloud.stream_index_filter','default': '0', 'description': 'texture stream index for pointcloud'},

    rs_left = launch_ros.actions.Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="rs_left",
        parameters=[{"serial_no": "_827112070053"},
        {"depth_module.profile": "848x480x15"},
        {"rgb_camera.profile": "640x480x15"},
        {"enable_infra1": False},
        {"enable_infra2": False},
        {"infra_rgb": False},
        {"align_depth.enable": True}]
    )

    rs_right = launch_ros.actions.Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="rs_right",
        parameters=[{"serial_no": "_825312070259"},
        {"depth_module.profile": "848x480x15"},
        {"rgb_camera.profile": "640x480x15"},
        {"enable_infra1": False},
        {"enable_infra2": False},
        {"infra_rgb": False},
        {"align_depth.enable": True}]
    )

    tf_rs_front = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        # arguments=['-1.43', '0.0325', '0.156', '0.0', '0.0', '0.0', '1.0', 'body', 'camera_link'],
        arguments=['0.23', '0.0325', '0.156', '0.0', '0.0', '0.0', '1.0', 'body', 'camera_link'],
        # arguments=['0.583', '0.0325', '0.156', '0.0', '0.005', '0.0', '1.0', 'body', 'camera_link'], #120.0
    )

    return launch.LaunchDescription([
        rs_front,
        rs_left,
        rs_right,
        #tf_rs_front,
    ])
