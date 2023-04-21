# Follow and build from source, compulsory!
# https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md#building-from-source-using-rsusb-backend
# Refer
# https://dev.intelrealsense.com/docs/multiple-depth-cameras-configuration?_ga=2.7010101.649757684.1676046093-271033209.1676046093

import launch
import launch_ros.actions
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    rs_front = launch_ros.actions.Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="rs_front",
        parameters=[{"serial_no": "_826212070098"},
        {"depth_module.profile": "848x480x15"},
        {"rgb_camera.profile": "640x480x15"},
        {"enable_infra1": False},
        {"enable_infra2": False},
        {"infra_rgb": False},
        {"align_depth.enable": True}]
    )

    rs_left = launch_ros.actions.Node(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="rs_left",
        parameters=[{"serial_no": "_828112074248"},
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

    return launch.LaunchDescription([
        rs_front,
        rs_left,
        rs_right,
    ])
