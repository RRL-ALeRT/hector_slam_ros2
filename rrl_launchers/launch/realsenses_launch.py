# Follow and build from source, compulsory!
# https://github.com/IntelRealSense/librealsense/blob/master/doc/installation_jetson.md#building-from-source-using-rsusb-backend
# Refer
# https://dev.intelrealsense.com/docs/multiple-depth-cameras-configuration?_ga=2.7010101.649757684.1676046093-271033209.1676046093

import launch
import launch_ros.actions

def delayed_node_action(package, executable, namespace, parameters, respawn=True, delay=0):
    return launch.actions.TimerAction(
        period=delay,
        actions=[
            launch_ros.actions.Node(
                package=package,
                executable=executable,
                namespace=namespace,
                parameters=parameters,
                respawn=respawn,
                remappings=[(f"/{namespace}/depth/color/points", "/velodyne_points")]
            )
        ]
    )

def generate_launch_description():
    # Define the delay duration
    delay_duration = 3  # in seconds

    rs_front_params = [{
        "serial_no": "_825312073924",
        "depth_module.profile": "848x480x30",
        "rgb_camera.profile": "640x480x30",
        "enable_color": True,
        "align_depth.enable": True,
        "enable_infra1": False,
        "enable_infra2": False,
        "infra_rgb": False,
        "camera_name": "rs_front",
        "pointcloud.enable": True,
        "publish_tf": False
    }]
    rs_left_params = [{
        "serial_no": "_827112070053",
        "depth_module.profile": "848x480x15",
        "rgb_camera.profile": "640x480x15",
        "enable_color": True,
        "align_depth.enable": True,
        "enable_infra1": False,
        "enable_infra2": False,
        "infra_rgb": False,
        "camera_name": "rs_left",
        "pointcloud.enable": True,
        "publish_tf": False
    }]
    rs_right_params = [{
        "serial_no": "_825312070259",
        "depth_module.profile": "848x480x15",
        "rgb_camera.profile": "640x480x15",
        "enable_color": True,
        "align_depth.enable": True,
        "enable_infra1": False,
        "enable_infra2": False,
        "infra_rgb": False,
        "camera_name": "rs_right",
        "pointcloud.enable": True,
        "publish_tf": False
    }]
    
    rs_front_action = delayed_node_action(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="rs_front",
        parameters=rs_front_params,
        respawn=True,
        delay=float(delay_duration)
    )
    rs_left_action = delayed_node_action(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="rs_left",
        parameters=rs_left_params,
        respawn=True,
        delay=float(delay_duration * 2)
    )
    rs_right_action = delayed_node_action(
        package="realsense2_camera",
        executable="realsense2_camera_node",
        namespace="rs_right",
        parameters=rs_right_params,
        respawn=True,
        delay=float(delay_duration * 3)
    )

    tf_rs_front = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.396', '0.0325', '0.155', '-0.5', '0.5', '-0.5', '0.5', 'body', 'rs_front_color_optical_frame'],
    )

    tf_rs_left = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['-0.175', '0.1159', '0.39', '0.001', '-0.2311465', '0.6462265', '0.7144206', 'body', 'rs_left_color_optical_frame'],
    )

    tf_rs_right = launch_ros.actions.Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['-0.111', '-0.1156', '0.139', '0.1830127', '0.1830127', '0.6830127', '-0.6830127', 'body', 'rs_right_color_optical_frame'],
    )

    return launch.LaunchDescription([
        rs_front_action,
        # rs_left_action,
        # rs_right_action,
        tf_rs_front,
        # tf_rs_left,
        # tf_rs_right,
    ])
