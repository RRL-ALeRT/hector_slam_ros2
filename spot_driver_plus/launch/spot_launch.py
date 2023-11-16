import launch
import launch_ros
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import DeclareLaunchArgument
from ament_index_python.packages import get_package_share_directory
import os
from launch.actions import ExecuteProcess


def generate_launch_description():
    spot_plus = LaunchDescription()

    # Declare the 'config_file' argument
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(get_package_share_directory('spot_driver_plus'), 'config', 'spot_params.yaml'),
        description='Path to the config file'
    )
    spot_plus.add_action(config_file_arg)

    os.environ["SPOT_URDF_EXTRAS"] = os.path.join(get_package_share_directory('spot_driver_plus'), 'urdf', 'gen3.urdf.xacro')
    # Include and launch the child launch file
    child_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(get_package_share_directory('spot_driver'), 'launch', 'spot_driver.launch.py')),
        launch_arguments={'config_file': launch.substitutions.LaunchConfiguration('config_file')}.items()
    )
    spot_plus.add_action(child_launch)

    pcl_combined = launch_ros.actions.Node(
                        package='spot_driver_plus',
                        executable='get_pcl.py',
                        output='screen',
                    )
    #spot_plus.add_action(pcl_combined)

    battery_screen = launch_ros.actions.Node(
                        package='spot_driver_plus',
                        executable='battery_screen',
                        output='screen',
                    )
    spot_plus.add_action(battery_screen)

    map_vision_node = launch_ros.actions.Node(
        package='spot_driver_plus',
        executable='map_vision',
        output='screen',
    )
    spot_plus.add_action(map_vision_node)
    #map_vision_tf2 = launch_ros.actions.Node(
    #    package='tf2_ros',
    #    executable='static_transform_publisher',
    #    output='screen',
    #    arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'map', 'vision'],
    #)
    #spot_plus.add_action(map_vision_tf2)

    back_image = ExecuteProcess(
                        cmd=['ros2', 'run', 'spot_cpp_ros2', 'get_image', 'back'],
                        output='screen'
                    )
    # spot_plus.add_action(back_image)

    return spot_plus
