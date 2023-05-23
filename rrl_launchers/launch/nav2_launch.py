import os
import launch
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription

def generate_launch_description():
    package_dir = get_package_share_directory('rrl_launchers')
    use_sim_time = LaunchConfiguration('use_sim_time', default=False)

    nav2_params = os.path.join(package_dir, 'params', 'nav2_params.yaml')

    rviz_config = os.path.join(package_dir, 'resource', 'nav2.rviz')
    rviz = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['--display-config=' + rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
    )

    nav2 = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(
            get_package_share_directory('nav2_bringup'), 'launch', 'bringup_launch.py')),
        launch_arguments=[
            ('map', '/map'),
            ('use_sim_time', use_sim_time),
            ('params_file', nav2_params),
        ],
    )

    return LaunchDescription([
        # rviz,
        nav2,
    ])