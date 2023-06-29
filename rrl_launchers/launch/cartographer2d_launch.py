# Copyright 2019 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
#
# Author: Darby Lim

import os
# import xacro
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir


def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')
    package_dir = get_package_share_directory('rrl_launchers')
    
    # urdf_dir = os.path.join(package_dir, 'urdf')
    # urdf_file = os.path.join(urdf_dir, 'spot.urdf.xacro')
    # doc = xacro.process_file(urdf_file)
    # robot_desc = doc.toprettyxml(indent='  ')
    
    cartographer_config_dir = package_dir + '/params'
    print(cartographer_config_dir)
    configuration_basename = LaunchConfiguration('configuration_basename',
                                                 default='cartographer2d.lua')

    resolution = LaunchConfiguration('resolution', default='0.05')
    publish_period_sec = LaunchConfiguration('publish_period_sec', default='1.0')

    # rviz_config_dir = os.path.join(get_package_share_directory('turtlebot3_cartographer'),
    #                                'rviz', 'tb3_cartographer.rviz')
	
    map_dad_vision = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['0.0', '0.0', '0.0', '0.0', '0.0', '0.0', '1.0', 'map', 'dad_vision'],
    )

    topic_remapping_imu = Node(
        package='my_package',
        executable='my_node',
        output='screen',
    )

    body_head = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        output='screen',
        arguments=['−0.208066478', '−0.01821628', '0.004368715', '0.0', '0.0', '0.0', '1.0', 'body', 'head'],
    )
    
    dad_vision = Node(
            package='rrl_launchers',
            executable='dad_vision',
            name='dad_vision',
            output='screen')
    # robot_state_publisher_node= Node(
    #         package='robot_state_publisher',
    #         executable='robot_state_publisher',
    #         parameters=[
    #             {'robot_decription':robot_desc},
    #             {'use_sim_time': LaunchConfiguration(use_sim_time)}],
    #         output='screen'),

    return LaunchDescription([
        DeclareLaunchArgument(
            'configuration_basename',
            default_value=configuration_basename,
            description='Name of lua file for cartographer'),
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',
            description='Use simulation (Gazebo) clock if true'),

        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{'use_sim_time': use_sim_time}],
            remappings=[("/points2", "/velodyne_points"),
                        # ("/odom", "/odometry"),
                        # ("/imu", "/imu_data"),
                        ],
            arguments=['-configuration_directory', cartographer_config_dir,
                       '-configuration_basename', configuration_basename],
            ),

        DeclareLaunchArgument(
            'resolution',
            default_value=resolution,
            description='Resolution of a grid cell in the published occupancy grid'),

        DeclareLaunchArgument(
            'publish_period_sec',
            default_value=publish_period_sec,
            description='OccupancyGrid publishing period'),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource([ThisLaunchFileDir(), '/occupancy_grid_launch.py']),
            launch_arguments={'use_sim_time': use_sim_time, 'resolution': resolution,
                              'publish_period_sec': publish_period_sec}.items(),
        ),
        
        # IncludeLaunchDescription(
        #     PythonLaunchDescriptionSource([os.path.join(
        #  get_package_share_directory('bluespace_ai_xsens_ros_mti_driver'), 'launch'),
        #  '/xsens_mti_node.launch.py'])),
        
        # map_dad_vision,
        body_head,
        topic_remapping_imu,
        # dad_vision,
        # robot_state_publisher_node,
        # Node(
        #     package='rviz2',
        #     executable='rviz2',
        #     name='rviz2',
        #     arguments=['-d', rviz_config_dir],
        #     parameters=[{'use_sim_time': use_sim_time}],
        #     output='screen'),
    ])
