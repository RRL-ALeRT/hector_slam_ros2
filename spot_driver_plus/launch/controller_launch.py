import launch_ros
from launch import LaunchDescription


def generate_launch_description():
    spot_controller = LaunchDescription()

    spot_kinova_controller = launch_ros.actions.Node(
                        package='spot_driver_plus',
                        executable='spot_kinova_controller',
                        output='screen',
                    )
    spot_controller.add_action(spot_kinova_controller)

    joy_node = launch_ros.actions.Node(
                        package='joy',
                        executable='joy_node',
                        output='screen',
                    )
    spot_controller.add_action(joy_node)

    return spot_controller
