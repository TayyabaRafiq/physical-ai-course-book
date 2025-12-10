from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ros2_package_boilerplate',
            executable='talker_node',
            name='sim_talker',
            output='screen'
        ),
        Node(
            package='ros2_package_boilerplate',
            executable='listener_node',
            name='sim_listener',
            output='screen'
        )
    ])