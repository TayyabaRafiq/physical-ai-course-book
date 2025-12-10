#!/usr/bin/env python3
"""
Launch file for all VLA action servers.

Starts all action servers needed for VLA robot control:
- PickObject server
- PlaceObject server
- NavigateToPoint server
- InspectObject server
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description with all VLA action servers."""

    # PickObject action server
    pick_server = Node(
        package='vla_action_servers',
        executable='pick_object_server',
        name='pick_object_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False,
        }]
    )

    # PlaceObject action server
    place_server = Node(
        package='vla_action_servers',
        executable='place_object_server',
        name='place_object_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False,
        }]
    )

    # NavigateToPoint action server
    navigate_server = Node(
        package='vla_action_servers',
        executable='navigate_to_point_server',
        name='navigate_to_point_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False,
        }]
    )

    # InspectObject action server
    inspect_server = Node(
        package='vla_action_servers',
        executable='inspect_object_server',
        name='inspect_object_server',
        output='screen',
        emulate_tty=True,
        parameters=[{
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        pick_server,
        place_server,
        navigate_server,
        inspect_server,
    ])
