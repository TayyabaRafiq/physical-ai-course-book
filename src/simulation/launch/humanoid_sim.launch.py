"""
ROS 2 Launch File for VLA Integration Simulation
Starts Gazebo with household world and humanoid robot
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation (Gazebo) clock if true'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo GUI'
    )

    world_file_arg = DeclareLaunchArgument(
        'world',
        default_value='household_world.sdf',
        description='Gazebo world file name (in src/simulation/worlds/)'
    )

    # Get paths
    # Note: In actual deployment, these would use FindPackageShare
    # For now, using relative paths from project root
    project_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '../../..'))
    world_file = os.path.join(project_root, 'src/simulation/worlds', LaunchConfiguration('world'))
    robot_description_file = os.path.join(project_root, 'src/simulation/robot_description/humanoid.urdf')

    # Read robot description
    with open(robot_description_file, 'r') as f:
        robot_desc = f.read()

    # Gazebo simulation server
    gazebo_server = ExecuteProcess(
        cmd=['gz', 'sim', '-r', '-s', world_file],
        output='screen',
        condition=None  # Always run server
    )

    # Gazebo GUI (conditional on 'gui' argument)
    gazebo_gui = ExecuteProcess(
        cmd=['gz', 'sim', '-g'],
        output='screen',
        condition=None  # TODO: Add condition based on 'gui' arg
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'robot_description': robot_desc
        }]
    )

    # ROS-Gazebo Bridge
    # Bridge /clock topic for time synchronization
    clock_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        arguments=['/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock'],
        output='screen'
    )

    # Joint State Publisher (for visualization)
    joint_state_publisher = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        name='joint_state_publisher',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }]
    )

    # Spawn robot in Gazebo
    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        arguments=[
            '-name', 'humanoid_robot',
            '-topic', '/robot_description',
            '-x', '0.0',
            '-y', '0.0',
            '-z', '0.1'
        ],
        output='screen'
    )

    return LaunchDescription([
        # Arguments
        use_sim_time_arg,
        gui_arg,
        world_file_arg,

        # Nodes and processes
        gazebo_server,
        gazebo_gui,
        robot_state_publisher,
        clock_bridge,
        joint_state_publisher,
        spawn_robot,
    ])
