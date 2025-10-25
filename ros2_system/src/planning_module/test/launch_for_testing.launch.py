#!/usr/bin/env python3
"""
Launch file for running planning module nodes during testing.
This is a simplified version of the main launch file for test purposes.
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """Generate launch description for testing"""

    # Get package directory
    pkg_dir = get_package_share_directory('planning_module')
    config_file = os.path.join(pkg_dir, 'config', 'planning.yaml')

    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Node configurations
    sort_node = Node(
        package='planning_module',
        executable='sort_node',
        name='sort_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True
    )

    verification_node = Node(
        package='planning_module',
        executable='verification_node',
        name='verification_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True
    )

    integrity_node = Node(
        package='planning_module',
        executable='integrity_node',
        name='integrity_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True
    )

    moveit2_interface_node = Node(
        package='planning_module',
        executable='moveit2_interface_node',
        name='moveit2_interface_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True
    )

    return LaunchDescription([
        use_sim_time_arg,
        sort_node,
        verification_node,
        integrity_node,
        moveit2_interface_node,
    ])
