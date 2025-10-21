#!/usr/bin/env python3
"""
Launch file for planning module
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('planning_module'),
            'config',
            'planning.yaml'
        ]),
        description='Path to planning config file'
    )

    # Sort node
    sort_node = Node(
        package='planning_module',
        executable='sort_node',
        name='sort_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True
    )

    # Verification node
    verification_node = Node(
        package='planning_module',
        executable='verification_node',
        name='verification_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True
    )

    # Integrity node
    integrity_node = Node(
        package='planning_module',
        executable='integrity_node',
        name='integrity_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True
    )

    # MoveIt2 interface node
    moveit2_interface_node = Node(
        package='planning_module',
        executable='moveit2_interface_node',
        name='moveit2_interface_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        sort_node,
        verification_node,
        integrity_node,
        moveit2_interface_node
    ])
