#!/usr/bin/env python3
"""
Launch file for recognition module
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
            FindPackageShare('recognition_module'),
            'config',
            'recognition.yaml'
        ]),
        description='Path to recognition config file'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )

    log_level = LaunchConfiguration('log_level')

    # Recognition node (for real camera)
    recognition_node = Node(
        package='recognition_module',
        executable='recognition_node',
        name='recognition_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        log_level_arg,
        recognition_node
    ])
