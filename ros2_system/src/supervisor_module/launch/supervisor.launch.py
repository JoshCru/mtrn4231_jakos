#!/usr/bin/env python3
"""
Launch file for supervisor module
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
            FindPackageShare('supervisor_module'),
            'config',
            'system_controller.yaml'
        ]),
        description='Path to system controller config file'
    )

    # System controller node
    system_controller_node = Node(
        package='supervisor_module',
        executable='system_controller_node',
        name='system_controller_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        system_controller_node
    ])
