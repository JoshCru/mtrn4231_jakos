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

    # Weight estimation node
    weight_estimation_node = Node(
        package='recognition_module',
        executable='weight_estimation_node',
        name='weight_estimation_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        weight_estimation_node
    ])
