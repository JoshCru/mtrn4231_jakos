#!/usr/bin/env python3
"""
Launch file for recognition module with mock camera (for testing/simulation)
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

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock if true'
    )

    # Mock camera node (simulates Lenovo 510 RGBD)
    mock_camera_node = Node(
        package='recognition_module',
        executable='mock_camera_node',
        name='mock_camera_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True
    )

    # Recognition node
    recognition_node = Node(
        package='recognition_module',
        executable='recognition_node',
        name='recognition_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        mock_camera_node,
        recognition_node
    ])
