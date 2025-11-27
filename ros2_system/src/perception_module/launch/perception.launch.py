#!/usr/bin/env python3
"""
Launch file for perception module
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
            FindPackageShare('perception_module'),
            'config',
            'perception.yaml'
        ]),
        description='Path to perception config file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level'
    )

    log_level = LaunchConfiguration('log_level')

    # RGBD Camera Node
    rgbd_camera_node = Node(
        package='perception_module',
        executable='rgbd_camera_node',
        name='rgbd_camera_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True
    )

    # OpenCV Processor Node
    opencv_processor_node = Node(
        package='perception_module',
        executable='opencv_processor_node',
        name='opencv_processor_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True
    )

    # PointCloud Processor Node
    pointcloud_processor_node = Node(
        package='perception_module',
        executable='pointcloud_processor_node',
        name='pointcloud_processor_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        arguments=['--ros-args', '--log-level', log_level],
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        use_sim_time_arg,
        log_level_arg,
        rgbd_camera_node,
        opencv_processor_node,
        pointcloud_processor_node
    ])
