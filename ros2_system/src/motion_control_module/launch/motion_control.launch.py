#!/usr/bin/env python3
"""
Launch file for motion control module
"""
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('motion_control_module'),
            'config',
            'motion_control.yaml'
        ]),
        description='Path to motion control config file'
    )

    # Gripper controller node (lifecycle)
    gripper_controller_node = LifecycleNode(
        package='motion_control_module',
        executable='gripper_controller_node',
        name='gripper_controller_node',
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        gripper_controller_node
    ])
