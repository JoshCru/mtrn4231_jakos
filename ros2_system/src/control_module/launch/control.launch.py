#!/usr/bin/env python3
"""
Launch file for control module
"""
from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import DeclareLaunchArgument, RegisterEventHandler
from launch.event_handlers import OnProcessExit
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition


def generate_launch_description():
    # Declare arguments
    config_file_arg = DeclareLaunchArgument(
        'config_file',
        default_value=PathJoinSubstitution([
            FindPackageShare('control_module'),
            'config',
            'control.yaml'
        ]),
        description='Path to control config file'
    )

    # Robot driver node (lifecycle)
    robot_driver_node = LifecycleNode(
        package='control_module',
        executable='robot_driver_node',
        name='robot_driver_node',
        namespace='',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True
    )

    # Pick operation node
    pick_operation_node = Node(
        package='control_module',
        executable='pick_operation_node',
        name='pick_operation_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True
    )

    # Place operation node
    place_operation_node = Node(
        package='control_module',
        executable='place_operation_node',
        name='place_operation_node',
        output='screen',
        parameters=[LaunchConfiguration('config_file')],
        emulate_tty=True
    )

    return LaunchDescription([
        config_file_arg,
        robot_driver_node,
        pick_operation_node,
        place_operation_node
    ])
