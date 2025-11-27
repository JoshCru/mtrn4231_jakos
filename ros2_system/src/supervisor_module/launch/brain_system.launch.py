#!/usr/bin/env python3
"""
Launch file for Brain Node system - Central Orchestrator with Dashboard
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for brain node system"""

    # Declare launch arguments
    dashboard_arg = DeclareLaunchArgument(
        'launch_dashboard',
        default_value='true',
        description='Whether to launch the Brain Dashboard UI'
    )

    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Whether to auto-start the system'
    )

    # Brain Node (C++)
    brain_node = Node(
        package='supervisor_module',
        executable='brain_node',
        name='brain_node',
        output='screen',
        parameters=[{
            'discovery_rate': 0.5,
            'health_check_rate': 1.0,
            'status_publish_rate': 2.0,
            'metrics_publish_rate': 0.2,
            'max_history_size': 100,
            'max_event_history': 1000,
        }],
        emulate_tty=True,
    )

    # System Controller Node (C++)
    system_controller_node = Node(
        package='supervisor_module',
        executable='system_controller_node',
        name='system_controller_node',
        output='screen',
        parameters=[{
            'num_target_areas': 3,
            'auto_start': LaunchConfiguration('auto_start'),
            'publish_rate': 1.0,
        }],
        emulate_tty=True,
    )

    # Brain Dashboard (Python with PyQt5) - launched with delay
    brain_dashboard = ExecuteProcess(
        cmd=['ros2', 'run', 'supervisor_module', 'brain_dashboard'],
        output='screen',
        shell=False,
    )

    # Delayed dashboard launch to ensure nodes are up
    delayed_dashboard = TimerAction(
        period=2.0,
        actions=[brain_dashboard]
    )

    return LaunchDescription([
        dashboard_arg,
        auto_start_arg,
        brain_node,
        system_controller_node,
        delayed_dashboard,
    ])
