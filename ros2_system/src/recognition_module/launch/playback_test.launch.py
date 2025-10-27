#!/usr/bin/env python3
"""
Launch file for testing recognition node with recorded bag files
Plays back a bag file and runs the recognition node
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    LogInfo,
)
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directory
    pkg_share = get_package_share_directory('recognition_module')
    config_file = os.path.join(pkg_share, 'config', 'recognition.yaml')

    # Fallback to source directory during development
    if not os.path.exists(config_file):
        pkg_share = str(Path(__file__).parent.parent)
        config_file = os.path.join(pkg_share, 'config', 'recognition.yaml')

    # Default bag directory
    default_bag_dir = str(Path.home() / 'Documents' / 'mtrn4231_jakos' / 'test_bags')

    # Launch arguments
    bag_file_arg = DeclareLaunchArgument(
        'bag_file',
        default_value='',
        description='Path to bag file (or directory) to play'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time from bag'
    )

    rate_arg = DeclareLaunchArgument(
        'rate',
        default_value='1.0',
        description='Playback rate multiplier'
    )

    loop_arg = DeclareLaunchArgument(
        'loop',
        default_value='false',
        description='Loop playback'
    )

    start_paused_arg = DeclareLaunchArgument(
        'start_paused',
        default_value='false',
        description='Start playback paused'
    )

    run_recognition_arg = DeclareLaunchArgument(
        'run_recognition',
        default_value='true',
        description='Run recognition node alongside playback'
    )

    # Bag playback
    bag_play = ExecuteProcess(
        cmd=[
            'ros2', 'bag', 'play',
            LaunchConfiguration('bag_file'),
            '--rate', LaunchConfiguration('rate'),
            '--loop', LaunchConfiguration('loop'),
            '--start-paused', LaunchConfiguration('start_paused'),
            '--clock'
        ],
        output='screen'
    )

    # Recognition node
    recognition_node = Node(
        package='recognition_module',
        executable='recognition_node',
        name='recognition_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        condition=IfCondition(LaunchConfiguration('run_recognition'))
    )

    # Build launch description
    ld = LaunchDescription()

    # Add arguments
    ld.add_action(bag_file_arg)
    ld.add_action(use_sim_time_arg)
    ld.add_action(rate_arg)
    ld.add_action(loop_arg)
    ld.add_action(start_paused_arg)
    ld.add_action(run_recognition_arg)

    # Info message
    ld.add_action(LogInfo(msg='Starting bag playback with recognition node'))

    # Start nodes
    ld.add_action(bag_play)
    ld.add_action(recognition_node)

    return ld
