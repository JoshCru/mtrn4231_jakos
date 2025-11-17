#!/usr/bin/env python3
"""
Launch file for weight detection module
"""
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    # Weight detector node
    weight_detector_node = Node(
        package='weight_detection_module',
        executable='weight_detector',
        name='weight_detector',
        namespace='',
        output='screen',
        emulate_tty=True
    )

    return LaunchDescription([
        weight_detector_node
    ])
