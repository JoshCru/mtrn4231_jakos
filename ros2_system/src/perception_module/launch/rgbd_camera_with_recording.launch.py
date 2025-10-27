#!/usr/bin/env python3
"""
Launch file for RGBD camera node with bag recording enabled
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from datetime import datetime


def generate_launch_description():
    # Generate default bag path with timestamp
    timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
    default_bag_path = os.path.join(
        os.path.expanduser("~"),
        "Documents",
        "mtrn4231_jakos",
        "test_bags",
        f"rgbd_camera_{timestamp}"
    )

    # Declare arguments
    enable_recording_arg = DeclareLaunchArgument(
        'enable_recording',
        default_value='true',
        description='Enable ROS bag recording'
    )

    bag_path_arg = DeclareLaunchArgument(
        'bag_path',
        default_value=default_bag_path,
        description='Path to save ROS bag'
    )

    publish_rate_arg = DeclareLaunchArgument(
        'publish_rate',
        default_value='30.0',
        description='Camera publish rate in Hz'
    )

    image_width_arg = DeclareLaunchArgument(
        'image_width',
        default_value='640',
        description='Image width in pixels'
    )

    image_height_arg = DeclareLaunchArgument(
        'image_height',
        default_value='480',
        description='Image height in pixels'
    )

    # RGBD Camera Node
    rgbd_camera_node = Node(
        package='perception_module',
        executable='rgbd_camera_node',
        name='rgbd_camera_node',
        output='screen',
        parameters=[{
            'enable_recording': LaunchConfiguration('enable_recording'),
            'bag_path': LaunchConfiguration('bag_path'),
            'publish_rate': LaunchConfiguration('publish_rate'),
            'image_width': LaunchConfiguration('image_width'),
            'image_height': LaunchConfiguration('image_height'),
        }],
        emulate_tty=True
    )

    return LaunchDescription([
        enable_recording_arg,
        bag_path_arg,
        publish_rate_arg,
        image_width_arg,
        image_height_arg,
        rgbd_camera_node,
    ])
