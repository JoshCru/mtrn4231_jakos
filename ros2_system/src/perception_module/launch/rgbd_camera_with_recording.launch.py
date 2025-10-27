#!/usr/bin/env python3
"""
Launch file for RGBD camera node with bag recording enabled

This launch file starts the rgbd_camera_node which subscribes to
RealSense camera topics and optionally records them to a ROS bag.
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

    color_topic_arg = DeclareLaunchArgument(
        'color_topic',
        default_value='/camera/color/image_raw',
        description='Topic name for color/RGB images'
    )

    depth_topic_arg = DeclareLaunchArgument(
        'depth_topic',
        default_value='/camera/depth/image_raw',
        description='Topic name for depth images'
    )

    color_info_topic_arg = DeclareLaunchArgument(
        'color_info_topic',
        default_value='/camera/color/camera_info',
        description='Topic name for color camera info'
    )

    depth_info_topic_arg = DeclareLaunchArgument(
        'depth_info_topic',
        default_value='/camera/depth/camera_info',
        description='Topic name for depth camera info'
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
            'color_topic': LaunchConfiguration('color_topic'),
            'depth_topic': LaunchConfiguration('depth_topic'),
            'color_info_topic': LaunchConfiguration('color_info_topic'),
            'depth_info_topic': LaunchConfiguration('depth_info_topic'),
        }],
        emulate_tty=True
    )

    return LaunchDescription([
        enable_recording_arg,
        bag_path_arg,
        color_topic_arg,
        depth_topic_arg,
        color_info_topic_arg,
        depth_info_topic_arg,
        rgbd_camera_node,
    ])
