#!/usr/bin/env python3
"""
Launch file to add RealSense camera with pointcloud to existing UR5e setup
This version does NOT require perception_module to be built
Run this AFTER you've started the robot and MoveIt
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 1. Hand-eye calibration - Static transform from base_link to camera_link
    # These values come from your calibration
    camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_broadcaster',
        arguments=[
            '1.27677', '0.0175114', '0.673798',  # x, y, z (translation)
            '-0.414096', '-0.019425', '0.910018', '0.00376407',  # qx, qy, qz, qw (rotation)
            'base_link', 'camera_link'
        ],
        output='screen'
    )

    # 2. RealSense camera launch with pointcloud enabled
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'depth_module.profile': '640x480x30',
            'pointcloud.enable': 'true',
            'align_depth.enable': 'true',
            'camera_name': 'camera',
            'camera_namespace': 'camera',
        }.items()
    )

    return LaunchDescription([
        camera_transform,
        realsense_launch,
    ])
