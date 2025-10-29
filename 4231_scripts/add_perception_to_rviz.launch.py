#!/usr/bin/env python3
"""
Launch file to add RealSense perception to existing UR5e setup
Run this AFTER you've started the robot with setupRealur5e.sh
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # 1. Hand-eye calibration - Static transform from base_link to camera_link
    camera_transform = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_link_broadcaster',
        arguments=[
            '1.27677', '0.0175114', '0.673798',  # x, y, z
            '-0.414096', '-0.019425', '0.910018', '0.00376407',  # qx, qy, qz, qw
            'base_link', 'camera_link'
        ],
        output='screen'
    )

    # 2. RealSense camera launch
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

    # 3. Perception module launch (delayed to allow camera to start)
    # Note: Only include if perception_module is built
    try:
        perception_launch = TimerAction(
            period=5.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource([
                        PathJoinSubstitution([
                            FindPackageShare('perception_module'),
                            'launch',
                            'perception.launch.py'
                        ])
                    ])
                )
            ]
        )
        has_perception = True
    except:
        has_perception = False
        perception_launch = None

    ld = LaunchDescription([
        camera_transform,
        realsense_launch,
    ])

    if has_perception and perception_launch:
        ld.add_action(perception_launch)

    return ld
