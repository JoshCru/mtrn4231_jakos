#!/usr/bin/env python3
"""
Launch file for UR5e robot with RealSense perception and motion planning in RViz
This launch file combines:
- RealSense depth camera with pointcloud
- Hand-eye calibration transform
- Perception module for object detection
- MoveIt motion planning with RViz
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    # Launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.0.100',
        description='IP address of the UR robot'
    )

    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur5e',
        description='Type of UR robot'
    )

    # 1. Hand-eye calibration - Static transform from base_link to camera_link
    # These values come from your calibration: 4_calib_camera_pose.launch
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

    # 4. UR robot driver (delayed to allow transforms to be set up)
    ur_control_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ur_robot_driver'),
                        'launch',
                        'ur_control.launch.py'
                    ])
                ]),
                launch_arguments={
                    'ur_type': LaunchConfiguration('ur_type'),
                    'robot_ip': LaunchConfiguration('robot_ip'),
                    'use_fake_hardware': 'false',
                    'launch_rviz': 'false',
                }.items()
            )
        ]
    )

    # 5. MoveIt with RViz (delayed to allow robot driver to initialize)
    moveit_launch = TimerAction(
        period=12.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('ur_moveit_config'),
                        'launch',
                        'ur_moveit.launch.py'
                    ])
                ]),
                launch_arguments={
                    'ur_type': LaunchConfiguration('ur_type'),
                    'robot_ip': LaunchConfiguration('robot_ip'),
                    'launch_rviz': 'true',
                }.items()
            )
        ]
    )

    return LaunchDescription([
        robot_ip_arg,
        ur_type_arg,
        camera_transform,
        realsense_launch,
        perception_launch,
        ur_control_launch,
        moveit_launch,
    ])
