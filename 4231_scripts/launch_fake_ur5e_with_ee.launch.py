#!/usr/bin/env python3
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


def generate_launch_description():
    # Paths
    ur_robot_driver_launch = PathJoinSubstitution([
        FindPackageShare('ur_robot_driver'),
        'launch',
        'ur_control.launch.py'
    ])

    ur_moveit_launch = PathJoinSubstitution([
        FindPackageShare('ur_moveit_config'),
        'launch',
        'ur_moveit.launch.py'
    ])

    # Launch ur_control with custom URDF
    ur_control = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(ur_robot_driver_launch),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': 'yyy.yyy.yyy.yyy',
            'initial_joint_controller': 'joint_trajectory_controller',
            'use_fake_hardware': 'true',
            'launch_rviz': 'false',
            'description_file': 'ur5e_with_end_effector.urdf.xacro',
            'description_package': 'motion_control_module',
        }.items()
    )

    # Launch moveit with delay and custom URDF
    ur_moveit = TimerAction(
        period=5.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(ur_moveit_launch),
                launch_arguments={
                    'ur_type': 'ur5e',
                    'launch_rviz': 'true',
                    'use_fake_hardware': 'true',
                    'description_file': 'ur5e_with_end_effector.urdf.xacro',
                    'description_package': 'motion_control_module',
                }.items()
            )
        ]
    )

    return LaunchDescription([
        ur_control,
        ur_moveit,
    ])
