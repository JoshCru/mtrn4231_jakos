#!/usr/bin/env python3
"""
Master launch file for UR5e with gripper - combines driver and MoveIt
This replaces the setupRealur5eSystemVisualisation.sh script
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, TimerAction, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # Declare arguments
    declared_arguments = []

    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.0.100",
            description="IP address of the UR robot",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "ur_type",
            default_value="ur5e",
            description="Type/series of used UR robot.",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "launch_rviz",
            default_value="true",
            description="Launch RViz with MoveIt?",
        )
    )

    declared_arguments.append(
        DeclareLaunchArgument(
            "use_fake_hardware",
            default_value="false",
            description="Use fake hardware for testing?",
        )
    )

    # Initialize Arguments
    robot_ip = LaunchConfiguration("robot_ip")
    ur_type = LaunchConfiguration("ur_type")
    launch_rviz = LaunchConfiguration("launch_rviz")
    use_fake_hardware = LaunchConfiguration("use_fake_hardware")

    # Config file paths - use defaults from ur_description
    joint_limit_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "joint_limits.yaml"]
    )
    kinematics_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "default_kinematics.yaml"]
    )
    physical_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "physical_parameters.yaml"]
    )
    visual_params = PathJoinSubstitution(
        [FindPackageShare("ur_description"), "config", ur_type, "visual_parameters.yaml"]
    )

    # 1. Launch UR Robot Driver with custom description (gripper included)
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ur_robot_driver"),
                "launch",
                "ur_control.launch.py"
            ])
        ),
        launch_arguments={
            "ur_type": ur_type,
            "robot_ip": robot_ip,
            "use_fake_hardware": use_fake_hardware,
            "launch_rviz": "false",  # Don't launch RViz from driver
            "description_file": "ur5e_with_end_effector.urdf.xacro",
            "description_package": "motion_control_module",
            # Use ur_description defaults for config files
            "joint_limit_params": joint_limit_params,
            "kinematics_params": kinematics_params,
            "physical_params": physical_params,
            "visual_params": visual_params,
        }.items(),
    )

    # 2. Activate the scaled_joint_trajectory_controller (after driver starts)
    activate_controller = TimerAction(
        period=5.0,  # Wait 5 seconds for driver to start
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'control', 'switch_controllers',
                     '--activate', 'scaled_joint_trajectory_controller'],
                output='screen',
                shell=False
            )
        ],
    )

    # 3. Launch MoveIt with gripper configuration (delayed to allow driver and controller to start)
    moveit_launch = TimerAction(
        period=12.0,  # Wait 12 seconds for driver and controller to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    PathJoinSubstitution([
                        FindPackageShare("motion_control_module"),
                        "launch",
                        "ur5e_moveit_with_gripper.launch.py"
                    ])
                ),
                launch_arguments={
                    "robot_ip": robot_ip,
                    "ur_type": ur_type,
                    "launch_rviz": launch_rviz,
                }.items(),
            )
        ],
    )

    return LaunchDescription(
        declared_arguments + [
            ur_control_launch,
            activate_controller,
            moveit_launch,
        ]
    )
