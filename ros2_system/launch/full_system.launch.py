#!/usr/bin/env python3
"""
Master launch file for the complete sort-by-weight robot system
Launches all modules with proper namespacing
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    # Supervisor module
    supervisor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('supervisor_module'),
                'launch',
                'supervisor.launch.py'
            ])
        ])
    )

    # Perception module
    perception_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('perception_module'),
                'launch',
                'perception.launch.py'
            ])
        ]),
        launch_arguments={
            'use_sim_time': LaunchConfiguration('use_sim_time')
        }.items()
    )

    # Recognition module
    recognition_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('recognition_module'),
                'launch',
                'recognition.launch.py'
            ])
        ])
    )

    # Planning module
    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('planning_module'),
                'launch',
                'planning.launch.py'
            ])
        ])
    )

    # Control module
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('control_module'),
                'launch',
                'control.launch.py'
            ])
        ])
    )

    # Motion control module
    motion_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('motion_control_module'),
                'launch',
                'motion_control.launch.py'
            ])
        ])
    )

    # Optional: RViz for visualization
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        output='screen',
        arguments=['-d', PathJoinSubstitution([
            FindPackageShare('supervisor_module'),  # Or create dedicated config package
            'config',
            'sort_system.rviz'
        ])],
        condition=IfCondition('false')  # Disabled by default
    )

    return LaunchDescription([
        use_sim_time_arg,

        # Launch all modules
        supervisor_launch,
        perception_launch,
        recognition_launch,
        planning_launch,
        control_launch,
        motion_control_launch,

        # Optional visualization
        # rviz_node,
    ])
