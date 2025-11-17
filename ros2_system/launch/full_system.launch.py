#!/usr/bin/env python3
"""
Master launch file for the complete sort-by-weight robot system
Launches all modules with proper namespacing
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction, TimerAction, ExecuteProcess, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.substitutions import FindPackageShare
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    launch_dashboard_arg = DeclareLaunchArgument(
        'launch_dashboard',
        default_value='true',
        description='Launch Brain Dashboard UI'
    )

    log_level_arg = DeclareLaunchArgument(
        'log_level',
        default_value='info',
        description='Logging level (debug, info, warn, error, fatal)',
        choices=['debug', 'info', 'warn', 'error', 'fatal']
    )

    # Get log level configuration
    log_level = LaunchConfiguration('log_level')

    # Brain Node - Central Orchestrator
    brain_node = Node(
        package='supervisor_module',
        executable='brain_node',
        name='brain_node',
        output='screen',
        parameters=[{
            'discovery_rate': 0.5,
            'health_check_rate': 1.0,
            'status_publish_rate': 2.0,
            'metrics_publish_rate': 0.2,
        }],
        emulate_tty=True,
        arguments=['--ros-args', '--log-level', log_level],
    )

    # Supervisor module (System Controller)
    supervisor_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('supervisor_module'),
                'launch',
                'supervisor.launch.py'
            ])
        ]),
        launch_arguments={
            'log_level': log_level
        }.items()
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
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'log_level': log_level
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
        ]),
        launch_arguments={
            'log_level': log_level
        }.items()
    )

    # Planning module
    planning_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('planning_module'),
                'launch',
                'planning.launch.py'
            ])
        ]),
        launch_arguments={
            'log_level': log_level
        }.items()
    )

    # Control module
    control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('control_module'),
                'launch',
                'control.launch.py'
            ])
        ]),
        launch_arguments={
            'log_level': log_level
        }.items()
    )

    # Motion control module - REMOVED: gripper_controller_node moved to control_module
    # motion_control_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         PathJoinSubstitution([
    #             FindPackageShare('motion_control_module'),
    #             'launch',
    #             'motion_control.launch.py'
    #         ])
    #     ])
    # )

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

    # Brain Dashboard (delayed launch to ensure nodes are up)
    brain_dashboard = ExecuteProcess(
        cmd=['ros2', 'run', 'supervisor_module', 'brain_dashboard'],
        output='screen',
        shell=False,
    )

    delayed_dashboard = TimerAction(
        period=3.0,
        actions=[brain_dashboard]
    )

    return LaunchDescription([
        use_sim_time_arg,
        launch_dashboard_arg,
        log_level_arg,

        # Brain Node - Central Orchestrator (launch first)
        brain_node,

        # Launch all modules
        supervisor_launch,
        perception_launch,
        recognition_launch,
        planning_launch,
        control_launch,
        # motion_control_launch,  # REMOVED: gripper_controller_node moved to control_module

        # Brain Dashboard UI (delayed)
        delayed_dashboard,

        # Optional visualization
        # rviz_node,
    ])
