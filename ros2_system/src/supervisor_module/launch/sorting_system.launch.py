#!/usr/bin/env python3
"""
Sorting System Master Launch File

This launch file can be configured to run in different modes:
1. Full simulation (fake robot + simulated perception)
2. Real robot with simulated perception (for testing without Kevin/Asad)
3. Full real system (real robot + real perception + real weight calibration)

Usage:
  ros2 launch supervisor_module sorting_system.launch.py use_fake_hardware:=true use_simulated_perception:=true
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess, TimerAction
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, FindExecutable
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare launch arguments
    use_fake_hardware_arg = DeclareLaunchArgument(
        'use_fake_hardware',
        default_value='true',
        description='Use fake hardware (simulation) or real robot'
    )

    use_simulated_perception_arg = DeclareLaunchArgument(
        'use_simulated_perception',
        default_value='true',
        description='Use simulated perception or real perception (Kevin\'s nodes)'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz with custom config'
    )

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='yyy.yyy.yyy.yyy',
        description='IP address of the real robot (ignored if use_fake_hardware:=true)'
    )

    # Get launch configurations
    use_fake_hardware = LaunchConfiguration('use_fake_hardware')
    use_simulated_perception = LaunchConfiguration('use_simulated_perception')
    launch_rviz = LaunchConfiguration('launch_rviz')
    robot_ip = LaunchConfiguration('robot_ip')

    # Find package shares
    motion_control_share = FindPackageShare('motion_control_module')

    # ==================== UR5e Driver ====================
    ur_driver = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': robot_ip,
            'initial_joint_controller': 'joint_trajectory_controller',
            'use_fake_hardware': use_fake_hardware,
            'launch_rviz': 'false',  # We'll launch RViz separately with custom config
            'description_file': 'ur5e_with_end_effector.urdf.xacro',
            'description_package': 'motion_control_module',
        }.items()
    )

    # ==================== MoveIt ====================
    moveit_launch = TimerAction(
        period=8.0,  # Wait for UR driver to initialize
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        FindPackageShare('motion_control_module'),
                        'launch',
                        'ur5e_moveit_with_gripper.launch.py'
                    ])
                ]),
                launch_arguments={
                    'ur_type': 'ur5e',
                    'launch_rviz': launch_rviz,
                    'use_fake_hardware': use_fake_hardware,
                }.items()
            )
        ]
    )

    # ==================== Visualization ====================
    # Safety boundary visualizer
    safety_viz = TimerAction(
        period=18.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    FindExecutable(name='python3'),
                    PathJoinSubstitution([
                        motion_control_share,
                        'share',
                        'motion_control_module',
                        'scripts',
                        'safety_boundary_collision.py'
                    ])
                ],
                output='screen'
            )
        ]
    )

    # Simulated perception node (only if use_simulated_perception:=true)
    simulated_perception = TimerAction(
        period=18.0,
        actions=[
            Node(
                package='supervisor_module',
                executable='simulated_perception_node',
                name='simulated_perception_node',
                parameters=[{
                    'num_objects': 4,
                    'publish_rate': 5.0,
                    'randomize_positions': True
                }],
                output='screen',
                condition=IfCondition(use_simulated_perception)
            )
        ]
    )

    # ==================== Go Home ====================
    go_home = TimerAction(
        period=21.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'motion_control_module', 'go_home', '5.0'
                ],
                output='screen'
            )
        ]
    )

    # ==================== Cartesian Controller ====================
    cartesian_controller = TimerAction(
        period=24.0,
        actions=[
            Node(
                package='motion_control_module',
                executable='cartesian_controller_node',
                name='cartesian_controller_node',
                output='screen'
            )
        ]
    )

    # ==================== Sorting Brain ====================
    sorting_brain = TimerAction(
        period=27.0,
        actions=[
            Node(
                package='supervisor_module',
                executable='sorting_brain_node',
                name='sorting_brain_node',
                output='screen'
            )
        ]
    )

    return LaunchDescription([
        # Arguments
        use_fake_hardware_arg,
        use_simulated_perception_arg,
        launch_rviz_arg,
        robot_ip_arg,

        # Launch sequence
        ur_driver,
        moveit_launch,
        safety_viz,
        simulated_perception,
        go_home,
        cartesian_controller,
        sorting_brain,
    ])
