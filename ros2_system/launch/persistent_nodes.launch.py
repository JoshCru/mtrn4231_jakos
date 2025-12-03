#!/usr/bin/env python3
"""
Persistent Nodes Launch File

Launches long-running infrastructure nodes that stay active during operation:
  - UR Driver
  - MoveIt
  - Safety Visualizer
  - Gripper Controller
  - Cartesian Controller

Usage:
  ros2 launch persistent_nodes.launch.py
  ros2 launch persistent_nodes.launch.py sim_robot:=true
  ros2 launch persistent_nodes.launch.py robot_ip:=192.168.0.100 launch_rviz:=true

Options:
  sim_robot:=true/false       Use simulated robot (fake hardware) (default: false)
  robot_ip:=IP                Robot IP address (default: 192.168.0.100)
  launch_rviz:=true/false     Launch RViz with MoveIt (default: true)
  launch_safety:=true/false   Launch Safety Visualizer (default: true)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
    LogInfo
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessStart
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression,
    TextSubstitution
)
from launch_ros.actions import Node, LifecycleNode
from launch_ros.substitutions import FindPackageShare
from launch_ros.event_handlers import OnStateTransition
from launch_ros.events.lifecycle import ChangeState
from lifecycle_msgs.msg import Transition
import os


def generate_launch_description():
    # ==================== Launch Arguments ====================
    sim_robot_arg = DeclareLaunchArgument(
        'sim_robot',
        default_value='false',
        description='Use simulated robot (fake hardware)'
    )

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.0.100',
        description='Robot IP address'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz with MoveIt'
    )

    launch_safety_arg = DeclareLaunchArgument(
        'launch_safety',
        default_value='true',
        description='Launch Safety Visualizer'
    )

    # ==================== Launch Configurations ====================
    sim_robot = LaunchConfiguration('sim_robot')
    robot_ip = LaunchConfiguration('robot_ip')
    launch_rviz = LaunchConfiguration('launch_rviz')
    launch_safety = LaunchConfiguration('launch_safety')

    # ==================== Package Paths ====================
    motion_control_pkg = FindPackageShare('motion_control_package')
    ur_robot_driver_pkg = FindPackageShare('ur_robot_driver')

    # ==================== 1. UR Driver ====================
    ur_driver_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([ur_robot_driver_pkg, 'launch', 'ur_control.launch.py'])
        ]),
        launch_arguments={
            'ur_type': 'ur5e',
            'robot_ip': robot_ip,
            'use_fake_hardware': sim_robot,
            'launch_rviz': 'false',
            'description_file': 'ur5e_with_end_effector.urdf.xacro',
            'description_package': 'motion_control_package',
        }.items()
    )

    # Log UR driver started
    ur_driver_log = RegisterEventHandler(
        OnProcessStart(
            target_action=ur_driver_launch,
            on_start=[
                LogInfo(msg='UR5e Driver started. Waiting for initialization...')
            ]
        )
    )

    # ==================== 2. MoveIt ====================
    moveit_launch = TimerAction(
        period=10.0,  # Wait 10 seconds for UR driver to initialize
        actions=[
            LogInfo(msg='Starting MoveIt with Gripper...'),
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource([
                    PathJoinSubstitution([
                        motion_control_pkg,
                        'launch',
                        'ur5e_moveit_with_gripper.launch.py'
                    ])
                ]),
                launch_arguments={
                    'robot_ip': robot_ip,
                    'ur_type': 'ur5e',
                    'launch_rviz': launch_rviz,
                }.items()
            )
        ]
    )

    # ==================== 3. Safety Visualizer ====================
    safety_visualizer = TimerAction(
        period=20.0,  # Wait for MoveIt to be ready
        actions=[
            ExecuteProcess(
                cmd=[
                    'python3',
                    PathJoinSubstitution([
                        motion_control_pkg,
                        'scripts',
                        'safety_boundary_collision.py'
                    ])
                ],
                name='safety_visualizer',
                output='screen',
                condition=IfCondition(launch_safety)
            )
        ]
    )

    # ==================== 4. Gripper Controller ====================
    gripper_controller = TimerAction(
        period=22.0,
        actions=[
            LogInfo(msg='Starting Gripper Controller (lifecycle node)...'),
            LifecycleNode(
                package='control_package',
                executable='gripper_controller_node',
                name='gripper_controller_node',
                namespace='',
                output='screen',
                parameters=[{
                    'simulation_mode': sim_robot
                }]
            )
        ]
    )

    # Configure gripper controller (lifecycle transition)
    configure_gripper = TimerAction(
        period=24.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/gripper_controller_node', 'configure'],
                name='configure_gripper',
                output='screen'
            )
        ]
    )

    # Activate gripper controller (lifecycle transition)
    activate_gripper = TimerAction(
        period=25.0,
        actions=[
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/gripper_controller_node', 'activate'],
                name='activate_gripper',
                output='screen'
            )
        ]
    )

    # ==================== 5. Cartesian Controller ====================
    cartesian_controller = TimerAction(
        period=28.0,
        actions=[
            LogInfo(msg='Starting Cartesian Controller...'),
            Node(
                package='motion_control_package',
                executable='cartesian_controller_node',
                name='cartesian_controller_node',
                output='screen',
                parameters=[{
                    'use_fake_hardware': sim_robot
                }]
            )
        ]
    )

    # ==================== Success Message ====================
    success_message = TimerAction(
        period=32.0,
        actions=[
            LogInfo(msg='========================================'),
            LogInfo(msg='   All persistent nodes launched!'),
            LogInfo(msg='========================================'),
        ]
    )

    # ==================== Launch Description ====================
    return LaunchDescription([
        # Arguments
        sim_robot_arg,
        robot_ip_arg,
        launch_rviz_arg,
        launch_safety_arg,

        # Launch sequence
        ur_driver_launch,
        ur_driver_log,
        moveit_launch,
        safety_visualizer,
        gripper_controller,
        configure_gripper,
        activate_gripper,
        cartesian_controller,
        success_message,
    ])
