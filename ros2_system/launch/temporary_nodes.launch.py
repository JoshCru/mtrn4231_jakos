#!/usr/bin/env python3
"""
Temporary Nodes Launch File

Launches temporary/one-off nodes that can be started and stopped:
  - Go Home
  - Perception (Real/Simulated)
  - Weight Detection (Real/Simulated/Off)
  - Position Check (for simulated perception)
  - Application Mode (Sorting System or Simple Pick & Weigh)

Usage:
  # Sorting with simulated perception
  ros2 launch temporary_nodes.launch.py mode:=sorting sim_perception:=true

  # Simple pick and weigh with real weight
  ros2 launch temporary_nodes.launch.py mode:=simple real_weight:=true

  # Full hybrid setup
  ros2 launch temporary_nodes.launch.py mode:=sorting sim_perception:=true \
      sim_weight:=true go_home:=true position_check:=true autorun:=true

Options:
  mode:=sorting/simple         Application mode (default: sorting)
  sim_perception:=true/false   Use simulated perception (default: false)
  real_weight:=true/false      Use real weight detection (default: false)
  sim_weight:=true/false       Use simulated weight (default: true)
  weight_detector_impl:=cpp/python  Weight detector implementation (default: cpp)
  go_home:=true/false          Run go_home before other nodes (default: true)
  position_check:=true/false   Run position check for sim perception (default: false)
  autorun:=true/false          Auto-start application (default: false)
  grip_weight:=GRAMS           Grip weight for simple mode (default: 100)
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    TimerAction,
    LogInfo,
    OpaqueFunction
)
from launch.conditions import IfCondition, UnlessCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    PythonExpression
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def launch_setup(context, *args, **kwargs):
    """Dynamic launch setup based on mode"""

    # Get launch configurations
    mode = LaunchConfiguration('mode').perform(context)
    sim_perception = LaunchConfiguration('sim_perception').perform(context)
    real_weight = LaunchConfiguration('real_weight').perform(context)
    sim_weight = LaunchConfiguration('sim_weight').perform(context)
    weight_detector_impl = LaunchConfiguration('weight_detector_impl').perform(context)
    go_home = LaunchConfiguration('go_home').perform(context)
    position_check = LaunchConfiguration('position_check').perform(context)
    autorun = LaunchConfiguration('autorun').perform(context)
    grip_weight = LaunchConfiguration('grip_weight').perform(context)

    # Package paths
    motion_control_pkg = FindPackageShare('motion_control_package')
    supervisor_pkg = FindPackageShare('supervisor_package')
    weight_detection_pkg = FindPackageShare('weight_detection_package')

    actions = []
    delay = 0.0

    # ==================== 1. Go Home (optional) ====================
    if go_home.lower() == 'true':
        actions.append(
            ExecuteProcess(
                cmd=[
                    'ros2', 'run', 'motion_control_package', 'go_home', '5.0'
                ],
                name='go_home',
                output='screen'
            )
        )
        delay += 8.0  # Wait for go_home to complete

    # ==================== 2. Perception ====================
    if sim_perception.lower() == 'true':
        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    LogInfo(msg='Starting Simulated Perception...'),
                    Node(
                        package='perception_package',
                        executable='simulated_perception_node',
                        name='simulated_perception_node',
                        output='screen',
                        parameters=[{
                            'num_objects': 4,
                            'publish_rate': 5.0,
                            'randomize_positions': True
                        }]
                    )
                ]
            )
        )
        delay += 2.0

    # ==================== 3. Weight Detection ====================
    if real_weight.lower() == 'true':
        # Determine executable based on implementation choice
        if weight_detector_impl == 'python':
            weight_executable = 'weight_detector_py'
        else:
            weight_executable = 'weight_detector'

        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    LogInfo(msg=f'Starting Real Weight Detection ({weight_detector_impl})...'),
                    Node(
                        package='weight_detection_package',
                        executable=weight_executable,
                        name='weight_detector',
                        output='screen'
                    )
                ]
            )
        )
        delay += 2.0

        # Launch PlotJuggler for visualization
        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    ExecuteProcess(
                        cmd=[
                            PathJoinSubstitution([
                                FindPackageShare('weight_detection_package').find('weight_detection_package'),
                                '..',
                                '..',
                                '..',
                                'plot_weight.sh'
                            ])
                        ],
                        name='plot_juggler',
                        output='screen',
                        shell=True
                    )
                ]
            )
        )
        delay += 2.0
    elif sim_weight.lower() == 'true':
        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    LogInfo(msg='Weight Detection: SIMULATED (using simulated weights)')
                ]
            )
        )
        delay += 0.5

    # ==================== 4. Position Check (optional) ====================
    if position_check.lower() == 'true' and sim_perception.lower() == 'true':
        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    LogInfo(msg='Running Position Check for Simulated Perception...'),
                    ExecuteProcess(
                        cmd=[
                            'python3',
                            PathJoinSubstitution([
                                motion_control_pkg,
                                'scripts',
                                'check_simulated_positions.py'
                            ])
                        ],
                        name='position_check',
                        output='screen'
                    )
                ]
            )
        )
        delay += 15.0  # Wait for position check to complete

    # ==================== 5. Application Mode ====================
    if mode == 'sorting':
        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    LogInfo(msg='Starting Sorting Brain...'),
                    Node(
                        package='supervisor_package',
                        executable='sorting_brain_node',
                        name='sorting_brain_node',
                        output='screen'
                    )
                ]
            )
        )
        delay += 2.0

        # Autorun sorting
        if autorun.lower() == 'true':
            actions.append(
                TimerAction(
                    period=delay,
                    actions=[
                        LogInfo(msg='AUTORUN: Starting sorting...'),
                        ExecuteProcess(
                            cmd=[
                                'ros2', 'topic', 'pub', '--once',
                                '/sorting/command', 'std_msgs/msg/String',
                                '{data: "start"}'
                            ],
                            name='autorun_sorting',
                            output='screen'
                        )
                    ]
                )
            )

    elif mode == 'simple':
        actions.append(
            TimerAction(
                period=delay,
                actions=[
                    LogInfo(msg=f'Running Simple Pick and Weigh (grip_weight: {grip_weight}g)...'),
                    Node(
                        package='motion_control_package',
                        executable='simple_pick_and_weigh_node',
                        name='simple_pick_and_weigh_node',
                        output='screen',
                        parameters=[{
                            'grip_weight': int(grip_weight),
                            'initial_positioning': True
                        }]
                    )
                ]
            )
        )

    # ==================== Success Message ====================
    actions.append(
        TimerAction(
            period=delay + 1.0,
            actions=[
                LogInfo(msg='========================================'),
                LogInfo(msg='   Temporary nodes launched!'),
                LogInfo(msg='========================================'),
            ]
        )
    )

    return actions


def generate_launch_description():
    # ==================== Launch Arguments ====================
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='sorting',
        description='Application mode: sorting or simple',
        choices=['sorting', 'simple']
    )

    sim_perception_arg = DeclareLaunchArgument(
        'sim_perception',
        default_value='false',
        description='Use simulated perception'
    )

    real_weight_arg = DeclareLaunchArgument(
        'real_weight',
        default_value='false',
        description='Use real weight detection'
    )

    sim_weight_arg = DeclareLaunchArgument(
        'sim_weight',
        default_value='true',
        description='Use simulated weight detection'
    )

    weight_detector_impl_arg = DeclareLaunchArgument(
        'weight_detector_impl',
        default_value='cpp',
        description='Weight detector implementation: cpp or python',
        choices=['cpp', 'python']
    )

    go_home_arg = DeclareLaunchArgument(
        'go_home',
        default_value='true',
        description='Run go_home action before other nodes'
    )

    position_check_arg = DeclareLaunchArgument(
        'position_check',
        default_value='false',
        description='Run position check (for simulated perception)'
    )

    autorun_arg = DeclareLaunchArgument(
        'autorun',
        default_value='false',
        description='Auto-start application after launch'
    )

    grip_weight_arg = DeclareLaunchArgument(
        'grip_weight',
        default_value='100',
        description='Weight for gripper angle (simple mode only)'
    )

    # ==================== Launch Description ====================
    return LaunchDescription([
        # Arguments
        mode_arg,
        sim_perception_arg,
        real_weight_arg,
        sim_weight_arg,
        weight_detector_impl_arg,
        go_home_arg,
        position_check_arg,
        autorun_arg,
        grip_weight_arg,

        # Dynamic launch setup
        OpaqueFunction(function=launch_setup)
    ])
