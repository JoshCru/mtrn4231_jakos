#!/usr/bin/env python3
"""
Launch file for Gazebo weight sorting simulation
Starts Gazebo with weight sorting world and recognition node
"""

import os
from pathlib import Path
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    IncludeLaunchDescription,
    SetEnvironmentVariable,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Get package directories
    pkg_share = get_package_share_directory('recognition_module')

    # Paths
    world_file = os.path.join(pkg_share, 'worlds', 'weight_sorting.world')
    models_path = os.path.join(pkg_share, 'models')
    config_file = os.path.join(pkg_share, 'config', 'recognition.yaml')

    # Ensure world file exists
    if not os.path.exists(world_file):
        # Fallback to source directory during development
        pkg_share = str(Path(__file__).parent.parent)
        world_file = os.path.join(pkg_share, 'worlds', 'weight_sorting.world')
        models_path = os.path.join(pkg_share, 'models')
        config_file = os.path.join(pkg_share, 'config', 'recognition.yaml')

    # Set Gazebo model path to include our models
    gazebo_models_path = os.environ.get('GAZEBO_MODEL_PATH', '')
    if gazebo_models_path:
        gazebo_models_path = f"{models_path}:{gazebo_models_path}"
    else:
        gazebo_models_path = models_path

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='true',
        description='Use simulation time'
    )

    world_arg = DeclareLaunchArgument(
        'world',
        default_value=world_file,
        description='Path to world file'
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Start Gazebo with GUI'
    )

    verbose_arg = DeclareLaunchArgument(
        'verbose',
        default_value='false',
        description='Verbose output from Gazebo'
    )

    # Set environment variables
    set_gazebo_model_path = SetEnvironmentVariable(
        'GAZEBO_MODEL_PATH',
        gazebo_models_path
    )

    # Gazebo server
    gzserver = ExecuteProcess(
        cmd=['gzserver',
             LaunchConfiguration('world'),
             '-s', 'libgazebo_ros_init.so',
             '-s', 'libgazebo_ros_factory.so'],
        output='screen'
    )

    # Gazebo client (GUI)
    gzclient = ExecuteProcess(
        cmd=['gzclient'],
        output='screen',
        condition=LaunchConfiguration('gui')
    )

    # Recognition node
    recognition_node = Node(
        package='recognition_module',
        executable='recognition_node',
        name='recognition_node',
        output='screen',
        parameters=[
            config_file,
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ]
    )

    # Build launch description
    ld = LaunchDescription()

    # Add launch arguments
    ld.add_action(use_sim_time_arg)
    ld.add_action(world_arg)
    ld.add_action(gui_arg)
    ld.add_action(verbose_arg)

    # Set environment
    ld.add_action(set_gazebo_model_path)

    # Start Gazebo
    ld.add_action(gzserver)
    ld.add_action(gzclient)

    # Start recognition node
    ld.add_action(recognition_node)

    return ld
