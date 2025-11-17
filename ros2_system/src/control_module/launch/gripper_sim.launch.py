"""
Launch gripper controller in SIMULATION mode (no hardware required)
Use this for testing on your Mac or when hardware is not available
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('motion_control_module')
    config_file = os.path.join(pkg_dir, 'config', 'gripper_params.yaml')

    return LaunchDescription([
        Node(
            package='motion_control_module',
            executable='gripper_controller_node',
            name='gripper_controller_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'simulation_mode': True,  # Override to enable simulation
                }
            ],
            emulate_tty=True,
        ),
    ])
