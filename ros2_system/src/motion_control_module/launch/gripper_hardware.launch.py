"""
Launch gripper controller in HARDWARE mode (Teensy 4.1 required)
Use this on your Linux VM when Teensy is connected
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('motion_control_module')
    config_file = os.path.join(pkg_dir, 'config', 'gripper_params.yaml')

    # Declare launch arguments
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value='/dev/ttyACM0',
        description='Serial port for Teensy (e.g., /dev/ttyACM0, /dev/cu.usbmodem*)'
    )

    return LaunchDescription([
        serial_port_arg,

        Node(
            package='motion_control_module',
            executable='gripper_controller_node',
            name='gripper_controller_node',
            output='screen',
            parameters=[
                config_file,
                {
                    'simulation_mode': False,  # Hardware mode
                    'serial_port': LaunchConfiguration('serial_port'),
                }
            ],
            emulate_tty=True,
        ),
    ])
