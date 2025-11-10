from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the system dashboard UI"""

    dashboard_node = Node(
        package='supervisor_module',
        executable='system_dashboard',
        name='system_dashboard',
        output='screen',
        parameters=[{
            'use_sim_time': False,
        }]
    )

    return LaunchDescription([
        dashboard_node
    ])
