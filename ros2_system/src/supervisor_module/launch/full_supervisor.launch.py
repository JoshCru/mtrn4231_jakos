from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    """
    Launch both the system controller backend and the dashboard UI
    """

    # Get package directory
    pkg_dir = get_package_share_directory('supervisor_module')
    config_file = os.path.join(pkg_dir, 'config', 'supervisor.yaml')

    # Launch arguments
    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation time'
    )

    auto_start_arg = DeclareLaunchArgument(
        'auto_start',
        default_value='false',
        description='Automatically start system on launch'
    )

    # System controller node (C++)
    system_controller = Node(
        package='supervisor_module',
        executable='system_controller_node',
        name='system_controller_node',
        output='screen',
        parameters=[
            config_file if os.path.exists(config_file) else {},
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'auto_start': LaunchConfiguration('auto_start'),
                'num_target_areas': 3,
                'publish_rate': 1.0,
            }
        ]
    )

    # Dashboard UI (Python/PyQt5)
    dashboard = Node(
        package='supervisor_module',
        executable='system_dashboard',
        name='system_dashboard',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
        }]
    )

    return LaunchDescription([
        use_sim_time_arg,
        auto_start_arg,
        system_controller,
        dashboard,
    ])
