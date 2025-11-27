"""
Launch gripper controller in SIMULATION mode with button interface
Use this for testing on your Mac or when hardware is not available
Includes keyboard interface for manual control
"""

from launch import LaunchDescription
from launch_ros.actions import Node, LifecycleNode
from launch.actions import RegisterEventHandler, EmitEvent
from launch.event_handlers import OnProcessExit
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
from lifecycle_msgs.msg import Transition
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('motion_control_module')
    config_file = os.path.join(pkg_dir, 'config', 'gripper_params.yaml')

    # Gripper controller (lifecycle node)
    gripper_controller = LifecycleNode(
        package='motion_control_module',
        executable='gripper_controller_node',
        name='gripper_controller_node',
        namespace='',
        output='screen',
        parameters=[
            config_file,
            {
                'simulation_mode': True,  # Override to enable simulation
            }
        ],
        emulate_tty=True,
    )

    # Auto-configure the lifecycle node
    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=lambda node: node.name == 'gripper_controller_node',
            transition_id=Transition.TRANSITION_CONFIGURE,
        )
    )

    # Auto-activate the lifecycle node after configuration
    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=gripper_controller,
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=lambda node: node.name == 'gripper_controller_node',
                        transition_id=Transition.TRANSITION_ACTIVATE,
                    )
                ),
            ],
        )
    )

    # Button interface (regular node)
    button_interface = Node(
        package='motion_control_module',
        executable='gripper_button_interface',
        name='gripper_button_interface',
        output='screen',
        emulate_tty=True,
    )

    return LaunchDescription([
        gripper_controller,
        configure_event,
        activate_event,
        button_interface,
    ])
