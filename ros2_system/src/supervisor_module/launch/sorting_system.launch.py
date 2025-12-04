#!/usr/bin/env python3
"""
Sorting System Master Launch File

This launch file replaces the runSimulation.sh, runHybrid.sh, and runRealRobot.sh scripts.

Three modes available:
1. SIMULATION: Fake robot + simulated perception + simulated weights
   ros2 launch supervisor_module sorting_system.launch.py mode:=simulation

2. HYBRID: Real robot + simulated perception + optional real weights
   ros2 launch supervisor_module sorting_system.launch.py mode:=hybrid robot_ip:=192.168.0.100

3. REAL: Real robot + real perception + real weights
   ros2 launch supervisor_module sorting_system.launch.py mode:=real robot_ip:=192.168.0.100

Additional options:
  - autorun:=true              Auto-start sorting without dashboard
  - launch_rviz:=true/false    Launch RViz visualization
  - real_weight_detection:=true Use real weight detector in hybrid mode
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    ExecuteProcess,
    TimerAction,
    RegisterEventHandler,
    LogInfo,
    EmitEvent
)
from launch.conditions import IfCondition, UnlessCondition
from launch.event_handlers import OnProcessStart, OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
    FindExecutable,
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
    mode_arg = DeclareLaunchArgument(
        'mode',
        default_value='simulation',
        description='Mode: simulation, hybrid, or real',
        choices=['simulation', 'hybrid', 'real']
    )

    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.0.100',
        description='IP address of the real robot (used in hybrid/real modes)'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='true',
        description='Launch RViz visualization'
    )

    autorun_arg = DeclareLaunchArgument(
        'autorun',
        default_value='false',
        description='Automatically start sorting without dashboard'
    )

    real_weight_detection_arg = DeclareLaunchArgument(
        'real_weight_detection',
        default_value='false',
        description='Use real weight detector in hybrid mode (default: use simulated weights)'
    )

    # Get launch configurations
    mode = LaunchConfiguration('mode')
    robot_ip = LaunchConfiguration('robot_ip')
    launch_rviz = LaunchConfiguration('launch_rviz')
    autorun = LaunchConfiguration('autorun')
    real_weight_detection = LaunchConfiguration('real_weight_detection')

    # Derived configurations
    use_fake_hardware = PythonExpression(["'", mode, "' == 'simulation'"])
    use_simulated_perception = PythonExpression(["'", mode, "' != 'real'"])
    gripper_simulation_mode = PythonExpression(["'", mode, "' == 'simulation'"])

    # Find package shares
    motion_control_share = FindPackageShare('motion_control_module')

    # ==================== FastDDS Profile Setup ====================
    # Create FastDDS profile for reliable communication
    fastdds_profile_setup = ExecuteProcess(
        cmd=['bash', '-c', '''
cat > /tmp/fastdds_profile.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomUdpTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>CustomUdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF
echo "FastDDS profile created at /tmp/fastdds_profile.xml"
        '''],
        output='screen',
        shell=True
    )

    # ==================== UR5e Driver ====================
    ur_driver = TimerAction(
        period=1.0,  # Start after FastDDS profile is created
        actions=[
            LogInfo(msg=['[1/9] Starting UR5e Driver (mode: ', mode, ')...']),
            IncludeLaunchDescription(
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
                    'launch_rviz': 'false',
                    'description_file': 'ur5e_with_end_effector.urdf.xacro',
                    'description_package': 'motion_control_module',
                }.items()
            )
        ]
    )

    # ==================== MoveIt ====================
    moveit_launch = TimerAction(
        period=10.0,  # Wait for UR driver to initialize
        actions=[
            LogInfo(msg='[2/9] Starting MoveIt with RViz...'),
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
                    'robot_ip': robot_ip,
                }.items()
            )
        ]
    )

    # ==================== Go Home ====================
    go_home = TimerAction(
        period=22.0,
        actions=[
            LogInfo(msg='[3/9] Moving robot to HOME position...'),
            ExecuteProcess(
                cmd=['ros2', 'run', 'motion_control_module', 'go_home', '5.0'],
                output='screen'
            )
        ]
    )

    # ==================== Safety Visualizer ====================
    safety_viz = TimerAction(
        period=27.0,
        actions=[
            LogInfo(msg='[4/9] Starting Safety Boundary Visualizer...'),
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

    # ==================== Perception ====================
    # Simulated perception (only in simulation/hybrid modes)
    simulated_perception = TimerAction(
        period=29.0,
        actions=[
            LogInfo(
                msg='[5/9] Starting Simulated Perception...',
                condition=IfCondition(use_simulated_perception)
            ),
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

    # Real perception reminder (only in real mode)
    real_perception_reminder = TimerAction(
        period=29.0,
        actions=[
            LogInfo(
                msg='[5/9] NOTE: Expecting Perception nodes to be running externally!',
                condition=UnlessCondition(use_simulated_perception)
            )
        ]
    )

    # ==================== Weight Detection ====================
    # Real weight detector (only in real mode OR hybrid with real_weight_detection flag)
    use_real_weight = PythonExpression([
        "'", mode, "' == 'real' or ('", mode, "' == 'hybrid' and '",
        real_weight_detection, "' == 'true')"
    ])

    weight_detector = TimerAction(
        period=31.0,
        actions=[
            LogInfo(
                msg='[6/9] Starting Real Weight Detection (Asad\'s module)...',
                condition=IfCondition(use_real_weight)
            ),
            Node(
                package='weight_detection_module',
                executable='weight_detector',
                name='weight_detector',
                output='screen',
                condition=IfCondition(use_real_weight)
            )
        ]
    )

    weight_skip_msg = TimerAction(
        period=31.0,
        actions=[
            LogInfo(
                msg='[6/9] Skipping Weight Detection (using simulated weights from perception)...',
                condition=UnlessCondition(use_real_weight)
            )
        ]
    )

    # ==================== Gripper Controller (Lifecycle) ====================
    gripper_controller = TimerAction(
        period=33.0,
        actions=[
            LogInfo(msg=['[7/9] Starting Gripper Controller (simulation_mode: ', gripper_simulation_mode, ')...']),
            Node(
                package='control_module',
                executable='gripper_controller_node',
                name='gripper_controller_node',
                parameters=[{
                    'simulation_mode': gripper_simulation_mode
                }],
                output='screen'
            )
        ]
    )

    # Activate gripper controller (lifecycle transitions)
    activate_gripper_configure = TimerAction(
        period=35.0,
        actions=[
            LogInfo(msg='   Configuring gripper controller (lifecycle)...'),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/gripper_controller_node', 'configure'],
                output='screen'
            )
        ]
    )

    activate_gripper_activate = TimerAction(
        period=37.0,
        actions=[
            LogInfo(msg='   Activating gripper controller (lifecycle)...'),
            ExecuteProcess(
                cmd=['ros2', 'lifecycle', 'set', '/gripper_controller_node', 'activate'],
                output='screen'
            )
        ]
    )

    # ==================== Cartesian Controller ====================
    cartesian_controller = TimerAction(
        period=41.0,
        actions=[
            LogInfo(msg='[8/9] Starting Cartesian Controller...'),
            Node(
                package='motion_control_module',
                executable='cartesian_controller_node',
                name='cartesian_controller_node',
                parameters=[{
                    'use_fake_hardware': use_fake_hardware
                }],
                output='screen'
            )
        ]
    )

    # ==================== Sorting Brain ====================
    sorting_brain = TimerAction(
        period=44.0,
        actions=[
            LogInfo(msg='[9/9] Starting Sorting Brain Node...'),
            Node(
                package='supervisor_module',
                executable='sorting_brain_node',
                name='sorting_brain_node',
                output='screen'
            )
        ]
    )

    # ==================== System Ready Message ====================
    system_ready = TimerAction(
        period=46.0,
        actions=[
            LogInfo(msg='=========================================='),
            LogInfo(msg='   All systems launched!'),
            LogInfo(msg='=========================================='),
        ]
    )

    # ==================== Autorun (Optional) ====================
    autorun_start = TimerAction(
        period=48.0,
        actions=[
            LogInfo(
                msg='AUTORUN: Starting sorting automatically...',
                condition=IfCondition(autorun)
            ),
            ExecuteProcess(
                cmd=['ros2', 'topic', 'pub', '--once', '/sorting/command',
                     'std_msgs/msg/String', 'data: start'],
                output='screen',
                condition=IfCondition(autorun)
            )
        ]
    )

    manual_control_msg = TimerAction(
        period=48.0,
        actions=[
            LogInfo(
                msg='To control the system, run: ./launchDashboard.sh',
                condition=UnlessCondition(autorun)
            )
        ]
    )

    # ==================== Launch Description ====================
    return LaunchDescription([
        # Arguments
        mode_arg,
        robot_ip_arg,
        launch_rviz_arg,
        autorun_arg,
        real_weight_detection_arg,

        # Setup
        fastdds_profile_setup,

        # Launch sequence
        ur_driver,
        moveit_launch,
        go_home,
        safety_viz,
        simulated_perception,
        real_perception_reminder,
        weight_detector,
        weight_skip_msg,
        gripper_controller,
        activate_gripper_configure,
        activate_gripper_activate,
        cartesian_controller,
        sorting_brain,
        system_ready,
        autorun_start,
        manual_control_msg,
    ])
