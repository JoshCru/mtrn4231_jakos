"""
Launch UR5e controller for FAKE/SIMULATION robot WITH END-EFFECTOR
This assumes you've already started the fake UR5e using the setupFakeur5e.sh script
"""

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import Command, FindExecutable, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get package directory
    pkg_dir = get_package_share_directory('motion_control_module')
    config_file = os.path.join(pkg_dir, 'config', 'ur5e_params.yaml')

    # Load robot description (URDF) with end-effector
    robot_description_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("motion_control_module"),
                "urdf",
                "ur5e_with_end_effector.urdf.xacro"
            ]),
            " ",
            "robot_ip:=xxx.yyy.zzz.www",
            " ",
            "joint_limit_params:=",
            PathJoinSubstitution([
                FindPackageShare("ur_description"),
                "config",
                "ur5e",
                "joint_limits.yaml"
            ]),
            " ",
            "kinematics_params:=",
            PathJoinSubstitution([
                FindPackageShare("ur_description"),
                "config",
                "ur5e",
                "default_kinematics.yaml"
            ]),
            " ",
            "physical_params:=",
            PathJoinSubstitution([
                FindPackageShare("ur_description"),
                "config",
                "ur5e",
                "physical_parameters.yaml"
            ]),
            " ",
            "visual_params:=",
            PathJoinSubstitution([
                FindPackageShare("ur_description"),
                "config",
                "ur5e",
                "visual_parameters.yaml"
            ]),
            " ",
            "name:=ur",
            " ",
            "ur_type:=ur5e",
            " ",
            "prefix:=",
            " ",
        ]
    )

    # Load robot semantic description (SRDF)
    robot_description_semantic_content = Command(
        [
            PathJoinSubstitution([FindExecutable(name="xacro")]),
            " ",
            PathJoinSubstitution([
                FindPackageShare("ur_moveit_config"),
                "srdf",
                "ur.srdf.xacro"
            ]),
            " ",
            "name:=ur",
            " ",
            "prefix:=",
            " ",
        ]
    )

    # Robot state publisher to publish the robot description with end-effector
    robot_state_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[
            {"robot_description": ParameterValue(robot_description_content, value_type=str)},
            {"use_sim_time": True},
        ],
    )

    return LaunchDescription([
        robot_state_publisher,
    ])
