#!/bin/bash

source ../ros2_system/install/setup.bash

gnome-terminal -t "DriverServer" -- bash -c 'ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 use_fake_hardware:=false launch_rviz:=false description_file:=ur5e_with_end_effector.urdf.xacro description_package:=motion_control_module; exec bash'

sleep 10

gnome-terminal -t "MoveitServer" -- bash -c 'ros2 launch motion_control_module ur5e_moveit_with_gripper.launch.py robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true; exec bash'

sleep 5

gnome-terminal -t "SafetyBoundaryVisuals" -- bash -c 'source ../ros2_system/install/setup.bash && python3 ../ros2_system/install/motion_control_module/share/motion_control_module/scripts/safety_boundary_visualizer.py; exec bash'

sleep 2

gnome-terminal -t "SafetyBoundaryCollision" -- bash -c 'source ../ros2_system/install/setup.bash && python3 ../ros2_system/install/motion_control_module/share/motion_control_module/scripts/safety_boundary_collision.py; exec bash'