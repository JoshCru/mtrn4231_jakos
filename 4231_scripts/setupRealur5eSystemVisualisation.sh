#!/bin/bash

source ../ros2_system/install/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 use_fake_hardware:=false launch_rviz:=false description_file:=ur5e_with_end_effector.urdf.xacro description_package:=motion_control_package &

sleep 10

ros2 launch motion_control_package ur5e_moveit_with_gripper.launch.py robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true &

echo "Waiting for MoveIt to initialize..."
sleep 5

# Wait for robot_description_semantic parameter to be available
echo "Waiting for robot_description_semantic parameter..."
timeout 30 bash -c 'until ros2 param list /move_group 2>/dev/null | grep -q robot_description_semantic; do sleep 1; done' || echo "Warning: robot_description_semantic not found, continuing anyway..."

sleep 2

python3 ../ros2_system/install/motion_control_package/share/motion_control_package/scripts/safety_boundary_visualizer.py &

sleep 2

python3 ../ros2_system/install/motion_control_package/share/motion_control_package/scripts/safety_boundary_collision.py &