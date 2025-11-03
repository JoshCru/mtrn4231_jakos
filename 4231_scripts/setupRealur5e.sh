# Source both workspaces
source /home/mtrn/4231/ros_ur_driver/install/setup.bash
source /home/mtrn/Documents/mtrn4231_jakos/ros2_system/install/setup.bash

gnome-terminal -t "DriverServer" -- bash -c 'source /home/mtrn/4231/ros_ur_driver/install/setup.bash && source /home/mtrn/Documents/mtrn4231_jakos/ros2_system/install/setup.bash && ros2 launch /home/mtrn/Documents/mtrn4231_jakos/4231_scripts/ur_control_fixed.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 initial_joint_controller:=scaled_joint_trajectory_controller use_fake_hardware:=false launch_rviz:=false description_file:=ur5e_with_end_effector.urdf.xacro description_package:=motion_control_module; exec bash'

sleep 10

gnome-terminal -t "MoveitServer" -- bash -c 'source /home/mtrn/4231/ros_ur_driver/install/setup.bash && source /home/mtrn/Documents/mtrn4231_jakos/ros2_system/install/setup.bash && ros2 launch /home/mtrn/Documents/mtrn4231_jakos/4231_scripts/ur_moveit_fixed.launch.py ur_type:=ur5e launch_rviz:=true use_fake_hardware:=false description_file:=ur5e_with_end_effector.urdf.xacro description_package:=motion_control_module; exec bash'





