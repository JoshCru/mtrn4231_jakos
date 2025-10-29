#!/bin/bash

# Launch UR robot driver
gnome-terminal -t "DriverServer" -e 'ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 use_fake_hardware:=false launch_rviz:=false'

sleep 10

# Launch RealSense camera
gnome-terminal -t "RealSense" -e 'ros2 launch realsense2_camera rs_launch.py depth_module.profile:=640x480x30 pointcloud.enable:=true'

sleep 5

# Launch hand-eye calibration (camera transform)
gnome-terminal -t "CameraTransform" -e 'roslaunch /Users/joshcru/Documents/Uni/MTRN4231/Git\ Folder/mtrn4231_jakos/4231_hand_eye_calibration/4_calib_camera_pose.launch'

sleep 2

# Launch perception module (processes pointcloud)
gnome-terminal -t "Perception" -e 'ros2 launch perception_module perception.launch.py'

sleep 2

# Launch MoveIt with RViz for motion planning
gnome-terminal -t "MoveitServer" -e 'ros2 launch ur_moveit_config ur_moveit.launch.py robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true'
