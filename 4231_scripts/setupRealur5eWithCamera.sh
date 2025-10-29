#!/bin/bash

# Step 1: Launch UR robot driver
echo "Launching UR Robot Driver..."
gnome-terminal -t "DriverServer" -e 'bash -c "ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 use_fake_hardware:=false launch_rviz:=false; exec bash"'

sleep 10

# Step 2: Launch MoveIt with RViz
echo "Launching MoveIt with RViz..."
gnome-terminal -t "MoveitServer" -e 'bash -c "ros2 launch ur_moveit_config ur_moveit.launch.py robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true; exec bash"'

sleep 5

# Step 3: Launch RealSense camera with hand-eye calibration
echo "Launching RealSense Camera and Perception..."
gnome-terminal -t "Perception" -e 'bash -c "cd ~/Downloads/mtrn4231_jakos && ros2 launch 4231_scripts/add_perception_to_rviz.launch.py; exec bash"'

echo ""
echo "============================================"
echo "All components launched!"
echo "============================================"
echo ""
echo "To add PointCloud to RViz:"
echo "1. In RViz, click 'Add' button"
echo "2. Select 'PointCloud2'"
echo "3. Set Topic to: /camera/depth/color/points"
echo "4. Set Fixed Frame to: base_link"
echo ""
echo "TF frames:"
echo "  base_link -> camera_link"
echo ""
