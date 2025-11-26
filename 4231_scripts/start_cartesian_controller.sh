#!/bin/bash

# Start the Direct Cartesian Controller

source ../ros2_system/install/setup.bash
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_profile.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

echo "Starting Direct Cartesian Controller..."
echo "Action server: /move_to_cartesian_direct"
echo ""
echo "For fake hardware (current setup):"
python3 ../ros2_system/install/motion_control_module/share/motion_control_module/scripts/direct_cartesian_controller.py

# Uncomment below and comment above for real hardware:
# python3 ../ros2_system/install/motion_control_module/share/motion_control_module/scripts/direct_cartesian_controller.py --ros-args -p use_scaled_controller:=true
