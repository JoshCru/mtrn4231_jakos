#!/bin/bash
# Build script for gripper controller system

echo "Building gripper controller system..."
echo ""

cd "$(dirname "$0")"

echo "Step 1: Building sort_interfaces (messages/services)..."
colcon build --packages-select sort_interfaces --symlink-install

echo ""
echo "Step 2: Sourcing interfaces..."
source install/setup.bash

echo ""
echo "Step 3: Building motion_control_module..."
colcon build --packages-select motion_control_module --symlink-install

echo ""
echo "Done! Source the workspace:"
echo "  source install/setup.bash"
echo ""
echo "Then run:"
echo "  Simulation: ros2 launch motion_control_module gripper_sim_with_interface.launch.py"
echo "  Hardware:   ros2 launch motion_control_module gripper_hardware_with_interface.launch.py"
