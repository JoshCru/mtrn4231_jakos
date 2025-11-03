#!/bin/bash
# Test script for square movement with real UR5e

echo "======================================"
echo "Real UR5e Square Movement Test"
echo "======================================"
echo ""
echo "This script will:"
echo "1. Check if the real UR5e is running"
echo "2. Run the square movement pattern"
echo ""

# Check if joint_states topic exists
if ! ros2 topic list | grep -q "/joint_states"; then
    echo "ERROR: /joint_states topic not found!"
    echo ""
    echo "Please start the real UR5e first using:"
    echo "  cd ~/Documents/mtrn4231_jakos/4231_scripts"
    echo "  ./setupRealur5e.sh"
    echo ""
    exit 1
fi

echo "✓ Real UR5e is running"
echo ""

# Check if trajectory controller is available
if ! ros2 topic list | grep -q "joint_trajectory_controller"; then
    echo "WARNING: joint_trajectory_controller not found"
    echo "Available topics:"
    ros2 topic list | grep -i "trajectory\|follow"
    echo ""
fi

echo "Starting square movement..."
echo ""

# Run the square movement script
python3 ros2_system/src/motion_control_module/scripts/square_movement_simple.py

echo ""
echo "======================================"
echo "Test complete!"
echo "======================================"
