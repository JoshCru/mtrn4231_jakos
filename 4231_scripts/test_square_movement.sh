#!/bin/bash
# Test script for square movement - supports both fake and real UR5e

echo "======================================"
echo "UR5e Square Movement Test"
echo "======================================"
echo ""

# Check if argument provided
if [ -z "$1" ]; then
    echo "Choose hardware mode:"
    echo "1) Real hardware"
    echo "2) Fake hardware"
    read -p "Enter choice (1 or 2): " choice

    if [ "$choice" == "1" ]; then
        MODE="REAL"
        SETUP_SCRIPT="setupRealur5e.sh"
    elif [ "$choice" == "2" ]; then
        MODE="FAKE"
        SETUP_SCRIPT="setupFakeur5e.sh"
    else
        echo "Invalid choice. Exiting."
        exit 1
    fi
else
    # Argument provided
    if [ "$1" == "real" ]; then
        MODE="REAL"
        SETUP_SCRIPT="setupRealur5e.sh"
    elif [ "$1" == "fake" ]; then
        MODE="FAKE"
        SETUP_SCRIPT="setupFakeur5e.sh"
    else
        echo "Usage: $0 [real|fake]"
        exit 1
    fi
fi

echo ""
echo "Testing with $MODE UR5e"
echo ""
echo "This script will:"
echo "1. Check if the $MODE UR5e is running"
echo "2. Run the square movement pattern"
echo ""

# Check if joint_states topic exists
if ! ros2 topic list | grep -q "/joint_states"; then
    echo "ERROR: /joint_states topic not found!"
    echo ""
    echo "Please start the $MODE UR5e first using:"
    echo "  cd ~/Documents/mtrn4231_jakos/4231_scripts"
    echo "  ./$SETUP_SCRIPT"
    echo ""
    exit 1
fi

echo "✓ $MODE UR5e is running"
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
