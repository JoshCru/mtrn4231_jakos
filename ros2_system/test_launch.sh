#!/bin/bash
# Quick test script for the ROS2 sort system

echo "========================================="
echo "ROS2 Sort System - Launch Test"
echo "========================================="
echo ""

# Source workspace
echo "Sourcing workspace..."
source install/setup.bash

echo ""
echo "Launching system in background..."
echo "Press Ctrl+C to stop"
echo ""

# Launch in background
ros2 launch launch/full_system.launch.py &
LAUNCH_PID=$!

# Wait for nodes to start
sleep 5

echo ""
echo "Checking running nodes..."
ros2 node list

echo ""
echo "System is running! Press Ctrl+C to stop."
echo ""
echo "In another terminal, try:"
echo "  source install/setup.bash"
echo "  ros2 topic list"
echo "  ros2 service call /system/start sort_interfaces/srv/SystemCommand \"{command: 'start'}\""
echo ""

# Wait for user interrupt
wait $LAUNCH_PID
