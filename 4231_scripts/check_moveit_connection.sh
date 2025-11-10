#!/bin/bash
# Check if MoveIt action server is running and joint_movement_controller can connect

echo "=========================================="
echo "Checking MoveIt Action Server"
echo "=========================================="

echo ""
echo "Looking for /move_action action server..."
ros2 action list | grep move_action

echo ""
echo "=========================================="
echo "Action server info:"
echo "=========================================="
ros2 action info /move_action

echo ""
echo "=========================================="
echo "Checking for joint_movement_controller node:"
echo "=========================================="
ros2 node list | grep joint_movement

echo ""
echo "=========================================="
echo "Checking for /move_to_joint_position service:"
echo "=========================================="
ros2 service list | grep move_to_joint_position

echo ""
echo "=========================================="
echo "If you see:"
echo "  - /move_action → MoveIt is running ✓"
echo "  - joint_movement_controller → Controller is running ✓"
echo "  - /move_to_joint_position → Service is available ✓"
echo "=========================================="
