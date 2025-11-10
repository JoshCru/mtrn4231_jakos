#!/bin/bash
# Comprehensive test to verify MoveIt communication with joint_movement_controller

echo "=========================================="
echo "MoveIt Communication Diagnostic Test"
echo "=========================================="
echo ""

# Step 1: Check if MoveIt is running
echo "Step 1: Checking if MoveIt action server is available..."
if ros2 action list | grep -q "/move_action"; then
    echo "✓ MoveIt action server found at /move_action"
else
    echo "✗ ERROR: /move_action not found!"
    echo "  Please start MoveIt first:"
    echo "  ros2 launch motion_control_module ur5e_real_with_gripper.launch.py"
    exit 1
fi
echo ""

# Step 2: Check action server details
echo "Step 2: Checking action server details..."
ros2 action info /move_action
echo ""

# Step 3: Check if joint_movement_controller is running
echo "Step 3: Checking if joint_movement_controller is running..."
if ros2 node list | grep -q "joint_movement_controller"; then
    echo "✓ joint_movement_controller node found"
else
    echo "✗ ERROR: joint_movement_controller not found!"
    echo "  Please start it first:"
    echo "  ros2 run supervisor_module joint_movement_controller"
    exit 1
fi
echo ""

# Step 4: Check if service is available
echo "Step 4: Checking if /move_to_joint_position service is available..."
if ros2 service list | grep -q "/move_to_joint_position"; then
    echo "✓ Service /move_to_joint_position found"
    echo ""
    echo "Service type:"
    ros2 service type /move_to_joint_position
else
    echo "✗ ERROR: /move_to_joint_position service not found!"
    exit 1
fi
echo ""

# Step 5: Check controller status
echo "Step 5: Checking robot controller status..."
ros2 control list_controllers
echo ""

echo "=========================================="
echo "All checks passed! System is ready."
echo "=========================================="
echo ""
echo "Now you can test by running:"
echo "  ros2 run supervisor_module joint_movement_client_example"
echo ""
echo "Watch for these logs in joint_movement_controller:"
echo "  1. 'Calling async_send_goal...' - Goal being sent"
echo "  2. 'Waiting for goal acceptance...' - Waiting for MoveIt"
echo "  3. 'Goal accepted by MoveIt, executing...' - MoveIt accepted"
echo "  4. 'Received feedback from MoveIt: ...' - MoveIt sending updates"
echo "  5. 'Received result with error code: ...' - MoveIt finished"
echo ""
