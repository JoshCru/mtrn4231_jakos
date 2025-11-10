#!/bin/bash
# Script to check and activate controllers

echo "============================================"
echo "Checking controller status..."
echo "============================================"

ros2 control list_controllers

echo ""
echo "============================================"
echo "Activating scaled_joint_trajectory_controller..."
echo "============================================"

ros2 control switch_controllers --activate scaled_joint_trajectory_controller

echo ""
echo "============================================"
echo "Controller status after activation:"
echo "============================================"

ros2 control list_controllers
