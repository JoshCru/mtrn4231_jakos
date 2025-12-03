#!/bin/bash
# Test script to verify gripper service is available

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/../ros2_system"

echo "=========================================="
echo "   Testing Gripper Service Lifecycle"
echo "=========================================="
echo ""

# Source ROS2
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

echo "Step 1: Starting gripper controller in simulation mode..."
ros2 run control_package gripper_controller_node --ros-args -p simulation_mode:=true &
GRIPPER_PID=$!
echo "   PID: $GRIPPER_PID"

sleep 3

echo ""
echo "Step 2: Checking node state (should be unconfigured)..."
ros2 lifecycle list /gripper_controller_node || echo "ERROR: Node not found!"

echo ""
echo "Step 3: Configuring node..."
ros2 lifecycle set /gripper_controller_node configure
sleep 1

echo ""
echo "Step 4: Checking node state (should be inactive)..."
ros2 lifecycle list /gripper_controller_node

echo ""
echo "Step 5: Activating node..."
ros2 lifecycle set /gripper_controller_node activate
sleep 2

echo ""
echo "Step 6: Checking node state (should be active)..."
ros2 lifecycle list /gripper_controller_node

echo ""
echo "Step 7: Checking if service is available..."
ros2 service list | grep gripper_control || echo "ERROR: Service not found!"

echo ""
echo "Step 8: Testing service call (open gripper, 5s wait)..."
echo "This should take approximately 5 seconds..."
start_time=$(date +%s)
ros2 service call /motion_control/gripper_control sort_interfaces/srv/GripperControl "{command: 'W', weight: 0, wait_time_sec: 5.0}"
end_time=$(date +%s)
elapsed=$((end_time - start_time))
echo "   Service call took ${elapsed} seconds"

if [ $elapsed -ge 4 ]; then
    echo "   ✓ SUCCESS: Service waited correctly!"
else
    echo "   ✗ FAILURE: Service returned too quickly (expected ~5s)"
fi

echo ""
echo "Cleaning up..."
kill $GRIPPER_PID 2>/dev/null || true
echo "Done."
