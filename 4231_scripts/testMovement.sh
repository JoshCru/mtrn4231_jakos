#!/bin/bash
# =============================================================================
# Simple Movement Test - Just UR Driver + MoveIt + basic movement
#
# Usage:
#   ./testMovement.sh [ROBOT_IP] [--step]
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/../ros2_system"

ROBOT_IP="192.168.0.100"
STEP_MODE=false

for arg in "$@"; do
    case $arg in
        --step|-s)
            STEP_MODE=true
            ;;
        *)
            if [[ $arg =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
                ROBOT_IP=$arg
            fi
            ;;
    esac
done

wait_for_enter() {
    if [ "$STEP_MODE" = true ]; then
        read -p "Press Enter to continue..."
    fi
}

echo "==========================================="
echo "   Movement Test"
echo "==========================================="
echo "  Robot IP: $ROBOT_IP"
echo "==========================================="
echo ""
echo "IMPORTANT: Make sure you press PLAY on the teach pendant"
echo "           to start the External Control program!"
echo ""
read -p "Press Enter when ready..."

source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

# 1. UR Driver
echo "[1/3] Starting UR5e Driver..."
wait_for_enter
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=$ROBOT_IP use_fake_hardware:=false launch_rviz:=false description_file:=ur5e_with_end_effector.urdf.xacro description_package:=motion_control_package &
UR_PID=$!
sleep 10

# Check controller status
echo "Checking controllers..."
ros2 control list_controllers

# 2. MoveIt
echo "[2/3] Starting MoveIt..."
wait_for_enter
ros2 launch motion_control_package ur5e_moveit_with_gripper.launch.py robot_ip:=$ROBOT_IP ur_type:=ur5e launch_rviz:=true &
MOVEIT_PID=$!

echo "Waiting for MoveIt to initialize..."
sleep 5

echo "Waiting for robot_description_semantic parameter..."
timeout 30 bash -c 'until ros2 param list /move_group 2>/dev/null | grep -q robot_description_semantic; do sleep 1; done' || echo "Warning: robot_description_semantic not found, continuing anyway..."
sleep 2

# 3. Cartesian Controller
echo "[3/3] Starting Cartesian Controller..."
wait_for_enter
ros2 run motion_control_package cartesian_controller_node --ros-args -p use_fake_hardware:=false &
CARTESIAN_PID=$!
sleep 3

echo ""
echo "=========================================="
echo "   Ready for movement tests!"
echo "=========================================="
echo ""
echo "Test commands you can run in another terminal:"
echo ""
echo "  # Go to home position"
echo "  ros2 run motion_control_package go_home 5.0"
echo ""
echo "  # Move to a position (x, y, z in mm)"
echo "  ros2 topic pub --once /cartesian/target geometry_msgs/msg/Point \"{x: 400.0, y: 0.0, z: 300.0}\""
echo ""
echo "  # Check current TCP position"
echo "  ros2 topic echo /tcp_position --once"
echo ""
echo "Press Ctrl+C to stop..."

wait -n

echo "Shutting down..."
kill $UR_PID $MOVEIT_PID $CARTESIAN_PID 2>/dev/null || true
echo "Done."
