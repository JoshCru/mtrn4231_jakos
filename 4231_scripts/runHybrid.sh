#!/bin/bash
# =============================================================================
# Hybrid Mode (Real Robot + Simulated Perception)
#
# Usage:
#   ./runHybrid.sh [ROBOT_IP] [--autorun] [--step]
#
# Options:
#   ROBOT_IP    Robot IP address (default: 192.168.0.100)
#   --autorun   Automatically start sorting without dashboard
#   --step      Pause before each node launch (for testing)
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/../ros2_system"

# Parse arguments
ROBOT_IP="192.168.0.100"
AUTORUN=false
STEP_MODE=false

for arg in "$@"; do
    case $arg in
        --autorun|-a)
            AUTORUN=true
            ;;
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

# Helper function for step mode
wait_for_enter() {
    if [ "$STEP_MODE" = true ]; then
        read -p "Press Enter to continue..."
    fi
}

echo "==========================================="
echo "   Sorting System - HYBRID MODE"
echo "==========================================="
echo "  Robot IP: $ROBOT_IP"
echo "  Autorun:  $AUTORUN"
echo "  Step:     $STEP_MODE"
echo "==========================================="
echo ""

source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

# 1. UR Driver
echo "[1/7] Starting UR5e Driver..."
wait_for_enter
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=$ROBOT_IP use_fake_hardware:=false launch_rviz:=false description_file:=ur5e_with_end_effector.urdf.xacro description_package:=motion_control_module &
UR_PID=$!
sleep 10

# Activate joint trajectory controller
echo "Activating joint_trajectory_controller..."
ros2 control switch_controllers --activate joint_trajectory_controller --deactivate scaled_joint_trajectory_controller 2>/dev/null || \
ros2 control switch_controllers --activate joint_trajectory_controller 2>/dev/null || \
echo "Warning: Could not switch controllers, may already be active"
sleep 2

# 2. MoveIt
echo "[2/7] Starting MoveIt..."
wait_for_enter
ros2 launch motion_control_module ur5e_moveit_with_gripper.launch.py robot_ip:=$ROBOT_IP ur_type:=ur5e launch_rviz:=true &
MOVEIT_PID=$!

echo "Waiting for MoveIt to initialize..."
sleep 5

echo "Waiting for robot_description_semantic parameter..."
timeout 30 bash -c 'until ros2 param list /move_group 2>/dev/null | grep -q robot_description_semantic; do sleep 1; done' || echo "Warning: robot_description_semantic not found, continuing anyway..."
sleep 2

# 3. Safety Visualizer
echo "[3/7] Starting Safety Visualizer..."
wait_for_enter
python3 "${ROS2_WS}/install/motion_control_module/share/motion_control_module/scripts/safety_boundary_collision.py" &
SAFETY_PID=$!
sleep 2

# 4. Simulated Perception
echo "[4/7] Starting Simulated Perception..."
wait_for_enter
ros2 run supervisor_module simulated_perception_node --ros-args -p num_objects:=4 -p publish_rate:=5.0 -p randomize_positions:=true &
PERCEPTION_PID=$!
sleep 2

# 5. Gripper Controller
echo "[5/7] Starting Gripper Controller..."
wait_for_enter
ros2 run control_module gripper_controller_node --ros-args -p simulation_mode:=false &
GRIPPER_PID=$!
sleep 2

echo "Activating gripper controller (lifecycle)..."
ros2 lifecycle set /gripper_controller_node configure
sleep 1
ros2 lifecycle set /gripper_controller_node activate
sleep 3

# 6. Cartesian Controller
echo "[6/7] Starting Cartesian Controller..."
wait_for_enter
ros2 run motion_control_module cartesian_controller_node &
CARTESIAN_PID=$!
sleep 3

# 7. Sorting Brain
echo "[7/7] Starting Sorting Brain..."
wait_for_enter
ros2 run supervisor_module sorting_brain_node &
SORTING_PID=$!

echo ""
echo "=========================================="
echo "   All systems launched!"
echo "=========================================="
echo ""

if [ "$AUTORUN" = true ]; then
    echo "AUTORUN: Starting sorting..."
    sleep 2
    ros2 topic pub --once /sorting/command std_msgs/msg/String "data: 'start'"
    echo "Sorting started!"
else
    echo "Run ./launchDashboard.sh to control, or use --autorun"
fi

echo ""
echo "Press Ctrl+C to stop..."

wait -n

echo "Shutting down..."
kill $UR_PID $MOVEIT_PID $GRIPPER_PID $CARTESIAN_PID $SAFETY_PID $PERCEPTION_PID $SORTING_PID 2>/dev/null || true
echo "Done."
