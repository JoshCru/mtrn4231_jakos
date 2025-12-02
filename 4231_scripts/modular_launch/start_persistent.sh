#!/bin/bash
# =============================================================================
# Persistent Nodes Launcher
#
# Launches long-running nodes that stay active during operation:
#   - UR Driver
#   - MoveIt
#   - Safety Visualizer
#   - Gripper Controller
#   - Cartesian Controller
#
# Usage:
#   ./start_persistent.sh [OPTIONS]
#
# Options:
#   --sim-robot           Use simulated robot (fake hardware)
#   --real-robot          Use real robot (default)
#   --robot-ip IP         Robot IP address (default: 192.168.0.100)
#   --no-rviz             Don't launch RViz with MoveIt
#   --no-safety           Don't launch Safety Visualizer
#   --step                Pause before each node launch
#   --help                Show this help message
#
# Examples:
#   ./start_persistent.sh --sim-robot
#   ./start_persistent.sh --real-robot --robot-ip 192.168.0.100
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/../../ros2_system"

# Default configuration
SIM_ROBOT=false
ROBOT_IP="192.168.0.100"
LAUNCH_RVIZ=true
LAUNCH_SAFETY=true
STEP_MODE=false

# Parse arguments
show_help() {
    head -n 25 "$0" | tail -n 21
    exit 0
}

for arg in "$@"; do
    case $arg in
        --sim-robot)
            SIM_ROBOT=true
            ;;
        --real-robot)
            SIM_ROBOT=false
            ;;
        --robot-ip)
            shift
            ROBOT_IP=$1
            ;;
        --no-rviz)
            LAUNCH_RVIZ=false
            ;;
        --no-safety)
            LAUNCH_SAFETY=false
            ;;
        --step|-s)
            STEP_MODE=true
            ;;
        --help|-h)
            show_help
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

echo "=========================================="
echo "   PERSISTENT NODES LAUNCHER"
echo "=========================================="
echo "  Robot Mode:       $([ "$SIM_ROBOT" = true ] && echo "SIMULATED" || echo "REAL")"
echo "  Robot IP:         $ROBOT_IP"
echo "  Launch RViz:      $LAUNCH_RVIZ"
echo "  Launch Safety:    $LAUNCH_SAFETY"
echo "  Step Mode:        $STEP_MODE"
echo "=========================================="
echo ""

source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

# Track PIDs for cleanup
PIDS=()

# 1. UR Driver
echo "[1/5] Starting UR5e Driver..."
wait_for_enter

if [ "$SIM_ROBOT" = true ]; then
    echo "  Mode: SIMULATED (fake_hardware)"
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e \
        robot_ip:=$ROBOT_IP \
        use_fake_hardware:=true \
        launch_rviz:=false \
        description_file:=ur5e_with_end_effector.urdf.xacro \
        description_package:=motion_control_module &
else
    echo "  Mode: REAL ROBOT"
    ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e \
        robot_ip:=$ROBOT_IP \
        use_fake_hardware:=false \
        launch_rviz:=false \
        description_file:=ur5e_with_end_effector.urdf.xacro \
        description_package:=motion_control_module &
fi
UR_PID=$!
PIDS+=($UR_PID)

echo "Waiting for UR driver to initialize..."
sleep 10

echo "Checking controllers..."
ros2 control list_controllers

if [ "$SIM_ROBOT" = false ]; then
    echo ""
    echo "*** IMPORTANT: Now load ros.urp on the UR5e teach pendant ***"
    echo "*** Connect to 192.168.0.77:50002 ***"
    read -p "Press Enter after connecting the robot..."
    echo ""
fi

# 2. MoveIt
echo "[2/5] Starting MoveIt..."
wait_for_enter

ros2 launch motion_control_module ur5e_moveit_with_gripper.launch.py \
    robot_ip:=$ROBOT_IP \
    ur_type:=ur5e \
    launch_rviz:=$LAUNCH_RVIZ &
MOVEIT_PID=$!
PIDS+=($MOVEIT_PID)

echo "Waiting for MoveIt to initialize..."
sleep 8

echo "Waiting for robot_description_semantic parameter..."
timeout 30 bash -c 'until ros2 param list /move_group 2>/dev/null | grep -q robot_description_semantic; do sleep 1; done' || echo "Warning: robot_description_semantic not found, continuing anyway..."
sleep 2

# 3. Safety Visualizer (optional)
if [ "$LAUNCH_SAFETY" = true ]; then
    echo "[3/5] Starting Safety Visualizer..."
    wait_for_enter
    python3 "${ROS2_WS}/install/motion_control_module/share/motion_control_module/scripts/safety_boundary_collision.py" &
    SAFETY_PID=$!
    PIDS+=($SAFETY_PID)
    sleep 2
else
    echo "[3/5] Skipping Safety Visualizer (--no-safety)"
    SAFETY_PID=""
fi

# 4. Gripper Controller
echo "[4/5] Starting Gripper Controller..."
wait_for_enter

ros2 run control_module gripper_controller_node \
    --ros-args \
    -p simulation_mode:=$SIM_ROBOT &
GRIPPER_PID=$!
PIDS+=($GRIPPER_PID)
sleep 2

echo "Activating gripper controller (lifecycle)..."
ros2 lifecycle set /gripper_controller_node configure
sleep 1
ros2 lifecycle set /gripper_controller_node activate
sleep 3

# 5. Cartesian Controller
echo "[5/5] Starting Cartesian Controller..."
wait_for_enter

ros2 run motion_control_module cartesian_controller_node \
    --ros-args \
    -p use_fake_hardware:=$SIM_ROBOT &
CARTESIAN_PID=$!
PIDS+=($CARTESIAN_PID)
sleep 3

echo ""
echo "=========================================="
echo "   All persistent nodes launched!"
echo "=========================================="
echo ""
echo "Running nodes:"
echo "  - UR Driver (PID: $UR_PID)"
echo "  - MoveIt (PID: $MOVEIT_PID)"
[ -n "$SAFETY_PID" ] && echo "  - Safety Visualizer (PID: $SAFETY_PID)"
echo "  - Gripper Controller (PID: $GRIPPER_PID)"
echo "  - Cartesian Controller (PID: $CARTESIAN_PID)"
echo ""
echo "Press Ctrl+C to stop all nodes..."
echo ""

# Cleanup function
cleanup() {
    echo ""
    echo "Shutting down all persistent nodes..."
    for pid in "${PIDS[@]}"; do
        kill $pid 2>/dev/null || true
    done
    echo "Done."
    exit 0
}

trap cleanup SIGINT SIGTERM

# Wait for any process to exit
wait -n

# If we get here, one process died unexpectedly
echo ""
echo "WARNING: A node has stopped unexpectedly!"
cleanup
