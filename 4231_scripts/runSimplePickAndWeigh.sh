#!/bin/bash
# =============================================================================
# Simple Pick and Weigh - Real or Simulated Robot
#
# Usage:
#   ./runSimplePickAndWeigh.sh [ROBOT_IP] [--step] [--sim-perception] [--grip-weight GRAMS] [--fake]
#
# Options:
#   ROBOT_IP           Robot IP address (default: 192.168.0.100)
#   --step             Pause before each node launch (for testing)
#   --sim-perception   Enable simulated perception (default: off)
#   --grip-weight      Weight for gripper angle in grams (default: 100)
#   --fake             Use fake hardware (simulation mode)
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/../ros2_system"

# Parse arguments
ROBOT_IP="192.168.0.100"
STEP_MODE=false
SIM_PERCEPTION=false
GRIP_WEIGHT=100
USE_FAKE_HARDWARE=false

while [[ $# -gt 0 ]]; do
    case $1 in
        --step|-s)
            STEP_MODE=true
            shift
            ;;
        --sim-perception|-p)
            SIM_PERCEPTION=true
            shift
            ;;
        --grip-weight|-g)
            GRIP_WEIGHT="$2"
            shift 2
            ;;
        --fake|-f)
            USE_FAKE_HARDWARE=true
            shift
            ;;
        *)
            if [[ $1 =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
                ROBOT_IP=$1
            fi
            shift
            ;;
    esac
done

# Helper function for step mode
wait_for_enter() {
    if [ "$STEP_MODE" = true ]; then
        read -p "Press Enter to continue..."
    fi
}

if [ "$USE_FAKE_HARDWARE" = true ]; then
    MODE_TEXT="SIMULATION"
else
    MODE_TEXT="REAL ROBOT"
fi

echo "==========================================="
echo "   Simple Pick and Weigh - ${MODE_TEXT}"
echo "==========================================="
echo "  Robot IP:         $ROBOT_IP"
echo "  Fake Hardware:    $USE_FAKE_HARDWARE"
echo "  Step Mode:        $STEP_MODE"
echo "  Sim Perception:   $SIM_PERCEPTION"
echo "  Grip Weight:      ${GRIP_WEIGHT}g"
echo "==========================================="
echo ""

source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"


# 1. UR Driver
echo "[1/7] Starting UR5e Driver..."
wait_for_enter
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=$ROBOT_IP \
    initial_joint_controller:=scaled_joint_trajectory_controller \
    use_fake_hardware:=$USE_FAKE_HARDWARE \
    launch_rviz:=false \
    description_file:=ur5e_with_end_effector.urdf.xacro \
    description_package:=motion_control_module &
UR_PID=$!

echo "Waiting for UR driver to initialize..."
sleep 10

echo "Checking controllers..."
ros2 control list_controllers

if [ "$USE_FAKE_HARDWARE" = false ]; then
    echo ""
    echo "*** IMPORTANT: Now load ros.urp on the UR5e teach pendant ***"
    echo "*** Connect to 192.168.0.77:50002 ***"
    read -p "Press Enter after connecting the robot..."
    echo ""
else
    echo ""
    echo "*** SIMULATION MODE: Skipping robot connection prompt ***"
    echo ""
fi

# 2. MoveIt
echo "[2/7] Starting MoveIt with RViz..."
wait_for_enter
ros2 launch motion_control_module ur5e_moveit_with_gripper.launch.py \
    robot_ip:=$ROBOT_IP \
    ur_type:=ur5e \
    launch_rviz:=true &
MOVEIT_PID=$!

echo "Waiting for MoveIt to initialize..."
sleep 8

echo "Waiting for robot_description_semantic parameter..."
timeout 30 bash -c 'until ros2 param list /move_group 2>/dev/null | grep -q robot_description_semantic; do sleep 1; done' || echo "Warning: robot_description_semantic not found, continuing anyway..."
sleep 2

# 3. Go Home
echo "[3/7] Moving robot to HOME position..."
wait_for_enter
ros2 run motion_control_module go_home 5.0
echo "Robot at home position"
sleep 2


# 5. Simulated Perception (optional)
PERCEPTION_PID=""
if [ "$SIM_PERCEPTION" = true ]; then
    echo "[5/7] Starting Simulated Perception..."
    wait_for_enter
    ros2 run supervisor_module simulated_perception_node \
        --ros-args \
        -p num_objects:=4 \
        -p publish_rate:=5.0 \
        -p randomize_positions:=true &
    PERCEPTION_PID=$!
    sleep 2
else
    echo "[5/7] Skipping Simulated Perception (--sim-perception not set)"
fi

# 6. Weight Detection
WEIGHT_PID=""
if [ "$USE_FAKE_HARDWARE" = true ]; then
    echo "[6/7] Starting Real Weight Detection..."
    wait_for_enter
    ros2 run weight_detection_module weight_detector &
    WEIGHT_PID=$!
    sleep 2

    # Launch PlotJuggler for weight visualization (after weight detector starts)
    echo "Starting PlotJuggler for weight visualization..."
    "${ROS2_WS}/plot_weight.sh" &
    PLOT_PID=$!
    sleep 2
else
    echo "[6/7] Skipping Weight Detection (fake hardware - no real weight sensor)"
    PLOT_PID=""
fi

# 7. Gripper Controller
echo "[7/7] Starting Gripper Controller..."
wait_for_enter
ros2 run control_module gripper_controller_node \
    --ros-args \
    -p simulation_mode:=$USE_FAKE_HARDWARE &
GRIPPER_PID=$!
sleep 2

echo "Activating gripper controller (lifecycle)..."
ros2 lifecycle set /gripper_controller_node configure
sleep 1
ros2 lifecycle set /gripper_controller_node activate
sleep 3

# 8. Cartesian Controller
echo "[8/8] Starting Cartesian Controller..."
wait_for_enter
ros2 run motion_control_module cartesian_controller_node \
    --ros-args \
    -p use_fake_hardware:=false &
CARTESIAN_PID=$!
sleep 3

echo ""
echo "=========================================="
echo "   All systems launched!"
echo "=========================================="
echo ""
echo "Now running simple pick and weigh (C++ version)..."
echo ""

# Run the C++ simple pick and weigh node with initial positioning enabled
ros2 run motion_control_module simple_pick_and_weigh_node \
    --ros-args \
    -p grip_weight:=${GRIP_WEIGHT} \
    -p initial_positioning:=true

echo ""
echo "Pick and weigh complete!"
echo ""
echo "Press Ctrl+C to stop all systems..."

# Wait for any process to exit
wait -n

echo ""
echo "Shutting down..."
kill $UR_PID $MOVEIT_PID $GRIPPER_PID $CARTESIAN_PID $SAFETY_PID $WEIGHT_PID $PERCEPTION_PID $PLOT_PID 2>/dev/null || true
echo "Done."
