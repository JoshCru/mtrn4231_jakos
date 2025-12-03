#!/bin/bash
# =============================================================================
# Simulation Mode (Fake Hardware) - Modular Launch
#
# Usage:
#   ./runSimulationModular.sh [--autorun] [--step]
#
# Options:
#   --autorun   Automatically start sorting without dashboard
#   --step      Pause before each node launch (for testing)
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/../ros2_system"

# Parse arguments
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
    esac
done

# Helper function for step mode
wait_for_enter() {
    if [ "$STEP_MODE" = true ]; then
        read -p "Press Enter to continue..."
    fi
}

echo "==========================================="
echo "   Sorting System - SIMULATION MODE"
echo "==========================================="
echo "  Autorun: $AUTORUN"
echo "  Step:    $STEP_MODE"
echo "==========================================="
echo "Note: Simulated perception - position check will run automatically"
echo ""

source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

# 1. UR Driver (Fake Hardware)
echo "[1/8] Starting UR5e Driver (SIMULATION)..."
wait_for_enter
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=yyy.yyy.yyy.yyy \
    initial_joint_controller:=scaled_joint_trajectory_controller \
    use_fake_hardware:=true \
    launch_rviz:=false \
    description_file:=ur5e_with_end_effector.urdf.xacro \
    description_package:=motion_control_package &
UR_PID=$!

echo "Waiting for UR driver to initialize..."
sleep 5

echo "Checking controllers..."
ros2 control list_controllers

# 2. MoveIt
echo "[2/8] Starting MoveIt with RViz..."
wait_for_enter
ros2 launch motion_control_package ur5e_moveit_with_gripper.launch.py \
    ur_type:=ur5e \
    launch_rviz:=true \
    use_fake_hardware:=true &
MOVEIT_PID=$!

echo "Waiting for MoveIt to initialize..."
sleep 8

echo "Waiting for robot_description_semantic parameter..."
timeout 30 bash -c 'until ros2 param list /move_group 2>/dev/null | grep -q robot_description_semantic; do sleep 1; done' || echo "Warning: robot_description_semantic not found, continuing anyway..."
sleep 2

# 3. Go Home
echo "[3/8] Moving robot to HOME position..."
wait_for_enter
ros2 run motion_control_package go_home 5.0
echo "Robot at home position"
sleep 2

# 4. Safety Visualizer
echo "[4/8] Starting Safety Visualizer..."
wait_for_enter
python3 "${ROS2_WS}/install/motion_control_package/share/motion_control_package/scripts/safety_boundary_collision.py" &
SAFETY_PID=$!
sleep 2

# 5. Simulated Perception
echo "[5/8] Starting Simulated Perception..."
wait_for_enter
ros2 run perception_package simulated_perception_node \
    --ros-args \
    -p num_objects:=4 \
    -p publish_rate:=5.0 \
    -p randomize_positions:=true &
PERCEPTION_PID=$!
sleep 2

# 6. No Weight Detection (simulated weights from perception)
echo "[6/8] Skipping Weight Detection (using simulated weights)"

# 7. Gripper Controller (Simulation Mode)
echo "[7/8] Starting Gripper Controller (SIMULATION)..."
wait_for_enter
ros2 run control_package gripper_controller_node \
    --ros-args \
    -p simulation_mode:=true &
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
ros2 run motion_control_package cartesian_controller_node \
    --ros-args \
    -p use_fake_hardware:=true &
CARTESIAN_PID=$!
sleep 3

# Position Check (automatic for simulated perception)
echo ""
echo "=========================================="
echo "   SIMULATED PERCEPTION POSITION CHECK"
echo "=========================================="
echo ""
echo "Running position check for simulated weights..."
echo "The robot will visit each simulated weight position at Z_PICKUP."
echo ""
python3 "${ROS2_WS}/src/motion_control_package/scripts/check_simulated_positions.py"

if [ $? -eq 0 ]; then
    echo ""
    echo "Position check completed successfully!"
    echo ""
else
    echo ""
    echo "Position check failed or was cancelled!"
    read -p "Continue anyway? (y/n): " continue_response
    if [[ ! $continue_response =~ ^[Yy]$ ]]; then
        echo "Stopping..."
        kill $UR_PID $MOVEIT_PID $GRIPPER_PID $CARTESIAN_PID $SAFETY_PID $PERCEPTION_PID 2>/dev/null || true
        exit 1
    fi
fi

# 9. Sorting Brain
echo "[9/9] Starting Sorting Brain..."
wait_for_enter
ros2 run supervisor_package sorting_brain_node &
SORTING_PID=$!
sleep 2

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

# Wait for any process to exit
wait -n

echo ""
echo "Shutting down..."
kill $UR_PID $MOVEIT_PID $GRIPPER_PID $CARTESIAN_PID $SAFETY_PID $PERCEPTION_PID $SORTING_PID 2>/dev/null || true
echo "Done."
