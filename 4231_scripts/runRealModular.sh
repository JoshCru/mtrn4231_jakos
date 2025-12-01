#!/bin/bash
# =============================================================================
# Real Mode (Real Robot + Real Perception + Real Weight) - Modular Launch
#
# Usage:
#   ./runRealModular.sh [ROBOT_IP] [--autorun] [--step]
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
echo "   Sorting System - REAL MODE (Modular)"
echo "==========================================="
echo "  Robot IP: $ROBOT_IP"
echo "  Autorun:  $AUTORUN"
echo "  Step:     $STEP_MODE"
echo "==========================================="
echo ""
echo "PREREQUISITES:"
echo "  [✓] Robot powered on and booted"
echo "  [✓] Kevin's perception nodes running"
echo "  [✓] Workspace clear of obstacles"
echo "  [✓] Camera calibrated and connected"
echo ""
read -p "All prerequisites met? Press Enter to continue or Ctrl+C to cancel..."

source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

# Set FastDDS profile for reliable communication
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_profile.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Create FastDDS profile
cat > /tmp/fastdds_profile.xml << 'EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomUdpTransport</transport_id>
            <type>UDPv4</type>
        </transport_descriptor>
    </transport_descriptors>
    <participant profile_name="participant_profile" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>CustomUdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>
</profiles>
EOF

echo "FastDDS profile created"
echo ""

# 1. UR Driver
echo "[1/9] Starting UR5e Driver..."
wait_for_enter
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=$ROBOT_IP \
    initial_joint_controller:=scaled_joint_trajectory_controller \
    use_fake_hardware:=false \
    launch_rviz:=false \
    description_file:=ur5e_with_end_effector.urdf.xacro \
    description_package:=motion_control_module &
UR_PID=$!

echo "Waiting for UR driver to initialize..."
sleep 10

echo "Checking controllers..."
ros2 control list_controllers

echo ""
echo "*** IMPORTANT: Now load ros.urp on the UR5e teach pendant ***"
echo "*** Connect to 192.168.0.77:50002 ***"
read -p "Press Enter after connecting the robot..."
echo ""

# 2. MoveIt
echo "[2/9] Starting MoveIt with RViz..."
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
echo "[3/9] Moving robot to HOME position..."
wait_for_enter
ros2 run motion_control_module go_home 5.0
echo "Robot at home position"
sleep 2

# 4. Safety Visualizer
echo "[4/9] Starting Safety Visualizer..."
wait_for_enter
python3 "${ROS2_WS}/install/motion_control_module/share/motion_control_module/scripts/safety_boundary_collision.py" &
SAFETY_PID=$!
sleep 2

# 5. Real Perception (Kevin's nodes should be running externally)
echo "[5/9] NOTE: Expecting Kevin's perception nodes to be running externally!"
echo "           Checking for /detected_objects topic..."
timeout 5 ros2 topic echo /detected_objects --once 2>/dev/null && echo "Perception topic found!" || echo "Warning: /detected_objects not publishing yet"
sleep 2

# 6. Weight Detection (Real)
echo "[6/9] Starting Real Weight Detection..."
wait_for_enter
ros2 run weight_detection_module weight_detector &
WEIGHT_PID=$!
sleep 3

# 7. Gripper Controller
echo "[7/9] Starting Gripper Controller..."
wait_for_enter
ros2 run control_module gripper_controller_node \
    --ros-args \
    -p simulation_mode:=false &
GRIPPER_PID=$!
sleep 2

echo "Activating gripper controller (lifecycle)..."
ros2 lifecycle set /gripper_controller_node configure
sleep 1
ros2 lifecycle set /gripper_controller_node activate
sleep 3

# 8. Cartesian Controller
echo "[8/9] Starting Cartesian Controller..."
wait_for_enter
ros2 run motion_control_module cartesian_controller_node \
    --ros-args \
    -p use_fake_hardware:=false &
CARTESIAN_PID=$!
sleep 3

# 9. Sorting Brain
echo "[9/9] Starting Sorting Brain..."
wait_for_enter
ros2 run supervisor_module sorting_brain_node &
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
kill $UR_PID $MOVEIT_PID $GRIPPER_PID $CARTESIAN_PID $SAFETY_PID $WEIGHT_PID $SORTING_PID 2>/dev/null || true
echo "Done."
