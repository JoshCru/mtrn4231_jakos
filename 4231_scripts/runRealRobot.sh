#!/bin/bash
# =============================================================================
# Full Real System Mode
#
# Launches the sorting system with:
# - REAL robot hardware
# - REAL perception (Kevin's nodes - must run separately)
# - REAL weight detection (Asad's weight_detection_module - launched here)
# - RViz visualization
#
# Use this for the final integrated system.
#
# PREREQUISITES:
#   - Kevin's perception nodes must be running separately
#   - Robot must be powered on and in remote control mode
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/../ros2_system"

# Get robot IP from argument or use default
ROBOT_IP=${1:-"192.168.0.100"}

echo "==========================================="
echo "   Sorting System - FULL REAL MODE"
echo "==========================================="
echo ""
echo "Configuration:"
echo "  Robot:      REAL ($ROBOT_IP)"
echo "  Perception: REAL (Kevin's nodes - external)"
echo "  Weights:    REAL (Asad's weight_detector - launched here)"
echo ""
echo "PREREQUISITES CHECK:"
echo "  [ ] Robot powered on and in remote control mode"
echo "  [ ] Robot at safe home position"
echo "  [ ] Kevin's perception nodes running separately"
echo "  [ ] Workspace clear of obstacles"
echo ""
read -p "All prerequisites met? Press Enter to continue or Ctrl+C to cancel..."

# Source ROS2 and workspace
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

echo "[1/6] Starting UR5e Driver (REAL robot at $ROBOT_IP)..."
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=$ROBOT_IP \
    initial_joint_controller:=joint_trajectory_controller \
    use_fake_hardware:=false \
    launch_rviz:=false \
    description_file:=ur5e_with_end_effector.urdf.xacro \
    description_package:=motion_control_module &

UR_PID=$!
echo "   UR Driver PID: $UR_PID"

echo ""
echo "   Waiting for UR driver to initialize..."
sleep 8

echo ""
echo "[2/6] Starting MoveIt with RViz..."
ros2 launch motion_control_module ur5e_moveit_with_gripper.launch.py \
    ur_type:=ur5e \
    launch_rviz:=true \
    use_fake_hardware:=false &

MOVEIT_PID=$!
echo "   MoveIt PID: $MOVEIT_PID"

echo ""
echo "   Waiting for MoveIt to initialize..."
sleep 10

echo ""
echo "[3/8] Starting Visualizations..."
echo "   - Safety Boundary Visualizer"
python3 "${ROS2_WS}/install/motion_control_module/share/motion_control_module/scripts/safety_boundary_collision.py" &
SAFETY_PID=$!

echo "   Safety Visualizer PID: $SAFETY_PID"
echo ""
echo "   NOTE: Expecting Kevin's perception nodes to be running!"
echo "   Required topic: /perception/detected_objects (from Kevin)"

sleep 3

echo ""
echo "[4/8] Starting Weight Detection (Asad's module)..."
ros2 run weight_detection_module weight_detector &
WEIGHT_PID=$!
echo "   Weight Detector PID: $WEIGHT_PID"

sleep 2

echo ""
echo "[5/8] Moving robot to HOME position..."
ros2 run motion_control_module go_home 5.0
echo "   Robot at home position"

echo ""
echo "[6/8] Starting Gripper Controller (real hardware)..."
ros2 run control_module gripper_controller_node --ros-args -p simulation_mode:=false &

GRIPPER_PID=$!
echo "   Gripper Controller PID: $GRIPPER_PID"

sleep 2

echo ""
echo "   Activating gripper controller (lifecycle)..."
ros2 lifecycle set /gripper_controller_node configure
sleep 1
ros2 lifecycle set /gripper_controller_node activate
sleep 3
echo "   Gripper controller ready"

sleep 2

echo ""
echo "[7/8] Starting Cartesian Controller..."
ros2 run motion_control_module cartesian_controller_node &

CARTESIAN_PID=$!
echo "   Cartesian Controller PID: $CARTESIAN_PID"

sleep 3

echo ""
echo "[8/8] Starting Sorting Brain Node..."
ros2 run supervisor_module sorting_brain_node &

SORTING_PID=$!
echo "   Sorting Brain PID: $SORTING_PID"

echo ""
echo "=========================================="
echo "   All systems launched!"
echo "=========================================="
echo ""
echo "Running processes:"
echo "  - UR Driver (REAL):     PID $UR_PID"
echo "  - MoveIt + RViz:        PID $MOVEIT_PID"
echo "  - Safety Visualizer:    PID $SAFETY_PID"
echo "  - Weight Detector:      PID $WEIGHT_PID"
echo "  - Gripper Controller:   PID $GRIPPER_PID"
echo "  - Cartesian Controller: PID $CARTESIAN_PID"
echo "  - Sorting Brain:        PID $SORTING_PID"
echo ""
echo "External dependencies (must be running):"
echo "  - Kevin's perception nodes"
echo ""
echo "To control the system:"
echo "  Open another terminal and run: ./launchDashboard.sh"
echo "  Then click 'Start' in the dashboard"
echo ""
echo "Press Ctrl+C to stop all processes..."
echo ""

# Wait for any process to exit
wait -n

# If one process exits, kill the others
echo "A process exited, shutting down..."
kill $UR_PID $MOVEIT_PID $GRIPPER_PID $CARTESIAN_PID $SAFETY_PID $WEIGHT_PID $SORTING_PID 2>/dev/null || true

echo "Done."
