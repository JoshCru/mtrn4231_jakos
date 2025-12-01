#!/bin/bash
# =============================================================================
# Sorting System Simulation Launcher
#
# This script launches the complete sorting simulation:
# 1. UR5e driver with fake hardware
# 2. MoveIt motion planning
# 3. Visualization (safety boundaries, zones, weights) - BEFORE go_home
# 4. Go home
# 5. Cartesian controller
# 6. Sorting brain node
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/../ros2_system"

echo "=========================================="
echo "   Sorting System Simulation Launcher"
echo "=========================================="

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

# Set FastDDS profile for reliable communication
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_profile.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Create FastDDS profile if needed
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

echo ""
echo "[1/8] Starting UR5e Driver (fake hardware)..."
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=yyy.yyy.yyy.yyy \
    initial_joint_controller:=joint_trajectory_controller \
    use_fake_hardware:=true \
    launch_rviz:=false \
    description_file:=ur5e_with_end_effector.urdf.xacro \
    description_package:=motion_control_module &

UR_PID=$!
echo "   UR Driver PID: $UR_PID"

echo ""
echo "   Waiting for UR driver to initialize..."
sleep 5

echo ""
echo "[2/8] Starting MoveIt..."
ros2 launch motion_control_module ur5e_moveit_with_gripper.launch.py \
    ur_type:=ur5e \
    launch_rviz:=true \
    use_fake_hardware:=true &

MOVEIT_PID=$!
echo "   MoveIt PID: $MOVEIT_PID"

echo ""
echo "   Waiting for MoveIt to initialize..."
sleep 10

echo ""
echo "[3/8] Starting Visualizations (BEFORE go_home)..."
echo "   - Safety Boundary Visualizer"
python3 "${ROS2_WS}/install/motion_control_module/share/motion_control_module/scripts/safety_boundary_collision.py" &
SAFETY_PID=$!

echo "   - Simulated Perception Node (weights & zones)"
ros2 run supervisor_module simulated_perception_node \
    --ros-args -p num_objects:=4 -p publish_rate:=5.0 -p randomize_positions:=true &
PERCEPTION_PID=$!

echo "   Safety Visualizer PID: $SAFETY_PID"
echo "   Perception Node PID: $PERCEPTION_PID"

sleep 3

echo ""
echo "=========================================="
echo "   RViz Markers now visible!"
echo "=========================================="
echo "   Add these MarkerArray displays in RViz:"
echo "   - /safety_boundaries               (orange box)"
echo "   - /perception/weight_markers       (gold weights)"
echo "   - /perception/zone_markers         (green/blue zones)"
echo "   - /sorting/placed_weight_markers   (green placed weights)"
echo "   - /sorting/held_weight_marker      (orange held weight)"
echo "=========================================="
echo ""

echo "[4/8] Moving robot to HOME position..."
echo "   (Required for Cartesian path planning to work)"
ros2 run motion_control_module go_home 5.0
echo "   Robot at home position"

echo ""
echo "[5/8] Starting Cartesian Controller..."
ros2 run motion_control_module cartesian_controller_node &

CARTESIAN_PID=$!
echo "   Cartesian Controller PID: $CARTESIAN_PID"

sleep 3

echo ""
echo "[6/8] Starting Sorting Brain Node..."
ros2 run supervisor_module sorting_brain_node &

SORTING_PID=$!
echo "   Sorting Brain PID: $SORTING_PID"

echo ""
echo "=========================================="
echo "   All systems launched!"
echo "=========================================="
echo ""
echo "Running processes:"
echo "  - UR Driver (fake):     PID $UR_PID"
echo "  - MoveIt:               PID $MOVEIT_PID"
echo "  - Safety Visualizer:    PID $SAFETY_PID"
echo "  - Perception Node:      PID $PERCEPTION_PID"
echo "  - Cartesian Controller: PID $CARTESIAN_PID"
echo "  - Sorting Brain:        PID $SORTING_PID"
echo ""
echo "To monitor sorting status:"
echo "  ros2 topic echo /sorting/status"
echo "  ros2 topic echo /sorting/state"
echo ""
echo "To see detected objects:"
echo "  ros2 topic echo /perception/detected_objects"
echo ""
echo "Press Ctrl+C to stop all processes..."
echo ""

# Wait for any process to exit
wait -n

# If one process exits, kill the others
echo "A process exited, shutting down..."
kill $UR_PID $MOVEIT_PID $CARTESIAN_PID $SAFETY_PID $PERCEPTION_PID $SORTING_PID 2>/dev/null || true

echo "Done."
