#!/bin/bash
# =============================================================================
# Hybrid Mode (Real Robot + Simulated Perception)
#
# Usage:
#   ./runHybrid.sh [ROBOT_IP] [--autorun] [--real-weight-detection]
#
# Options:
#   ROBOT_IP                 Robot IP address (default: 192.168.0.100)
#   --autorun                Automatically start sorting without dashboard
#   --real-weight-detection  Use real C++ weight detector instead of simulated
# =============================================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/../ros2_system"

# Parse arguments
ROBOT_IP="192.168.0.100"
AUTORUN=false
REAL_WEIGHT_DETECTION=false

for arg in "$@"; do
    case $arg in
        --autorun|-a)
            AUTORUN=true
            ;;
        --real-weight-detection|-w)
            REAL_WEIGHT_DETECTION=true
            ;;
        *)
            if [[ $arg =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
                ROBOT_IP=$arg
            fi
            ;;
    esac
done

echo "==========================================="
echo "   Sorting System - HYBRID MODE"
echo "==========================================="
echo "  Robot IP:            $ROBOT_IP"
echo "  Autorun:             $AUTORUN"
echo "  Real Weight Detect:  $REAL_WEIGHT_DETECTION"
echo "==========================================="
echo ""

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

# Set FastDDS profile for reliable communication
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_profile.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Build launch arguments
LAUNCH_ARGS="mode:=hybrid robot_ip:=$ROBOT_IP"

if [ "$AUTORUN" = true ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS autorun:=true"
fi

if [ "$REAL_WEIGHT_DETECTION" = true ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS real_weight_detection:=true"
fi

# Launch using the unified launch file
ros2 launch full_system.launch.py $LAUNCH_ARGS
