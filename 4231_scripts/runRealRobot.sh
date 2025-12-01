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
# Usage:
#   ./runRealRobot.sh [ROBOT_IP] [--autorun]
#
# Options:
#   ROBOT_IP    Robot IP address (default: 192.168.0.100)
#   --autorun   Automatically start sorting without dashboard
#
# PREREQUISITES:
#   - Kevin's perception nodes must be running separately
#   - Robot must be powered on and in remote control mode
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/../ros2_system"

# Parse arguments
ROBOT_IP="192.168.0.100"
AUTORUN=false

for arg in "$@"; do
    case $arg in
        --autorun|-a)
            AUTORUN=true
            ;;
        *)
            # Assume it's the robot IP if it looks like an IP address
            if [[ $arg =~ ^[0-9]+\.[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
                ROBOT_IP=$arg
            fi
            ;;
    esac
done

echo "==========================================="
echo "   Sorting System - FULL REAL MODE"
echo "==========================================="
echo ""
echo "Configuration:"
echo "  Robot:      REAL ($ROBOT_IP)"
echo "  Perception: REAL (Kevin's nodes - external)"
echo "  Weights:    REAL (Asad's weight_detector - launched here)"
echo "  Autorun:    $AUTORUN"
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

# Build launch arguments
LAUNCH_ARGS="mode:=real robot_ip:=$ROBOT_IP"

if [ "$AUTORUN" = true ]; then
    LAUNCH_ARGS="$LAUNCH_ARGS autorun:=true"
fi

# Launch using the unified launch file
cd "${ROS2_WS}"
ros2 launch launch/full_system.launch.py $LAUNCH_ARGS
