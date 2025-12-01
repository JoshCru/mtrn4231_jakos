#!/bin/bash
# =============================================================================
# Full Simulation Mode
#
# Launches the sorting system with:
# - Fake robot hardware (simulation)
# - Simulated perception (4 random weights)
# - Simulated weight estimation
# - RViz visualization
#
# Use this for testing without any real hardware.
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/../ros2_system"

echo "==========================================="
echo "   Sorting System - SIMULATION MODE"
echo "==========================================="
echo ""
echo "Configuration:"
echo "  Robot:      FAKE (simulation)"
echo "  Perception: SIMULATED"
echo "  Weights:    SIMULATED"
echo ""

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

# Set FastDDS profile for reliable communication
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_profile.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Launch using the unified launch file
ros2 launch full_system.launch.py mode:=simulation
