#!/bin/bash
# =============================================================================
# Sorting System Dashboard Launcher
#
# This script launches the UI dashboard for monitoring and controlling
# the sorting system.
#
# Usage:
#   ./launchDashboard.sh
#
# Note: The sorting system must already be running (via runSortingSimulation.sh)
# =============================================================================

set -e

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS2_WS="${SCRIPT_DIR}/../ros2_system"

echo "==========================================="
echo "   Sorting System Dashboard"
echo "==========================================="

# Source ROS2 and workspace
source /opt/ros/humble/setup.bash
source "${ROS2_WS}/install/setup.bash"

echo ""
echo "Starting dashboard UI..."
echo ""
echo "Dashboard Controls:"
echo "  ▶ Start   - Begin sorting operation"
echo "  ⏸ Stop    - Pause sorting"
echo "  🔄 Reset   - Clear all sorted weights"
echo "  ⚠ E-Stop  - Emergency stop"
echo ""

# Launch the dashboard
ros2 run supervisor_module system_dashboard

echo ""
echo "Dashboard closed."
