#!/bin/bash
# Build script for ROS2 Sort System

set -e  # Exit on error

echo "========================================="
echo "ROS2 Sort-by-Weight System - Build Script"
echo "========================================="
echo ""

# Colors for output
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Check if we're in the right directory
if [ ! -d "src" ]; then
    echo -e "${RED}Error: src directory not found. Please run this script from the workspace root.${NC}"
    exit 1
fi

# Source ROS2
echo -e "${YELLOW}Sourcing ROS2 Humble...${NC}"
source /opt/ros/humble/setup.bash

# Install dependencies
echo ""
echo -e "${YELLOW}Installing dependencies with rosdep...${NC}"
rosdep install --from-paths src --ignore-src -r -y

# Clean build (optional - uncomment if needed)
# echo ""
# echo -e "${YELLOW}Cleaning previous build...${NC}"
# rm -rf build install log

# Build
echo ""
echo -e "${YELLOW}Building all packages...${NC}"
colcon build --symlink-install --cmake-args -DCMAKE_BUILD_TYPE=Release

# Check build result
if [ $? -eq 0 ]; then
    echo ""
    echo -e "${GREEN}=========================================${NC}"
    echo -e "${GREEN}Build completed successfully!${NC}"
    echo -e "${GREEN}=========================================${NC}"
    echo ""
    echo "To use the system, source the workspace:"
    echo "  source install/setup.bash"
    echo ""
    echo "Then launch the complete system:"
    echo "  ros2 launch launch/full_system.launch.py"
    echo ""
    echo "Or launch individual modules:"
    echo "  ros2 launch supervisor_module supervisor.launch.py"
    echo "  ros2 launch perception_module perception.launch.py"
    echo "  ros2 launch recognition_module recognition.launch.py"
    echo "  ros2 launch planning_module planning.launch.py"
    echo "  ros2 launch control_module control.launch.py"
    echo "  ros2 launch motion_control_module motion_control.launch.py"
    echo ""
else
    echo ""
    echo -e "${RED}=========================================${NC}"
    echo -e "${RED}Build failed! Check errors above.${NC}"
    echo -e "${RED}=========================================${NC}"
    exit 1
fi
