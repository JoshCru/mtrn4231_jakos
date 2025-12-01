#!/bin/bash
# Test script for comparing C++ weight detection node with snapping and unsnapped outputs

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Change to the script's directory
cd "$(dirname "$0")"

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}C++ Weight Detection Snapping Comparison Test (200g)${NC}"
echo -e "${BLUE}======================================${NC}"

# Step 1: Build the package
echo -e "\n${GREEN}[1/4] Building weight_detection_module...${NC}"
colcon build --packages-select weight_detection_module
if [ $? -ne 0 ]; then
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi

# Step 2: Source the setup file
echo -e "\n${GREEN}[2/4] Sourcing install/setup.bash...${NC}"
source install/setup.bash

# Step 3: Launch nodes in background
echo -e "\n${GREEN}[3/4] Launching weight detection nodes...${NC}"

echo -e "${BLUE}Starting C++ node (useSnapping=true -> /estimated_mass_snapped)...${NC}"
ros2 run weight_detection_module weight_detector --ros-args -p useSnapping:=true -r /estimated_mass:=/estimated_mass_snapped &
SNAPPED_PID=$!

sleep 1

echo -e "${BLUE}Starting C++ node (useSnapping=false -> /estimated_mass_unsnapped)...${NC}"
ros2 run weight_detection_module weight_detector --ros-args -p useSnapping:=false -r /estimated_mass:=/estimated_mass_unsnapped &
UNSNAPPED_PID=$!

sleep 2

# Step 4: Play rosbag
echo -e "\n${GREEN}[4/4] Playing rosbag...${NC}"
echo -e "${BLUE}Bag: rosbag2_200_3cm_lift${NC}"
ros2 bag play ../rosbags2/rosbag2_200_3cm_lift

# Cleanup function
cleanup() {
    echo -e "\n${RED}Stopping all nodes...${NC}"
    kill $SNAPPED_PID 2>/dev/null
    kill $UNSNAPPED_PID 2>/dev/null
    # Force kill the actual binaries
    pkill -f "weight_detection_module/weight_detector" 2>/dev/null
    echo -e "${GREEN}Done!${NC}"
}

# Register cleanup on exit
trap cleanup EXIT

echo -e "\n${GREEN}Test complete!${NC}"
echo -e "${BLUE}Press Ctrl+C to stop all nodes${NC}"

# Wait for user to stop
wait
