#!/bin/bash
# Test script for C++ weight detection node with 200g rosbag

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Change to the script's directory
cd "$(dirname "$0")"

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}C++ Weight Detection Test - 200g${NC}"
echo -e "${BLUE}======================================${NC}"

# Step 1: Build the package
echo -e "\n${GREEN}[1/3] Building weight_detection_package...${NC}"
colcon build --packages-select weight_detection_package
if [ $? -ne 0 ]; then
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi

# Step 2: Source the setup file
echo -e "\n${GREEN}[2/3] Sourcing install/setup.bash...${NC}"
source install/setup.bash

# Step 3: Launch C++ node in background
echo -e "\n${GREEN}[3/3] Launching C++ weight detection node...${NC}"
echo -e "${BLUE}Starting C++ node (/estimated_mass)...${NC}"
ros2 run weight_detection_package weight_detector &
CPP_PID=$!

sleep 2

# Step 4: Play rosbag
echo -e "\n${GREEN}[4/4] Playing rosbag...${NC}"
echo -e "${BLUE}Bag: rosbag2_200_3cm_lift${NC}"
ros2 bag play ../rosbags2/rosbag2_200_3cm_lift

# Cleanup function
cleanup() {
    echo -e "\n${RED}Stopping C++ node...${NC}"
    kill $CPP_PID 2>/dev/null
    # Force kill the actual binary in case ros2 run didn't propagate the signal
    pkill -f "weight_detection_package/weight_detector" 2>/dev/null
    echo -e "${GREEN}Done!${NC}"
}

# Register cleanup on exit
trap cleanup EXIT

echo -e "\n${GREEN}Test complete!${NC}"
echo -e "${BLUE}Press Ctrl+C to stop the node${NC}"

# Wait for user to stop
wait
