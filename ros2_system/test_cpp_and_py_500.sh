#!/bin/bash
# Test script for comparing C++ and Python weight detection nodes

# Colors for output
GREEN='\033[0;32m'
BLUE='\033[0;34m'
RED='\033[0;31m'
NC='\033[0m' # No Color

# Change to the script's directory
cd "$(dirname "$0")"

echo -e "${BLUE}======================================${NC}"
echo -e "${BLUE}Weight Detection Comparison Test${NC}"
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

echo -e "${BLUE}Starting Python node (/estimated_mass_py)...${NC}"
ros2 run weight_detection_module weight_detector_py --ros-args -r /estimated_mass:=/estimated_mass_py &
PY_PID=$!

sleep 1

echo -e "${BLUE}Starting C++ node (/estimated_mass_cpp)...${NC}"
ros2 run weight_detection_module weight_detector --ros-args -r /estimated_mass:=/estimated_mass_cpp &
CPP_PID=$!

sleep 2

# Step 4: Play rosbag
echo -e "\n${GREEN}[4/4] Playing rosbag...${NC}"
echo -e "${BLUE}Bag: rosbag2_500_3cm_lift${NC}"
ros2 bag play ../rosbags2/rosbag2_500_3cm_lift

# Cleanup function
cleanup() {
    echo -e "\n${RED}Stopping all nodes...${NC}"
    kill $PY_PID 2>/dev/null
    kill $CPP_PID 2>/dev/null
    # Force kill the actual binaries
    pkill -f "weight_detection_module/weight_detector" 2>/dev/null
    pkill -f "weight_detector_py" 2>/dev/null
    echo -e "${GREEN}Done!${NC}"
}

# Register cleanup on exit
trap cleanup EXIT

echo -e "\n${GREEN}Test complete!${NC}"
echo -e "${BLUE}Press Ctrl+C to stop all nodes${NC}"

# Wait for user to stop
wait
