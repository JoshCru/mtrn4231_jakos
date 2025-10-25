#!/bin/bash
# Convenient test runner script for recognition module
# Usage: ./run_tests.sh [options]

set -e

# Colors for output
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# Default values
WORKSPACE_ROOT="$(cd "$(dirname "$0")/../../.." && pwd)"
VERBOSE=""
KEEP_ALIVE=false

# Help message
show_help() {
    cat << EOF
Usage: ./run_tests.sh [OPTIONS]

Run integration tests for the recognition module with mock camera.

OPTIONS:
    -h, --help          Show this help message
    -v, --verbose       Run tests with verbose output
    -k, --keep-alive    Keep nodes running after tests (for debugging)

EXAMPLES:
    ./run_tests.sh                          # Run all tests
    ./run_tests.sh --verbose                # Run with verbose output
    ./run_tests.sh -k                       # Run and keep nodes alive

EOF
}

# Parse arguments
while [[ $# -gt 0 ]]; do
    case $1 in
        -h|--help)
            show_help
            exit 0
            ;;
        -v|--verbose)
            VERBOSE="-v -s"
            shift
            ;;
        -k|--keep-alive)
            KEEP_ALIVE=true
            shift
            ;;
        *)
            echo -e "${RED}Unknown option: $1${NC}"
            show_help
            exit 1
            ;;
    esac
done

# Print header
echo -e "${GREEN}========================================${NC}"
echo -e "${GREEN}Recognition Module Test Runner${NC}"
echo -e "${GREEN}========================================${NC}"
echo ""

# Check if workspace exists
if [ ! -d "$WORKSPACE_ROOT" ]; then
    echo -e "${RED}Error: Workspace not found at $WORKSPACE_ROOT${NC}"
    exit 1
fi

echo "Workspace: $WORKSPACE_ROOT"
echo ""

# Build the workspace
echo -e "${YELLOW}Step 1: Building workspace...${NC}"
cd "$WORKSPACE_ROOT"
colcon build --packages-select recognition_module sort_interfaces
if [ $? -ne 0 ]; then
    echo -e "${RED}Build failed!${NC}"
    exit 1
fi
echo -e "${GREEN}Build successful!${NC}"
echo ""

# Source the workspace
echo -e "${YELLOW}Step 2: Sourcing workspace...${NC}"
source "$WORKSPACE_ROOT/install/setup.bash"
echo -e "${GREEN}Workspace sourced!${NC}"
echo ""

# Launch nodes in background
echo -e "${YELLOW}Step 3: Launching recognition module with mock camera...${NC}"
ros2 launch recognition_module recognition_with_mock_camera.launch.py > /tmp/recognition_nodes.log 2>&1 &
LAUNCH_PID=$!
echo "Launch PID: $LAUNCH_PID"
echo "Waiting for nodes to start..."
sleep 5  # Recognition needs a bit more time for PCL initialization
echo -e "${GREEN}Nodes launched!${NC}"
echo ""

# Function to cleanup on exit
cleanup() {
    if [ "$KEEP_ALIVE" = false ]; then
        echo ""
        echo -e "${YELLOW}Cleaning up...${NC}"
        kill $LAUNCH_PID 2>/dev/null || true
        wait $LAUNCH_PID 2>/dev/null || true
        echo -e "${GREEN}Cleanup complete!${NC}"
    else
        echo ""
        echo -e "${YELLOW}Keeping nodes alive (PID: $LAUNCH_PID)${NC}"
        echo -e "${YELLOW}To stop nodes, run: kill $LAUNCH_PID${NC}"
        echo ""
        echo "You can now manually test:"
        echo "  ros2 topic echo /recognition/estimated_weights"
        echo "  ros2 topic echo /camera/pointcloud"
    fi
}
trap cleanup EXIT

# Run tests
echo -e "${YELLOW}Step 4: Running tests...${NC}"
echo ""

cd "$WORKSPACE_ROOT"

# Run recognition tests
echo -e "${GREEN}Running recognition_node tests...${NC}"
colcon test --packages-select recognition_module --pytest-args "$VERBOSE"

TEST_RESULT=$?

echo ""
echo -e "${YELLOW}Step 5: Test results...${NC}"
echo ""

# Show test results
colcon test-result --verbose

echo ""

if [ $TEST_RESULT -eq 0 ]; then
    echo -e "${GREEN}========================================${NC}"
    echo -e "${GREEN}All tests PASSED! ✓${NC}"
    echo -e "${GREEN}========================================${NC}"
else
    echo -e "${RED}========================================${NC}"
    echo -e "${RED}Some tests FAILED! ✗${NC}"
    echo -e "${RED}========================================${NC}"
    echo ""
    echo "Check logs for details:"
    echo "  - Node logs: /tmp/recognition_nodes.log"
    echo "  - Test logs: $WORKSPACE_ROOT/log/latest_test/"
fi

exit $TEST_RESULT
