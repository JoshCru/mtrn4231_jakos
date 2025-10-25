# Planning Module Integration Tests

This directory contains integration tests for all planning module nodes.

## Test Overview

### Test Files

| Test File | Node Under Test | Test Count | Description |
|-----------|----------------|------------|-------------|
| `test_sort_node.py` | sort_node | 7 tests | Sorting decision logic, weight matching, boundary cases |
| `test_verification_node.py` | verification_node | 10 tests | Action server, weight verification, tolerance checking |
| `test_integrity_node.py` | integrity_node | 13 tests | Workspace validation service, environment monitoring |
| `test_moveit2_interface_node.py` | moveit2_interface_node | 12 tests | Action servers (stub implementation) |

**Total: 42 integration tests**

---

## Running the Tests

### Prerequisites

1. **Build the workspace:**
   ```bash
   cd /path/to/ros2_system
   colcon build --packages-select planning_module sort_interfaces
   source install/setup.bash
   ```

2. **Ensure all nodes are launched:**
   The tests require the actual planning nodes to be running. Launch them with:
   ```bash
   ros2 launch planning_module planning.launch.py
   ```

### Run All Tests

```bash
# From workspace root
colcon test --packages-select planning_module

# View test results
colcon test-result --verbose
```

### Run Individual Test Files

```bash
# Sort node tests
colcon test --packages-select planning_module --pytest-args test/test_sort_node.py

# Verification node tests
colcon test --packages-select planning_module --pytest-args test/test_verification_node.py

# Integrity node tests
colcon test --packages-select planning_module --pytest-args test/test_integrity_node.py

# MoveIt2 interface tests
colcon test --packages-select planning_module --pytest-args test/test_moveit2_interface_node.py
```

### Run Specific Test Cases

```bash
# Run a specific test
colcon test --packages-select planning_module --pytest-args "test/test_sort_node.py::TestSortNode::test_weight_based_exact_match -v"

# Run tests matching a pattern
colcon test --packages-select planning_module --pytest-args "test/test_verification_node.py -k tolerance -v"
```

### Run Tests Without Launching Nodes (for debugging)

If you want to run the test file directly (useful for debugging):

```bash
# Terminal 1: Launch planning nodes
ros2 launch planning_module planning.launch.py

# Terminal 2: Run test directly
cd /path/to/planning_module
python3 test/test_sort_node.py
```

---

## Test Details

### 1. Sort Node Tests (`test_sort_node.py`)

Tests the sorting decision logic:

- ✅ `test_weight_based_exact_match` - Exact weight range matching
- ✅ `test_weight_based_boundary_lower` - Lower boundary of weight range
- ✅ `test_weight_based_boundary_upper` - Upper boundary of weight range
- ✅ `test_weight_out_of_range_fallback` - Fallback to closest bin
- ✅ `test_multiple_objects_sequential` - Multiple objects in sequence
- ✅ `test_low_confidence_handling` - Low confidence weight estimates

**What it tests:**
- Weight-based sorting strategy
- Boundary conditions
- Fallback behavior
- Sequential processing

**Dependencies:**
- Subscribes: `/recognition/estimated_weights`, `/system/target_areas`
- Publishes: `/planning/sort_decisions`

---

### 2. Verification Node Tests (`test_verification_node.py`)

Tests weight verification action server:

- ✅ `test_action_server_available` - Action server startup
- ✅ `test_verification_success_within_tolerance` - Successful verification
- ✅ `test_verification_failure_outside_tolerance` - Failed verification
- ✅ `test_verification_exact_match` - Exact weight match
- ✅ `test_verification_boundary_tolerance` - Tolerance boundary
- ✅ `test_verification_with_noisy_measurements` - Sensor noise handling
- ✅ `test_topic_based_verification_success` - Topic-based output
- ✅ `test_multiple_concurrent_verifications` - Sequential requests

**What it tests:**
- VerifyWeight action server
- Tolerance calculations
- Moving average filtering
- Error percentage computation

**Dependencies:**
- Subscribes: `/recognition/estimated_weights`, `/motion_control/force_feedback`
- Publishes: `/planning/verification_result`
- Action Server: `/planning/verify_weight`

---

### 3. Integrity Node Tests (`test_integrity_node.py`)

Tests workspace validation and safety monitoring:

- ✅ `test_service_available` - Service startup
- ✅ `test_environment_status_published` - Periodic status updates (10 Hz)
- ✅ `test_validate_workspace_within_bounds` - Valid waypoints
- ✅ `test_validate_workspace_x_out_of_bounds` - X boundary violations
- ✅ `test_validate_workspace_y_out_of_bounds` - Y boundary violations
- ✅ `test_validate_workspace_z_below_minimum` - Z min violations
- ✅ `test_validate_workspace_z_above_maximum` - Z max violations
- ✅ `test_validate_workspace_boundary_values` - Exact boundaries
- ✅ `test_validate_workspace_empty_waypoints` - Empty trajectory
- ✅ `test_validate_workspace_many_waypoints` - 100 waypoint trajectory
- ✅ `test_validate_workspace_mixed_valid_invalid` - Mixed waypoints
- ✅ `test_emergency_stop_status` - Emergency stop handling
- ✅ `test_periodic_status_updates` - Update rate verification

**What it tests:**
- ValidateWorkspace service
- Workspace bounds checking
- Environment status publishing
- Safety monitoring

**Dependencies:**
- Subscribes: `/camera/pointcloud`, `/system/status`
- Publishes: `/planning/environment_status`
- Service: `/planning/validate_workspace`

**Workspace Bounds (from config):**
- X: [-0.6, 0.6] meters
- Y: [-0.6, 0.6] meters
- Z: [0.0, 0.6] meters

---

### 4. MoveIt2 Interface Tests (`test_moveit2_interface_node.py`)

Tests motion planning action servers (stub implementation):

- ✅ `test_plan_pick_action_server_available` - Pick action server
- ✅ `test_plan_place_action_server_available` - Place action server
- ✅ `test_plan_pick_basic_request` - Basic pick planning
- ✅ `test_plan_place_basic_request` - Basic place planning
- ✅ `test_plan_pick_with_custom_planning_time` - Custom time limits
- ✅ `test_plan_pick_without_collision_avoidance` - Collision flag
- ✅ `test_plan_pick_various_poses` - Multiple poses
- ✅ `test_plan_place_various_poses` - Multiple place poses
- ✅ `test_sequential_pick_and_place_planning` - Sequential requests
- ✅ `test_concurrent_planning_requests` - Overlapping requests
- ✅ `test_planning_with_empty_group_name` - Error handling
- ✅ `test_planning_with_zero_planning_time` - Edge cases

**What it tests:**
- PlanTrajectory action servers
- Request handling
- Parameter validation
- Concurrent request behavior

**Dependencies:**
- Action Servers: `/planning/plan_pick`, `/planning/plan_place`
- Publishes: `/planning/trajectory`

**Note:** Current implementation is a stub. Tests verify action server availability and response structure.

---

## Expected Test Results

### Currently Passing Tests

With the current stub implementation:

- ✅ **Sort Node:** All tests should pass
- ✅ **Verification Node:** Action server tests should pass
- ✅ **Integrity Node:** Service and workspace validation tests should pass
- ⚠️ **MoveIt2 Interface:** Action servers available but may return `success=False` (stub)

### Known Issues / TODO

1. **MoveIt2 Interface:**
   - Stub implementation returns `success=False`
   - No actual trajectory planning yet
   - Requires MoveIt2 configuration

2. **Integrity Node:**
   - Obstacle detection not fully implemented
   - Robot bounds checking needs TF integration

3. **Verification Node:**
   - Topic-based verification output may not publish in all cases
   - Implementation uses action server primarily

---

## Debugging Failed Tests

### Check Node Status

```bash
# List running nodes
ros2 node list

# Check node info
ros2 node info /sort_node
ros2 node info /verification_node
ros2 node info /integrity_node
ros2 node info /moveit2_interface_node
```

### Check Topic Communication

```bash
# List topics
ros2 topic list

# Echo messages
ros2 topic echo /planning/sort_decisions
ros2 topic echo /planning/environment_status

# Check topic info
ros2 topic info /recognition/estimated_weights
```

### Check Action Servers

```bash
# List action servers
ros2 action list

# Send test goal
ros2 action send_goal /planning/verify_weight sort_interfaces/action/VerifyWeight "{object_id: 1, estimated_weight: 100.0, tolerance: 10.0}"
```

### Check Service Servers

```bash
# List services
ros2 service list

# Call service
ros2 service call /planning/validate_workspace sort_interfaces/srv/ValidateWorkspace "{planned_waypoints: [{position: {x: 0.3, y: 0.2, z: 0.3}}]}"
```

### Increase Test Verbosity

```bash
# Run with verbose output
colcon test --packages-select planning_module --pytest-args "-v -s"

# Run with debug logging
colcon test --packages-select planning_module --event-handlers console_direct+
```

---

## Test Configuration

### Timeouts

- Action server wait: 5 seconds
- Service client wait: 5 seconds
- Goal acceptance: 2 seconds
- Result wait: 5-10 seconds
- Test timeout: 60 seconds (CMake)

### Test Parameters

Tests use parameters from `config/planning.yaml`:

```yaml
sort_node:
  sorting_strategy: "weight_based"
  weight_tolerance: 10.0

verification_node:
  default_tolerance: 10.0
  measurement_duration: 2.0
  filter_window_size: 10

integrity_node:
  workspace_min_x: -0.6
  workspace_max_x: 0.6
  workspace_min_y: -0.6
  workspace_max_y: 0.6
  workspace_min_z: 0.0
  workspace_max_z: 0.6
```

---

## Continuous Integration

To run tests in CI:

```bash
#!/bin/bash
# ci_test.sh

# Build
colcon build --packages-select planning_module sort_interfaces

# Source
source install/setup.bash

# Launch nodes in background
ros2 launch planning_module planning.launch.py &
LAUNCH_PID=$!

# Wait for nodes to start
sleep 3

# Run tests
colcon test --packages-select planning_module

# Get results
TEST_RESULT=$?
colcon test-result --verbose

# Cleanup
kill $LAUNCH_PID

# Exit with test result
exit $TEST_RESULT
```

---

## Adding New Tests

### Template for New Test

```python
#!/usr/bin/env python3
import unittest
import rclpy
from rclpy.node import Node

class MyTestNode(Node):
    def __init__(self):
        super().__init__('my_test_node')
        # Setup publishers/subscribers/clients

class TestMyNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.test_node = MyTestNode()

    def tearDown(self):
        self.test_node.destroy_node()

    def test_my_feature(self):
        # Your test here
        pass

if __name__ == '__main__':
    unittest.main()
```

### Add to CMakeLists.txt

```cmake
ament_add_pytest_test(test_my_node test/test_my_node.py
  TIMEOUT 60
)
```

---

## Resources

- [ROS2 Testing Guide](https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/Testing-Main.html)
- [pytest Documentation](https://docs.pytest.org/)
- [ROS2 Actions](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Actions/Understanding-ROS2-Actions.html)
- [ROS2 Services](https://docs.ros.org/en/humble/Tutorials/Beginner-CLI-Tools/Understanding-ROS2-Services/Understanding-ROS2-Services.html)

---

## Contact

For issues or questions about these tests, please contact the maintainer or open an issue in the repository.
