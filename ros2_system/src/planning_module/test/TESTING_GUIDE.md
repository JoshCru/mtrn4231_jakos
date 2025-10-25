# Planning Module Testing Guide

Complete guide for testing the planning module with 42 integration tests across 4 nodes.

---

## Quick Start

### Easiest Way (Recommended)

```bash
cd src/planning_module/test
./run_tests.sh
```

That's it! The script will:
1. Build the workspace
2. Launch all planning nodes
3. Run all 42 tests
4. Show results
5. Clean up automatically

---

## Test Runner Options

```bash
# Run all tests
./run_tests.sh

# Run specific node tests
./run_tests.sh sort
./run_tests.sh verification
./run_tests.sh integrity
./run_tests.sh moveit2

# Run with verbose output
./run_tests.sh --verbose
./run_tests.sh -v sort

# Keep nodes running after tests (for debugging)
./run_tests.sh --keep-alive
./run_tests.sh -k integrity

# Show help
./run_tests.sh --help
```

---

## Manual Testing

### Method 1: Using colcon test

```bash
# Build
cd /path/to/ros2_system
colcon build --packages-select planning_module sort_interfaces
source install/setup.bash

# Terminal 1: Launch nodes
ros2 launch planning_module planning.launch.py

# Terminal 2: Run tests
colcon test --packages-select planning_module

# View results
colcon test-result --verbose
```

### Method 2: Direct pytest execution

```bash
# Launch nodes first
ros2 launch planning_module planning.launch.py

# In another terminal
cd src/planning_module
python3 test/test_sort_node.py
python3 test/test_verification_node.py
python3 test/test_integrity_node.py
python3 test/test_moveit2_interface_node.py
```

---

## Test Coverage Summary

| Node | Tests | What's Tested |
|------|-------|---------------|
| **sort_node** | 7 | Weight matching, boundaries, fallback, sequential processing |
| **verification_node** | 10 | Action server, tolerance checking, filtering, error calculation |
| **integrity_node** | 13 | Workspace validation, bounds checking, environment monitoring |
| **moveit2_interface_node** | 12 | Action servers, request handling (stub implementation) |
| **TOTAL** | **42** | Complete integration coverage |

---

## Common Test Scenarios

### Test 1: Weight-Based Sorting

```bash
# Tests that objects are sorted to correct bins based on weight
./run_tests.sh sort
```

**Covers:**
- Exact weight range matching
- Boundary conditions (min/max weights)
- Fallback to closest bin when no exact match
- Sequential object processing

### Test 2: Weight Verification

```bash
# Tests verification of estimated vs actual weights
./run_tests.sh verification
```

**Covers:**
- Action server communication
- Tolerance calculations (within/outside limits)
- Moving average filtering for noisy sensors
- Error percentage computation

### Test 3: Workspace Safety

```bash
# Tests workspace bounds and safety validation
./run_tests.sh integrity
```

**Covers:**
- Workspace boundary validation (X, Y, Z limits)
- Service-based trajectory validation
- Periodic environment status monitoring (10 Hz)
- Emergency stop handling

### Test 4: Motion Planning Interface

```bash
# Tests motion planning action servers
./run_tests.sh moveit2
```

**Covers:**
- Pick and place action servers
- Request/response handling
- Parameter validation
- Concurrent request behavior

---

## Understanding Test Results

### Success Output

```
test_sort_node.py::TestSortNode::test_weight_based_exact_match PASSED
test_verification_node.py::TestVerificationNode::test_action_server_available PASSED
...

========================================
All tests PASSED! ✓
========================================
```

### Failure Output

```
test_sort_node.py::TestSortNode::test_weight_based_exact_match FAILED

FAILED test_sort_node.py::TestSortNode::test_weight_based_exact_match - AssertionError: ...

Check logs for details:
  - Node logs: /tmp/planning_nodes.log
  - Test logs: ros2_system/log/latest_test/
```

---

## Debugging Failed Tests

### Step 1: Check if nodes are running

```bash
ros2 node list
```

Expected output:
```
/sort_node
/verification_node
/integrity_node
/moveit2_interface_node
```

### Step 2: Check topic communication

```bash
# List all topics
ros2 topic list

# Monitor sort decisions
ros2 topic echo /planning/sort_decisions

# Monitor environment status
ros2 topic echo /planning/environment_status
```

### Step 3: Check action servers

```bash
# List action servers
ros2 action list

# Test action server manually
ros2 action send_goal /planning/verify_weight sort_interfaces/action/VerifyWeight \
  "{object_id: 1, estimated_weight: 100.0, tolerance: 10.0}"
```

### Step 4: Check services

```bash
# List services
ros2 service list

# Test service manually
ros2 service call /planning/validate_workspace sort_interfaces/srv/ValidateWorkspace \
  "{planned_waypoints: [{position: {x: 0.3, y: 0.2, z: 0.3}}]}"
```

### Step 5: Run with verbose logging

```bash
./run_tests.sh --verbose sort
```

### Step 6: Keep nodes alive for manual testing

```bash
./run_tests.sh --keep-alive sort
# Nodes will stay running after tests
# Manually interact with them for debugging
```

---

## Test Architecture

### Test Structure

Each test file follows this pattern:

```python
class TestNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()  # Initialize ROS2 once

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()  # Shutdown ROS2 at end

    def setUp(self):
        self.test_node = TestNodeClass()  # Create test node

    def tearDown(self):
        self.test_node.destroy_node()  # Cleanup after each test

    def test_feature(self):
        # Publish test inputs
        # Wait for outputs
        # Assert expected behavior
```

### Communication Pattern

```
Test Node                    Planning Node
    |                              |
    |---> Publish input msg ------>|
    |                              |
    |                         Process input
    |                              |
    |<--- Publish output msg ------|
    |                              |
   Assert output matches expected
```

---

## Continuous Integration

### CI Script Example

```bash
#!/bin/bash
# .github/workflows/test_planning.sh

set -e

# Build
colcon build --packages-select planning_module sort_interfaces

# Source
source install/setup.bash

# Launch nodes
ros2 launch planning_module planning.launch.py &
LAUNCH_PID=$!
sleep 3

# Run tests
colcon test --packages-select planning_module

# Get results
colcon test-result --verbose
TEST_RESULT=$?

# Cleanup
kill $LAUNCH_PID

exit $TEST_RESULT
```

### GitHub Actions Example

```yaml
name: Planning Module Tests

on: [push, pull_request]

jobs:
  test:
    runs-on: ubuntu-22.04
    steps:
      - uses: actions/checkout@v3

      - name: Setup ROS2 Humble
        uses: ros-tooling/setup-ros@v0.6
        with:
          required-ros-distributions: humble

      - name: Build
        run: |
          colcon build --packages-select planning_module sort_interfaces

      - name: Test
        run: |
          source install/setup.bash
          cd src/planning_module/test
          ./run_tests.sh
```

---

## Test Configuration

### Default Parameters (from planning.yaml)

```yaml
sort_node:
  sorting_strategy: "weight_based"
  weight_tolerance: 10.0

verification_node:
  default_tolerance: 10.0
  measurement_duration: 2.0
  filter_window_size: 10

integrity_node:
  check_rate: 10.0
  workspace_min_x: -0.6
  workspace_max_x: 0.6
  workspace_min_y: -0.6
  workspace_max_y: 0.6
  workspace_min_z: 0.0
  workspace_max_z: 0.6
  obstacle_detection_threshold: 0.05

moveit2_interface_node:
  planning_group: "ur_manipulator"
  default_planning_time: 5.0
```

### Test Timeouts

- Node startup wait: 3 seconds
- Action server availability: 5 seconds
- Service availability: 5 seconds
- Goal acceptance: 2 seconds
- Result wait: 5-10 seconds
- Test execution timeout: 60 seconds

---

## Expected Behavior

### ✅ Should Pass

All 42 tests should pass with the current implementation:

- **Sort node:** Full implementation, all logic working
- **Verification node:** Action server functional
- **Integrity node:** Service and monitoring working
- **MoveIt2 interface:** Action servers available (stub returns success=false)

### ⚠️ Known Limitations

1. **MoveIt2 Interface (Expected):**
   - Tests verify action server availability
   - Stub implementation returns `success=False`
   - No actual trajectory planning yet
   - This is expected and documented

2. **Integrity Node (Partial Implementation):**
   - Obstacle detection from pointcloud is stubbed
   - Robot bounds checking needs TF integration
   - Tests verify service structure, not full implementation

---

## Adding New Tests

### 1. Create test file

```bash
cd src/planning_module/test
touch test_my_feature.py
chmod +x test_my_feature.py
```

### 2. Write test

```python
#!/usr/bin/env python3
import unittest
import rclpy
from rclpy.node import Node

class MyFeatureTestNode(Node):
    def __init__(self):
        super().__init__('my_feature_test_node')
        # Setup publishers/subscribers/clients

class TestMyFeature(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        rclpy.shutdown()

    def setUp(self):
        self.test_node = MyFeatureTestNode()

    def tearDown(self):
        self.test_node.destroy_node()

    def test_my_feature(self):
        # Your test logic
        self.assertTrue(True)

if __name__ == '__main__':
    unittest.main()
```

### 3. Add to CMakeLists.txt

```cmake
ament_add_pytest_test(test_my_feature test/test_my_feature.py
  TIMEOUT 60
)
```

### 4. Run new test

```bash
./run_tests.sh  # Will include new test
```

---

## Troubleshooting

### Problem: "No module named 'sort_interfaces'"

**Solution:**
```bash
cd /path/to/ros2_system
colcon build --packages-select sort_interfaces
source install/setup.bash
```

### Problem: "Action server not available"

**Solution:**
```bash
# Check if nodes are running
ros2 node list

# If not, launch them
ros2 launch planning_module planning.launch.py
```

### Problem: "Test timeout"

**Solution:**
```bash
# Increase timeout in CMakeLists.txt
ament_add_pytest_test(test_name test/test_name.py
  TIMEOUT 120  # Increase from 60 to 120
)
```

### Problem: "Permission denied: run_tests.sh"

**Solution:**
```bash
chmod +x src/planning_module/test/run_tests.sh
```

### Problem: Tests pass individually but fail together

**Solution:**
- Add delays between tests
- Ensure proper cleanup in `tearDown()`
- Check for shared state between tests

---

## Resources

- **Test README:** `test/README.md` - Detailed test documentation
- **ROS2 Testing:** https://docs.ros.org/en/humble/Tutorials/Intermediate/Testing/
- **pytest Docs:** https://docs.pytest.org/
- **Planning Module:** `src/planning_module/` - Source code

---

## Summary

**Test Suite Coverage:**
- ✅ 42 integration tests
- ✅ 4 nodes tested
- ✅ Topics, actions, and services covered
- ✅ Automated test runner
- ✅ CI/CD ready

**Run Tests:**
```bash
cd src/planning_module/test
./run_tests.sh
```

**Questions?** See `test/README.md` for more details.
