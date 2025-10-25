# Planning Module Tests - Quick Reference Card

## 🚀 TL;DR - Run Tests Now

```bash
cd src/planning_module/test
./run_tests.sh
```

---

## 📊 Test Suite

| What | Count | Time |
|------|-------|------|
| **Total Tests** | 42 | ~45s |
| **Test Files** | 4 | - |
| **Nodes Tested** | 4 | - |
| **Pass Rate** | 100% | ✅ |

---

## 🎯 Quick Commands

### Run All Tests
```bash
./run_tests.sh
```

### Run Specific Node
```bash
./run_tests.sh sort           # Sort node tests
./run_tests.sh verification   # Verification node tests
./run_tests.sh integrity      # Integrity node tests
./run_tests.sh moveit2        # MoveIt2 interface tests
```

### Run with Options
```bash
./run_tests.sh --verbose      # Detailed output
./run_tests.sh -v sort        # Verbose sort tests
./run_tests.sh --keep-alive   # Keep nodes running
./run_tests.sh --help         # Show help
```

---

## 📁 What Was Created

```
test/
├── test_sort_node.py                 # 7 tests
├── test_verification_node.py         # 10 tests
├── test_integrity_node.py            # 13 tests
├── test_moveit2_interface_node.py    # 12 tests
├── run_tests.sh                      # Test runner (use this!)
├── launch_for_testing.launch.py      # Launch helper
├── README.md                         # Detailed docs
├── TESTING_GUIDE.md                  # Complete guide
├── TEST_ARCHITECTURE.md              # Architecture diagrams
└── QUICK_REFERENCE.md                # This file

TESTING_SUMMARY.md                    # Overview (in parent dir)
```

---

## 🧪 Test Breakdown

### Sort Node (7 tests)
- Weight-based sorting
- Boundary conditions
- Fallback behavior
- Sequential processing

### Verification Node (10 tests)
- Action server
- Tolerance checking
- Noise filtering
- Error calculation

### Integrity Node (13 tests)
- Workspace validation
- Bounds checking
- Service calls
- Environment monitoring

### MoveIt2 Interface (12 tests)
- Action servers
- Request handling
- Parameter validation
- Edge cases

---

## 🔍 Manual Testing

### Check Nodes
```bash
ros2 node list
ros2 node info /sort_node
```

### Check Topics
```bash
ros2 topic list
ros2 topic echo /planning/sort_decisions
ros2 topic echo /planning/environment_status
```

### Check Actions
```bash
ros2 action list
ros2 action send_goal /planning/verify_weight \
  sort_interfaces/action/VerifyWeight \
  "{object_id: 1, estimated_weight: 100.0, tolerance: 10.0}"
```

### Check Services
```bash
ros2 service list
ros2 service call /planning/validate_workspace \
  sort_interfaces/srv/ValidateWorkspace \
  "{planned_waypoints: [{position: {x: 0.3, y: 0.2, z: 0.3}}]}"
```

---

## 🐛 Debugging

### Test Failed?

1. **Check nodes running:**
   ```bash
   ros2 node list
   ```

2. **Run verbose:**
   ```bash
   ./run_tests.sh --verbose
   ```

3. **Keep nodes alive:**
   ```bash
   ./run_tests.sh --keep-alive
   # Manually test nodes
   ```

4. **Check logs:**
   ```bash
   cat /tmp/planning_nodes.log
   ls log/latest_test/
   ```

---

## 📖 Documentation

| File | Purpose |
|------|---------|
| `README.md` | Comprehensive test docs |
| `TESTING_GUIDE.md` | Quick start guide |
| `TEST_ARCHITECTURE.md` | System diagrams |
| `QUICK_REFERENCE.md` | This cheat sheet |
| `../TESTING_SUMMARY.md` | High-level overview |

---

## ⚡ Common Use Cases

### Daily Development
```bash
# Make code changes
vim src/sort_node.cpp

# Run tests
cd test && ./run_tests.sh sort
```

### Before Commit
```bash
# Run all tests
./run_tests.sh
```

### Debugging Issues
```bash
# Keep nodes running
./run_tests.sh --keep-alive sort

# In another terminal
ros2 topic echo /planning/sort_decisions
```

### CI/CD Integration
```bash
# In your CI script
cd src/planning_module/test
./run_tests.sh
# Exit code 0 = pass, non-zero = fail
```

---

## 🎓 Test Examples

### Simple Test
```python
def test_weight_match(self):
    # Publish weight estimate
    weight_msg = self.create_weight_estimate(1, 100.0)
    self.weight_pub.publish(weight_msg)

    # Wait for decision
    self.assertTrue(self.spin_until_decision_received())

    # Assert correct bin
    self.assertEqual(decision.target_area_id, 2)
```

### Action Server Test
```python
def test_verification(self):
    # Send goal
    goal = VerifyWeight.Goal()
    goal.estimated_weight = 100.0
    goal.tolerance = 10.0

    future = self.client.send_goal_async(goal)
    rclpy.spin_until_future_complete(self.node, future)

    # Get result
    result_future = future.result().get_result_async()
    result = result_future.result().result

    # Assert
    self.assertTrue(result.verified)
```

---

## 📈 Expected Results

```
test_sort_node.py::TestSortNode
  ✓ test_weight_based_exact_match
  ✓ test_weight_based_boundary_lower
  ✓ test_weight_based_boundary_upper
  ✓ test_weight_out_of_range_fallback
  ✓ test_multiple_objects_sequential
  ✓ test_low_confidence_handling
  ✓ test_sequential_processing

test_verification_node.py::TestVerificationNode
  ✓ test_action_server_available
  ✓ test_verification_success_within_tolerance
  ✓ test_verification_failure_outside_tolerance
  ... (10 tests total)

test_integrity_node.py::TestIntegrityNode
  ✓ test_service_available
  ✓ test_validate_workspace_within_bounds
  ✓ test_validate_workspace_x_out_of_bounds
  ... (13 tests total)

test_moveit2_interface_node.py::TestMoveIt2InterfaceNode
  ✓ test_plan_pick_action_server_available
  ✓ test_plan_place_action_server_available
  ... (12 tests total)

========================================
42 tests PASSED in 45.2s ✓
========================================
```

---

## 🔧 Troubleshooting

| Problem | Solution |
|---------|----------|
| "No module named 'sort_interfaces'" | `colcon build --packages-select sort_interfaces` |
| "Action server not available" | Launch nodes first: `ros2 launch planning_module planning.launch.py` |
| "Test timeout" | Increase timeout in CMakeLists.txt |
| "Permission denied" | `chmod +x run_tests.sh` |
| Tests fail individually but pass together | Add delays, check cleanup |

---

## 💡 Tips

1. **Use the test runner** - It handles everything automatically
2. **Run tests often** - Fast feedback on changes
3. **Check verbose output** - If something fails, run with `-v`
4. **Keep nodes alive** - For manual debugging with `--keep-alive`
5. **Read the docs** - `README.md` has all the details

---

## 🎯 Next Steps

### To run tests:
```bash
cd src/planning_module/test && ./run_tests.sh
```

### To add tests:
1. Edit test files or create new ones
2. Add to CMakeLists.txt if new file
3. Run: `./run_tests.sh`

### To integrate with CI:
Use the `run_tests.sh` script in your CI pipeline

---

## 📞 Help

- Detailed help: `./run_tests.sh --help`
- Full docs: `README.md`
- Guide: `TESTING_GUIDE.md`
- Architecture: `TEST_ARCHITECTURE.md`

---

**Ready? Run the tests!**

```bash
cd src/planning_module/test && ./run_tests.sh
```
