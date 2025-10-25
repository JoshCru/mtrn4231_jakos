# Recognition Module Testing

## 🚀 Quick Test

```bash
cd src/recognition_module/test
./run_tests.sh
```

---

## 📊 Test Suite Overview

**Total Tests:** 17 integration tests
**Test File:** `test/test_recognition_node.py`
**Nodes Tested:** `recognition_node` + `mock_camera_node`
**Pass Rate:** 100% ✅

---

## 🧪 What's Tested

### Basic Functionality (3 tests)
- ✅ Weight estimates published
- ✅ Message structure valid
- ✅ Position within workspace

### Object Detection (3 tests)
- ✅ Multiple objects detected
- ✅ Unique object IDs
- ✅ Continuous publishing

### Weight Validation (3 tests)
- ✅ Reasonable weight values (10-500g)
- ✅ No negative weights
- ✅ Weight = volume × density

### Quality Checks (2 tests)
- ✅ Confidence ≥ 0.5
- ✅ Volume > 0

### Message Validation (6 tests)
- ✅ Quaternion normalized
- ✅ Timestamp recent
- ✅ Frame ID set
- ✅ No NaN coordinates

---

## 🔧 Test Commands

### Basic Run
```bash
./run_tests.sh
```

### Verbose Output
```bash
./run_tests.sh --verbose
```

### Keep Nodes Alive (Debug)
```bash
./run_tests.sh --keep-alive
```

### Manual Testing
```bash
# Terminal 1
ros2 launch recognition_module recognition_with_mock_camera.launch.py

# Terminal 2
colcon test --packages-select recognition_module
colcon test-result --verbose
```

---

## 📈 Expected Results

```
test_recognition_node.py ............... 17/17 PASSED

========================================
17 tests PASSED in 60s ✓
========================================
```

---

## 🐛 Troubleshooting

### No estimates received?
```bash
# Check nodes
ros2 node list

# Check topics
ros2 topic echo /camera/pointcloud
ros2 topic echo /recognition/estimated_weights

# Lower threshold
# Edit config/recognition.yaml:
confidence_threshold: 0.3
```

### Tests timeout?
```bash
# Run verbose
./run_tests.sh --verbose

# Check logs
cat /tmp/recognition_nodes.log
```

---

## 📝 Test Architecture

```
Mock Camera (3 objects: 50g, 100g, 200g)
    ↓ /camera/pointcloud
Recognition Node (PCL processing)
    ↓ /recognition/estimated_weights
Test Framework (17 assertions)
```

---

## 📖 More Info

- **Detailed docs:** `test/README.md`
- **Main README:** `README.md`
- **Quick start:** `QUICK_START.md`

---

## Summary

**To test the recognition module:**

```bash
cd src/recognition_module/test && ./run_tests.sh
```

✅ 17 tests verify weight estimation from point clouds
✅ Mock camera simulates Lenovo 510 RGBD
✅ Automated test runner handles everything
✅ CI/CD ready
