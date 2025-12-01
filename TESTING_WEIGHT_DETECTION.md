# Testing Weight Detection System

This guide shows how to test the weight detection system at different levels.

## Quick Start - Simple Pick and Weigh

The easiest way to test weight detection:

```bash
# 1. Start the robot and required nodes (using modular script)
cd ~/Documents/mtrn4231_jakos/4231_scripts
./runHybridModular.sh --real-weight --step

# Wait for all systems to start, then connect robot via ros.urp

# 2. In a new terminal, run the simple pick and weigh test
cd ~/Documents/mtrn4231_jakos/ros2_system
source install/setup.bash
ros2 run motion_control_module simple_pick_and_weigh --ros-args -p grip_weight:=100
```

See [SIMPLE_PICK_AND_WEIGH_GUIDE.md](SIMPLE_PICK_AND_WEIGH_GUIDE.md) for details.

## Testing Levels

### Level 1: Weight Detection Module Only

Test just the weight detection without any motion:

```bash
# Terminal 1: Start UR driver
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.0.100 \
    use_fake_hardware:=false \
    initial_joint_controller:=scaled_joint_trajectory_controller

# Connect robot via ros.urp on teach pendant

# Terminal 2: Start weight detector
ros2 run weight_detection_module weight_detector

# Terminal 3: Monitor weight
ros2 topic echo /estimated_mass

# Terminal 4: Test calibration manually
ros2 service call /weight_detection/calibrate_baseline sort_interfaces/srv/CalibrateBaseline

# Watch calibration status
ros2 topic echo /weight_detection/calibration_status

# Move robot manually and watch weight change
```

### Level 2: Simple Pick and Weigh (Recommended)

Tests weight detection with automated motion but without sorting logic:

```bash
# Use runHybridModular.sh or runRealModular.sh to start all systems
./runHybridModular.sh --real-weight

# Then run simple pick and weigh
ros2 run motion_control_module simple_pick_and_weigh
```

**Advantages:**
- Tests complete calibration workflow
- Tests motion to correct heights
- No sorting complexity
- Easy to iterate and debug

### Level 3: Full Sorting System

Tests complete integration with sorting brain:

```bash
# Start full system
./runHybridModular.sh --real-weight

# Or with autorun
./runHybridModular.sh --real-weight --autorun
```

**Note:** Sorting brain needs to be fixed first (see [SORTING_BRAIN_FIXES_NEEDED.md](SORTING_BRAIN_FIXES_NEEDED.md))

## Verification Checklist

### ✅ Weight Detection Module
- [ ] Service `/weight_detection/calibrate_baseline` exists
- [ ] Topic `/estimated_mass` publishes Int32 messages
- [ ] Topic `/weight_detection/calibration_status` publishes Bool
- [ ] Calibration takes ~5.5 seconds
- [ ] Weight readings are stable after calibration

### ✅ Simple Pick and Weigh
- [ ] Robot moves to Z_DESCEND (212mm)
- [ ] Calibration service is called
- [ ] Calibration status goes true → false
- [ ] Gripper opens and closes correctly
- [ ] Robot returns to exact Z_DESCEND height
- [ ] Weight reading is displayed after 10 seconds
- [ ] Weight value is reasonable (0-500g range)

### ✅ Full Sorting System (After Fixes)
- [ ] Perception publishes perceived_weight
- [ ] Sorting brain calls calibration before each pick
- [ ] Gripper angle set based on perceived_weight
- [ ] Actual weight read from /estimated_mass
- [ ] Sorting decision based on actual_weight
- [ ] Objects sorted into correct bins

## Common Issues and Solutions

### Issue: Weight always reads 0
**Solution:**
- Ensure robot is holding something (gripper closed)
- Verify robot is at Z_DESCEND height (212mm)
- Wait full 10 seconds for stabilization
- Check calibration completed successfully

### Issue: Weight very inaccurate
**Solution:**
- Ensure Z_DESCEND height matches calibration height EXACTLY
- Increase stabilization wait time to 15 seconds
- Recalibrate baseline
- Check gripper hasn't changed (affects torques)

### Issue: Calibration times out
**Solution:**
- Check weight_detection_module is running
- Verify `/joint_states` topic is publishing
- Check robot is connected (ros.urp loaded)
- Look for error messages in weight detector output

### Issue: Weight readings unstable
**Solution:**
- Increase stabilization time (10s → 15s)
- Ensure robot is completely still
- Check for vibrations or movement
- Verify all controllers are properly initialized

## Expected Weight Accuracy

Based on Asad's README:

**With snapping (useSnapping: true):**
- Discrete values: 0g, 50g, 100g, 200g, 500g
- Best for known weight sets

**Without snapping (useSnapping: false):**
- Continuous values rounded to nearest 5g
- ±5g accuracy with proper calibration
- Better for arbitrary weights

## PlotJuggler Visualization

Monitor weight measurements in real-time:

```bash
cd ~/Documents/mtrn4231_jakos/ros2_system
./plot_weight.sh
```

- Click "Yes" to start streaming
- Change buffer size to 90 seconds (top left)
- Watch weight estimates update

Expected: Step plot showing discrete weight changes (with snapping) or smooth transitions (without).

## Next Steps

1. **Test simple_pick_and_weigh** to verify weight detection works
2. **Fix sorting_brain_node** using [SORTING_BRAIN_FIXES_NEEDED.md](SORTING_BRAIN_FIXES_NEEDED.md)
3. **Test full system** with corrected sorting brain
4. **Tune parameters** (stabilization time, snapping mode, etc.)

## Parameters to Tune

### Weight Detection Module
```bash
# Enable discrete snapping
ros2 run weight_detection_module weight_detector --ros-args -p useSnapping:=true

# Disable snapping for continuous values
ros2 run weight_detection_module weight_detector --ros-args -p useSnapping:=false
```

### Simple Pick and Weigh
```bash
# Set perceived weight for gripper
ros2 run motion_control_module simple_pick_and_weigh --ros-args -p grip_weight:=200
```

### Sorting Brain (Future)
- `weight_stabilization_sec`: Time to wait at Z_DESCEND (default: 7s, recommend: 10s)
- Bin thresholds for sorting decisions

## Documentation References

- [Weight Detection Module README](ros2_system/src/weight_detection_module/README.md)
- [Simple Pick and Weigh Guide](SIMPLE_PICK_AND_WEIGH_GUIDE.md)
- [Sorting Brain Fixes](SORTING_BRAIN_FIXES_NEEDED.md)
- [System Modes](SYSTEM_MODES.md)
