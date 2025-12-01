# Simple Pick and Weigh Tool

A standalone script that performs a basic pick-and-weigh operation for testing the weight detection system.

## What It Does

From the robot's current X-Y position, the script:

1. **Moves to Z_DESCEND (212mm)** - Calibration height
2. **Calls weight calibration** - `/weight_detection/calibrate_baseline`
3. **Waits for calibration** - Monitors `/weight_detection/calibration_status` (~5.5 seconds)
4. **Sets gripper angle** - Based on perceived weight (default: 100g)
5. **Opens gripper** - Waits 3 seconds
6. **Descends to Z_PICKUP (182mm)** - Pickup height
7. **Closes gripper** - Waits 3 seconds
8. **Lifts to Z_DESCEND (212mm)** - MUST be same height as calibration!
9. **Waits 10 seconds** - For weight measurement to stabilize
10. **Reads weight** - From `/estimated_mass` topic
11. **Displays result** - Shows measured weight in grams

## Prerequisites

Before running, ensure:
- ✅ UR5e robot connected and controllers running
- ✅ Weight detection module running: `ros2 run weight_detection_module weight_detector`
- ✅ Gripper controller running and activated (lifecycle)
- ✅ Cartesian controller running: `ros2 run motion_control_module cartesian_controller_node`
- ✅ Robot positioned above an object to pick

## Usage

### Basic Usage (Default 100g grip):
```bash
cd ~/Documents/mtrn4231_jakos/ros2_system
source install/setup.bash
ros2 run motion_control_module simple_pick_and_weigh
```

### With Custom Grip Weight:
```bash
# For a 200g object (sets gripper angle for 200g):
ros2 run motion_control_module simple_pick_and_weigh --ros-args -p grip_weight:=200

# For a 500g object:
ros2 run motion_control_module simple_pick_and_weigh --ros-args -p grip_weight:=500
```

## Default Position

The script uses a default X-Y position:
- **X: -600.0 mm** (center of workspace)
- **Y: -100.0 mm** (center of workspace)

To use a different position, you can modify the `default_x` and `default_y` values in the script (lines ~143-144).

## Expected Output

```
[INFO] [simple_pick_and_weigh]: Simple Pick and Weigh Node initialized
[INFO] [simple_pick_and_weigh]: Grip weight: 100g
[INFO] [simple_pick_and_weigh]: Waiting 2 seconds for all systems to be ready...

============================================================
STARTING SIMPLE PICK AND WEIGH
============================================================

[INFO] [simple_pick_and_weigh]: Using position: X=-600.0, Y=-100.0

[INFO] [simple_pick_and_weigh]: [1/10] Moving to Z_DESCEND (calibration height)...
[INFO] [simple_pick_and_weigh]: Moving to: X=-600.0, Y=-100.0, Z=212.0

[INFO] [simple_pick_and_weigh]: [2/10] Calling calibration service...
[INFO] [simple_pick_and_weigh]: 📞 Calling weight calibration service...
[INFO] [simple_pick_and_weigh]: Calibration service response: Calibration started

[INFO] [simple_pick_and_weigh]: [3/10] Waiting for calibration (~5.5 seconds)...
[INFO] [simple_pick_and_weigh]: ⏳ Waiting for calibration to complete...
[INFO] [simple_pick_and_weigh]: 🔧 Weight calibration in progress...
[INFO] [simple_pick_and_weigh]: ✅ Weight calibration complete!

[INFO] [simple_pick_and_weigh]: [4/10] Setting gripper angle for 100g...
[INFO] [simple_pick_and_weigh]: Gripper: SET_ANGLE(100g)

[INFO] [simple_pick_and_weigh]: [5/10] Opening gripper...
[INFO] [simple_pick_and_weigh]: Gripper: OPEN

[INFO] [simple_pick_and_weigh]: [6/10] Descending to Z_PICKUP...
[INFO] [simple_pick_and_weigh]: Moving to: X=-600.0, Y=-100.0, Z=182.0

[INFO] [simple_pick_and_weigh]: [7/10] Closing gripper...
[INFO] [simple_pick_and_weigh]: Gripper: CLOSE

[INFO] [simple_pick_and_weigh]: [8/10] Lifting to Z_DESCEND (weighing height)...
[INFO] [simple_pick_and_weigh]: Moving to: X=-600.0, Y=-100.0, Z=212.0

[INFO] [simple_pick_and_weigh]: [9/10] Waiting 10 seconds for weight to stabilize...
[INFO] [simple_pick_and_weigh]:     ⏱️  10 seconds remaining...
[INFO] [simple_pick_and_weigh]:     ⏱️  9 seconds remaining...
...
[INFO] [simple_pick_and_weigh]:     ⏱️  1 seconds remaining...

[INFO] [simple_pick_and_weigh]: [10/10] Reading weight from /estimated_mass...

============================================================
⚖️  MEASURED WEIGHT: 187 grams
============================================================

[INFO] [simple_pick_and_weigh]: ✅ Pick and weigh sequence complete!
```

## Troubleshooting

### "Move service not available"
- Ensure cartesian_controller_node is running
- Check: `ros2 service list | grep move_to_cartesian`

### "Gripper service not available"
- Ensure gripper_controller_node is running and activated
- Check lifecycle state: `ros2 lifecycle get /gripper_controller_node`

### "Weight calibration service not available"
- Ensure weight_detection_module is running
- Check: `ros2 service list | grep calibrate_baseline`

### "No weight measurement received"
- Ensure weight_detection_module is running
- Check topic: `ros2 topic echo /estimated_mass`
- Verify calibration completed successfully
- Ensure robot returned to exact Z_DESCEND height

### Weight seems inaccurate
- Verify robot is at exact Z_DESCEND height for both calibration and weighing
- Ensure 10 second stabilization period completed
- Try recalibrating by running the script again
- Check that gripper hasn't changed (weight affects calibration)

## Integration with Full System

This script demonstrates the exact same weighing procedure used in the sorting_brain_node:
1. Calibrate at Z_DESCEND
2. Pick object
3. Return to Z_DESCEND (same height!)
4. Wait for stabilization
5. Read weight

Use this to test weight detection independently before running the full sorting system.

## Modifying for Your Needs

### To pick from a specific position:
Edit lines ~143-144 in `simple_pick_and_weigh.py`:
```python
default_x = -500.0  # Your X position
default_y = -200.0  # Your Y position
```

### To change stabilization time:
Edit line ~254:
```python
for i in range(15, 0, -1):  # Wait 15 seconds instead of 10
```

### To change heights:
Edit the class variables at the top:
```python
Z_DESCEND = 212.0  # Calibration/weighing height
Z_PICKUP = 182.0   # Pickup height
```

## Notes

- **Critical**: The robot MUST return to the exact same Z height for weighing as used for calibration
- The script waits 10 seconds for weight stabilization - do not reduce this time
- Gripper angle is set based on `grip_weight` parameter, but actual weight is measured independently
- The weight measurement is completely independent of the gripper angle setting
