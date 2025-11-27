# Weight Detection Integration

## Overview

Asad's `weight_detection_module` has been successfully integrated into the sorting system. The module uses UR5e joint torques to estimate the mass of objects held by the gripper.

## Integration Details

### Topic Interface

**Published Topic:** `/recognition/estimated_mass`
- **Message Type:** `std_msgs/Int32`
- **Values:** Weight in grams [0, 20, 50, 100, 200, 500]
- **Description:** Real-time weight estimation based on joint torque analysis

### How It Works

1. **Weight Detector subscribes to:** `/joint_states`
   - Reads joint torques from all 6 joints
   - Applies Kalman filtering to reduce noise
   - Computes moment arms using forward kinematics

2. **Physics-Based Estimation:**
   - Calculates torque changes from baseline (gripper empty)
   - Uses moment arm × gravity × mass relationship
   - Applies calibration factors for accuracy

3. **Weight Published to:** `/recognition/estimated_mass`
   - Sorting brain subscribes to this topic
   - Values are snapped to known weight set [20, 50, 100, 200, 500]g
   - Zero output when no object detected

### Visualization

The weight detector includes matplotlib visualization showing:
- Joint torques (raw and filtered) for all 6 joints
- Real-time mass estimation
- Calibration factor

## Usage Modes

### Simulation Mode

```bash
cd 4231_scripts
./runSimulation.sh
```

**Weight Source:** `simulated_perception_node`
- Publishes to same topic: `/recognition/estimated_mass`
- Uses assumed perfect values (100g object → 100g reading)
- No torque calculations needed
- Useful for testing sorting logic without real hardware

### Real Robot Mode

```bash
cd 4231_scripts
./runRealRobot.sh
```

**Weight Source:** `weight_detection_module`
- Automatically launched by the script
- Reads real joint torques from UR5e
- Physics-based mass estimation
- Includes visualization window

### Hybrid Mode

```bash
cd 4231_scripts
./runHybrid.sh <robot_ip>
```

**Weight Source:** `simulated_perception_node`
- Real robot hardware but simulated weights
- Useful for testing robot motion without needing perception

## System Changes Made

### Files Modified

1. **weight_detector.py**
   - Changed topic from `/estimated_mass` → `/recognition/estimated_mass`
   - Maintains namespace consistency

2. **sorting_brain_node.py**
   - Changed subscription from `WeightEstimate` → `Int32`
   - Now subscribes to `/recognition/estimated_mass`
   - Simplified callback (no object_id matching needed)

3. **simulated_perception_node.py**
   - Changed publication from `WeightEstimate` → `Int32`
   - Publishes to `/recognition/estimated_mass`
   - Matches real weight detector interface

4. **runRealRobot.sh**
   - Added weight_detector launch at step [4/7]
   - Launches after visualizations, before go_home
   - Includes 2-second initialization delay

### Files Created

- None (no adapter needed - direct integration)

## Topic Flow

```
┌─────────────────────┐
│  /joint_states      │  (from UR driver)
└──────────┬──────────┘
           │
           v
┌─────────────────────┐
│ weight_detector     │  (Asad's module)
│  - Read torques     │
│  - Apply Kalman     │
│  - Compute mass     │
└──────────┬──────────┘
           │
           v
┌─────────────────────┐
│ /recognition/       │  (Int32)
│ estimated_mass      │
└──────────┬──────────┘
           │
           v
┌─────────────────────┐
│ sorting_brain_node  │
│  - Receive weight   │
│  - Decide placement │
│  - Trigger sort     │
└─────────────────────┘
```

## Calibration Service

The weight detector provides a calibration service:

**Service:** `/weight_detection/calibrate_baseline`
- **Type:** `sort_interfaces/srv/CalibrateBaseline`
- **Purpose:** Reset baseline torques (gripper empty)
- **When to use:** After gripper opens/closes, or if drift occurs

Currently, the sorting brain does not automatically call this service, but it could be added to the pick operation for improved accuracy.

## Troubleshooting

### Weight readings are zero

- Check `/joint_states` topic is publishing: `ros2 topic echo /joint_states`
- Verify weight detector is running: `ros2 node list | grep weight`
- Check matplotlib window for torque data

### Weight readings are incorrect

- Try recalibrating: `ros2 service call /weight_detection/calibrate_baseline sort_interfaces/srv/CalibrateBaseline`
- Check robot is at stable position (not moving)
- Verify gripper is properly closed around object

### Matplotlib window not appearing

- Check X11 forwarding if using SSH
- Verify python3-matplotlib is installed
- Run with `--ros-args --log-level debug` for more info

## Future Improvements

1. **Auto-calibration:** Have sorting brain call calibration service after each object placement
2. **Confidence filtering:** Reject weight estimates with low confidence/high variance
3. **Multi-measurement:** Take multiple weight readings and average for robustness
4. **Dynamic thresholds:** Adjust weight snapping thresholds based on expected weight set

## Testing

### Test Weight Detection in Simulation

```bash
# Terminal 1: Run simulation
cd 4231_scripts
./runSimulation.sh

# Terminal 2: Monitor weight topic
ros2 topic echo /recognition/estimated_mass

# Terminal 3: Launch dashboard and press Start
cd 4231_scripts
./launchDashboard.sh
```

### Test Weight Detection with Real Robot

```bash
# Terminal 1: Run real robot
cd 4231_scripts
./runRealRobot.sh

# Terminal 2: Monitor weight topic
ros2 topic echo /recognition/estimated_mass

# You should see weight values updating as objects are picked
```

## Summary

The weight_detection_module is now fully integrated:
- ✅ Publishes to correct topic with correct message type
- ✅ Launches automatically in real robot mode
- ✅ Simulation mode uses fake weights on same topic
- ✅ Sorting brain subscribes and processes weights correctly
- ✅ No manual intervention needed - works out of the box

The integration is clean, modular, and maintains separation between simulation and real hardware modes.
