# Sorting System Operating Modes

The sorting system can run in three different modes, allowing you to easily toggle between simulation and real hardware as you develop and test.

## Quick Mode Selection

```bash
cd 4231_scripts

# Full Simulation (no hardware needed)
./runSimulation.sh

# Hybrid (real robot + simulated perception)
./runHybrid.sh [robot_ip]

# Full Real System (everything real)
./runRealRobot.sh [robot_ip]

# Dashboard (open in separate terminal)
./launchDashboard.sh
```

## Mode Details

### 1. Simulation Mode (`runSimulation.sh`)

**What it does:**
- ✅ Simulated robot (fake hardware)
- ✅ Simulated perception (4 random weights)
- ✅ Simulated weight estimation
- ✅ RViz visualization

**When to use:**
- Development and testing without any hardware
- Algorithm development
- Demonstrating the system
- Learning the system workflow

**How to run:**
```bash
./runSimulation.sh
# In another terminal:
./launchDashboard.sh
```

### 2. Hybrid Mode (`runHybrid.sh`)

**What it does:**
- ✅ **REAL robot hardware**
- ✅ Simulated perception (4 random weights)
- ✅ Simulated weight estimation
- ✅ RViz visualization

**When to use:**
- Testing robot movements without Kevin's or Asad's nodes
- Debugging motion planning issues
- Verifying safety boundaries on real hardware
- Testing before full integration

**Prerequisites:**
- Robot powered on and in remote control mode
- Robot at safe position
- Workspace clear

**How to run:**
```bash
# Default IP (192.168.0.100)
./runHybrid.sh

# Custom IP
./runHybrid.sh 192.168.1.100
```

### 3. Full Real System (`runRealRobot.sh`)

**What it does:**
- ✅ **REAL robot hardware**
- ✅ **REAL perception (Kevin's nodes)**
- ✅ **REAL weight estimation (Asad's nodes)**
- ✅ RViz visualization

**When to use:**
- Final integrated system
- Full system testing with all team members' nodes
- Production runs
- Demonstrations with actual hardware

**Prerequisites:**
- Robot powered on and in remote control mode
- Robot at safe home position
- **Kevin's perception nodes running** → Publishing to `/perception/detected_objects`
- **Asad's weight calibration nodes running** → Publishing to `/recognition/estimated_weights`
- Workspace clear and safe

**How to run:**
```bash
# Start Kevin's perception nodes (in separate terminal)
ros2 run recognition_module perception_node  # or whatever Kevin's launch command is

# Start Asad's calibration nodes (in separate terminal)
ros2 run recognition_module weight_calibration_node  # or whatever Asad's launch command is

# Start the sorting system
./runRealRobot.sh 192.168.0.100

# Start the dashboard (in another terminal)
./launchDashboard.sh
```

## Integration with Kevin's and Asad's Nodes

### Kevin's Perception Module

**Required topic:** `/perception/detected_objects` (type: `DetectedObjects`)

**Message format:**
```python
# DetectedObjects.msg
BoundingBox[] objects

# BoundingBox.msg
int32 id
float32 x_min  # mm from base
float32 x_max  # mm from base
float32 y_min  # mm from base
float32 y_max  # mm from base
```

**What the sorting brain expects:**
- Objects detected in the picking area: X[-787, -420], Y[-252, 50] mm
- Each object has a unique ID
- Coordinates in millimeters relative to base_link

### Asad's Weight Calibration Module

**Required topic:** `/recognition/estimated_weights` (type: `WeightEstimate`)

**Message format:**
```python
# WeightEstimate.msg
int32 object_id      # Should match the ID from Kevin's detection
float32 estimated_weight  # in grams
float32 confidence   # 0.0 to 1.0
```

**What the sorting brain expects:**
- Weight published when object is picked (triggered by `/perception/remove_object`)
- Weight in grams (100, 200, 500, etc.)
- Object ID matches the currently picked object

**Integration flow:**
1. Brain picks up object with ID=5
2. Brain publishes to `/perception/remove_object` with ID=5
3. Asad's node receives this and publishes weight estimate for ID=5
4. Brain receives weight and proceeds with sorting

## System Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                    Launch Modes                              │
│                                                               │
│  runSimulation.sh    runHybrid.sh      runRealRobot.sh      │
│       │                  │                   │               │
│       └──────────────────┴───────────────────┘               │
│                          │                                   │
│                          ▼                                   │
│           supervisor_module/sorting_system.launch.py        │
│                          │                                   │
│       ┌──────────────────┼──────────────────┐               │
│       ▼                  ▼                  ▼               │
│   UR Driver          MoveIt          Sorting Brain          │
│   (fake/real)                                               │
│                                                              │
│   Perception Node                                           │
│   (simulated/Kevin's)                                       │
└──────────────────────────────────────────────────────────────┘
```

## Dashboard Control

All three modes use the same dashboard for control:

1. **Launch system** (one of the three modes)
2. **Wait** for initialization (~30 seconds)
3. **Launch dashboard**: `./launchDashboard.sh`
4. **Click ▶ Start** to begin sorting
5. **Monitor** in RViz and dashboard

## Troubleshooting

### "Simulated perception not working"
- Check: `ros2 topic echo /perception/detected_objects`
- Should see 4 objects being published

### "Kevin's nodes not detected"
- Check: `ros2 topic echo /perception/detected_objects`
- Make sure Kevin's nodes are running first
- Verify topic name matches exactly

### "Asad's weights not received"
- Check: `ros2 topic echo /recognition/estimated_weights`
- Make sure Asad's nodes are running
- Verify they're subscribed to `/perception/remove_object`

### "Robot not moving"
- Check robot is in remote control mode
- Verify IP address is correct
- Check: `ros2 topic list | grep joint`
- Should see `/joint_states` and `/scaled_joint_trajectory_controller/`

## Advanced: Custom Launch

You can also launch with custom parameters:

```bash
ros2 launch supervisor_module sorting_system.launch.py \
    use_fake_hardware:=true \
    use_simulated_perception:=false \
    launch_rviz:=true \
    robot_ip:=192.168.0.100
```

**Available parameters:**
- `use_fake_hardware` (true/false) - Simulated vs real robot
- `use_simulated_perception` (true/false) - Simulated vs Kevin's nodes
- `launch_rviz` (true/false) - Show RViz visualization
- `robot_ip` (IP address) - Real robot IP (ignored if fake hardware)

## Files Overview

### Launch Scripts (in `4231_scripts/`)
- `runSimulation.sh` - Full simulation
- `runHybrid.sh` - Real robot + simulated perception
- `runRealRobot.sh` - Full real system
- `launchDashboard.sh` - UI control panel
- `setupFakeur5e.sh` - Legacy setup (prefer runSimulation.sh)
- `setupRealur5eSystemVisualisation.sh` - Legacy setup (prefer runRealRobot.sh)

### Launch File
- `ros2_system/src/supervisor_module/launch/sorting_system.launch.py`
  - Master launch file with configurable parameters
  - Handles all three modes

### Node Files
- `sorting_brain_node.py` - Main orchestrator
- `simulated_perception_node.py` - Simulated weight detection
- `system_dashboard.py` - UI control panel
- `cartesian_controller_node.cpp` - Motion control

## Next Steps

1. **Now (Simulation)**: Use `runSimulation.sh` for development
2. **Soon (Hybrid)**: Use `runHybrid.sh` to test with real robot
3. **Later (Integration)**:
   - Get Kevin's perception topic working
   - Get Asad's weight calibration topic working
   - Switch to `runRealRobot.sh` for full system
