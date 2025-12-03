# Complete System Workflows

This document provides the complete commands and workflows for all system configurations.

---

## Table of Contents
1. [Scenario 1: Real Robot + Fake Perception + Fake Weight](#scenario-1-real-robot--fake-perception--fake-weight)
2. [Scenario 2: Real Robot + Fake Perception + Real Weight](#scenario-2-real-robot--fake-perception--real-weight)
3. [Scenario 3: Simple Pick and Weigh + Real Weight + Recording](#scenario-3-simple-pick-and-weigh--real-weight--recording)

---

## Scenario 1: Real Robot + Fake Perception + Fake Weight

**Configuration:**
- ✅ REAL Robot (UR5e)
- ✅ FAKE Perception (simulated_perception_node)
- ✅ FAKE Weight Detection (uses estimated_mass from simulated perception)
- ✅ REAL Gripper (Arduino)
- ✅ Go Home First
- ✅ Position Check

This is the **Hybrid Mode** - safest for testing sorting logic without real sensors.

### Method 1: Using Shell Scripts

```bash
# Terminal 1: Start Persistent Nodes (Infrastructure)
cd ~/Documents/mtrn4231_jakos/4231_scripts/modular_launch
./start_persistent.sh --real-robot --robot-ip 192.168.0.100

# Wait for all nodes to initialize (~30 seconds)
# Load ros.urp on teach pendant when prompted
# Connect to 192.168.0.77:50002

# Terminal 2: Start Temporary Nodes (Sorting System)
./start_temporary.sh \
    --mode sorting \
    --sim-perception \
    --sim-weight \
    --go-home \
    --position-check \
    --autorun
```

### Method 2: Using Launch Files

```bash
# Terminal 1: Start Persistent Nodes
cd ~/Documents/mtrn4231_jakos/ros2_system
source install/setup.bash
ros2 launch persistent_nodes.launch.py robot_ip:=192.168.0.100

# Wait for all nodes to initialize
# Load ros.urp on teach pendant when prompted

# Terminal 2: Start Temporary Nodes
ros2 launch temporary_nodes.launch.py \
    mode:=sorting \
    sim_perception:=true \
    sim_weight:=true \
    go_home:=true \
    position_check:=true \
    autorun:=true
```

### What Happens:
1. UR5e driver connects to real robot
2. MoveIt starts for motion planning
3. Gripper controller connects to Arduino
4. Cartesian controller ready
5. Robot moves to HOME position
6. Simulated perception generates fake object positions with estimated masses
7. Position check verifies robot can reach all positions
8. Sorting brain starts and automatically begins sorting
9. System uses estimated_mass from simulated perception (no weight sensor needed)

---

## Scenario 2: Real Robot + Fake Perception + Real Weight

**Configuration:**
- ✅ REAL Robot (UR5e)
- ✅ FAKE Perception (simulated_perception_node)
- ✅ REAL Weight Detection (actual weight sensor)
- ✅ REAL Gripper (Arduino)
- ✅ Go Home First
- ✅ Position Check
- 📊 PlotJuggler for weight visualization

This mode tests real weight sensing with fake positions.

### Method 1A: Using Shell Scripts (C++ Weight Detector - DEFAULT)

```bash
# Terminal 1: Start Persistent Nodes
cd ~/Documents/mtrn4231_jakos/4231_scripts/modular_launch
./start_persistent.sh --real-robot --robot-ip 192.168.0.100

# Terminal 2: Start Temporary Nodes with REAL WEIGHT (C++)
./start_temporary.sh \
    --mode sorting \
    --sim-perception \
    --real-weight \
    --weight-detector-cpp \
    --go-home \
    --position-check \
    --autorun
```

### Method 1B: Using Shell Scripts (Python Weight Detector)

```bash
# Terminal 1: Same as above

# Terminal 2: Start Temporary Nodes with REAL WEIGHT (Python)
./start_temporary.sh \
    --mode sorting \
    --sim-perception \
    --real-weight \
    --weight-detector-python \
    --go-home \
    --position-check \
    --autorun
```

### Method 2A: Using Launch Files (C++ Weight Detector - DEFAULT)

```bash
# Terminal 1: Start Persistent Nodes
cd ~/Documents/mtrn4231_jakos/ros2_system
source install/setup.bash
ros2 launch persistent_nodes.launch.py robot_ip:=192.168.0.100

# Terminal 2: Start Temporary Nodes with REAL WEIGHT (C++)
ros2 launch temporary_nodes.launch.py \
    mode:=sorting \
    sim_perception:=true \
    real_weight:=true \
    weight_detector_impl:=cpp \
    go_home:=true \
    position_check:=true \
    autorun:=true
```

### Method 2B: Using Launch Files (Python Weight Detector)

```bash
# Terminal 1: Same as above

# Terminal 2: Start Temporary Nodes with REAL WEIGHT (Python)
ros2 launch temporary_nodes.launch.py \
    mode:=sorting \
    sim_perception:=true \
    real_weight:=true \
    weight_detector_impl:=python \
    go_home:=true \
    position_check:=true \
    autorun:=true
```

### What Happens:
1. Same as Scenario 1, but with real weight sensor
2. PlotJuggler automatically opens showing:
   - `/estimated_mass` - Real-time weight readings
   - `/calibration_status` - Whether baseline is calibrated
3. Weight detector subscribes to `/joint_states` to detect weight
4. System uses REAL weight measurements instead of simulated estimates
5. Sorting brain uses actual weight for classification

### Topics Published by Weight Detector:
- `/estimated_mass` (std_msgs/Int32) - Weight in grams
- `/calibration_status` (std_msgs/Bool) - Calibration status

### Services Provided by Weight Detector:
- `/calibrate_baseline` (sort_interfaces/srv/CalibrateBaseline) - Calibrate zero weight

---

## Scenario 3: Simple Pick and Weigh + Real Weight + Recording

**Configuration:**
- ✅ REAL Robot (UR5e)
- ✅ REAL Weight Detection (choose C++ or Python)
- ✅ REAL Gripper (Arduino)
- ✅ Go Home First
- 📊 PlotJuggler visualization
- 🔴 ROS2 Bag Recording (60 seconds after position confirmation)

This mode performs a single pick-and-weigh cycle with full data recording.

### Method 1A: Using Shell Scripts (C++ Weight Detector)

```bash
# Terminal 1: Start Persistent Nodes
cd ~/Documents/mtrn4231_jakos/4231_scripts/modular_launch
./start_persistent.sh --real-robot --robot-ip 192.168.0.100

# Wait for initialization, load ros.urp on pendant

# Terminal 2: Run Simple Pick and Weigh (C++)
./start_temporary.sh \
    --mode simple \
    --real-weight \
    --weight-detector-cpp \
    --go-home \
    --grip-weight 100

# Terminal 3 (AFTER you press Enter for position confirmation):
# Start 60-second rosbag recording
cd ~/Documents/mtrn4231_jakos/ros2_system
ros2 bag record -d 60 \
    /estimated_mass \
    /calibration_status \
    /joint_states \
    /tf \
    /tf_static \
    /gripper/state \
    -o rosbags/simple_pick_weigh_$(date +%Y%m%d_%H%M%S)
```

### Method 1B: Using Shell Scripts (Python Weight Detector)

```bash
# Terminal 1: Same as above

# Terminal 2: Run Simple Pick and Weigh (Python)
./start_temporary.sh \
    --mode simple \
    --real-weight \
    --weight-detector-python \
    --go-home \
    --grip-weight 100

# Terminal 3: Same rosbag command as above
```

### Method 2A: Using Launch Files (C++ Weight Detector)

```bash
# Terminal 1: Start Persistent Nodes
cd ~/Documents/mtrn4231_jakos/ros2_system
source install/setup.bash
ros2 launch persistent_nodes.launch.py robot_ip:=192.168.0.100

# Terminal 2: Run Simple Pick and Weigh (C++)
ros2 launch temporary_nodes.launch.py \
    mode:=simple \
    real_weight:=true \
    weight_detector_impl:=cpp \
    go_home:=true \
    grip_weight:=100

# Terminal 3 (AFTER go_home completes):
# Start 60-second rosbag recording
ros2 bag record -d 60 \
    /estimated_mass \
    /calibration_status \
    /joint_states \
    /tf \
    /tf_static \
    /gripper/state \
    -o rosbags/simple_pick_weigh_$(date +%Y%m%d_%H%M%S)
```

### Method 2B: Using Launch Files (Python Weight Detector)

```bash
# Terminal 1: Same as above

# Terminal 2: Run Simple Pick and Weigh (Python)
ros2 launch temporary_nodes.launch.py \
    mode:=simple \
    real_weight:=true \
    weight_detector_impl:=python \
    go_home:=true \
    grip_weight:=100

# Terminal 3: Same rosbag command as above
```

### What Happens:
1. Persistent nodes start (UR driver, MoveIt, gripper, cartesian controller)
2. Robot moves to HOME position
3. PlotJuggler opens showing real-time weight data
4. **WAIT** for position prompt, then press Enter
5. **START ROSBAG** in Terminal 3 immediately (60-second recording)
6. Simple pick and weigh executes:
   - Move to pickup position
   - Open gripper
   - Lower to grasp
   - Close gripper with specified force (100g)
   - Lift object
   - Wait for weight stabilization
   - Read weight from `/estimated_mass`
   - Report weight
   - Place object
   - Return home
7. Rosbag automatically stops after 60 seconds

### Recorded Topics:
- `/estimated_mass` - Weight measurements
- `/calibration_status` - Calibration state
- `/joint_states` - Robot joint positions/torques
- `/tf` & `/tf_static` - Transform trees
- `/gripper/state` - Gripper status

### After Recording:
```bash
# View rosbag info
ros2 bag info rosbags/simple_pick_weigh_YYYYMMDD_HHMMSS

# Play back recording
ros2 bag play rosbags/simple_pick_weigh_YYYYMMDD_HHMMSS

# Replay in PlotJuggler
plotjuggler
# File → Open Rosbag → Select your bag file
```

---

## Quick Reference Table

| Scenario | Robot | Perception | Weight | Launch Command |
|----------|-------|------------|--------|----------------|
| 1 | Real | Fake | Fake | `ros2 launch temporary_nodes.launch.py mode:=sorting sim_perception:=true sim_weight:=true go_home:=true position_check:=true autorun:=true` |
| 2 (C++) | Real | Fake | Real (C++) | `ros2 launch temporary_nodes.launch.py mode:=sorting sim_perception:=true real_weight:=true weight_detector_impl:=cpp go_home:=true position_check:=true autorun:=true` |
| 2 (Py) | Real | Fake | Real (Python) | `ros2 launch temporary_nodes.launch.py mode:=sorting sim_perception:=true real_weight:=true weight_detector_impl:=python go_home:=true position_check:=true autorun:=true` |
| 3 (C++) | Real | N/A | Real (C++) | `ros2 launch temporary_nodes.launch.py mode:=simple real_weight:=true weight_detector_impl:=cpp go_home:=true grip_weight:=100` |
| 3 (Py) | Real | N/A | Real (Python) | `ros2 launch temporary_nodes.launch.py mode:=simple real_weight:=true weight_detector_impl:=python go_home:=true grip_weight:=100` |

---

## Important Notes

### 1. Calibrating Weight Detector
Before first use or after robot restart:
```bash
# Call calibration service (no object in gripper)
ros2 service call /calibrate_baseline sort_interfaces/srv/CalibrateBaseline
```

### 2. Checking Weight Detector Status
```bash
# View current weight
ros2 topic echo /estimated_mass

# View calibration status
ros2 topic echo /calibration_status

# List all topics
ros2 topic list
```

### 3. PlotJuggler Configuration
PlotJuggler should open automatically when `--real-weight` is used. If not:
```bash
cd ~/Documents/mtrn4231_jakos/ros2_system
./plot_weight.sh
```

### 4. Grip Weight Parameter
The `grip_weight` parameter (default: 100g) controls gripper closing force:
- Light objects (50-100g): Use `--grip-weight 50` or `grip_weight:=50`
- Medium objects (100-200g): Use `--grip-weight 150` or `grip_weight:=150`
- Heavy objects (200-500g): Use `--grip-weight 250` or `grip_weight:=250`

### 5. Emergency Stop
Press `Ctrl+C` in each terminal to cleanly stop all nodes.

### 6. Robot Safety
- Always have emergency stop button accessible
- Ensure workspace is clear
- Monitor robot during operation
- Use simulated mode first for testing logic

---

## Troubleshooting

### Weight readings are unstable
- Ensure robot is stationary when measuring
- Check calibration: `ros2 topic echo /calibration_status`
- Recalibrate if needed: `ros2 service call /calibrate_baseline sort_interfaces/srv/CalibrateBaseline`

### PlotJuggler doesn't open
```bash
# Check if PlotJuggler is installed
which plotjuggler

# Install if missing
sudo apt install ros-humble-plotjuggler-ros
```

### Robot won't connect
- Check robot IP: `ping 192.168.0.100`
- Verify ros.urp is loaded on teach pendant
- Check external control is connected (192.168.0.77:50002)

### Rosbag file too large
- Reduce recording duration: Change `-d 60` to `-d 30`
- Record fewer topics: Remove `/tf` and `/tf_static` if not needed

---

**Created:** 2025-12-03
**Version:** 1.0
