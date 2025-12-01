# Robot Testing Guide

This guide provides the exact commands to run for testing on both **real hardware** and **simulation**.

---

## Real Robot Testing

### Prerequisites:
- ✅ Robot powered on and booted
- ✅ Real weight detection hardware connected
- ✅ Real gripper and Arduino connected
- ✅ Real weights available for testing
- ⚠️ NO real perception yet (using simulated perception)
- ✅ Robot at safe home position
- ✅ Workspace clear of obstacles

---

### Test 1: Simple Pick and Weigh (Real Robot)

**What it tests:**
- Real robot hardware
- Real weight detection
- Real gripper control
- Initial positioning check at Z_PICKUP
- Single weight pick and weigh operation

**Command:**
```bash
./runSimplePickAndWeigh.sh 192.168.0.100 --grip-weight 100
```

**Optional flags:**
```bash
# Test with different grip weights
./runSimplePickAndWeigh.sh 192.168.0.100 --grip-weight 200
./runSimplePickAndWeigh.sh 192.168.0.100 --grip-weight 500

# Step-by-step mode (for debugging)
./runSimplePickAndWeigh.sh 192.168.0.100 --step
```

**What happens:**
1. Launches all systems (UR driver, MoveIt, gripper, weight detector, PlotJuggler)
2. **Initial position check**: Robot drops to Z_PICKUP with gripper open
3. Wait for Enter to verify weight placement
4. Robot lifts to Z_HOME
5. Full pick and weigh sequence begins
6. Weight measurement displayed after configured time (10s or 15s based on grip_weight)

---

### Test 2: Hybrid Sorting System (Real Robot + Simulated Perception)

**What it tests:**
- Real robot hardware
- Real weight detection
- Real gripper control
- Simulated perception (4 random weights)
- **Automatic position check** for all simulated weights
- Full sorting brain logic

**Command:**
```bash
./runHybridModular.sh 192.168.0.100
```

**Optional flags:**
```bash
# Step-by-step mode (for debugging)
./runHybridModular.sh 192.168.0.100 --step

# Auto-start sorting without dashboard
./runHybridModular.sh 192.168.0.100 --autorun

# Use real weight detection (default is simulated)
./runHybridModular.sh 192.168.0.100 --real-weight
```

**What happens:**
1. Launches all systems including simulated perception
2. **Automatic position check**: Robot visits each simulated weight at Z_PICKUP with gripper open
3. Press Enter at each position to verify placement
4. Robot returns to Z_HOME
5. Sorting brain starts (manual control via dashboard or auto-start with --autorun)
6. Full sorting operation with all 4 simulated weights

**Important Notes:**
- Position check runs automatically when simulated perception is active
- Each weight position is checked before sorting begins
- Real weight detection will measure actual weights during sorting

---

## Simulation Testing

### Prerequisites:
- ✅ No physical hardware required
- ✅ Simulated robot, gripper, and weights
- ✅ RViz visualization

---

### Test 3: Simple Pick and Weigh (Simulation)

**What it tests:**
- Fake robot hardware (simulation)
- Simulated weight detection
- Simulated gripper control
- Initial positioning check
- Single weight pick and weigh operation

**Command:**
```bash
./runSimplePickAndWeigh.sh 192.168.0.100 --fake --grip-weight 100
```

**Optional flags:**
```bash
# Test with different grip weights
./runSimplePickAndWeigh.sh 192.168.0.100 --fake --grip-weight 200
./runSimplePickAndWeigh.sh 192.168.0.100 --fake --grip-weight 500

# Add simulated perception (for testing with fake objects)
./runSimplePickAndWeigh.sh 192.168.0.100 --fake --sim-perception

# Step-by-step mode
./runSimplePickAndWeigh.sh 192.168.0.100 --fake --step
```

**What happens:**
- Same as real robot test, but all hardware is simulated
- Initial position check still runs
- Movements visible in RViz

---

### Test 4: Full Sorting Simulation

**What it tests:**
- Fake robot hardware (simulation)
- Simulated perception (4 random weights)
- Simulated weight detection
- Simulated gripper control
- **Automatic position check** for all simulated weights
- Full sorting brain logic

**Command:**
```bash
./runSimulationModular.sh
```

**Optional flags:**
```bash
# Step-by-step mode (for debugging)
./runSimulationModular.sh --step

# Auto-start sorting without dashboard
./runSimulationModular.sh --autorun
```

**What happens:**
1. Launches all simulated systems
2. **Automatic position check**: Robot visits each simulated weight at Z_PICKUP with gripper open
3. Press Enter at each position to verify simulated placement
4. Robot returns to Z_HOME
5. Sorting brain starts
6. Full sorting operation with all 4 simulated weights visible in RViz

---

## Quick Reference

### Real Robot Commands:
```bash
# Simple pick and weigh
./runSimplePickAndWeigh.sh 192.168.0.100 --grip-weight 100

# Hybrid sorting (real robot + sim perception)
./runHybridModular.sh 192.168.0.100
```

### Simulation Commands:
```bash
# Simple pick and weigh (simulation)
./runSimplePickAndWeigh.sh 192.168.0.100 --fake --grip-weight 100

# Full sorting (simulation)
./runSimulationModular.sh
```

---

## Configuration Files

All robot parameters are configured in:
- `robot_config.yaml` - Z heights, orientation, weight settings, etc.

Current configuration:
- Z_HOME: 371.0 mm
- Z_DESCEND: 210.0 mm
- Z_PICKUP: 180.0 mm
- Z_PLACE: 180.0 mm
- Orientation: RX=2.221, RY=2.221, RZ=0.0

Weight-specific timings:
- 500g: 15 seconds weighting time
- 200g: 12 seconds weighting time
- 100g: 15 seconds weighting time
- Default: 10 seconds weighting time

---

## Troubleshooting

### Robot doesn't move:
1. Check that robot is connected on teach pendant (192.168.0.77:50002)
2. Verify ros.urp program is loaded on teach pendant
3. Check controller status: `ros2 control list_controllers`

### Position check fails:
1. Ensure simulated perception is running: `ros2 topic list | grep detected_objects`
2. Check topic: `ros2 topic echo /perception/detected_objects`

### Weight detection not working:
1. Check Arduino connection
2. Verify weight detector is running: `ros2 topic echo /estimated_mass`
3. Check PlotJuggler for weight visualization

### Gripper not responding:
1. Verify gripper controller lifecycle state
2. Check gripper service: `ros2 service list | grep gripper`

---

## Expected Test Results

### Simple Pick and Weigh:
- Initial position check shows robot at pickup height with open gripper
- Robot successfully picks weight after user confirms position
- Weight measurement appears after configured stabilization time
- PlotJuggler shows weight data

### Hybrid Sorting:
- Position check visits all 4 simulated weight positions
- Each position can be verified before sorting starts
- Sorting brain successfully picks and sorts all weights
- Real weight detection measures actual weight values during sorting
