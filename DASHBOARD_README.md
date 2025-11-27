# Sorting System Dashboard

UI-based control and monitoring for the UR5e weight sorting system.

## Features

- **Real-time State Monitoring**: Visual display of current sorting state with color-coded status
- **Weight Display**: Shows current weight being processed and sorted weights list
- **System Controls**:
  - ▶ **Start**: Begin sorting operation
  - ⏸ **Stop**: Pause sorting
  - 🔄 **Reset**: Clear all sorted weights and restart
  - ⚠ **Emergency Stop**: Immediately halt all operations
- **Status Log**: Real-time event logging with timestamps
- **Visual Feedback**: Color-coded state indicators

## Quick Start

### 1. Launch the Sorting System

First, start the main sorting simulation:

```bash
cd 4231_scripts
./runSortingSimulation.sh
```

Wait for all systems to initialize (about 20 seconds).

### 2. Launch the Dashboard

In a **separate terminal**:

```bash
cd 4231_scripts
./launchDashboard.sh
```

The dashboard UI will open in a new window.

### 3. Start Sorting

Click the **▶ Start** button in the dashboard to begin the sorting operation.

## Dashboard Layout

```
┌─────────────────────────────────────────────────────────┐
│            🤖 UR5e Weight Sorting System                │
├──────────────────────────┬──────────────────────────────┤
│ Current State            │ System Status Log            │
│   WAITING_FOR_DETECTION  │ [12:34:56] System started   │
│                          │ [12:34:58] Object detected  │
│ Current Weight           │ [12:35:02] Picking...       │
│   150 g                  │ [12:35:05] Weighing...      │
│                          │ [12:35:08] Placed at pos 1  │
│ Sorted Weights           │                             │
│  [100g] [150g] [200g]    │                             │
│                          │                             │
│ Controls                 │                             │
│  [  ▶ Start  ]          │                             │
│  [  ⏸ Stop   ]          │                             │
│  [  🔄 Reset  ]         │                             │
│  [⚠ Emergency Stop]      │                             │
└──────────────────────────┴──────────────────────────────┘
```

## State Colors

- 🟢 **IDLE** - Gray (ready to start)
- 🔵 **WAITING_FOR_DETECTION** - Blue (scanning for objects)
- 🟠 **PICKING** - Orange (moving to pick)
- 🟣 **WEIGHING** - Purple (measuring weight)
- 🟡 **DECIDING_PLACEMENT** - Yellow (calculating position)
- 🔴 **REARRANGING** - Red-Orange (moving existing weights)
- 🟢 **PLACING** - Green (placing weight)
- 🔴 **ERROR** - Red (error state)

## Commands

### Start
- **Function**: Transitions system from IDLE to WAITING_FOR_DETECTION
- **When to use**: After launching the system or after a stop
- **Note**: Cannot start if already running

### Stop
- **Function**: Pauses the sorting operation and returns to IDLE
- **When to use**: To temporarily halt sorting
- **Note**: Current operation will complete first

### Reset
- **Function**: Clears all sorted weights and resets the system
- **When to use**: To start a fresh sorting session
- **Warning**: All progress will be lost

### Emergency Stop
- **Function**: Immediately halts all operations and sets ERROR state
- **When to use**: Safety emergencies or critical issues
- **Warning**: System will need to be reset after emergency stop

## ROS2 Topics

The dashboard communicates via these topics:

**Published:**
- `/sorting/command` (String) - Control commands

**Subscribed:**
- `/sorting/state` (String) - Current state machine state
- `/sorting/status` (String) - Status messages and events
- `/perception/weight_estimate` (WeightEstimate) - Weight measurements

## Troubleshooting

### Dashboard won't start
- Ensure the main sorting system is running first
- Check that ROS2 workspace is built: `colcon build --packages-select supervisor_module`
- Verify ROS2 is sourced: `source /opt/ros/humble/setup.bash`

### Commands not working
- Check ROS2 topic connection: `ros2 topic echo /sorting/command`
- Verify sorting_brain_node is running: `ros2 node list | grep sorting`

### Dashboard shows "UNKNOWN" state
- The sorting_brain_node may not be running
- Check with: `ros2 topic echo /sorting/state`

## Integration with Existing System

The dashboard is fully integrated with the existing sorting system:

1. **Non-intrusive**: The system works with or without the dashboard
2. **Real-time sync**: All state changes are reflected immediately
3. **Dual control**: Can use dashboard or command line (`ros2 topic pub`)
4. **Visualization**: Works alongside RViz for complete system monitoring

## Development

Dashboard source code:
- **Node**: `ros2_system/src/supervisor_module/supervisor_module/system_dashboard.py`
- **Launch**: `4231_scripts/launchDashboard.sh`

To modify the dashboard:
1. Edit the Python file
2. Rebuild: `colcon build --packages-select supervisor_module`
3. Relaunch the dashboard

## Example Workflow

1. **Launch system**: `./runSortingSimulation.sh`
2. **Wait for initialization** (20 seconds)
3. **Launch dashboard**: `./launchDashboard.sh` (in separate terminal)
4. **Start sorting**: Click ▶ Start button
5. **Monitor**: Watch state transitions and weight sorting in real-time
6. **Stop when done**: Click ⏸ Stop or 🔄 Reset

## Tips

- Keep both RViz and the dashboard open for complete visibility
- Use the status log to debug issues
- Emergency stop is for safety - use Stop for normal pausing
- Reset clears all progress - use carefully
