# Arduino Gripper Integration

## Overview

The Arduino-controlled servo gripper has been fully integrated with the sorting system, supporting weight-specific grip angles and proper timing for pick-and-place operations.

## Arduino Code Changes

### File: `arduino/servo.ino`

**Serial Protocol:**
- **Baud rate:** 115200 (updated from 9600)
- **Commands:**
  - `W` - Open gripper
  - `S` - Close gripper
  - `E <weight>` - Set grip angle (e.g., "E 100", "E 200", "E 500")

**Grip Angles:**
- 100g → closed position = 12°
- 200g → closed position = 20°
- 500g → closed position = 30°

## ROS2 Changes

### 1. GripperControl Service (`sort_interfaces/srv/GripperControl.srv`)

**Extended fields:**
```
string command          # "open", "close", "w", "s", "e", or "edit"
int32 weight           # For edit command: 100, 200, or 500
float32 wait_time_sec  # Wait time after command (default 5.0)
---
bool success
string message
float32 final_position
```

### 2. Gripper Controller Node (`control_module/src/gripper_controller_node.cpp`)

**New functionality:**
- Added `send_edit_command(int weight)` function
- Updated `handle_gripper_control()` to support:
  - 'e' or 'edit' commands with weight parameter
  - Configurable wait_time_sec (default 5.0 seconds)
  - Commands: 'W', 'S', 'e' (uppercase matching Arduino)

### 3. Sorting Brain Node (`supervisor_module/supervisor_module/sorting_brain_node.py`)

**New functions:**
- `_extract_weight_from_class_name()` - Parses weight from object's class_name field
- Updated `gripper_control(command, weight, wait_time_sec)` - Extended parameters

**Pick Sequence (Updated):**
1. Extract perceived weight from object's class_name (e.g., "100g" → 100)
2. Send `'e'` command with weight to set grip angle
3. Send `'W'` command to open gripper + wait 5 seconds
4. Move to descend position (staged movement)
5. Descend to pickup height
6. Send `'S'` command to close gripper + wait 5 seconds
7. Lift and continue with sorting

**Place Sequence (Updated):**
1. Move to place position (staged movement)
2. Descend to place height
3. Send `'W'` command to open gripper + wait 5 seconds
4. Retreat

### 4. Simulated Perception Node (`supervisor_module/supervisor_module/simulated_perception_node.py`)

**Changes:**
- Object `class_name` now includes weight: `'100g'`, `'200g'`, `'500g'`
- This allows sorting_brain to set correct grip angle before picking

### 5. Util Arduino Serial (`util_arduino_serial/util_arduino_serial/util_arduino_serial.py`)

**Changes:**
- Baud rate updated to 115200 to match Arduino code

## Workflow

### Simulation Mode

1. **Perception**: `simulated_perception_node` publishes objects with `class_name = '100g'`, etc.
2. **Detection**: Sorting brain detects object and reads class_name
3. **Grip Setup**: Sends `'E 100'` to Arduino (or simulated)
4. **Open**: Sends `'W'` to Arduino, waits 5 seconds
5. **Move**: Robot moves to pick position
6. **Close**: Sends `'S'` to Arduino, waits 5 seconds
7. **Weight Measurement**: `simulated_perception_node` publishes assumed weight
8. **Sort & Place**: Robot sorts and places, opening gripper with `'W'` + 5s wait

### Real Robot Mode

1. **Perception**: Kevin's perception publishes objects with `class_name` indicating weight
2. **Detection**: Sorting brain detects object and reads class_name
3. **Grip Setup**: Sends `'E <weight>'` to Arduino via serial
4. **Open**: Sends `'W'` to Arduino, waits 5 seconds (physical gripper opens)
5. **Move**: Robot moves to pick position
6. **Close**: Sends `'S'` to Arduino, waits 5 seconds (physical gripper closes)
7. **Weight Measurement**: Asad's `weight_detection_module` measures actual weight via joint torques
8. **Sort & Place**: Robot sorts and places, opening gripper with `'W'` + 5s wait

## Timing Details

All gripper operations include 5-second waits:
- **After open (`W`)**: 5 seconds (allows gripper to fully open)
- **After close (`S`)**: 5 seconds (allows gripper to fully close and stabilize)
- **Edit commands (`E <weight>`)**: No wait (instant angle update)

This ensures the gripper has completed its motion before the robot moves.

## Hardware Setup

### Arduino Upload
1. Open `arduino/servo.ino` in Arduino IDE
2. Select your board (e.g., Arduino Uno)
3. Select port (e.g., `/dev/ttyACM0`)
4. Upload sketch
5. **Important**: After upload, manually set gripper to open position (align marker to 'o')
6. Open Serial Monitor (115200 baud)
7. Press 'F' to initialize

### ROS2 Connection
The gripper controller node automatically connects to `/dev/ttyACM0` at 115200 baud. Configure in launch file if needed:

```yaml
gripper_controller_node:
  ros__parameters:
    serial_port: "/dev/ttyACM0"
    baud_rate: 115200
    simulation_mode: false  # Set true for simulation without hardware
```

## Testing

### Test Arduino Commands Manually

```bash
# Open gripper
echo "W" > /dev/ttyACM0

# Close gripper
echo "S" > /dev/ttyACM0

# Set grip for 100g
echo "E 100" > /dev/ttyACM0

# Set grip for 500g
echo "E 500" > /dev/ttyACM0
```

### Test via ROS2 Service

```bash
# Set grip angle for 200g
ros2 service call /motion_control/gripper_control sort_interfaces/srv/GripperControl "{command: 'e', weight: 200, wait_time_sec: 0.0}"

# Open gripper with 5s wait
ros2 service call /motion_control/gripper_control sort_interfaces/srv/GripperControl "{command: 'W', weight: 0, wait_time_sec: 5.0}"

# Close gripper with 5s wait
ros2 service call /motion_control/gripper_control sort_interfaces/srv/GripperControl "{command: 'S', weight: 0, wait_time_sec: 5.0}"
```

### Test Full Pick Sequence

```bash
# Terminal 1: Run simulation
cd 4231_scripts
./runSimulation.sh

# Terminal 2: Monitor gripper commands
ros2 topic echo /motion_control/gripper_state

# Terminal 3: Launch dashboard and press Start
cd 4231_scripts
./launchDashboard.sh
```

You should see in the logs:
```
[sorting_brain_node]: Step: Setting grip angle for 100g...
[gripper_controller_node]: Grip angle set for 100g
[sorting_brain_node]: Step: Opening gripper (waiting 5s)...
[gripper_controller_node]: Opening gripper via service, waiting 5.0s...
[sorting_brain_node]: Step: Closing gripper (waiting 5s)...
[gripper_controller_node]: Closing gripper via service, waiting 5.0s...
```

## Troubleshooting

### Gripper not responding

**Check serial connection:**
```bash
ls -l /dev/ttyACM*
# Should show /dev/ttyACM0

# Check permissions
sudo usermod -a -G dialout $USER
# Log out and log back in
```

**Check Arduino is running:**
```bash
# Install screen
sudo apt install screen

# Monitor Arduino output
screen /dev/ttyACM0 115200
# Press Ctrl+A, then K to exit
```

### Wrong grip angle

The grip angles are calibrated for specific weight cylinders. If objects slip or are crushed:

1. Open `arduino/servo.ino`
2. Adjust closed positions in `setWeightTarget()`:
   ```cpp
   case 100:
       newClosed = 12;  // Adjust this value
       break;
   ```
3. Re-upload to Arduino
4. Test with: `echo "E 100" > /dev/ttyACM0` then `echo "S" > /dev/ttyACM0`

### Timing too long

If 5-second waits are too long, modify in `sorting_brain_node.py`:

```python
# Change from:
if not self.gripper_control('W', wait_time_sec=5.0):

# To (e.g., 3 seconds):
if not self.gripper_control('W', wait_time_sec=3.0):
```

Or modify default in `gripper_controller_node.cpp`:
```cpp
float wait_time_sec = request->wait_time_sec > 0 ? request->wait_time_sec : 3.0f;  // Change from 5.0f
```

## Integration with Kevin's Perception

Kevin's perception node should publish objects with `class_name` field indicating the weight:
- For a 100g weight: `object.class_name = "100g"` (or "weight_100g", etc.)
- For a 200g weight: `object.class_name = "200g"`
- For a 500g weight: `object.class_name = "500g"`

The sorting brain will parse the numeric value and set the correct grip angle before picking.

## Summary

The gripper integration is now complete:
- ✅ Arduino code supports "E <weight>" commands
- ✅ GripperControl service extended with weight and wait_time parameters
- ✅ Gripper controller node handles all commands with proper timing
- ✅ Sorting brain extracts weight from perception and sets grip angle
- ✅ All pick/place operations include 5-second waits
- ✅ Simulation mode works without hardware
- ✅ Real robot mode ready for Arduino connection

The system is ready for end-to-end testing with physical hardware!
