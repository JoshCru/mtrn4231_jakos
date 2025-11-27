# Gripper Control Module

This module contains all gripper control functionality for the MTRN4231 project, including Arduino/Teensy serial communication.

## Overview

The gripper control has been integrated into the `control_module` package, which now handles:
- **Robot arm control** (robot_driver_node, pick/place operations)
- **Gripper control** (Arduino/Teensy serial communication)

## Components

### Arduino Serial Communication

The gripper communicates with a Teensy 4.1 microcontroller via serial port. The serial communication is handled by:

**Header file**: `include/simple_serial.hpp`
- Cross-platform C++ serial communication wrapper
- Works on Linux and macOS
- Supports configurable baud rates (9600, 115200)
- Provides synchronous read/write operations

**Key features**:
- Non-blocking reads with timeout
- Line-based reading (for sensor data)
- Automatic port configuration (8N1 mode)

### Gripper Nodes

#### 1. gripper_controller_node.cpp
Main lifecycle node for gripper control with features:
- Serial communication with Teensy/Arduino
- Force/weight sensing
- Simulation mode support
- Real-time force feedback publishing
- Service interfaces for calibration and control

**Topics**:
- `/motion_control/gripper_command` (subscribe) - Float32: 0.0 (open) to 1.0 (close)
- `/motion_control/gripper_state` (publish) - Current gripper position
- `/motion_control/force_feedback` (publish) - Force sensor data

**Services**:
- `/motion_control/calibrate_gripper` - Calibrate force sensor
- `/motion_control/gripper_control` - Command-based control ("open"/"close"/"w"/"s")

**Parameters**:
- `simulation_mode`: Enable/disable hardware (default: false)
- `serial_port`: Device path (Linux: `/dev/ttyACM0`, macOS: `/dev/cu.usbmodem*`)
- `baud_rate`: Serial baud rate (default: 115200)
- `publish_rate`: Sensor data publish rate in Hz (default: 20.0)

#### 2. gripper_button_interface.cpp
Button-based interface for manual gripper control during testing.

## Configuration Files

- `config/gripper_params.yaml` - Gripper parameters (calibration, thresholds, etc.)
- `urdf/gripper.urdf` - URDF description of gripper
- `meshes/gripper.stl` - 3D mesh for visualization

## Launch Files

- `gripper_hardware.launch.py` - Launch with real hardware
- `gripper_sim.launch.py` - Launch in simulation mode
- `gripper_hardware_with_interface.launch.py` - Hardware + button interface
- `gripper_sim_with_interface.launch.py` - Simulation + button interface

## Protocol

The Teensy/Arduino communicates using a simple text-based protocol:

**Commands to Teensy** (from ROS):
- `"w"` - Open gripper
- `"s"` - Close gripper

**Responses from Teensy** (to ROS):
- `"W<value>\n"` - Weight reading in grams (e.g., "W150.5\n")

## Migration from util_arduino_serial

The old Python-based `util_arduino_serial` package has been replaced by this C++ implementation:

**Old approach** (Python):
- Separate Python node for serial communication
- Limited integration with gripper control
- Simple command forwarding

**New approach** (C++):
- Integrated C++ serial communication via `simple_serial.hpp`
- Direct integration in gripper controller
- Better performance and lower latency
- Lifecycle node support for proper initialization/cleanup

## Building

The gripper nodes are built as part of the control_module:

```bash
cd ~/ros2_ws
colcon build --packages-select control_module
source install/setup.bash
```

## Testing

Test files are located in `test/`:
- `test_gripper_hardware.py` - Hardware integration tests
- `test_gripper_simulation.py` - Simulation tests

## Usage Example

Launch gripper in hardware mode:
```bash
ros2 launch control_module gripper_hardware.launch.py
```

Control gripper via topic:
```bash
# Open gripper
ros2 topic pub /motion_control/gripper_command std_msgs/msg/Float32 "data: 0.0"

# Close gripper
ros2 topic pub /motion_control/gripper_command std_msgs/msg/Float32 "data: 1.0"
```

Control gripper via service:
```bash
# Open gripper
ros2 service call /motion_control/gripper_control sort_interfaces/srv/GripperControl "{command: 'open'}"

# Close gripper
ros2 service call /motion_control/gripper_control sort_interfaces/srv/GripperControl "{command: 'close'}"
```

Monitor force feedback:
```bash
ros2 topic echo /motion_control/force_feedback
```
