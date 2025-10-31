# Gripper Controller Setup Guide

This guide explains how to use the gripper controller system for both simulation and hardware modes.

## Overview

The gripper control system consists of:
- **gripper_controller_node**: Main ROS2 lifecycle node that handles serial communication with Teensy/Arduino
- **gripper_button_interface**: Interactive keyboard interface for manual control
- **Arduino sketch**: Firmware for Teensy/Arduino to control the servo gripper

## Architecture

```
┌─────────────────────────┐
│ gripper_button_interface│
│   (Keyboard: w/s/q)     │
└───────────┬─────────────┘
            │ Service Call
            ▼
┌─────────────────────────┐
│ gripper_controller_node │
│  (ROS2 Lifecycle Node)  │
└───────────┬─────────────┘
            │ Serial (w/s)
            ▼
┌─────────────────────────┐
│   Teensy/Arduino        │
│   (Servo Controller)    │
└─────────────────────────┘
```

## Serial Protocol

### ROS2 → Arduino
- `w` - Open gripper
- `s` - Close gripper

### Arduino → ROS2
- `W<weight>\n` - Weight measurement in grams (e.g., "W152.5\n")

## Setup Instructions

### 1. Build the ROS2 Package

```bash
cd ~/ros2_ws
colcon build --packages-select sort_interfaces motion_control_module
source install/setup.bash
```

### 2. Upload Arduino Sketch to Teensy

1. Open Arduino IDE
2. Load `ros2_system/arduino_sketches/gripper_controller_ros2.ino`
3. Select your Teensy board (Tools → Board → Teensy 4.1)
4. Select the serial port (Tools → Port)
5. Upload the sketch

**Important**: The sketch is configured for:
- Servo on pin 9
- Open position: 90 degrees
- Closed position: 15 degrees (for 200g weight)
- Optional force sensor on pin A0

Adjust these values in the sketch if needed for your gripper.

### 3. Find Your Serial Port

**On macOS:**
```bash
ls /dev/cu.usbmodem*
```

**On Linux:**
```bash
ls /dev/ttyACM*
```

Note the port name (e.g., `/dev/ttyACM0` or `/dev/cu.usbmodem141301`)

## Usage

### Simulation Mode (No Hardware Required)

Perfect for testing on your Mac or when the Teensy isn't available:

```bash
# Launch with button interface
ros2 launch motion_control_module gripper_sim_with_interface.launch.py

# Or just the controller (for programmatic control)
ros2 launch motion_control_module gripper_sim.launch.py
```

### Hardware Mode (Teensy Required)

**On Linux VM with Teensy connected:**

```bash
# Launch with button interface (auto-detects /dev/ttyACM0)
ros2 launch motion_control_module gripper_hardware_with_interface.launch.py

# Or specify custom serial port
ros2 launch motion_control_module gripper_hardware_with_interface.launch.py serial_port:=/dev/ttyACM1
```

**On macOS with Teensy:**

```bash
ros2 launch motion_control_module gripper_hardware_with_interface.launch.py serial_port:=/dev/cu.usbmodem141301
```

### Using the Button Interface

Once launched, you'll see:
```
======================================
    GRIPPER MANUAL CONTROL
======================================
  [W] - Open gripper
  [S] - Close gripper
  [H] - Show this help
  [Q] - Quit
======================================
```

Simply press `W` to open or `S` to close the gripper!

## Programmatic Control

### From Python

```python
import rclpy
from rclpy.node import Node
from sort_interfaces.srv import GripperControl

class GripperControlExample(Node):
    def __init__(self):
        super().__init__('gripper_example')
        self.client = self.create_client(GripperControl, '/motion_control/gripper_control')

    def open_gripper(self):
        request = GripperControl.Request()
        request.command = 'open'  # or 'w'
        future = self.client.call_async(request)
        return future

    def close_gripper(self):
        request = GripperControl.Request()
        request.command = 'close'  # or 's'
        future = self.client.call_async(request)
        return future

# Usage
rclpy.init()
node = GripperControlExample()
node.open_gripper()
```

### From Command Line

```bash
# Open gripper
ros2 service call /motion_control/gripper_control sort_interfaces/srv/GripperControl "{command: 'open'}"

# Close gripper
ros2 service call /motion_control/gripper_control sort_interfaces/srv/GripperControl "{command: 'close'}"
```

## Monitoring

### View Force Feedback

```bash
ros2 topic echo /motion_control/force_feedback
```

Output:
```yaml
header:
  stamp: ...
gripper_force: 1.53  # Newtons
measured_weight: 150.2  # grams
gripper_position: 1.0  # 0.0=open, 1.0=closed
object_detected: true
raw_sensor_value: 150.2
```

### View Gripper State

```bash
ros2 topic echo /motion_control/gripper_state
```

## Lifecycle Management

The gripper controller is a lifecycle node with states:

```bash
# Check current state
ros2 lifecycle get /gripper_controller_node

# Manually control lifecycle
ros2 lifecycle set /gripper_controller_node configure
ros2 lifecycle set /gripper_controller_node activate
ros2 lifecycle set /gripper_controller_node deactivate
ros2 lifecycle set /gripper_controller_node cleanup
```

The launch files automatically configure and activate the node.

## Configuration

Edit `config/gripper_params.yaml` to customize:

```yaml
gripper_controller_node:
  ros__parameters:
    simulation_mode: false
    serial_port: "/dev/ttyACM0"
    baud_rate: 115200
    publish_rate: 20.0

    # Weight sensor calibration
    weight_calibration_factor: 1.0
    zero_offset: 0.0
    object_detection_threshold: 50.0  # grams
```

## Troubleshooting

### "Failed to open serial port"
- Check that the Teensy is connected
- Verify the serial port name with `ls /dev/tty*`
- Check permissions: `sudo chmod 666 /dev/ttyACM0`
- On Linux, add user to dialout group: `sudo usermod -a -G dialout $USER`

### "Service call timed out"
- Ensure gripper_controller_node is running: `ros2 node list`
- Check node is activated: `ros2 lifecycle get /gripper_controller_node`

### Gripper not moving
- Check Arduino serial monitor (9600 baud) to see if commands are received
- Verify servo is connected to pin 9
- Check power supply to servo

### Weight readings are wrong
- Calibrate using the service:
  ```bash
  ros2 service call /motion_control/calibrate_gripper sort_interfaces/srv/CalibrateGripper "{tare_weight_sensor: true}"
  ```
- Adjust `weight_calibration_factor` in config file

## Integration with Other Nodes

The gripper controller publishes force feedback that other nodes can use:

```python
# Subscribe to force feedback in your planning/control nodes
self.force_sub = self.create_subscription(
    ForceFeedback,
    '/motion_control/force_feedback',
    self.force_callback,
    10
)

def force_callback(self, msg):
    if msg.object_detected:
        self.get_logger().info(f'Grasping object: {msg.measured_weight}g')
```

## Hardware Setup

### Wiring
- Servo signal → Teensy Pin 9
- Servo power → 5V supply (NOT Teensy 5V!)
- Servo ground → Common ground with Teensy
- Force sensor (optional) → Teensy Pin A0

### Servo Calibration
1. Run the gripper manually and observe positions
2. Update `open_pos` and `closed_pos` in the Arduino sketch
3. Adjust `moveRes` and `waitTime` for desired speed

## Next Steps

- Add force-based grip detection
- Implement automatic weight measurement on grasp
- Integrate with motion planning
- Add safety limits and error recovery
