# Motion Control Module

Gripper controller for the ROS2 pick and sort system using Teensy 4.1.

## Features

- ✅ **Dual Mode Operation**: Simulation mode (no hardware) or Hardware mode (with Teensy)
- ✅ **Lifecycle Node**: Safe startup, shutdown, and state management
- ✅ **Serial Communication**: Cross-platform support (macOS, Linux)
- ✅ **Weight Sensing**: Real-time force/weight measurement
- ✅ **Object Detection**: Automatic detection based on weight threshold
- ✅ **Calibration Service**: Tare and calibrate weight sensor
- ✅ **Comprehensive Tests**: Both simulation and hardware tests

## Quick Start

### Simulation Mode (No Hardware)

```bash
# Build
cd ~/ros2_system
colcon build --packages-select motion_control_module
source install/setup.bash

# Run
ros2 launch motion_control_module gripper_sim.launch.py

# Test
ros2 topic pub --once /motion_control/gripper_command std_msgs/msg/Float32 "data: 1.0"
ros2 topic echo /motion_control/force_feedback
```

### Hardware Mode (With Teensy 4.1)

1. **Setup Teensy**: Follow `TEENSY_SETUP.md`
2. **Upload sketch**: `arduino_sketches/teensy_gripper_controller.ino`
3. **Run**:

```bash
ros2 launch motion_control_module gripper_hardware.launch.py serial_port:=/dev/ttyACM0
```

## Topics

**Subscribed:**
- `/motion_control/gripper_command` (std_msgs/Float32): Gripper position (0.0 = open, 1.0 = closed)

**Published:**
- `/motion_control/gripper_state` (std_msgs/Float32): Current gripper position
- `/motion_control/force_feedback` (sort_interfaces/ForceFeedback): Force and weight data

## Services

- `/motion_control/calibrate_gripper` (sort_interfaces/srv/CalibrateGripper): Calibrate weight sensor

## Parameters

See `config/gripper_params.yaml` for all parameters. Key ones:

- `simulation_mode`: Enable/disable hardware (default: false)
- `serial_port`: Teensy serial port (default: /dev/ttyACM0)
- `baud_rate`: Serial baud rate (default: 115200)
- `publish_rate`: Feedback publish rate in Hz (default: 20.0)
- `object_detection_threshold`: Weight threshold in grams (default: 50.0)

## Testing

**Simulation tests:**
```bash
cd test
python3 test_gripper_simulation.py
```

**Hardware tests** (requires Teensy):
```bash
cd test
python3 test_gripper_hardware.py
```

## Hardware Requirements

- Teensy 4.1 microcontroller
- Servo motor (standard hobby servo)
- Force/weight sensor (FSR, load cell, etc.)
- USB cable

## Files

```
motion_control_module/
├── src/
│   └── gripper_controller_node.cpp    # Main ROS2 node
├── include/
│   └── simple_serial.hpp              # Serial communication library
├── config/
│   └── gripper_params.yaml            # Configuration parameters
├── launch/
│   ├── gripper_sim.launch.py          # Simulation mode launch
│   └── gripper_hardware.launch.py     # Hardware mode launch
├── test/
│   ├── test_gripper_simulation.py     # Simulation tests
│   └── test_gripper_hardware.py       # Hardware tests
├── TEENSY_SETUP.md                    # Teensy setup guide
└── README.md                          # This file
```

## Documentation

- **Setup Guide**: See `TEENSY_SETUP.md` for detailed Teensy setup instructions
- **Architecture**: See `ros2_system/ARCHITECTURE.md` for system overview
- **Testing**: See test files for examples

## Troubleshooting

**Q: Node won't start in hardware mode**
- Check Teensy is connected: `ls /dev/ttyACM*` or `ls /dev/cu.usbmodem*`
- Verify sketch is uploaded (open Arduino Serial Monitor)
- Check permissions (Linux): `sudo chmod 666 /dev/ttyACM0`
- Try simulation mode first to verify ROS2 setup

**Q: No force feedback messages**
- Check node is in Active state (lifecycle)
- Verify topics: `ros2 topic list | grep motion_control`
- Check logs: Add `--screen` to launch command

**Q: Servo not moving**
- Verify wiring (signal to pin 9)
- Test in Arduino Serial Monitor: send `G90`
- Check servo power supply

**Q: Weight readings are noisy**
- Increase `filter_window_size` in config
- Calibrate sensor: `ros2 service call /motion_control/calibrate_gripper ...`
- Check hardware filtering (add capacitor)

## Next Steps

After getting the gripper controller working:
1. Test with control module (pick/place operations)
2. Integrate with full pick-and-sort system
3. Calibrate with actual objects
4. Tune detection thresholds

## License

MIT
