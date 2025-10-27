# Teensy 4.1 Gripper Controller Setup Guide

This guide will help you set up the Teensy 4.1 gripper controller for the ROS2 pick and sort system.

## Hardware Requirements

- **Teensy 4.1** microcontroller board
- **Servo motor** (standard hobby servo, 0-180 degrees)
- **Force/weight sensor** (FSR, load cell, or strain gauge)
- **USB cable** (Micro-B USB)
- **Power supply** (5V for servo, if needed)
- **Jumper wires**

## Software Requirements

### 1. Install Arduino IDE

Download and install the Arduino IDE from: https://www.arduino.cc/en/software

- **macOS**: Download the `.dmg` file or use Homebrew: `brew install --cask arduino`
- **Linux**: Download the AppImage or use your package manager
- **Windows**: Download the installer

### 2. Install Teensyduino

Teensyduino is an add-on for Arduino IDE that adds support for Teensy boards.

1. Download Teensyduino from: https://www.pjrc.com/teensy/td_download.html
2. Run the installer
3. Select your Arduino IDE installation directory
4. Install all libraries (recommended)

**macOS Note**: You may need to grant security permissions:
- System Settings → Privacy & Security → Allow applications downloaded from: App Store and identified developers

### 3. Verify Installation

1. Open Arduino IDE
2. Go to `Tools` → `Board` → `Teensyduino` → `Teensy 4.1`
3. If you see "Teensy 4.1", installation was successful!

## Hardware Wiring

### Teensy 4.1 Pinout

```
Teensy 4.1 Pin Connections:

Pin 9  → Servo Signal (PWM)
GND    → Servo Ground
5V*    → Servo Power (if needed)

Pin A0 (14) → Force Sensor Signal
GND         → Force Sensor Ground
3.3V        → Force Sensor Power (if needed)

USB    → Computer (for programming and serial communication)
```

*Note: If your servo draws significant current, power it externally and connect grounds together.

### Servo Motor Connection

```
Servo Wire Colors (typical):
- Brown/Black  → Ground (GND)
- Red          → Power (5V)
- Orange/White → Signal (Pin 9)
```

### Force Sensor Connection (Example: FSR - Force Sensitive Resistor)

```
Simple FSR Circuit:
3.3V ---[10kΩ]--- A0 ---[FSR]--- GND

Or for Load Cell with HX711:
HX711 DT  → A0
HX711 SCK → A1 (or any digital pin)
HX711 VCC → 3.3V or 5V
HX711 GND → GND
```

## Upload Code to Teensy

### Step 1: Open the Sketch

1. Navigate to: `ros2_system/arduino_sketches/teensy_gripper_controller.ino`
2. Double-click to open in Arduino IDE

### Step 2: Configure Arduino IDE

1. **Select Board**: `Tools` → `Board` → `Teensyduino` → `Teensy 4.1`
2. **Select USB Type**: `Tools` → `USB Type` → `Serial`
3. **Select CPU Speed**: `Tools` → `CPU Speed` → `600 MHz` (default is fine)
4. **Select Port**: `Tools` → `Port` → Select your Teensy port
   - **macOS**: `/dev/cu.usbmodem*` (e.g., `/dev/cu.usbmodem141301`)
   - **Linux**: `/dev/ttyACM0` or `/dev/ttyUSB0`
   - **Windows**: `COM3` or similar

### Step 3: Compile and Upload

1. Click the **Verify** button (✓) to compile
   - Should see: "Done compiling"
2. Click the **Upload** button (→) to upload to Teensy
   - Teensy Loader window will open automatically
   - Code will be uploaded and Teensy will reboot
3. Look for "Reboot OK" in the Teensy Loader

### Step 4: Test Serial Communication

1. Open Serial Monitor: `Tools` → `Serial Monitor`
2. Set baud rate to **115200**
3. You should see:
   ```
   Teensy 4.1 Gripper Controller Ready
   Commands: G<angle> (0-180), C (calibrate)
   Calibrating... Remove all weight from sensor.
   ...
   OK:Calibrated
   W0.0
   W0.0
   ...
   ```

### Step 5: Test Commands

In the Serial Monitor, try these commands:

```
G0      → Open gripper (0 degrees)
G90     → Mid position (90 degrees)
G180    → Close gripper (180 degrees)
C       → Calibrate weight sensor
S       → Print status
```

You should see responses like:
```
OK:G0
OK:G90
OK:G180
```

## ROS2 Integration

### Find Your Serial Port

**macOS:**
```bash
ls /dev/cu.usbmodem*
# Example output: /dev/cu.usbmodem141301
```

**Linux:**
```bash
ls /dev/ttyACM*
# Example output: /dev/ttyACM0
```

### Update Configuration

Edit `config/gripper_params.yaml`:

```yaml
/**:
  ros__parameters:
    simulation_mode: false  # Set to false for hardware
    serial_port: "/dev/cu.usbmodem141301"  # Your port here
    baud_rate: 115200
```

**Or** specify port at launch time:
```bash
ros2 launch motion_control_module gripper_hardware.launch.py serial_port:=/dev/cu.usbmodem141301
```

### Grant Serial Port Access (Linux Only)

```bash
# Add your user to the dialout group
sudo usermod -a -G dialout $USER

# Log out and log back in, or:
newgrp dialout

# Alternatively, set permissions for this session only:
sudo chmod 666 /dev/ttyACM0
```

## Testing

### 1. Build the ROS2 Package

```bash
cd ~/ros2_system
colcon build --packages-select motion_control_module
source install/setup.bash
```

### 2. Run Simulation Mode (No Hardware)

Test without Teensy connected:

```bash
ros2 launch motion_control_module gripper_sim.launch.py
```

In another terminal:
```bash
# Send gripper command
ros2 topic pub --once /motion_control/gripper_command std_msgs/msg/Float32 "data: 0.5"

# Check gripper state
ros2 topic echo /motion_control/gripper_state

# Check force feedback
ros2 topic echo /motion_control/force_feedback
```

### 3. Run Hardware Mode (With Teensy)

Connect Teensy and run:

```bash
ros2 launch motion_control_module gripper_hardware.launch.py
```

Watch for log messages:
```
[INFO] [gripper_controller_node]: Hardware mode: Opening serial port /dev/cu.usbmodem141301 @ 115200 baud
[INFO] [gripper_controller_node]: Serial port opened successfully
[INFO] [gripper_controller_node]: Gripper controller activated
```

Test with commands:
```bash
# Close gripper
ros2 topic pub --once /motion_control/gripper_command std_msgs/msg/Float32 "data: 1.0"

# Open gripper
ros2 topic pub --once /motion_control/gripper_command std_msgs/msg/Float32 "data: 0.0"
```

### 4. Run Automated Tests

**Simulation tests** (no hardware needed):
```bash
cd ~/ros2_system/src/motion_control_module/test
python3 test_gripper_simulation.py
```

**Hardware tests** (Teensy required):
```bash
cd ~/ros2_system/src/motion_control_module/test
python3 test_gripper_hardware.py
```

## Calibration

### Weight Sensor Calibration

1. **Tare (Zero) the sensor:**
   ```bash
   ros2 service call /motion_control/calibrate_gripper sort_interfaces/srv/CalibrateGripper "{tare_weight_sensor: true, calibrate_position: false}"
   ```

2. **Two-point calibration** (in Teensy Serial Monitor):
   - Modify `teensy_gripper_controller.ino` to call `calibrateWithKnownWeight(100.0)` with a known weight
   - Or update `calibrationFactor` in the Arduino code based on your sensor's datasheet

### Gripper Position Calibration

Update in `config/gripper_params.yaml`:
```yaml
min_servo_angle: 0      # Fully open
max_servo_angle: 180    # Fully closed
```

## Troubleshooting

### Teensy Not Detected

**macOS:**
- Check System Settings → Privacy & Security → Allow USB devices
- Try a different USB port or cable
- Press the white button on Teensy to enter bootloader mode

**Linux:**
```bash
# Check if Teensy is detected
lsusb | grep -i teensy

# Check serial port permissions
ls -l /dev/ttyACM0

# Add udev rule (if needed)
sudo cp /path/to/teensyduino/00-teensy.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
```

### Serial Port Permission Denied

```bash
# Linux
sudo chmod 666 /dev/ttyACM0

# Or permanently:
sudo usermod -a -G dialout $USER
# Then log out and log back in
```

### ROS2 Node Can't Open Serial Port

1. Check the port name:
   ```bash
   ls /dev/cu.usbmodem*  # macOS
   ls /dev/ttyACM*       # Linux
   ```

2. Close any other programs using the port (Arduino Serial Monitor, etc.)

3. Check logs:
   ```bash
   ros2 launch motion_control_module gripper_hardware.launch.py --screen
   ```

4. Test with simulation mode first:
   ```bash
   ros2 launch motion_control_module gripper_sim.launch.py
   ```

### Servo Not Moving

- Check wiring (especially signal wire to pin 9)
- Check servo power supply (may need external 5V)
- Test in Arduino Serial Monitor: send `G90`, `G0`, `G180`
- Verify servo works with Arduino example: `File` → `Examples` → `Servo` → `Sweep`

### Force Sensor Readings Are Noisy

1. Add hardware filtering (capacitor across sensor)
2. Increase `filter_window_size` in config:
   ```yaml
   filter_window_size: 20  # More filtering
   ```
3. Adjust ALPHA in Teensy code (lower = more filtering)

### Compilation Errors

- Make sure Teensyduino is installed
- Select `Tools` → `Board` → `Teensy 4.1`
- Update Arduino IDE and Teensyduino to latest versions
- Check that Servo library is included (comes with Arduino IDE)

## Next Steps

Once the gripper controller is working:

1. **Calibrate weight sensor** with known weights
2. **Tune detection threshold** in `config/gripper_params.yaml`
3. **Test with other nodes** (control, planning, etc.)
4. **Implement** in full pick-and-place system

## Additional Resources

- **Teensy 4.1 Documentation**: https://www.pjrc.com/store/teensy41.html
- **Teensyduino Reference**: https://www.pjrc.com/teensy/teensyduino.html
- **Arduino Servo Library**: https://www.arduino.cc/reference/en/libraries/servo/
- **ROS2 Humble Documentation**: https://docs.ros.org/en/humble/

## Support

If you encounter issues:
1. Check this guide's troubleshooting section
2. Review ROS2 logs with `--screen` flag
3. Test Teensy independently with Serial Monitor
4. Verify hardware connections

Happy building! 🤖
