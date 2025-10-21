# Quick Start Guide

## What Was Created

A complete ROS2 (Humble, C++17) system with **7 packages**, **13 nodes**, **7 custom messages**, **4 action definitions**, and **3 services**.

## System Summary

### Packages Created:
1. **sort_interfaces** - 7 messages, 4 actions, 3 services
2. **supervisor_module** - 1 node (system_controller)
3. **perception_module** - 3 nodes (camera, opencv, pointcloud)
4. **recognition_module** - 1 node (weight estimation)
5. **planning_module** - 4 nodes (sort, verification, integrity, moveit2)
6. **control_module** - 3 nodes (robot driver, pick, place)
7. **motion_control_module** - 1 node (gripper controller)

### Total Files Created:
- **13 C++ nodes** (`.cpp` files)
- **7 CMakeLists.txt** files
- **7 package.xml** files
- **6 launch files** (including master `full_system.launch.py`)
- **6 YAML config files**
- **7 message definitions**
- **4 action definitions**
- **3 service definitions**
- **1 Arduino sketch** template
- **Build script** and documentation

## First-Time Setup

### 1. Build the System
```bash
cd ~/mtrn4231_jakos/ros2_system
./build_all.sh
```

### 2. Source the Workspace
```bash
source install/setup.bash
```

### 3. Hardware Setup (if available)

#### Arduino:
1. Upload `arduino_sketches/gripper_controller.ino` to your Arduino
2. Connect Arduino via USB
3. Update serial port in `src/motion_control_module/config/motion_control.yaml` if needed

#### UR5e Robot:
1. Update robot IP in `src/control_module/config/control.yaml`
2. Ensure network connectivity

#### Camera:
1. Connect RGBD camera (RealSense or similar)
2. Update parameters in `src/perception_module/config/perception.yaml`

## Running the System

### Option 1: Launch Everything
```bash
ros2 launch launch/full_system.launch.py
```

### Option 2: Launch Modules Individually

In separate terminals:

```bash
# Terminal 1: Supervisor
ros2 launch supervisor_module supervisor.launch.py

# Terminal 2: Perception
ros2 launch perception_module perception.launch.py

# Terminal 3: Recognition
ros2 launch recognition_module recognition.launch.py

# Terminal 4: Planning
ros2 launch planning_module planning.launch.py

# Terminal 5: Control
ros2 launch control_module control.launch.py

# Terminal 6: Motion Control
ros2 launch motion_control_module motion_control.launch.py
```

## Lifecycle Node Management

Two nodes use lifecycle management for safe operation:

### Robot Driver
```bash
# Configure
ros2 lifecycle set /robot_driver_node configure

# Activate
ros2 lifecycle set /robot_driver_node activate
```

### Gripper Controller
```bash
# Configure
ros2 lifecycle set /gripper_controller_node configure

# Activate
ros2 lifecycle set /gripper_controller_node activate
```

## Basic System Commands

### Start Sorting
```bash
ros2 service call /system/start sort_interfaces/srv/SystemCommand "{command: 'start'}"
```

### Stop System
```bash
ros2 service call /system/stop sort_interfaces/srv/SystemCommand "{command: 'stop'}"
```

### Emergency Stop
```bash
ros2 service call /system/emergency_stop sort_interfaces/srv/SystemCommand "{command: 'emergency_stop'}"
```

## Testing Individual Components

### Test Camera
```bash
ros2 topic echo /camera/color/image_raw
ros2 topic echo /camera/pointcloud
```

### Test Object Detection
```bash
ros2 topic echo /perception/detected_objects
```

### Test Weight Estimation
```bash
ros2 topic echo /recognition/estimated_weights
```

### Test Gripper
```bash
# Open gripper
ros2 topic pub /motion_control/gripper_command std_msgs/msg/Float32 "data: 0.0"

# Close gripper
ros2 topic pub /motion_control/gripper_command std_msgs/msg/Float32 "data: 1.0"

# Monitor force feedback
ros2 topic echo /motion_control/force_feedback
```

### Calibrate Gripper
```bash
ros2 service call /motion_control/calibrate_gripper sort_interfaces/srv/CalibrateGripper "{tare_weight_sensor: true, calibrate_position: true, calibrate_force: false}"
```

## Monitoring the System

### View All Topics
```bash
ros2 topic list
```

### View System Graph
```bash
rqt_graph
```

### View Node Info
```bash
ros2 node list
ros2 node info /system_controller_node
```

### Monitor Specific Topic
```bash
# Weight estimates
ros2 topic echo /recognition/estimated_weights

# Sort decisions
ros2 topic echo /planning/sort_decisions

# Robot status
ros2 topic echo /control/robot_status
```

## Development Workflow

### 1. Implement Hardware-Specific Code

All nodes contain `TODO` comments marking where you need to add actual hardware integration:

**Priority TODOs:**
1. `rgbd_camera_node.cpp:38` - Initialize RealSense SDK
2. `opencv_processor_node.cpp:67` - Implement object detection (YOLO/SSD)
3. `gripper_controller_node.cpp:127` - Add serial communication
4. `robot_driver_node.cpp:141` - Connect to UR5e via RTDE

### 2. Search for TODOs
```bash
grep -r "TODO" src/*/src/*.cpp
```

### 3. After Making Changes
```bash
# Rebuild only changed packages
colcon build --packages-select <package_name>

# Or rebuild everything
./build_all.sh
```

### 4. Test Your Changes
```bash
# Launch the modified module
ros2 launch <module_name> <launch_file>

# Monitor relevant topics
ros2 topic echo <topic_name>
```

## Common Issues

### Build Errors
- **Missing dependencies**: Run `rosdep install --from-paths src --ignore-src -r -y`
- **MoveIt2 errors**: MoveIt2 code is commented out by default - requires proper setup
- **OpenCV not found**: Install `libopencv-dev`

### Runtime Errors
- **Serial port permission denied**: Add user to dialout group: `sudo usermod -a -G dialout $USER`
- **Camera not found**: Check USB connection and permissions
- **Robot not responding**: Verify IP address and network connectivity

### No Data on Topics
- **Check node status**: `ros2 node list`
- **Check topic connections**: `ros2 topic info <topic_name>`
- **Verify lifecycle state**: `ros2 lifecycle get /node_name`

## Next Steps

1. **Configure Target Areas**: Edit `supervisor_module/config/system_controller.yaml`
2. **Tune Detection Parameters**: Edit `perception_module/config/perception.yaml`
3. **Calibrate Weight Sensor**: Use calibration service
4. **Add MoveIt2 Support**: Configure MoveIt2 for your robot
5. **Implement ML Models**: Add weight estimation model in `weight_estimation_node.cpp`

## Documentation

- **README.md** - Detailed system documentation
- **ARCHITECTURE.md** - System architecture and data flow
- **This file** - Quick start guide

## Getting Help

Check TODO comments in code for specific implementation guidance:
```bash
grep -n "TODO" src/planning_module/src/moveit2_interface_node.cpp
```

Each node includes extensive comments explaining what needs to be implemented for your specific hardware.

---

**System Status**: ✅ All stub implementations complete and ready for hardware integration!
