### Documentation
| Module | Documentation |
|--------|---------------|
| Weight Detection | [README.md](ros2_system/src/weight_detection_module/README.md) |
|        |               |

### Installation

#### Dependencies

```bash
sudo apt update
sudo apt install -y \
    ros-humble-moveit \
    ros-humble-control-msgs \
    ros-humble-cv-bridge \
    ros-humble-image-transport \
    ros-humble-pcl-conversions \
    ros-humble-pcl-ros \
    ros-humble-rclcpp-lifecycle \
    ros-humble-rclcpp-action \
    libopencv-dev \
    libpcl-dev \    
    python3-numpy \
    python3-matplotlib
```
#### ROS2 Packages

```bash
cd ./ros2_system
colcon build
source install/setup.bash
ros2 launch launch/full_system.launch.py

```

### Arduino Commands
Prereq: Ensure Servo in open position (align marker on the gear to 'o' position)
# ROS2 Sort-by-Weight Robot System

A comprehensive ROS2 (Humble) system for robotic sorting using a UR5e arm with RGBD camera and weight-sensing gripper.

## System Architecture

The system is organized into modular packages:

### 1. **sort_interfaces** - Custom Message Definitions
Custom messages, services, and actions used throughout the system.

**Messages:**
- `TargetArea.msg` - Sorting bin definitions
- `DetectedObjects.msg` / `BoundingBox.msg` - 2D object detection
- `WeightEstimate.msg` - Estimated object weight
- `SortDecision.msg` - Sorting target selection
- `EnvironmentStatus.msg` - Safety status
- `ForceFeedback.msg` - Gripper force/weight sensor

**Actions:**
- `PickObject.action` - Pick operation
- `PlaceObject.action` - Place operation
- `PlanTrajectory.action` - Motion planning
- `VerifyWeight.action` - Weight verification

**Services:**
- `SystemCommand.srv` - System control
- `ValidateWorkspace.srv` - Safety validation
- `CalibrateGripper.srv` - Gripper calibration

### 2. **supervisor_module** - System Controller
- `system_controller_node` - Central system coordinator
  - Publishes start/stop commands
  - Defines target sorting areas
  - Monitors system status

### 3. **perception_module** - Vision System
- `rgbd_camera_node` - RGBD camera interface (RealSense/similar)
- `opencv_processor_node` - 2D object detection
- `pointcloud_processor_node` - 3D segmentation and localization

### 4. **recognition_module** - Weight Estimation
- `weight_estimation_node` - Predicts weight from volume analysis

### 5. **planning_module** - Decision Making & Motion Planning
- `sort_node` - Sorting algorithm and bin selection
- `verification_node` - Compare estimated vs actual weight
- `integrity_node` - Environment safety monitoring
- `moveit2_interface_node` - MoveIt2 trajectory planning

### 6. **control_module** - Robot Control
- `robot_driver_node` - UR5e interface (lifecycle node)
- `pick_operation_node` - Pick state machine
- `place_operation_node` - Place state machine
- `gripper_controller_node` - Arduino servo gripper with weight sensor (lifecycle node)

### 7. **motion_control_module** - Robot Arm Motion Control
- `joint_movement_controller` - MoveIt-based joint position controller service
- Robot URDF/SRDF descriptions and meshes
- Launch files for UR5e with gripper
- See [JOINT_MOVEMENT_README.md](ros2_system/src/motion_control_module/JOINT_MOVEMENT_README.md) for detailed usage

## Building the System

```bash
# Navigate to workspace root
cd ~/mtrn4231_jakos/ros2_system

# Install dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build all packages
colcon build --symlink-install

# Source the workspace
source install/setup.bash
```

## Running the System

### Launch Complete System
```bash
ros2 launch launch/full_system.launch.py
```

### Launch Individual Modules
```bash
# Supervisor
ros2 launch supervisor_module supervisor.launch.py

# Perception
ros2 launch perception_module perception.launch.py

# Recognition
ros2 launch recognition_module recognition.launch.py

# Planning
ros2 launch planning_module planning.launch.py

# Control
ros2 launch control_module control.launch.py

# Motion Control (Joint Movement Controller)
ros2 run motion_control_module joint_movement_controller
```

### Controlling Log Output

The system supports configurable logging levels to switch between debug and production modes.

**Debug Mode** (verbose - see all messages):
```bash
ros2 launch launch/full_system.launch.py log_level:=debug
```

**Production Mode** (minimal - only warnings and errors):
```bash
ros2 launch launch/full_system.launch.py log_level:=warn
```

**Error Only** (silent except for errors):
```bash
ros2 launch launch/full_system.launch.py log_level:=error
```

**Available Log Levels:**
- `debug` - Most verbose, shows everything including debug messages
- `info` - Default level, shows informational messages
- `warn` - Only warnings and errors
- `error` - Only error messages
- `fatal` - Only fatal errors

**Alternative: Environment Variable**
```bash
# Suppress all INFO messages globally
export RCUTILS_LOGGING_SEVERITY_THRESHOLD=WARNING
ros2 launch launch/full_system.launch.py
```

## Configuration

Each module has a corresponding YAML configuration file in `<module>/config/`.

Key configuration files:
- `supervisor_module/config/system_controller.yaml` - Target areas, system settings
- `perception_module/config/perception.yaml` - Camera and detection parameters
- `planning_module/config/planning.yaml` - Sorting strategy, workspace limits
- `control_module/config/control.yaml` - Robot IP, pick/place parameters, gripper serial port

## Hardware Setup

### Required Hardware:
1. **UR5e Robot Arm** - Configure IP in `control_module/config/control.yaml`
2. **RGBD Camera** (RealSense D435/D455) - USB connection
3. **Arduino with Servo Gripper** - Serial connection (default: `/dev/ttyACM0`)
4. **Force/Weight Sensor** - Connected to Arduino analog pin

### Arduino Setup:
Upload the gripper control sketch to your Arduino. The node expects:
- Serial communication at 115200 baud
- Command format: `G<angle>\n` (e.g., `G90\n` for 90 degrees)
- Response format: `W<weight>\n` (e.g., `W150.5\n` for 150.5 grams)

## Lifecycle Management

Some nodes use ROS2 lifecycle management for safe startup/shutdown:
- `robot_driver_node`
- `gripper_controller_node`

Lifecycle commands:
```bash
# Configure node
ros2 lifecycle set /robot_driver_node configure

# Activate node
ros2 lifecycle set /robot_driver_node activate

# Deactivate node
ros2 lifecycle set /robot_driver_node deactivate
```

## System Control

### Start the system:
```bash
ros2 service call /system/start sort_interfaces/srv/SystemCommand "{command: 'start'}"
```

### Stop the system:
```bash
ros2 service call /system/stop sort_interfaces/srv/SystemCommand "{command: 'stop'}"
```

### Emergency stop:
```bash
ros2 service call /system/emergency_stop sort_interfaces/srv/SystemCommand "{command: 'emergency_stop'}"
```

## Testing Individual Components

### Test object detection:
```bash
ros2 topic echo /perception/detected_objects
```

### Test weight estimation:
```bash
ros2 topic echo /recognition/estimated_weights
```

### Monitor gripper state:
```bash
ros2 topic echo /motion_control/force_feedback
```

### Manual gripper control:
```bash
# Open gripper (0.0)
ros2 topic pub /motion_control/gripper_command std_msgs/msg/Float32 "data: 0.0"

# Close gripper (1.0)
ros2 topic pub /motion_control/gripper_command std_msgs/msg/Float32 "data: 1.0"
```

## Development Notes

### TODO Items in Code:
Most nodes contain `TODO` comments indicating where hardware-specific code needs to be added:

1. **rgbd_camera_node.cpp** - RealSense SDK integration
2. **opencv_processor_node.cpp** - Actual object detection algorithm (YOLO, SSD, etc.)
3. **pointcloud_processor_node.cpp** - PCL integration for 3D segmentation
4. **weight_estimation_node.cpp** - Volume calculation and ML model
5. **moveit2_interface_node.cpp** - MoveIt2 MoveGroupInterface setup
6. **robot_driver_node.cpp** - UR RTDE or ros2_control integration
7. **gripper_controller_node.cpp** - Serial communication implementation

### Adding MoveIt2 Support:
This system is designed to work with MoveIt2. You'll need to:
1. Create/obtain a URDF for UR5e + gripper
2. Generate MoveIt2 config package
3. Launch move_group node alongside the system
4. Uncomment MoveIt2 code in `moveit2_interface_node.cpp`

## Visualization

Launch RViz2 to visualize:
- Robot model and state
- Camera point cloud
- Detected objects (markers)
- Planned trajectories
- Target sorting areas

```bash
ros2 run rviz2 rviz2
```

Add these displays:
- RobotModel
- PointCloud2 (`/camera/pointcloud`)
- MarkerArray (`/perception/object_markers`)
- Image (`/perception/debug_image`)

## Troubleshooting

### Camera not publishing:
- Check USB connection
- Verify RealSense udev rules are installed
- Check camera permissions

### Arduino not responding:
- Verify serial port: `ls /dev/ttyACM*`
- Check baud rate matches (115200)
- Ensure user is in `dialout` group: `sudo usermod -a -G dialout $USER`

### MoveIt2 planning fails:
- Ensure move_group node is running
- Check URDF is loaded correctly
- Verify planning group name matches configuration

### Robot not moving:
- Check robot_driver_node lifecycle state
- Verify robot IP and network connection
- Ensure robot is not in protective stop

## Contributing

This is a template/stub implementation. To complete the system:
1. Implement TODO items for your specific hardware
2. Tune parameters in config files
3. Add error handling and recovery behaviors
4. Implement actual ML models for weight estimation
5. Add comprehensive testing

## License

MIT License

## Author

MTRN4231 - Advanced Robotics Project
**Commands:**
- 'W' - open gripper
- 'S' - close gripper
- 'E \<weight\>' - edit close (grip) angle
