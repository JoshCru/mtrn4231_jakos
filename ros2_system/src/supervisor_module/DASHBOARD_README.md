# System Dashboard UI - Brain Module

## Overview

The System Dashboard is a comprehensive PyQt5-based graphical user interface for monitoring and controlling the MTRN4231 Sort-by-Weight Robot System. It serves as the "brain module" that provides centralized control and real-time monitoring of all subsystems.

## Features

### 1. **System Control Panel**
- **Start/Stop System**: Control the overall system state
- **Emergency Stop**: Immediate halt of all operations with visual feedback
- **System Status Display**: Real-time status with color-coded indicators
  - Green: Running
  - Orange: Stopped
  - Red: Emergency Stop
  - Gray: Idle/Unknown

### 2. **Gripper Control**
- **Manual Control**: Open/close gripper with buttons
- **Position Display**: Real-time gripper position (0.0 - 1.0)
- **Visual Progress Bar**: Gripper state visualization
- **Force Feedback**:
  - Measured weight in grams
  - Object detection status
  - Real-time force sensor readings
- **Calibration**: Tare weight sensor on demand

### 3. **Robot Arm Status**
- Robot status display
- Real-time joint states for all 6 joints
- Position monitoring

### 4. **Module Status Overview**
Monitors all system modules:
- **Supervisor**: System controller status
- **Perception**: Camera and object detection
- **Recognition**: Weight estimation
- **Planning**: Sort decisions and trajectory planning
- **Control**: Robot arm driver
- **Gripper**: Gripper controller

Each module shows:
- Current status
- Last update timestamp
- Active/inactive state

### 5. **Perception Data Display**
- Number of detected objects
- Real-time object detection updates

### 6. **Recognition Data Display**
- Estimated weight with confidence
- Weight estimation updates

### 7. **Planning Data Display**
- Latest sort decisions
- Target bin assignments
- Object routing information

### 8. **Topic Monitor**
Real-time scrolling log of all topic activity:
- Timestamped messages
- Force feedback data
- Object detection events
- Weight estimates
- Sort decisions
- Automatic scrolling with 100-line buffer

### 9. **System Logs**
Comprehensive logging with:
- Timestamps
- Log levels (INFO, WARN, ERROR)
- Color-coded messages
- Service call results
- System events

## Installation

### Prerequisites

Install PyQt5:
```bash
sudo apt-get install python3-pyqt5
# Or on macOS:
brew install pyqt5
```

### Build the Package

```bash
cd ~/ros2_ws
colcon build --packages-select supervisor_module
source install/setup.bash
```

## Usage

### Launch Dashboard with System Controller

To launch both the system controller backend and the dashboard UI:

```bash
# Terminal 1: Launch system controller
ros2 run supervisor_module system_controller_node

# Terminal 2: Launch dashboard
ros2 run supervisor_module system_dashboard
```

Or use the launch file:

```bash
ros2 launch supervisor_module dashboard.launch.py
```

### Full System Launch

To launch the entire system with the dashboard:

```bash
# Launch all modules
ros2 launch <your_main_launch_file>

# In separate terminal, launch dashboard
ros2 run supervisor_module system_dashboard
```

## Dashboard Tabs

### Tab 1: System Control
Main control interface with:
- System start/stop/emergency stop
- Gripper manual control
- Robot arm status
- Real-time sensor feedback

### Tab 2: Module Status
Overview of all system modules with:
- Status table for 6 modules
- Perception, Recognition, and Planning data displays
- Last update timestamps

### Tab 3: Topic Monitor
Scrolling log of all ROS2 topic messages:
- Real-time message display
- Automatic line limiting (100 lines)
- Clear button for cleanup

### Tab 4: System Logs
System-level event logging:
- Service call results
- Error messages
- Warning notifications
- Color-coded by severity

## Topics Monitored

The dashboard subscribes to:

```
/system/status                      (String)
/motion_control/gripper_state       (Float32)
/motion_control/force_feedback      (ForceFeedback)
/perception/detected_objects        (DetectedObjects)
/recognition/estimated_weights      (WeightEstimate)
/planning/sort_decisions            (SortDecision)
/control/robot_status               (String)
/control/joint_states               (JointState)
```

## Services Used

The dashboard can call:

```
/system/start                       (SystemCommand)
/system/stop                        (SystemCommand)
/system/emergency_stop              (SystemCommand)
/motion_control/gripper_control     (GripperControl)
/motion_control/calibrate_gripper   (CalibrateGripper)
```

## Architecture

### Backend (ROS2 Node)
- `SystemDashboardNode`: ROS2 node running in separate thread
- Subscribes to all monitoring topics
- Provides service clients for control commands
- Emits Qt signals for thread-safe UI updates

### Frontend (PyQt5 GUI)
- `SystemDashboard`: Main window with tabbed interface
- Thread-safe signal/slot mechanism for updates
- Real-time visualization of all system data
- Interactive control buttons

### Threading Model
- ROS2 executor runs in separate `ROS2Thread`
- Qt event loop runs in main thread
- PyQt signals ensure thread-safe communication

## Customization

### Adding New Topics

To monitor additional topics, edit `system_dashboard.py`:

1. Add signal:
```python
new_topic_signal = pyqtSignal(object)
```

2. Add subscriber in `SystemDashboardNode.__init__()`:
```python
self.create_subscription(MsgType, '/topic/name', self.callback, 10)
```

3. Add callback:
```python
def callback(self, msg):
    self.new_topic_signal.emit(msg)
```

4. Connect signal in `SystemDashboard.__init__()`:
```python
self.ros_node.new_topic_signal.connect(self.update_method)
```

5. Add update method:
```python
def update_method(self, msg):
    # Update UI widgets
    pass
```

### Adding New Services

To add service clients:

1. Create client in `SystemDashboardNode.__init__()`:
```python
self.new_service_client = self.create_client(SrvType, '/service/name')
```

2. Add button and handler in dashboard:
```python
def call_new_service(self):
    request = SrvType.Request()
    # Set request fields
    future = self.ros_node.new_service_client.call_async(request)
    future.add_done_callback(lambda f: self.log_message(f.result().message))
```

### Styling

Modify button styles, colors, and layout in the `create_*_tab()` methods.

## Troubleshooting

### Dashboard Won't Launch
```bash
# Check PyQt5 installation
python3 -c "from PyQt5.QtWidgets import QApplication"

# Verify ROS2 package
ros2 pkg list | grep supervisor_module
```

### Services Not Available
```bash
# Check if system_controller_node is running
ros2 node list | grep system_controller

# List available services
ros2 service list | grep system
```

### No Topic Updates
```bash
# Verify topics are being published
ros2 topic list
ros2 topic hz /system/status

# Check if nodes are running
ros2 node list
```

## Safety Features

1. **Emergency Stop Button**:
   - Prominently placed with red color
   - Immediately halts all operations
   - Sends emergency stop command to all modules

2. **Visual Feedback**:
   - Color-coded status indicators
   - Real-time weight monitoring
   - Object detection confirmation

3. **Service Timeout Protection**:
   - 1-second timeout on all service calls
   - Error logging when services unavailable

## Future Enhancements

Potential additions:
- [ ] Camera feed visualization
- [ ] 3D robot pose visualization
- [ ] Trajectory preview
- [ ] Historical data plotting
- [ ] Configuration file editing
- [ ] Performance metrics dashboard
- [ ] Multi-robot support
- [ ] Data export functionality

## Technical Details

- **Language**: Python 3
- **GUI Framework**: PyQt5
- **ROS2**: Humble/Iron/Rolling
- **Threading**: QThread for ROS2 executor
- **Update Rate**: Real-time (on message receipt)

## Contact

For issues or questions about the dashboard:
1. Check system logs tab for error messages
2. Verify all prerequisite packages are installed
3. Ensure system_controller_node is running
4. Check ROS2 topic/service availability
