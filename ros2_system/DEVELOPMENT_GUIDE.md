# Development & Testing Guide

## Development Roadmap

Follow this order for incremental development and testing:

```
Phase 1: Foundation (No Hardware Required)
Phase 2: Gripper & Weight Sensing
Phase 3: Camera & Vision
Phase 4: Robot Integration
Phase 5: MoveIt2 & Full Integration
```

---

## Phase 1: Foundation Testing (Day 1)

**Goal**: Verify the ROS2 system builds and basic communication works.

### 1.1 Build the System
```bash
cd ~/mtrn4231_jakos/ros2_system
./build_all.sh
source install/setup.bash
```

**Expected**: All packages build successfully.

### 1.2 Test Individual Modules

#### Test Supervisor Module
```bash
# Terminal 1: Launch supervisor
ros2 launch supervisor_module supervisor.launch.py

# Terminal 2: Check topics
ros2 topic list | grep system

# Terminal 3: Monitor target areas
ros2 topic echo /system/target_areas

# Terminal 4: Test system control
ros2 service call /system/start sort_interfaces/srv/SystemCommand "{command: 'start'}"
```

**What to verify:**
- ✓ Node starts without errors
- ✓ Target areas are published (3 areas by default)
- ✓ System status changes to "running"
- ✓ Services respond correctly

#### Test Message Flow Without Hardware
```bash
# Terminal 1: Launch all modules
ros2 launch launch/full_system.launch.py

# Terminal 2: View computational graph
rqt_graph

# Terminal 3: Monitor topic list
ros2 topic list

# Terminal 4: Check node status
ros2 node list
```

**What to verify:**
- ✓ All 13 nodes start
- ✓ No critical errors (warnings about missing hardware are OK)
- ✓ Topic connections visible in rqt_graph

### 1.3 Test Simulated Data Flow

Create test publishers to simulate data:

```bash
# Simulate gripper force feedback
ros2 topic pub /motion_control/force_feedback sort_interfaces/msg/ForceFeedback "{
  header: {stamp: {sec: 0, nanosec: 0}, frame_id: 'gripper'},
  gripper_force: 1.5,
  measured_weight: 150.0,
  gripper_position: 0.8,
  object_detected: true,
  raw_sensor_value: 512.0
}" -r 10

# Monitor that verification node receives it
ros2 topic echo /planning/verification_result
```

**What to verify:**
- ✓ Data flows through the system
- ✓ Nodes process and republish data correctly

---

## Phase 2: Gripper & Weight Sensing (Days 2-3)

**Goal**: Get the Arduino gripper working with weight measurement.

### 2.1 Arduino Setup

1. **Upload the sketch:**
```bash
# Open Arduino IDE
arduino ~/mtrn4231_jakos/ros2_system/arduino_sketches/gripper_controller.ino

# Upload to Arduino
```

2. **Test serial communication:**
```bash
# Find Arduino port
ls /dev/ttyACM*

# Test with screen or minicom
screen /dev/ttyACM0 115200

# Type: G90 (should move servo to 90 degrees)
# You should see: W<weight> messages
```

### 2.2 Implement Serial Communication in Gripper Node

**File to edit**: `src/motion_control_module/src/gripper_controller_node.cpp`

**Key TODOs** (around these lines):
- Line 127: `// TODO: Open serial port`
- Line 229: `// TODO: Send command to Arduino via serial`
- Line 244: `// TODO: Read from Arduino via serial`

**Implementation steps:**

1. Add serial library to CMakeLists.txt:
```cmake
# In src/motion_control_module/CMakeLists.txt
find_package(serial REQUIRED)

ament_target_dependencies(gripper_controller_node
  # ... existing dependencies ...
  serial
)
```

2. Add to package.xml:
```xml
<depend>serial</depend>
```

3. Implement serial in the node (example):
```cpp
#include <serial/serial.h>

// In on_configure():
try {
    serial_ = std::make_unique<serial::Serial>(serial_port_, baud_rate_,
                                               serial::Timeout::simpleTimeout(1000));
    if (serial_->isOpen()) {
        RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
    }
} catch (serial::IOException& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
    return LifecycleCallbackReturn::FAILURE;
}

// In send_gripper_command():
if (serial_ && serial_->isOpen()) {
    std::string command = "G" + std::to_string(angle) + "\n";
    serial_->write(command);
}

// In read_force_sensor():
if (serial_ && serial_->isOpen() && serial_->available()) {
    std::string response = serial_->readline();
    if (response[0] == 'W') {
        return std::stof(response.substr(1));
    }
}
```

### 2.3 Test Gripper Control

```bash
# Terminal 1: Launch gripper controller
ros2 launch motion_control_module motion_control.launch.py

# Terminal 2: Configure and activate lifecycle
ros2 lifecycle set /gripper_controller_node configure
ros2 lifecycle set /gripper_controller_node activate

# Terminal 3: Test gripper commands
# Open gripper
ros2 topic pub /motion_control/gripper_command std_msgs/msg/Float32 "data: 0.0" --once

# Close gripper
ros2 topic pub /motion_control/gripper_command std_msgs/msg/Float32 "data: 1.0" --once

# Partial close
ros2 topic pub /motion_control/gripper_command std_msgs/msg/Float32 "data: 0.5" --once

# Terminal 4: Monitor force feedback
ros2 topic echo /motion_control/force_feedback
```

**What to verify:**
- ✓ Servo moves to commanded positions
- ✓ Force feedback publishes at ~20 Hz
- ✓ Weight readings change when you place objects in gripper
- ✓ Object detection threshold works

### 2.4 Calibrate Weight Sensor

```bash
# With gripper empty, tare the sensor
ros2 service call /motion_control/calibrate_gripper sort_interfaces/srv/CalibrateGripper "{
  calibrate_position: false,
  calibrate_force: false,
  tare_weight_sensor: true
}"

# Place known weights and verify readings
ros2 topic echo /motion_control/force_feedback --field measured_weight

# Adjust calibration_factor in config until readings are accurate
```

**Tuning tips:**
- Start with objects of known weight (50g, 100g, 200g)
- Adjust `weight_calibration_factor` in `motion_control.yaml`
- Set appropriate `object_detection_threshold`

---

## Phase 3: Camera & Vision (Days 4-6)

**Goal**: Get camera working and detect objects.

### 3.1 Camera Driver Selection

**Option A: RealSense Camera**

1. Install RealSense ROS2 wrapper:
```bash
sudo apt install ros-humble-realsense2-camera
```

2. Modify `rgbd_camera_node.cpp` to use RealSense wrapper OR simply use the wrapper:
```bash
# Instead of custom node, use RealSense wrapper directly
ros2 launch realsense2_camera rs_launch.py
```

3. Remap topics in launch file:
```python
# In perception.launch.py
realsense_node = Node(
    package='realsense2_camera',
    executable='realsense2_camera_node',
    parameters=[{
        'enable_color': True,
        'enable_depth': True,
        'enable_pointcloud': True,
    }],
    remappings=[
        ('/camera/camera/color/image_raw', '/camera/color/image_raw'),
        ('/camera/camera/depth/image_raw', '/camera/depth/image_raw'),
        ('/camera/camera/depth/color/points', '/camera/pointcloud'),
    ]
)
```

**Option B: Generic USB Camera**

Use `usb_cam` or `v4l2_camera`:
```bash
sudo apt install ros-humble-usb-cam
ros2 run usb_cam usb_cam_node_exe
```

### 3.2 Test Camera

```bash
# Terminal 1: Launch camera
ros2 launch perception_module perception.launch.py

# Terminal 2: Check camera topics
ros2 topic list | grep camera

# Terminal 3: View images
ros2 run rqt_image_view rqt_image_view

# Select /camera/color/image_raw from dropdown
```

**What to verify:**
- ✓ Camera publishes images at configured rate
- ✓ Point cloud is generated
- ✓ Image quality is acceptable

### 3.3 Implement Object Detection

**File to edit**: `src/perception_module/src/opencv_processor_node.cpp`

**Key TODO** (line 67): `// TODO: Implement actual object detection algorithm`

**Quick Start - Contour Detection** (already implemented as placeholder):
- The default contour detection may work for simple objects
- Tune parameters in `perception.yaml`:
  - `min_object_area`: 1000
  - `max_object_area`: 50000

**Advanced - Deep Learning (YOLO/SSD)**:

1. Add YOLO/SSD support:
```bash
# Install darknet_ros or use OpenCV DNN module
sudo apt install ros-humble-darknet-ros
```

2. Or integrate OpenCV DNN in the node:
```cpp
// Load YOLO model
cv::dnn::Net net = cv::dnn::readNetFromDarknet("yolov3.cfg", "yolov3.weights");

// In detect_objects():
cv::Mat blob = cv::dnn::blobFromImage(image, 1/255.0, cv::Size(416, 416));
net.setInput(blob);
std::vector<cv::Mat> outputs;
net.forward(outputs, net.getUnconnectedOutLayersNames());
// Parse detections...
```

### 3.4 Test Object Detection

```bash
# Terminal 1: Launch perception
ros2 launch perception_module perception.launch.py

# Terminal 2: Monitor detected objects
ros2 topic echo /perception/detected_objects

# Terminal 3: View debug image with bounding boxes
ros2 run rqt_image_view rqt_image_view
# Select /perception/debug_image

# Place objects in camera view and verify detection
```

**What to verify:**
- ✓ Objects are detected with bounding boxes
- ✓ Detection IDs are assigned
- ✓ Confidence values are reasonable
- ✓ Debug visualization shows correct boxes

### 3.5 Implement 3D Position Extraction

**File to edit**: `src/perception_module/src/pointcloud_processor_node.cpp`

**Key TODO** (line 95): `// TODO: Implement actual 3D segmentation`

**Option 1: PCL Integration** (recommended):

1. Add PCL to CMakeLists.txt:
```cmake
find_package(PCL REQUIRED)
include_directories(${PCL_INCLUDE_DIRS})
target_link_libraries(pointcloud_processor_node ${PCL_LIBRARIES})
```

2. Use PCL for segmentation:
```cpp
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl_conversions/pcl_conversions.h>

// Convert ROS msg to PCL
pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
pcl::fromROSMsg(*latest_pointcloud_, *cloud);

// Passthrough filter (crop to workspace)
pcl::PassThrough<pcl::PointXYZ> pass;
pass.setInputCloud(cloud);
pass.setFilterFieldName("z");
pass.setFilterLimits(workspace_limits_.min_z, workspace_limits_.max_z);
pass.filter(*cloud);

// Euclidean clustering
pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
tree->setInputCloud(cloud);

std::vector<pcl::PointIndices> cluster_indices;
pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
ec.setClusterTolerance(cluster_tolerance_);
ec.setMinClusterSize(min_cluster_size_);
ec.setMaxClusterSize(max_cluster_size_);
ec.setSearchMethod(tree);
ec.setInputCloud(cloud);
ec.extract(cluster_indices);

// Extract centroids
for (const auto& indices : cluster_indices) {
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cloud, indices, centroid);
    // Convert to Pose and publish
}
```

### 3.6 Test Full Perception Pipeline

```bash
# Terminal 1: Launch perception
ros2 launch perception_module perception.launch.py

# Terminal 2: Launch recognition
ros2 launch recognition_module recognition.launch.py

# Terminal 3: Monitor weight estimates
ros2 topic echo /recognition/estimated_weights

# Terminal 4: Launch RViz
ros2 run rviz2 rviz2

# In RViz, add:
# - PointCloud2 -> /camera/pointcloud
# - MarkerArray -> /perception/object_markers
# - Image -> /camera/color/image_raw
```

**What to verify:**
- ✓ Objects detected in 2D (bounding boxes)
- ✓ 3D positions extracted from point cloud
- ✓ Weight estimates generated
- ✓ Visualization shows all detected objects

---

## Phase 4: Robot Integration (Days 7-9)

**Goal**: Connect to UR5e and execute trajectories.

### 4.1 UR5e Driver Setup

**Option A: Use Universal Robots ROS2 Driver** (recommended):

```bash
# Install UR driver
sudo apt install ros-humble-ur-robot-driver

# Launch UR driver
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.100 \
  launch_rviz:=false
```

**Option B: Integrate into robot_driver_node**:

File: `src/control_module/src/robot_driver_node.cpp`
Line 141: `// TODO: Establish connection to robot`

### 4.2 Test Robot Connection

```bash
# Terminal 1: Launch UR driver
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e \
  robot_ip:=192.168.1.100

# Terminal 2: Check joint states
ros2 topic echo /joint_states

# Terminal 3: Test simple motion
ros2 topic pub /scaled_joint_trajectory_controller/joint_trajectory \
  trajectory_msgs/msg/JointTrajectory "{...}"
```

**What to verify:**
- ✓ Connection established
- ✓ Joint states publishing at 50+ Hz
- ✓ Can send test trajectories
- ✓ Emergency stop works

### 4.3 Test Pick Operation (Manual)

```bash
# Terminal 1: Launch control module
ros2 launch control_module control.launch.py

# Terminal 2: Launch gripper
ros2 launch motion_control_module motion_control.launch.py

# Activate lifecycle nodes
ros2 lifecycle set /robot_driver_node configure
ros2 lifecycle set /robot_driver_node activate
ros2 lifecycle set /gripper_controller_node configure
ros2 lifecycle set /gripper_controller_node activate

# Terminal 3: Send pick action goal
ros2 action send_goal /control/pick_object sort_interfaces/action/PickObject "{
  object_id: 1,
  target_pose: {
    position: {x: 0.4, y: 0.0, z: 0.1},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  },
  approach_distance: 0.1
}"
```

**What to verify:**
- ✓ Pick sequence executes (approach, grasp, lift, measure)
- ✓ Gripper opens/closes on command
- ✓ Weight is measured and published
- ✓ Action feedback shows state progression

---

## Phase 5: MoveIt2 & Full Integration (Days 10-14)

**Goal**: Add motion planning and complete the system.

### 5.1 MoveIt2 Setup

1. **Create/obtain URDF**:
```bash
# If you have UR description
sudo apt install ros-humble-ur-description

# Or create custom URDF with gripper attached
```

2. **Generate MoveIt config**:
```bash
# Use MoveIt Setup Assistant
ros2 launch moveit_setup_assistant setup_assistant.launch.py

# Configure:
# - Load UR5e URDF
# - Add gripper as end effector
# - Generate planning group "ur_manipulator"
# - Set end effector link: "tool0" or custom gripper link
# - Generate config package
```

3. **Launch MoveIt**:
```bash
ros2 launch <your_moveit_config> move_group.launch.py
```

### 5.2 Implement MoveIt2 Interface

**File**: `src/planning_module/src/moveit2_interface_node.cpp`
**Lines**: 80-140 (uncomment MoveIt2 code)

Key changes:
```cpp
// Uncomment in constructor:
move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
    shared_from_this(), planning_group_);

// Uncomment in execute_planning():
move_group_->setPlanningTime(goal->planning_time);
move_group_->setPoseTarget(goal->target_pose);
moveit::planning_interface::MoveGroupInterface::Plan plan;
bool success = (move_group_->plan(plan) == moveit::core::MoveItErrorCode::SUCCESS);
```

### 5.3 End-to-End System Test

```bash
# Launch everything
ros2 launch launch/full_system.launch.py

# In separate terminal, launch MoveIt
ros2 launch <your_moveit_config> move_group.launch.py

# Start the system
ros2 service call /system/start sort_interfaces/srv/SystemCommand "{command: 'start'}"

# Place objects in camera view and watch the system:
# 1. Detect objects
# 2. Estimate weights
# 3. Make sort decisions
# 4. Plan trajectories
# 5. Execute pick and place
# 6. Verify weights
```

---

## Testing Strategies

### Unit Testing Individual Nodes

Create test publishers/subscribers:

```bash
# Test gripper force feedback processing
ros2 topic pub /motion_control/force_feedback sort_interfaces/msg/ForceFeedback "{...}"

# Test weight estimation
ros2 topic pub /perception/object_positions geometry_msgs/msg/PoseArray "{...}"

# Monitor outputs
ros2 topic echo /recognition/estimated_weights
```

### Integration Testing

1. **Perception → Recognition**:
```bash
ros2 launch perception_module perception.launch.py
ros2 launch recognition_module recognition.launch.py
# Place objects, verify weight estimates
```

2. **Planning → Control**:
```bash
ros2 launch planning_module planning.launch.py
ros2 launch control_module control.launch.py
# Send sort decision, verify pick execution
```

### Debugging Tools

```bash
# View all topics and their rates
ros2 topic list -v

# Monitor specific topic frequency
ros2 topic hz /camera/color/image_raw

# Record data for playback
ros2 bag record -a -o test_run_1

# Playback recorded data
ros2 bag play test_run_1

# View node graph
rqt_graph

# Monitor all node output
ros2 run rqt_console rqt_console
```

### Common Issues & Solutions

| Issue | Solution |
|-------|----------|
| Serial port permission denied | `sudo usermod -a -G dialout $USER` then logout/login |
| Camera not found | Check `lsusb`, verify USB3 connection |
| MoveIt planning fails | Check workspace limits, collision objects |
| Weight readings noisy | Increase filter_window_size in config |
| Robot doesn't move | Verify lifecycle state, check emergency stop |
| Build errors | Run `rosdep install` again, check dependencies |

---

## Performance Benchmarks

Target performance metrics:

- Camera frame rate: **30 Hz**
- Object detection: **10 Hz** (after processing)
- Weight estimation: **10 Hz**
- Force feedback: **20 Hz**
- Planning time: **< 5 seconds**
- Pick operation: **15-20 seconds**
- Place operation: **10-15 seconds**
- Complete sort cycle: **< 1 minute**

---

## Safety Testing Checklist

Before running with real hardware:

- [ ] Emergency stop button accessible
- [ ] Workspace clear of obstacles
- [ ] Robot speed limited (velocity_scaling < 0.5)
- [ ] Collision checking enabled
- [ ] Gripper force limits configured
- [ ] Camera view covers entire workspace
- [ ] Weight sensor calibrated
- [ ] All lifecycle nodes properly configured

---

## Next Steps After Basic System Works

1. **Tune Parameters**: Adjust all configs for your specific setup
2. **Add Error Recovery**: Handle failed picks, missed objects
3. **Optimize Performance**: Reduce cycle time, improve detection
4. **Add Logging**: Record all operations for analysis
5. **Machine Learning**: Train better weight estimation models
6. **Multi-Object**: Handle multiple objects in one cycle

---

## Getting Help

If you encounter issues:

1. Check logs: `ros2 node info /node_name`
2. Search for TODO comments: `grep -rn "TODO" src/`
3. Test incrementally: Don't try everything at once
4. Use simulation first: Test without real hardware
5. Record data: Use ros2 bag for debugging

Good luck with your development! 🚀
