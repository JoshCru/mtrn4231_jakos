# RGBD Camera Node

## Overview

The `rgbd_camera_node` is a subscriber-based node that listens to Intel RealSense (or similar) RGBD camera topics and optionally records them to a ROS bag file.

## Features

- Subscribes to RGB image, depth image, and camera info topics
- Optional automatic ROS bag recording
- Configurable topic names
- Frame counting and logging
- Compatible with Intel RealSense cameras via realsense2_camera or v4l2_camera drivers

## Usage

### Step 1: Start your RealSense camera driver

For Intel RealSense cameras:
```bash
# If using realsense2_camera package
ros2 run realsense2_camera realsense2_camera_node

# OR if using v4l2_camera
ros2 run v4l2_camera v4l2_camera_node
```

### Step 2: Start the rgbd_camera_node

**Option A: Using the launch file (recommended)**
```bash
source install/setup.bash
ros2 launch perception_module rgbd_camera_with_recording.launch.py
```

**Option B: Run directly without recording**
```bash
source install/setup.bash
ros2 run perception_module rgbd_camera_node
```

**Option C: Run with custom parameters**
```bash
source install/setup.bash
ros2 run perception_module rgbd_camera_node --ros-args \
    -p enable_recording:=true \
    -p bag_path:=/path/to/save/bag \
    -p color_topic:=/camera/color/image_raw \
    -p depth_topic:=/camera/depth/image_raw
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `color_topic` | string | `/camera/color/image_raw` | Topic name for RGB/color images |
| `depth_topic` | string | `/camera/depth/image_raw` | Topic name for depth images |
| `color_info_topic` | string | `/camera/color/camera_info` | Topic name for color camera info |
| `depth_info_topic` | string | `/camera/depth/camera_info` | Topic name for depth camera info |
| `enable_recording` | bool | `false` | Enable ROS bag recording |
| `bag_path` | string | `""` | Path to save bag (auto-generated if empty) |

## Launch File Arguments

When using the launch file, you can override parameters:

```bash
ros2 launch perception_module rgbd_camera_with_recording.launch.py \
    enable_recording:=true \
    bag_path:=/custom/path/to/bag \
    color_topic:=/custom/camera/color/image_raw
```

## Recorded Topics

When recording is enabled, the node saves the following topics to a bag file:

- RGB/color images (`sensor_msgs/msg/Image`)
- Depth images (`sensor_msgs/msg/Image`)
- Color camera info (`sensor_msgs/msg/CameraInfo`)
- Depth camera info (`sensor_msgs/msg/CameraInfo`)

## Example Workflow

### Complete workflow with RealSense camera:

```bash
# Terminal 1: Start RealSense driver
ros2 run realsense2_camera realsense2_camera_node

# Terminal 2: Start recorder
source install/setup.bash
ros2 launch perception_module rgbd_camera_with_recording.launch.py

# Stop recording with Ctrl+C when done
# Bag will be saved to ~/Documents/mtrn4231_jakos/test_bags/rgbd_camera_<timestamp>
```

### View recorded data:

```bash
# Check bag info
ros2 bag info ~/Documents/mtrn4231_jakos/test_bags/rgbd_camera_<timestamp>

# Play back the bag
ros2 bag play ~/Documents/mtrn4231_jakos/test_bags/rgbd_camera_<timestamp>
```

## Testing Without a Camera

For testing purposes, a test publisher script is provided:

```bash
# Terminal 1: Start test publisher
python3 src/perception_module/scripts/test_camera_publisher.py

# Terminal 2: Start recorder
source install/setup.bash
ros2 run perception_module rgbd_camera_node --ros-args -p enable_recording:=true
```

## Node Information

**Package:** `perception_module`
**Executable:** `rgbd_camera_node`
**File:** `src/perception_module/src/rgbd_camera_node.cpp`

## Troubleshooting

### No topics detected
- Make sure your RealSense driver is running
- Check available topics: `ros2 topic list`
- Verify topic names match the configured parameters

### Recording not working
- Check that `enable_recording` parameter is set to `true`
- Verify you have write permissions to the bag path
- Check disk space is available

### Frame rate issues
- RealSense cameras typically publish at 30 Hz
- Check actual rate: `ros2 topic hz /camera/color/image_raw`
- Large bag files are normal for image data (expect ~15-20 MB/second)
