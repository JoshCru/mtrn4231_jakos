# RGBD Camera Node - Quick Start Guide

## Push to Git

```bash
cd /home/mtrn/Documents/mtrn4231_jakos/ros2_system

# Check what files changed
git status

# Add the modified files
git add src/perception_module/src/rgbd_camera_node.cpp
git add src/perception_module/CMakeLists.txt
git add src/perception_module/launch/rgbd_camera_with_recording.launch.py
git add src/perception_module/scripts/test_camera_publisher.py
git add src/perception_module/README_RGBD_CAMERA.md
git add RGBD_CAMERA_QUICKSTART.md

# Commit changes
git commit -m "Add RGBD camera subscriber and recorder node

- Modified rgbd_camera_node to subscribe to RealSense topics instead of publishing dummy data
- Added automatic ROS bag recording functionality
- Created launch file for easy recording
- Added test publisher for testing without hardware
- Updated CMakeLists.txt with rosbag2 dependencies

🤖 Generated with [Claude Code](https://claude.com/claude-code)

Co-Authored-By: Claude <noreply@anthropic.com>"

# Push to remote
git push origin perception
```

## Testing on Linux System with RealSense Camera

### 1. Clone/Pull the Latest Code

```bash
cd ~/Documents/mtrn4231_jakos/ros2_system
git pull origin perception
```

### 2. Build the Package

```bash
cd ~/Documents/mtrn4231_jakos/ros2_system
colcon build --packages-select perception_module
source install/setup.bash
```

### 3. Test the RealSense Camera Driver

First, verify your RealSense camera is detected:

```bash
# Check if camera is connected (for RealSense)
rs-enumerate-devices

# Start the RealSense driver
ros2 run realsense2_camera realsense2_camera_node
```

In another terminal, check if topics are being published:

```bash
source install/setup.bash

# List all topics
ros2 topic list

# You should see topics like:
# /camera/color/image_raw
# /camera/depth/image_raw
# /camera/color/camera_info
# /camera/depth/camera_info

# Check the frame rate
ros2 topic hz /camera/color/image_raw

# View a single image (to verify data)
ros2 topic echo /camera/color/image_raw --once
```

### 4. Test the rgbd_camera_node (Without Recording)

In a new terminal:

```bash
source install/setup.bash

# Run the node without recording (just to verify it subscribes)
ros2 run perception_module rgbd_camera_node

# You should see output like:
# [INFO] [rgbd_camera_node]: RGBD Camera Node initialized
# [INFO] [rgbd_camera_node]: Subscribing to:
# [INFO] [rgbd_camera_node]:   Color: /camera/color/image_raw
# [INFO] [rgbd_camera_node]:   Depth: /camera/depth/image_raw
# [INFO] [rgbd_camera_node]: Recording: DISABLED
# [INFO] [rgbd_camera_node]: Received color frame 30 (640x480, rgb8)
# [INFO] [rgbd_camera_node]: Received depth frame 30 (640x480, 16UC1)
```

### 5. Test with Recording Enabled

Stop the previous node (Ctrl+C), then:

```bash
source install/setup.bash

# Option A: Using the launch file (easiest)
ros2 launch perception_module rgbd_camera_with_recording.launch.py

# Option B: Run directly with recording
ros2 run perception_module rgbd_camera_node --ros-args \
    -p enable_recording:=true \
    -p bag_path:=~/Documents/mtrn4231_jakos/test_bags/my_recording

# Let it record for 10-30 seconds, then stop with Ctrl+C
```

### 6. Verify the Recording

```bash
# Check the bag was created
ls -lh ~/Documents/mtrn4231_jakos/test_bags/

# View bag information
ros2 bag info ~/Documents/mtrn4231_jakos/test_bags/rgbd_camera_*

# You should see:
# - /camera/color/image_raw (sensor_msgs/msg/Image)
# - /camera/depth/image_raw (sensor_msgs/msg/Image)
# - /camera/color/camera_info (sensor_msgs/msg/CameraInfo)
# - /camera/depth/camera_info (sensor_msgs/msg/CameraInfo)
```

### 7. Play Back the Recording (Optional)

```bash
# Stop the camera driver first
# Then play back the bag
ros2 bag play ~/Documents/mtrn4231_jakos/test_bags/rgbd_camera_*

# In another terminal, verify topics are being published
ros2 topic list
ros2 topic hz /camera/color/image_raw
```

## Complete Test Sequence

Here's a complete test from start to finish:

```bash
# Terminal 1: Start RealSense camera
source install/setup.bash
ros2 run realsense2_camera realsense2_camera_node

# Terminal 2: Check topics are working
source install/setup.bash
ros2 topic list
ros2 topic hz /camera/color/image_raw
# (Ctrl+C after confirming it's publishing)

# Terminal 2: Start recorder
ros2 launch perception_module rgbd_camera_with_recording.launch.py
# Let it run for 30 seconds, then Ctrl+C

# Terminal 2: Check the recording
ros2 bag info ~/Documents/mtrn4231_jakos/test_bags/rgbd_camera_*
```

## Troubleshooting

### Camera not detected
```bash
# Check USB connection
lsusb | grep Intel

# For RealSense, install viewer to test hardware
realsense-viewer
```

### Topics not appearing
```bash
# Verify the camera driver is running
ros2 node list

# Check what topics it's publishing
ros2 node info /camera
```

### Build errors
```bash
# Clean build
rm -rf build/ install/ log/
colcon build --packages-select perception_module
```

### Wrong topic names
If your camera publishes to different topic names, you can specify them:

```bash
ros2 run perception_module rgbd_camera_node --ros-args \
    -p color_topic:=/your_camera/color/image \
    -p depth_topic:=/your_camera/depth/image \
    -p enable_recording:=true
```

## Alternative: Using v4l2_camera

If you mentioned you have v4l2_camera working, you might be using a different driver:

```bash
# Start v4l2 camera
ros2 run v4l2_camera v4l2_camera_node

# Check what topic it publishes to
ros2 topic list

# If it only publishes /image_raw (not /camera/color/image_raw), remap it:
ros2 run perception_module rgbd_camera_node --ros-args \
    -p color_topic:=/image_raw \
    -p enable_recording:=true
```

## Success Indicators

✅ Camera driver starts without errors
✅ `ros2 topic list` shows camera topics
✅ rgbd_camera_node logs "Received color frame" messages
✅ Bag file is created in test_bags directory
✅ `ros2 bag info` shows all 4 topics with frame counts

## Performance Notes

- Expect ~15-20 MB/second of bag data (for 640x480 @ 30Hz)
- Recording for 1 minute = ~1 GB of data
- RGB images are ~900 KB each
- Depth images are ~600 KB each
