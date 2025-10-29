# UR5e with RealSense Perception Setup

This directory contains scripts and launch files to run the UR5e robot with RealSense depth camera perception and motion planning in RViz.

## Files

### Launch Scripts

1. **setupRealRvizWithPerception.sh** - Bash script that launches all components in separate terminals
   - Launches UR robot driver
   - Launches RealSense camera
   - Launches hand-eye calibration transform
   - Launches perception module
   - Launches MoveIt with RViz

2. **ur5e_with_perception.launch.py** - ROS2 launch file (recommended)
   - Single launch file that starts all components with proper timing
   - More robust than the bash script
   - Better integrated with ROS2

### Configuration Files

3. **ur5e_perception.rviz** - RViz configuration file
   - Pre-configured to display:
     - Robot model
     - Motion planning interface
     - PointCloud from RealSense camera
     - TF frames
     - Object detection markers

## Usage

### Option 1: Using ROS2 Launch File (Recommended)

```bash
cd /Users/joshcru/Documents/Uni/MTRN4231/Git\ Folder/mtrn4231_jakos
ros2 launch 4231_scripts/ur5e_with_perception.launch.py
```

### Option 2: Using Bash Script

```bash
cd /Users/joshcru/Documents/Uni/MTRN4231/Git\ Folder/mtrn4231_jakos/4231_scripts
./setupRealRvizWithPerception.sh
```

### Using Custom RViz Configuration

If you want to use the custom RViz config with pointcloud visualization:

```bash
# First launch the robot and perception
ros2 launch 4231_scripts/ur5e_with_perception.launch.py

# Then in another terminal, launch RViz with the custom config
rviz2 -d /Users/joshcru/Documents/Uni/MTRN4231/Git\ Folder/mtrn4231_jakos/4231_scripts/ur5e_perception.rviz
```

## What Gets Launched

1. **Static Transform** - Hand-eye calibration from `base_link` to `camera_link`
   - Position: (1.277, 0.018, 0.674) meters
   - Orientation: Quaternion (-0.414, -0.019, 0.910, 0.004)

2. **RealSense Camera** - Intel RealSense depth camera
   - Depth resolution: 640x480 @ 30fps
   - PointCloud enabled and aligned to color
   - Publishes to topics:
     - `/camera/depth/color/points` - PointCloud2
     - `/camera/color/image_raw` - RGB image
     - `/camera/depth/image_rect_raw` - Depth image

3. **Perception Module** - Processes camera data
   - `rgbd_camera_node` - Subscribes to camera topics
   - `pointcloud_processor_node` - Processes pointcloud data
   - `opencv_processor_node` - Image processing for object detection

4. **UR Robot Driver** - Controls the UR5e robot
   - Connects to robot at 192.168.0.100
   - Publishes joint states and accepts motion commands

5. **MoveIt with RViz** - Motion planning interface
   - Interactive motion planning
   - Visualizes robot, workspace, and obstacles
   - Shows pointcloud from camera in the workspace

## RViz Display Topics

The RViz configuration displays the following:
- **Robot Model** - Shows the UR5e robot
- **MotionPlanning** - Interactive motion planning widget
- **PointCloud2** - Real-time pointcloud from RealSense (`/camera/depth/color/points`)
- **TF** - Coordinate frame transforms (shows camera position relative to robot)
- **ObjectMarkers** - Detected object markers from perception module (`/perception/object_markers`)

## Troubleshooting

### PointCloud not visible in RViz
1. Check that the RealSense camera is connected and publishing:
   ```bash
   ros2 topic echo /camera/depth/color/points --no-arr
   ```
2. Verify the camera transform is being published:
   ```bash
   ros2 run tf2_ros tf2_echo base_link camera_link
   ```
3. In RViz, check that:
   - Fixed Frame is set to `base_link`
   - PointCloud2 topic is set to `/camera/depth/color/points`
   - PointCloud2 display is enabled

### Camera not publishing
- Check USB connection to RealSense camera
- Verify RealSense ROS2 package is installed:
  ```bash
  ros2 pkg list | grep realsense
  ```

### Motion planning not working
- Ensure robot driver is connected to the real robot
- Check robot IP address is correct (192.168.0.100)
- Verify robot is in remote control mode

## Notes

- The launch file includes delays between starting different components to ensure proper initialization
- The hand-eye calibration values come from `4231_hand_eye_calibration/4_calib_camera_pose.launch`
- All components use the `base_link` as the common reference frame
