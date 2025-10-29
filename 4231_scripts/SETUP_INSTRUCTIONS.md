# Setup Instructions for UR5e with RealSense Perception

## Quick Start (Recommended)

### Option 1: Sequential Launch (Easiest)

1. **First, make sure no other robot drivers are running:**
```bash
pkill -f ur_ros2_control_node
pkill -f dashboard_client
```

2. **Build the perception module (one-time setup):**
```bash
cd ~/Downloads/mtrn4231_jakos/ros2_system
colcon build --symlink-install --packages-select perception_module
source install/setup.bash
```

3. **Add to your .bashrc for automatic sourcing:**
```bash
echo "source ~/Downloads/mtrn4231_jakos/ros2_system/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

4. **Launch everything using the new script:**
```bash
cd ~/Downloads/mtrn4231_jakos/4231_scripts
./setupRealur5eWithCamera.sh
```

This will launch:
- UR robot driver (terminal 1)
- MoveIt with RViz (terminal 2)
- RealSense camera + hand-eye calibration (terminal 3)

### Option 2: Manual Launch (More Control)

**Terminal 1 - Robot Driver:**
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 use_fake_hardware:=false launch_rviz:=false
```

**Terminal 2 - MoveIt with RViz (wait 10 seconds):**
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true
```

**Terminal 3 - Camera & Perception (wait 5 more seconds):**
```bash
cd ~/Downloads/mtrn4231_jakos
ros2 launch 4231_scripts/add_perception_to_rviz.launch.py
```

## Viewing PointCloud in RViz

Once everything is running, add the pointcloud to RViz:

1. In RViz, click the **"Add"** button (bottom left)
2. Go to **"By topic"** tab
3. Find `/camera/depth/color/points` and select **PointCloud2**
4. Click **OK**

**PointCloud Display Settings:**
- Fixed Frame: `base_link`
- Topic: `/camera/depth/color/points`
- Size (m): `0.01`
- Style: `Flat Squares` or `Points`
- Color Transformer: `RGB8` (for colored points) or `AxisColor`

## Verifying Everything Works

### Check Camera is Publishing:
```bash
# See if pointcloud is being published
ros2 topic hz /camera/depth/color/points

# Echo a few messages
ros2 topic echo /camera/depth/color/points --no-arr
```

### Check TF Transform:
```bash
# Verify camera is positioned correctly relative to robot base
ros2 run tf2_ros tf2_echo base_link camera_link
```

Expected output:
```
Translation: [1.277, 0.018, 0.674]
Rotation: in Quaternion [-0.414, -0.019, 0.910, 0.004]
```

### List Available Topics:
```bash
ros2 topic list
```

Should include:
- `/camera/color/image_raw`
- `/camera/depth/image_rect_raw`
- `/camera/depth/color/points`
- `/camera/color/camera_info`
- `/camera/depth/camera_info`

## Troubleshooting

### Error: "RTDE client already connected"
**Problem:** Another program is controlling the robot.

**Solution:**
```bash
pkill -f ur_ros2_control_node
pkill -f dashboard_client
# Wait 5 seconds, then relaunch
```

### Error: "perception_module not found"
**Problem:** Workspace not built or not sourced.

**Solution:**
```bash
cd ~/Downloads/mtrn4231_jakos/ros2_system
colcon build --symlink-install
source install/setup.bash
```

### PointCloud not visible in RViz
**Checks:**
1. Is camera publishing? `ros2 topic hz /camera/depth/color/points`
2. Is Fixed Frame set to `base_link`?
3. Is PointCloud2 enabled (checkbox)?
4. Try increasing Size (m) to 0.02
5. Check if camera can see anything (not pointing at blank wall)

### Camera shows "No device connected"
**Checks:**
1. USB cable connected?
2. Try different USB port (must be USB 3.0)
3. Check permissions: `ls -la /dev/video*`
4. Reconnect camera and wait 5 seconds

### RViz crashes or freezes
**Solution:**
- Reduce pointcloud size in camera settings
- Use lower resolution: `640x480x30` instead of `1280x720x30`
- Disable aligned depth if not needed

## Camera Hand-Eye Calibration Values

The static transform from `base_link` to `camera_link`:
- **Translation (XYZ):** [1.277, 0.018, 0.674] meters
- **Rotation (Quaternion):** [-0.414, -0.019, 0.910, 0.004]

These values come from: `4231_hand_eye_calibration/4_calib_camera_pose.launch`

## File Locations

- **Bash script:** `4231_scripts/setupRealur5eWithCamera.sh`
- **Perception launch:** `4231_scripts/add_perception_to_rviz.launch.py`
- **RViz config:** `4231_scripts/ur5e_perception.rviz` (optional)
- **Original calibration:** `4231_hand_eye_calibration/4_calib_camera_pose.launch`

## Next Steps

Once you can see the pointcloud in RViz:
1. Position robot to view workspace
2. Place objects in camera view
3. Verify objects appear in pointcloud
4. Use for motion planning obstacle avoidance
