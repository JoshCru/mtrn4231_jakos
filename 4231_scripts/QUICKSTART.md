# Quick Start: UR5e with RealSense Camera in RViz

## Easy 3-Step Setup

### Step 1: Kill any existing robot connections
```bash
pkill -f ur_ros2_control_node
pkill -f dashboard_client
```

### Step 2: Launch everything
```bash
cd ~/Downloads/mtrn4231_jakos/4231_scripts
./setupRealur5eWithCamera.sh
```

This will open 3 terminals:
1. **DriverServer** - Robot driver (wait ~10 sec)
2. **MoveitServer** - RViz with motion planning (wait ~5 sec)
3. **Camera** - RealSense camera with pointcloud

### Step 3: Add PointCloud to RViz

In the RViz window:
1. Click **"Add"** button (bottom left panel)
2. Go to **"By topic"** tab
3. Expand `/camera/depth/color/points`
4. Select **PointCloud2**
5. Click **OK**

**Adjust settings:**
- In PointCloud2 panel, change:
  - **Size (m)**: `0.01` or `0.02`
  - **Color Transformer**: `RGB8` (for colored) or `AxisColor`
  - **Style**: `Flat Squares`

---

## Verify It's Working

### Check camera is publishing:
```bash
ros2 topic list | grep camera
```

Should show:
- `/camera/color/image_raw`
- `/camera/depth/image_rect_raw`
- `/camera/depth/color/points`

### Check pointcloud data:
```bash
ros2 topic hz /camera/depth/color/points
```

Should show: `~30 Hz`

### Check camera transform:
```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

Should show translation: `[1.277, 0.018, 0.674]`

---

## Manual Launch (Alternative)

If the script doesn't work, launch manually in 3 separate terminals:

**Terminal 1:**
```bash
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 use_fake_hardware:=false launch_rviz:=false
```

Wait 10 seconds...

**Terminal 2:**
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true
```

Wait 5 seconds...

**Terminal 3:**
```bash
cd ~/Downloads/mtrn4231_jakos
ros2 launch 4231_scripts/camera_only.launch.py
```

---

## Troubleshooting

### ❌ No pointcloud in RViz
- Check Fixed Frame is `base_link`
- Check PointCloud2 is enabled (checkbox)
- Try larger Size (m): `0.02`
- Ensure camera can see something (not pointing at wall)

### ❌ RTDE error
```bash
pkill -f ur_ros2_control
# Wait 5 seconds, then relaunch
```

### ❌ Camera not found
- Check USB 3.0 connection
- Reconnect camera, wait 5 seconds
- Check: `ls /dev/video*`

### ❌ RViz shows nothing
- Ensure robot driver finished starting before launching camera
- Check: `ros2 topic list` shows robot topics

---

## What the Camera Transform Does

The `camera_link_broadcaster` publishes a static transform that tells ROS where the camera is positioned relative to the robot base:

- **Position**: 1.28m forward, 0.02m left, 0.67m up from robot base
- **Orientation**: Rotated to match calibration

This allows RViz to correctly display the pointcloud in the robot's coordinate frame.

---

## Files Used

- `camera_only.launch.py` - Launches camera + transform (no perception_module needed)
- `setupRealur5eWithCamera.sh` - Automated 3-terminal launch
- Calibration from: `4231_hand_eye_calibration/4_calib_camera_pose.launch`
