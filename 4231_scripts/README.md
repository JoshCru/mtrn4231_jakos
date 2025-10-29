# 4231 Scripts - UR5e Robot Control and Perception

This directory contains scripts and launch files for operating the UR5e robot with RealSense depth camera perception.

## 📁 Files

### Launch Scripts
| File | Description |
|------|-------------|
| `setupRealur5e.sh` | Original script - launches UR5e with MoveIt and RViz |
| `setupFakeur5e.sh` | Original script - launches simulated UR5e |
| `setupRealur5eWithCamera.sh` | **NEW** - Launches UR5e + RealSense camera + RViz |
| `camera.sh` | Simple v4l2 camera test script |

### ROS2 Launch Files
| File | Description |
|------|-------------|
| `camera_only.launch.py` | **RECOMMENDED** - RealSense camera + hand-eye calibration |
| `add_perception_to_rviz.launch.py` | Camera + perception module (requires build) |
| `ur5e_with_perception.launch.py` | All-in-one launch (complex, use manual approach instead) |

### Configuration Files
| File | Description |
|------|-------------|
| `ur5e_perception.rviz` | RViz config with PointCloud2 display |

### Documentation
| File | Description |
|------|-------------|
| `QUICKSTART.md` | ⭐ **START HERE** - Simple 3-step guide |
| `SETUP_INSTRUCTIONS.md` | Detailed setup and troubleshooting |
| `SYSTEM_OVERVIEW.md` | Technical architecture and data flow |
| `README_PERCEPTION.md` | Original perception setup guide |
| `README.md` | This file |

## 🚀 Quick Start

**See [QUICKSTART.md](QUICKSTART.md) for the easiest setup!**

### TL;DR
```bash
# 1. Kill existing connections
pkill -f ur_ros2_control_node

# 2. Launch everything
cd ~/Downloads/mtrn4231_jakos/4231_scripts
./setupRealur5eWithCamera.sh

# 3. In RViz: Add → By topic → /camera/depth/color/points → PointCloud2
```

## 📚 Documentation Guide

**First time?** → Read `QUICKSTART.md`

**Need help?** → Check `SETUP_INSTRUCTIONS.md`

**Want to understand how it works?** → See `SYSTEM_OVERVIEW.md`

**Building perception module?** → See `README_PERCEPTION.md`

## 🔧 What Gets Launched

### setupRealur5eWithCamera.sh launches:

1. **Terminal 1: Robot Driver**
   - Connects to UR5e at 192.168.0.100
   - Publishes joint states and transforms

2. **Terminal 2: MoveIt + RViz**
   - Motion planning interface
   - 3D visualization

3. **Terminal 3: RealSense Camera**
   - Intel D435 depth camera
   - Pointcloud generation
   - Hand-eye calibration transform

## 📊 Key Topics

After launching, these topics will be available:

```bash
# Camera topics
/camera/color/image_raw           # RGB image
/camera/depth/image_rect_raw      # Depth image
/camera/depth/color/points        # PointCloud2 (use this in RViz!)

# Robot topics
/joint_states                     # Robot joint positions
/tf                               # Robot transforms
/robot_description                # URDF model

# Planning topics
/display_planned_path             # Planned trajectory
```

## 🎯 Camera Transform

The camera position relative to robot base (from hand-eye calibration):

```
Position: X=1.277m, Y=0.018m, Z=0.674m
Rotation: Quaternion [-0.414, -0.019, 0.910, 0.004]
```

Source: `../4231_hand_eye_calibration/4_calib_camera_pose.launch`

## ⚡ Common Commands

### Check camera is working:
```bash
ros2 topic hz /camera/depth/color/points
```

### View camera transform:
```bash
ros2 run tf2_ros tf2_echo base_link camera_link
```

### List all topics:
```bash
ros2 topic list
```

### Kill robot driver:
```bash
pkill -f ur_ros2_control_node
```

## 🐛 Troubleshooting

| Problem | Solution |
|---------|----------|
| RTDE error | `pkill -f ur_ros2_control_node` then relaunch |
| No pointcloud | Check Fixed Frame = `base_link` in RViz |
| Camera not found | Check USB 3.0 connection, reconnect camera |
| perception_module error | Use `camera_only.launch.py` instead |

**Full troubleshooting guide in `SETUP_INSTRUCTIONS.md`**

## 🔬 Advanced Usage

### Building Perception Module (Optional)
```bash
cd ~/Downloads/mtrn4231_jakos/ros2_system
colcon build --symlink-install --packages-select perception_module
source install/setup.bash
```

Then you can use `add_perception_to_rviz.launch.py` for object detection.

### Manual Launch (3 terminals)

**Terminal 1:**
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur5e robot_ip:=192.168.0.100 \
  use_fake_hardware:=false launch_rviz:=false
```

**Terminal 2:**
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
  robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true
```

**Terminal 3:**
```bash
ros2 launch 4231_scripts/camera_only.launch.py
```

## 📝 Credits

- Hand-eye calibration from Lab 4
- RealSense ROS2 wrapper: https://github.com/IntelRealSense/realsense-ros
- UR ROS2 driver: https://github.com/UniversalRobots/Universal_Robots_ROS2_Driver

## 🆘 Need Help?

1. Check the docs in order:
   - `QUICKSTART.md` → Quick 3-step guide
   - `SETUP_INSTRUCTIONS.md` → Detailed setup
   - `SYSTEM_OVERVIEW.md` → Architecture

2. Common issues are documented in each guide

3. Verify basics:
   ```bash
   ros2 topic list        # See available topics
   ros2 node list         # See running nodes
   ros2 topic hz <topic>  # Check topic frequency
   ```
