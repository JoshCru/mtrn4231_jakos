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

*All commands can be either lowercase or uppercase*

*Enter 'F' to begin code on bootup*

**Commands:**
- 'W' - open gripper
- 'S' - close gripper
- 'E \<angle\>' - edit close (grip) angle
