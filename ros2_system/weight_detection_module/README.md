# Weight Detection Module

ROS2 Humble package for weight detection using UR5e joint torques.

## Installation

```bash
cd ~/your_ros2_workspace/src
cp -r weight_detection_module .
cd ~/your_ros2_workspace
colcon build --packages-select weight_detection_module
source install/setup.bash
```

## Usage

```bash
ros2 run weight_detection_module weight_detector
```

## Topics

- Subscribes to: `/joint_states` (sensor_msgs/msg/JointState)
- Reads joint torques from the `effort` field

## Structure

- `weight_detector.py`: Main node that subscribes to joint torques
