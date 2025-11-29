# Weight Detection Module

ROS2 Humble package for weight detection using UR5e joint torques.

## Usage
Visualise the estimated mass plots using plotjuggler.


```bash
colcon build --packages-select weight_detection_module
source install/setup.bash
ros2 run weight_detection_module weight_detector
```

- Publishes current estimated mass to: `/estimated_mass`
- mass_msg:
  -  Type: **Int32**
  -  Values: [0, 20, 50, 100, 200, 500]
  -  Unit: Grams