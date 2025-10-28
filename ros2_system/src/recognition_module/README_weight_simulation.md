# Weight Simulation Script

This Python script provides a visual simulation of stainless steel weights that is compatible with the recognition module.

## Features

- **Visual Display**: Shows top-down and 3D views of weight objects
- **Real-time Animation**: Objects move slightly to simulate dynamic environment
- **ROS2 Integration**: Can publish point clouds for the recognition module
- **Configurable**: Support 1-4 objects with different weights

## Usage

### Visual Display Mode (Default)
```bash
cd /home/joshc/mtrn4231_jakos/ros2_system/src/recognition_module/scripts
python3 weight_simulation.py --mode display --objects 3
```

### ROS2 Publishing Mode
```bash
# Source ROS2 workspace first
source install/setup.bash

# Run publisher (sends point clouds to recognition module)
python3 weight_simulation.py --mode publish --objects 3 --rate 1.0
```

## Options

- `--mode`: `display` (visual only) or `publish` (ROS2 point cloud publishing)
- `--objects`: Number of objects to simulate (1-4, default: 3)
- `--rate`: Publishing rate in Hz for publish mode (default: 1.0)

## Object Specifications

The simulation creates objects that match the recognition module's expectations:

1. **50g weight**: 2.5cm × 2.5cm × 1cm at position (0.3, 0.15, 0.1)
2. **100g weight**: 3cm × 3cm × 1.5cm at position (0.35, -0.1, 0.12)
3. **200g weight**: 4cm × 4cm × 1.5cm at position (0.25, 0.0, 0.115)
4. **150g weight**: 3.5cm × 3.5cm × 1.2cm at position (0.4, 0.2, 0.11)

## Integration with Recognition Module

When using `--mode publish`:
- Publishes to `/camera/pointcloud` topic
- Uses `camera_link` frame_id
- Includes Gaussian noise (2mm stddev) for realistic simulation
- Compatible with recognition_node processing pipeline

## Testing the Recognition Module

1. **Start the recognition module**:
```bash
ros2 launch recognition_module recognition_with_mock_camera.launch.py
```

2. **Replace mock camera with simulation**:
```bash
# Stop the mock camera
ros2 lifecycle set mock_camera_node shutdown

# Start weight simulation publisher
python3 scripts/weight_simulation.py --mode publish --objects 3 --rate 1.0
```

3. **Monitor recognition output**:
```bash
ros2 topic echo /recognition/estimated_weights
```

## Dependencies

- Python 3
- matplotlib
- numpy
- ROS2 (for publish mode)
- sensor_msgs_py
- rclpy

Install dependencies:
```bash
pip install matplotlib numpy
# ROS2 dependencies should be installed with your ROS2 setup
```

## Visual Features

- **Top-down view**: Shows object positions with size representing height
- **3D view**: Full 3D visualization with object boundaries
- **Real-time updates**: 10 FPS animation with slight object movement
- **Color coding**: Silver objects on brown table surface
- **Labels**: Object names and weights displayed

## Simulation Accuracy

- Uses same dimensions as mock_camera_node
- Includes realistic point cloud noise
- Maintains correct object positions and orientations
- Generates dense point clouds (2mm resolution)