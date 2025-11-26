# Cartesian Movement System - Usage Guide

## Overview

The new `CartesianRobot` class provides a clean, safe interface for Cartesian movements, following the pattern from `ur_robot_driver/example.py`.

### Key Features:
- **Safety boundary checking** - Prevents movements outside safe workspace
- **Clean API** - Similar to UR robot driver example
- **Synchronous operations** - Easy to understand and debug
- **MoveIt integration** - Uses MoveIt for path planning with collision avoidance
- **Works with fake and real hardware** - Just change one parameter

## Safety Boundaries

All movements are checked against these boundaries (relative to `base_link`):

```
x >= -300mm
y >= -300mm
0mm <= z <= 655mm
```

Movements outside these boundaries will be **rejected** before execution.

## Quick Start

### 1. Start the Fake UR5e System

```bash
./setupFakeur5e.sh
```

Wait for all terminals to load and MoveIt to initialize.

### 2. Run Test Movements

```bash
# Test with pre-defined safe positions
python3 test_cartesian_robot.py
```

### 3. Move to Custom Position

```bash
# Using shell script
./move_cartesian.sh 100 400 300 -1.571 0.0 0.0

# Or using Python directly
python3 move_cartesian_simple.py 100 400 300 -1.571 0.0 0.0
```

## Using in Your Own Scripts

### Basic Usage

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cartesian_robot import CartesianRobot

rclpy.init()
node = Node("my_robot_script")

# Initialize robot (use_fake_hardware=True for simulation)
robot = CartesianRobot(node, use_fake_hardware=True)

# Move to position (x, y, z in mm, rx, ry, rz in radians)
robot.move_to_cartesian(
    x=100,      # 100mm
    y=400,      # 400mm
    z=300,      # 300mm
    rx=-1.571,  # -π/2 (gripper pointing down)
    ry=0.0,
    rz=0.0
)

node.destroy_node()
rclpy.shutdown()
```

### Multiple Movements

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cartesian_robot import CartesianRobot

rclpy.init()
node = Node("multi_move_example")
robot = CartesianRobot(node, use_fake_hardware=True)

# Define waypoints
waypoints = [
    (100, 400, 300, -1.571, 0.0, 0.0),  # Position 1
    (200, 300, 400, -1.571, 0.0, 0.0),  # Position 2
    (150, 350, 350, -1.571, 0.0, 0.0),  # Position 3
]

# Execute movements
for x, y, z, rx, ry, rz in waypoints:
    success = robot.move_to_cartesian(x, y, z, rx, ry, rz)
    if not success:
        node.get_logger().error("Movement failed!")
        break

node.destroy_node()
rclpy.shutdown()
```

### Error Handling

```python
try:
    robot.move_to_cartesian(100, 400, 300, -1.571, 0.0, 0.0)
except Exception as e:
    node.get_logger().error(f"Movement failed: {e}")
```

## Coordinate System

- **Frame**: All coordinates are relative to `base_link` (robot base)
- **Units**:
  - Position: **millimeters**
  - Rotation: **radians**
- **Rotation Format**: UR rotation vector (axis-angle representation)

### Common Orientations

```python
# Gripper pointing down (most common for pick/place)
rx=-1.571, ry=0.0, rz=0.0  # -π/2

# Gripper pointing forward
rx=0.0, ry=0.0, rz=0.0

# Gripper pointing up
rx=1.571, ry=0.0, rz=0.0   # π/2
```

## Real Robot Usage

To use with a real UR5e robot:

```python
# Change use_fake_hardware to False
robot = CartesianRobot(node, use_fake_hardware=False)
```

This will automatically use the `scaled_joint_trajectory_controller` instead of `joint_trajectory_controller`.

## How It Works

1. **Boundary Check**: Position validated against safety boundaries
2. **Path Planning**: MoveIt computes Cartesian path with collision checking
3. **Trajectory Execution**: Path sent directly to joint trajectory controller
4. **Feedback**: Real-time status updates during execution

## Advantages Over Old System

| Old System | New System |
|------------|------------|
| Action server + client architecture | Simple function calls |
| Async/await complexity | Synchronous, easy to follow |
| No built-in boundary checking | Safety boundaries enforced |
| Separate files | All in one module |
| Hard to integrate | Easy to import and use |

## Troubleshooting

### "Position outside safety boundaries"
- Check your x, y, z values are in mm and within safe range
- x >= -300mm, y >= -300mm, 0mm <= z <= 655mm

### "Only X% of path could be computed"
- Target position may cause collision with safety boundaries
- Try a different path or intermediate waypoints
- Check if MoveIt planning scene has proper collision objects

### "Joint states not received"
- Make sure `./setupFakeur5e.sh` is running
- Check that `/joint_states` topic is publishing: `ros2 topic echo /joint_states`

### "Could not reach service/action within timeout"
- Ensure MoveIt is fully initialized
- Wait longer after running `./setupFakeur5e.sh`
- Check that MoveIt services are available: `ros2 service list | grep compute_cartesian_path`

## Example Workflows

### Pick and Place

```python
# Move above object
robot.move_to_cartesian(100, 400, 400, -1.571, 0.0, 0.0)

# Move down to object
robot.move_to_cartesian(100, 400, 50, -1.571, 0.0, 0.0)

# Close gripper (add your gripper control here)

# Move up with object
robot.move_to_cartesian(100, 400, 400, -1.571, 0.0, 0.0)

# Move to place location
robot.move_to_cartesian(200, 300, 400, -1.571, 0.0, 0.0)

# Move down to place
robot.move_to_cartesian(200, 300, 50, -1.571, 0.0, 0.0)

# Open gripper (add your gripper control here)

# Move up
robot.move_to_cartesian(200, 300, 400, -1.571, 0.0, 0.0)
```

### Square Pattern

```python
# Define square corners
height = 300  # mm
size = 100    # mm
center_x, center_y = 150, 350

corners = [
    (center_x - size/2, center_y - size/2, height),  # Bottom-left
    (center_x + size/2, center_y - size/2, height),  # Bottom-right
    (center_x + size/2, center_y + size/2, height),  # Top-right
    (center_x - size/2, center_y + size/2, height),  # Top-left
    (center_x - size/2, center_y - size/2, height),  # Back to start
]

# Execute square pattern
for x, y, z in corners:
    robot.move_to_cartesian(x, y, z, -1.571, 0.0, 0.0)
```

## Files

- `cartesian_robot.py` - Main CartesianRobot class
- `test_cartesian_robot.py` - Test script with pre-defined movements
- `move_cartesian_simple.py` - Command-line movement script
- `move_cartesian.sh` - Shell wrapper for easy command-line use
- `CARTESIAN_USAGE.md` - This documentation
