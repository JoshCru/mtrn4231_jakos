# Cartesian Robot with Constraints - Quick Reference

## What's New

I've updated the `CartesianRobot` class with the features you requested:

### ✅ Fixed Orientation Constraint
- **End-effector always faces down**: `rx=3.14, ry=0.001, rz=0.001`
- Use the simplified `move(x, y, z)` method instead of `move_to_cartesian()`

### ✅ Home Position
- **Calibrated from joint angles**: `[0, -75, 90, -105, -90]` degrees
- **Cartesian coordinates**: `x=-588.63mm, y=-133.62mm, z=370.85mm`
- **Orientation**: `rx=2.222, ry=2.222, rz=0.0`
- Use `robot.go_home()` to return to home

### ✅ Smooth Paths for Small Movements
- Default `max_step=0.005m` (5mm) for smoother interpolation
- Prevents extravagant maneuvers on small movements
- Adjustable per movement if needed

### ✅ Updated Safety Boundaries
- **x**: >= -600mm (adjusted to include home position)
- **y**: >= -300mm
- **z**: 0mm to 655mm

## Quick Start

### 1. Start your system
```bash
./setupFakeur5e.sh
```

### 2. Test home position and constrained movements
```bash
./test_home.sh
```

## Usage Examples

### Basic Example - Using Constraints

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cartesian_robot import CartesianRobot

rclpy.init()
node = Node("my_robot")
robot = CartesianRobot(node, use_fake_hardware=True)

# Go to home position
robot.go_home()

# Move to positions - end-effector always faces down!
# Just specify x, y, z - orientation is automatic
robot.move(100, 400, 300)
robot.move(200, 300, 400)
robot.move(150, 350, 350)

# Return home
robot.go_home()

node.destroy_node()
rclpy.shutdown()
```

### Advanced Example - Custom max_step

```python
# For very small, precise movements, use smaller max_step
robot.move(100, 400, 300, max_step=0.002)  # 2mm steps - very smooth!

# For larger movements, default is fine
robot.move(200, 300, 400)  # Uses default 5mm steps
```

### Advanced Example - Custom Orientation (if needed)

```python
# If you need a different orientation for some reason
robot.move_to_cartesian(
    x=100, y=400, z=300,
    rx=3.14, ry=0.001, rz=0.001,  # Custom orientation
    check_bounds=True
)
```

## API Reference

### `robot.go_home()`
Move to the calibrated home position.

**Returns:** `True` if successful, `False` otherwise

**Example:**
```python
robot.go_home()
```

---

### `robot.move(x, y, z, max_step=0.005)`
Move to position with constrained orientation (end-effector down).

**Parameters:**
- `x, y, z` - Position in millimeters (relative to base_link)
- `max_step` - Cartesian interpolation step in meters (default 5mm)
  - Smaller = smoother but slower planning
  - Good for small movements: 2-5mm
  - Good for large movements: 5-10mm

**Returns:** `True` if successful, `False` otherwise

**Example:**
```python
# Standard movement
robot.move(100, 400, 300)

# Very smooth movement
robot.move(100, 400, 300, max_step=0.002)
```

---

### `robot.move_to_cartesian(x, y, z, rx, ry, rz, check_bounds=True)`
Move to position with custom orientation (advanced).

**Parameters:**
- `x, y, z` - Position in millimeters
- `rx, ry, rz` - Rotation vector in radians (UR axis-angle format)
- `check_bounds` - Enable safety boundary checking (default True)

**Returns:** `True` if successful, `False` otherwise

**Example:**
```python
robot.move_to_cartesian(100, 400, 300, 3.14, 0.001, 0.001)
```

## Configuration Constants

You can modify these in `cartesian_robot.py` if needed:

```python
# Default orientation (end-effector down)
DEFAULT_ORIENTATION = {
    'rx': 3.14,    # π radians
    'ry': 0.001,
    'rz': 0.001,
}

# Home position
HOME_POSITION = {
    'x': -588.63,   # mm
    'y': -133.62,   # mm
    'z': 370.85,    # mm
    'rx': 2.222,    # rad
    'ry': 2.222,    # rad
    'rz': 0.0,      # rad
}

# Safety boundaries
SAFETY_BOUNDS = {
    'x_min': -0.6,   # -600mm
    'x_max': 2.0,    # 2000mm
    'y_min': -0.3,   # -300mm
    'y_max': 2.0,    # 2000mm
    'z_min': 0.0,    # 0mm
    'z_max': 0.655,  # 655mm
}
```

## Typical Workflow

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from cartesian_robot import CartesianRobot

rclpy.init()
node = Node("pick_and_place")
robot = CartesianRobot(node, use_fake_hardware=True)

# Start at home
robot.go_home()

# Pick sequence
robot.move(100, 400, 400)  # Move above object
robot.move(100, 400, 50)   # Move down to object
# ... close gripper ...
robot.move(100, 400, 400)  # Lift object

# Place sequence
robot.move(200, 300, 400)  # Move to place location
robot.move(200, 300, 50)   # Move down to place
# ... open gripper ...
robot.move(200, 300, 400)  # Lift up

# Return home
robot.go_home()

node.destroy_node()
rclpy.shutdown()
```

## Testing

Run the test script to verify everything works:

```bash
./test_home.sh
```

This will:
1. Move to HOME position
2. Perform small constrained movements
3. Return to HOME

## Troubleshooting

### "Position outside safety boundaries"
- Home position requires x >= -600mm
- Check if your target is within bounds
- Modify `SAFETY_BOUNDS` in `cartesian_robot.py` if needed

### Robot takes strange paths
- Reduce `max_step` for smoother paths:
  ```python
  robot.move(100, 400, 300, max_step=0.002)  # 2mm steps
  ```

### "Only X% of path could be computed"
- Target may collide with safety boundaries
- Try intermediate waypoints
- Check MoveIt planning scene

## Advantages of Constrained Movement

| Without Constraints | With Constraints |
|---------------------|------------------|
| Must specify rx, ry, rz every time | Just specify x, y, z |
| May rotate unexpectedly | Always faces down |
| Long function calls | Clean, simple calls |
| Easy to make mistakes | Harder to make mistakes |

**Before:**
```python
robot.move_to_cartesian(100, 400, 300, 3.14, 0.001, 0.001)
robot.move_to_cartesian(200, 300, 400, 3.14, 0.001, 0.001)
robot.move_to_cartesian(150, 350, 350, 3.14, 0.001, 0.001)
```

**After:**
```python
robot.move(100, 400, 300)
robot.move(200, 300, 400)
robot.move(150, 350, 350)
```

Much cleaner! 🎉
