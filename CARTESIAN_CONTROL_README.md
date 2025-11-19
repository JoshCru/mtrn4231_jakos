# Cartesian Control System - Quick Reference

## Coordinate Frame

**ALL COORDINATES ARE WITH RESPECT TO BASE_LINK FRAME (robot base)**

- The origin (0, 0, 0) is at the base of the robot
- X, Y, Z are measured in **millimeters** from the robot base
- RX, RY, RZ use **UR rotation vector** format (axis-angle representation) in **radians**

## Files and Usage

### 1. Action-Based Controller (RECOMMENDED)

**Start the controller:**
```bash
source install/setup.bash
python3 ros2_system/src/motion_control_module/scripts/action_cartesian_controller.py
```

**Use the action client:**
```bash
# Full control (specify position and orientation)
python3 ros2_system/src/motion_control_module/scripts/action_cartesian_client.py <X> <Y> <Z> <RX> <RY> <RZ>

# Example: Move to X=100mm, Y=400mm, Z=800mm with gripper down
python3 ros2_system/src/motion_control_module/scripts/action_cartesian_client.py 100 400 800 -1.571 0.0 0.0
```

**Use the simplified client (gripper always faces down):**
```bash
# Only specify position - orientation is automatic
python3 ros2_system/src/motion_control_module/scripts/move_gripper_down.py <X> <Y> <Z>

# Example: Move to X=100mm, Y=400mm, Z=800mm
python3 ros2_system/src/motion_control_module/scripts/move_gripper_down.py 100 400 800
```

### 2. Service-Based Controllers (Legacy - has execution issues)

These are available but **NOT recommended** due to ROS2 executor threading limitations:
- `cartesian_movement_controller.py` - Original service-based controller
- `working_cartesian_controller.py` - Callback-based approach
- `final_cartesian_controller.py` - Time-based waiting approach

## Gripper Orientation

### Gripper Facing Down (Most Common)
```
RX = -1.571 rad (≈ -90°)
RY = 0.0 rad
RZ = 0.0 rad
```

### Your Home Position
From the base feature coordinates you provided:
```
X  = -589.22 mm
Y  = -131.78 mm
Z  = 371.73 mm
RX = 2.22 rad
RY = 2.22 rad
RZ = 0.004 rad
```

## Understanding Rotation Vectors

The rotation vector (RX, RY, RZ) uses **axis-angle representation**:
- The **direction** of the vector (RX, RY, RZ) is the axis of rotation
- The **magnitude** √(RX² + RY² + RZ²) is the angle in radians

Example: `(RX=-1.571, RY=0, RZ=0)` means:
- Rotate -90° (-π/2 rad) around the X-axis
- This points the gripper straight down

## Safety Boundaries

The system has collision boundaries at:
- **Table**: z = 0m (don't go below the table)
- **Back plane**: y = -0.3m
- **Side plane**: x = -0.3m
- **Ceiling**: z = 2.655m

MoveIt will refuse to plan paths that violate these boundaries.

## Checking Current Position

To see the current robot position:
```bash
python3 ros2_system/src/motion_control_module/scripts/check_tcp_position.py
```

This shows:
- Current joint angles
- Current X, Y, Z position (in mm)
- Current RX, RY, RZ orientation (in radians)
- For both `tool0` and `gripper_tip` frames

## Key Technical Details

1. **Frame**: All coordinates use `base_link` frame (set in controller code: `req.header.frame_id = 'base_link'`)
2. **End Effector**: Controlled from `gripper_tip` link (not `tool0`)
3. **Path Planning**: Uses Cartesian path planning for straight-line motion
4. **Action vs Service**: Actions are recommended for long-running movements (avoid executor deadlocks)
5. **Async Implementation**: The action controller uses proper Python async/await patterns

## Troubleshooting

### Path Planning Fails (0% computed)
- Target position is unreachable from current position
- Target violates safety boundaries
- Try a position closer to current location first

### Robot Plans But Doesn't Execute (Service-based controllers)
- This is a known issue with service-based architecture
- **Solution**: Use the action-based controller instead

### "No joint state available"
- Controller started before robot/simulation
- Restart the controller after the robot system is running
