# UR5e Square Box Movement

## Overview
This script demonstrates controlled square box movement with the UR5e robot while maintaining safety constraints to avoid singularities and maintain constant z-axis height.

## Features
- **Square pattern movement**: 15cm x 15cm square
- **Constant z-height**: Maintains 30cm height above base
- **Joint restrictions**: Shoulder and elbow joints are restricted to safe ranges to avoid:
  - Upward pointing configurations
  - Singularities
  - Unreachable positions
- **Smooth trajectories**: 3-second motion between waypoints

## Files Created
1. `ros2_system/src/motion_control_module/scripts/square_movement_simple.py` - Main script
2. `test_square_movement.sh` - Test wrapper script

## Usage

### Step 1: Start the Fake UR5e
```bash
cd ~/Documents/mtrn4231_jakos/4231_scripts
./setupFakeur5e.sh
```

This will open two terminals:
- **DriverServer**: UR robot driver with fake hardware
- **MoveitServer**: MoveIt motion planning with RViz visualization

Wait for both to fully start (about 10 seconds).

### Step 2: Run the Square Movement
In a new terminal:

```bash
cd ~/Documents/mtrn4231_jakos
python3 ros2_system/src/motion_control_module/scripts/square_movement_simple.py
```

Or use the test script:
```bash
./test_square_movement.sh
```

## What Happens
The robot will:
1. Move to home position (center)
2. Move to corner 1 (forward-right)
3. Move to corner 2 (forward-left)
4. Move to corner 3 (back-left)
5. Move to corner 4 (back-right)
6. Return to corner 1
7. Return to home position

Each movement takes about 3 seconds with a 1-second pause at each waypoint.

## Joint Configuration Details

The script uses predefined safe joint positions that avoid singularities:

- **shoulder_lift_joint**: Restricted to -2.0 to -0.5 rad (avoids upward pointing)
- **elbow_joint**: Restricted to -2.5 to -0.5 rad (maintains safe configuration)
- **wrist joints**: Full range allowed (no singularity risk)

## Customization

Edit these parameters in `square_movement_simple.py`:

```python
self.square_size = 0.15  # Square size in meters
self.z_height = 0.3      # Z-height in meters
self.center_x = 0.3      # Center X position
self.center_y = 0.0      # Center Y position
```

## Monitoring

You can monitor the movement in RViz (launched automatically with setupFakeur5e.sh) or check joint states:

```bash
# Monitor joint positions
ros2 topic echo /joint_states

# Monitor trajectory controller state
ros2 topic echo /joint_trajectory_controller/controller_state
```

## Troubleshooting

**Error: "/joint_states topic not found"**
- Make sure the fake UR5e is running (use setupFakeur5e.sh)

**Error: "Action server not available"**
- Wait longer for MoveIt to fully initialize
- Check that both terminals from setupFakeur5e.sh are running

**Movement looks wrong in RViz**
- The robot should move in a horizontal square pattern
- If it's moving strangely, check the joint limits in the script

## Technical Details

- Uses ROS2 action interface: `/joint_trajectory_controller/follow_joint_trajectory`
- Sends `FollowJointTrajectory` goals with 6 joint positions
- Waits for each trajectory to complete before moving to next waypoint
- All movements maintain the same end-effector orientation (pointing down)
