# Debugging MoveIt Communication

## Problem
`joint_movement_controller` is timing out because MoveIt is not sending result callbacks back.

## Enhanced Debugging Added

I've added extensive logging to `joint_movement_controller.cpp` to track exactly where communication breaks down:

### 1. **Feedback Callback** (Line 249-253)
```cpp
send_goal_options.feedback_callback = ...
```
- This will show if MoveIt is sending ANY updates during execution
- If you see "Received feedback from MoveIt: ..." → MoveIt IS communicating!
- If you don't see this → MoveIt is not sending feedback

### 2. **Goal Acceptance Check** (Line 258-271)
```cpp
if (rclcpp::spin_until_future_complete(...) != SUCCESS)
```
- Explicitly waits for MoveIt to accept/reject the goal
- If goal is rejected → MoveIt won't execute anything
- Logs: "Goal was accepted, waiting for result..." when successful

### 3. **Periodic Wait Logging** (Line 297-302)
- Logs every 2 seconds: "Still waiting for result... (X seconds elapsed)"
- Shows the node is alive and actively waiting
- If timeout (30s) → Shows "MoveIt did not send a result back!"

### 4. **Result Timing** (Line 282-286)
- When result IS received, logs how long it took
- Helps verify normal operation timing

## Testing Procedure

### Terminal 1: Start MoveIt
```bash
cd ~/Documents/mtrn4231_jakos/ros2_system
source install/setup.bash
ros2 launch motion_control_module ur5e_real_with_gripper.launch.py
```
**Wait until you see:** "You can start planning now!"

### Terminal 2: Start joint_movement_controller
```bash
cd ~/Documents/mtrn4231_jakos/ros2_system
source install/setup.bash
ros2 run supervisor_module joint_movement_controller
```
**Expected logs:**
```
========================================
Joint Movement Controller starting...
========================================
Creating action client for /move_action...
Subscribing to /joint_states...
Creating service /move_to_joint_position...
========================================
Waiting for MoveIt action server at /move_action...
========================================
[After MoveIt connects]
========================================
✓ Connected to MoveIt action server!
✓ Service ready at /move_to_joint_position
========================================
```

### Terminal 3: Run diagnostic check
```bash
cd ~/Documents/mtrn4231_jakos/4231_scripts
./test_moveit_communication.sh
```

### Terminal 4: Test the service
```bash
cd ~/Documents/mtrn4231_jakos/ros2_system
source install/setup.bash
ros2 run supervisor_module joint_movement_client_example
```

## What to Look For

### ✓ SUCCESS Case:
```
Received move request: Home
Moving to position: Home
...
Calling async_send_goal...
Waiting for goal acceptance...
Goal accepted by MoveIt, executing...
Received feedback from MoveIt: PLANNING    ← MoveIt is communicating!
Received feedback from MoveIt: EXECUTING   ← Robot is moving!
Received result with error code: 1          ← MoveIt finished!
Movement completed successfully!
Result received after 5.3 seconds!
```

### ✗ PROBLEM Case 1 - Goal Rejected:
```
Received move request: Home
Calling async_send_goal...
Waiting for goal acceptance...
Goal was rejected by server!   ← MoveIt rejected the goal
```
**Solution:** Check MoveIt planning group configuration

### ✗ PROBLEM Case 2 - No Feedback:
```
Received move request: Home
Calling async_send_goal...
Waiting for goal acceptance...
Goal was accepted, waiting for result...
Still waiting for result... (2.0 seconds elapsed)   ← No feedback received
Still waiting for result... (4.0 seconds elapsed)
...
Timeout waiting for movement to complete (30s)
MoveIt did not send a result back!
```
**This means:** Goal was accepted but MoveIt is not sending feedback/results
**Possible causes:**
- Result callback not being processed (executor issue)
- MoveIt planning failed silently
- Action communication broken

## Verifying Action Communication

Check if MoveIt's action server is actually active:
```bash
ros2 action list
ros2 action info /move_action
```

You should see:
```
Action: /move_action
Action clients: 1
    /joint_movement_controller
Action servers: 1
    /move_group
```

## Next Steps Based on Results

1. **If you see feedback** → MoveIt IS communicating, result callback might have an issue
2. **If no feedback** → Check if MoveIt is actually planning/executing
3. **If goal rejected** → SRDF/planning group configuration issue

Run the test and share the complete output from Terminal 2 (joint_movement_controller).
