# MoveIt Setup Guide for UR5e Robot

Complete setup instructions for both fake (simulation) and real UR5e robots.

---

## Prerequisites

### 1. Environment Setup

Add to your `~/.bashrc`:
```bash
source /opt/ros/humble/setup.bash
source /home/mtrn/4231/ros_ur_driver/install/setup.bash
source /home/mtrn/4231/ws_moveit2/install/setup.bash
source /home/mtrn/Documents/mtrn4231_jakos/ros2_system/install/setup.bash
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

**IMPORTANT:** Make sure these lines are NOT in your bashrc (they cause memory errors):
```bash
# DO NOT HAVE THESE:
export FASTRTPS_DEFAULT_PROFILES_FILE=
export RMW_FASTRTPS_USE_QOS_FROM_XML=1
```

### 2. Build the Workspace

```bash
cd /home/mtrn/Documents/mtrn4231_jakos/ros2_system
colcon build
source install/setup.bash
```

---

## Part A: Fake Robot (Simulation) Setup

### Step 1: Start the Fake UR5e System

Run the setup script:
```bash
cd /home/mtrn/Documents/mtrn4231_jakos/4231_scripts
./setupFakeur5e.sh
```

Or manually start each component in separate terminals:

**Terminal 1 - Robot Driver (Fake Hardware):**
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=yyy.yyy.yyy.yyy \
    use_fake_hardware:=true \
    launch_rviz:=false \
    initial_joint_controller:=joint_trajectory_controller
```

**Terminal 2 - MoveIt:**
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur5e \
    use_fake_hardware:=true \
    launch_rviz:=true
```

### Step 2: Verify System is Running

```bash
# Check topics
ros2 topic list | grep -E "joint_states|trajectory"

# Check services
ros2 service list | grep -E "compute_cartesian|move_to"

# Check controllers
ros2 control list_controllers
```

Expected controllers:
- `joint_trajectory_controller` [active]
- `joint_state_broadcaster` [active]

### Step 3: Start the Cartesian Controller Node

```bash
ros2 run motion_control_package cartesian_controller_node
```

You should see:
```
[INFO] Using controller: /joint_trajectory_controller/follow_joint_trajectory
[INFO] Connected to /compute_cartesian_path
[INFO] Connected to trajectory action server
[INFO] Joint states received
[INFO] Service available: /motion_control/move_to_cartesian
[INFO] Frame toggle service: /motion_control/use_gripper_tip
```

---

## Part B: Real Robot Setup

### Step 1: Network Configuration

1. Connect to the robot via Ethernet
2. Set your PC's IP to be on the same subnet as the robot (e.g., 192.168.1.x)
3. Robot IP is typically: `192.168.1.102` (check your robot's teach pendant)

### Step 2: Prepare the Robot

On the teach pendant:
1. Power on the robot
2. Release the brakes
3. Load and start the External Control URCap program

### Step 3: Start the Real Robot Driver

**Terminal 1 - Robot Driver:**
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
    ur_type:=ur5e \
    robot_ip:=192.168.1.102 \
    use_fake_hardware:=false \
    launch_rviz:=false \
    initial_joint_controller:=scaled_joint_trajectory_controller
```

Wait for: `Robot connected to reverse interface. Ready to receive control commands.`

**Terminal 2 - MoveIt:**
```bash
ros2 launch ur_moveit_config ur_moveit.launch.py \
    ur_type:=ur5e \
    use_fake_hardware:=false \
    launch_rviz:=true
```

### Step 4: Start the Cartesian Controller Node (Real Robot)

```bash
ros2 run motion_control_package cartesian_controller_node --ros-args -p use_fake_hardware:=false
```

This will use `/scaled_joint_trajectory_controller/follow_joint_trajectory` instead.

---

## Part C: Using the Cartesian Controller

### Service Interface

**Main Movement Service:** `/motion_control/move_to_cartesian`

```bash
# Move to position (coordinates in mm, rotation in radians)
ros2 service call /motion_control/move_to_cartesian sort_interfaces/srv/MoveToCartesian \
    "{x: -588.0, y: -133.0, z: 222.0, rx: -2.221, ry: 2.221, rz: 0.0}"
```

### Coordinate System

- Coordinates are in **millimeters** (mm)
- Rotations are in **radians** (axis-angle/rotation vector format)
- X and Y are **negated** internally (input -588 -> robot +588mm)
- Z is direct (input 222 -> robot 222mm)
- Reference frame: `base_link`

### Frame Toggle Service

Switch between gripper_tip and tool0 frames:

```bash
# Use tool0 (robot TCP)
ros2 service call /motion_control/use_gripper_tip std_srvs/srv/SetBool "{data: false}"

# Use gripper_tip (default)
ros2 service call /motion_control/use_gripper_tip std_srvs/srv/SetBool "{data: true}"
```

### Safety Boundaries

Current limits (in robot frame, meters):
- X: -0.6m to 2.0m
- Y: -0.7m to 2.0m
- Z: 0.0m to 0.655m

### Home Position

Joint angles for home position:
- shoulder_pan: 0°
- shoulder_lift: -75°
- elbow: 90°
- wrist_1: -105°
- wrist_2: -90°
- wrist_3: 0°

Move to home via joint trajectory:
```bash
ros2 action send_goal /joint_trajectory_controller/follow_joint_trajectory \
    control_msgs/action/FollowJointTrajectory \
    "{trajectory: {
        joint_names: [shoulder_pan_joint, shoulder_lift_joint, elbow_joint, wrist_1_joint, wrist_2_joint, wrist_3_joint],
        points: [{positions: [0.0, -1.309, 1.5708, -1.8326, -1.5708, 0.0], time_from_start: {sec: 3}}]
    }}"
```

---

## Part D: Troubleshooting

### FastRTPS Memory Errors

If you see:
```
Not enough memory in the buffer stream
```

**Fix:**
```bash
# Remove these from ~/.bashrc if present:
# export FASTRTPS_DEFAULT_PROFILES_FILE=
# export RMW_FASTRTPS_USE_QOS_FROM_XML=1

# Then in current terminal:
unset FASTRTPS_DEFAULT_PROFILES_FILE
unset RMW_FASTRTPS_USE_QOS_FROM_XML
```

### Shared Memory Port Errors

If you see:
```
Failed init_port fastrtps_port7419: open_and_lock_file failed
```

These are warnings and can be ignored. They occur when multiple ROS2 nodes try to use the same shared memory ports.

### Controller Not Found

If cartesian controller can't find the trajectory action server:

```bash
# Check available controllers
ros2 control list_controllers

# For fake robot, should see:
# joint_trajectory_controller [active]

# For real robot, should see:
# scaled_joint_trajectory_controller [active]
```

### Path Planning Fails

If you get "Only X% of path could be computed":
1. Check if target is within robot reach
2. Check for collisions with the environment
3. Try a different orientation
4. Move in smaller increments

### Robot Not Moving (Real Robot)

1. Check External Control program is running on teach pendant
2. Check robot is not in protective stop
3. Verify network connection: `ping 192.168.1.102`
4. Check driver received connection message

---

## Part E: Quick Reference

### Start Everything (Fake Robot)

```bash
# Terminal 1: Setup script
cd /home/mtrn/Documents/mtrn4231_jakos/4231_scripts && ./setupFakeur5e.sh

# Terminal 2: Cartesian controller
source /home/mtrn/Documents/mtrn4231_jakos/ros2_system/install/setup.bash
ros2 run motion_control_package cartesian_controller_node
```

### Start Everything (Real Robot)

```bash
# Terminal 1: Robot driver
ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.1.102 use_fake_hardware:=false launch_rviz:=false initial_joint_controller:=scaled_joint_trajectory_controller

# Terminal 2: MoveIt
ros2 launch ur_moveit_config ur_moveit.launch.py ur_type:=ur5e use_fake_hardware:=false launch_rviz:=true

# Terminal 3: Cartesian controller
ros2 run motion_control_package cartesian_controller_node --ros-args -p use_fake_hardware:=false
```

### Test Movement

```bash
# Check current TCP position
ros2 run tf2_ros tf2_echo base_link gripper_tip

# Move to a test position
ros2 service call /motion_control/move_to_cartesian sort_interfaces/srv/MoveToCartesian \
    "{x: -500.0, y: -200.0, z: 300.0, rx: -2.221, ry: 2.221, rz: 0.0}"
```
