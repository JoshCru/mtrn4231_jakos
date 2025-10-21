# System Architecture Overview

## Module Hierarchy

```
┌─────────────────────────────────────────────────────────────────┐
│                      SUPERVISOR MODULE                          │
│                  (system_controller_node)                       │
│         System Commands │ Target Areas │ Status                │
└────────────────┬────────────────────────────────────────────────┘
                 │
    ┌────────────┴──────────────┬────────────────────────┐
    │                           │                        │
┌───▼───────────────┐  ┌────────▼──────────┐  ┌─────────▼──────────┐
│  PERCEPTION       │  │   RECOGNITION     │  │    PLANNING        │
│  - RGBD Camera    │  │   - Weight Est.   │  │    - Sort Logic    │
│  - OpenCV Proc    │  │                   │  │    - Verification  │
│  - PointCloud     │  │                   │  │    - Integrity     │
│                   │  │                   │  │    - MoveIt2 Iface │
└───┬───────────────┘  └────────┬──────────┘  └─────────┬──────────┘
    │                           │                        │
    └───────────────────────────┴───────────┬────────────┘
                                            │
                                 ┌──────────▼──────────┐
                                 │     CONTROL         │
                                 │  - Robot Driver     │
                                 │  - Pick Operation   │
                                 │  - Place Operation  │
                                 └──────────┬──────────┘
                                            │
                                 ┌──────────▼──────────┐
                                 │  MOTION CONTROL     │
                                 │  - Gripper Ctrl     │
                                 │  (Arduino Interface)│
                                 └─────────────────────┘
```

## Data Flow

### Pick and Sort Operation Flow:

```
1. PERCEPTION
   Camera → OpenCV → PointCloud Processor
   ↓
   DetectedObjects + 3D Positions

2. RECOGNITION
   Volume Analysis → Weight Estimation
   ↓
   WeightEstimate

3. PLANNING - DECISION
   Sort Node → Select Target Bin
   ↓
   SortDecision (pick_pose, place_pose)

4. PLANNING - MOTION
   MoveIt2 Interface → Plan Trajectory
   ↓
   RobotTrajectory

5. CONTROL - PICK
   Pick Operation Node:
     a. Move to approach position
     b. Open gripper
     c. Move to grasp position
     d. Close gripper
     e. Lift object
     f. Measure actual weight
   ↓
   Actual Weight Reading

6. PLANNING - VERIFICATION
   Verification Node → Compare Weights
   ↓
   Verification Result (Pass/Fail)

7. CONTROL - PLACE
   Place Operation Node:
     a. Move to target bin (approach)
     b. Lower to placement height
     c. Release gripper
     d. Retreat to safe position
   ↓
   Object Sorted Successfully
```

## Topic Communication Map

### Published Topics:
```
SUPERVISOR:
  /system/commands (String)
  /system/target_areas (TargetArea)
  /system/status (String)

PERCEPTION:
  /camera/color/image_raw (Image)
  /camera/depth/image_raw (Image)
  /camera/pointcloud (PointCloud2)
  /camera/color/camera_info (CameraInfo)
  /perception/detected_objects (DetectedObjects)
  /perception/object_positions (PoseArray)
  /perception/object_markers (MarkerArray)
  /perception/debug_image (Image)

RECOGNITION:
  /recognition/estimated_weights (WeightEstimate)

PLANNING:
  /planning/sort_decisions (SortDecision)
  /planning/verification_result (Bool)
  /planning/environment_status (EnvironmentStatus)
  /planning/trajectory (DisplayTrajectory)

CONTROL:
  /control/joint_states (JointState)
  /control/robot_status (String)
  /control/actual_weight (Float32)

MOTION CONTROL:
  /motion_control/gripper_state (Float32)
  /motion_control/force_feedback (ForceFeedback)
```

### Action Servers:
```
PLANNING:
  /planning/plan_pick (PlanTrajectory)
  /planning/plan_place (PlanTrajectory)
  /planning/verify_weight (VerifyWeight)

CONTROL:
  /control/execute_trajectory (FollowJointTrajectory)
  /control/pick_object (PickObject)
  /control/place_object (PlaceObject)
```

### Services:
```
SUPERVISOR:
  /system/start (SystemCommand)
  /system/stop (SystemCommand)
  /system/emergency_stop (SystemCommand)

PLANNING:
  /planning/validate_workspace (ValidateWorkspace)

MOTION CONTROL:
  /motion_control/calibrate_gripper (CalibrateGripper)
```

## State Machines

### Pick Operation States:
```
IDLE → APPROACHING → PRE_GRASP → GRASPING → LIFTING →
MEASURING_WEIGHT → COMPLETE
                     ↓ (on error)
                   ERROR
```

### Place Operation States:
```
IDLE → APPROACHING → LOWERING → PLACING → RELEASING →
RETREATING → COMPLETE
              ↓ (on error)
            ERROR
```

### Robot Driver Lifecycle:
```
UNCONFIGURED → CONFIGURED → ACTIVE
     ↑              ↑           ↓
     └──────────────┴───────INACTIVE
```

## Safety Features

1. **Emergency Stop System**
   - Global emergency stop command propagated to all motion nodes
   - Lifecycle nodes can be safely deactivated
   - All trajectory execution halts immediately

2. **Workspace Validation**
   - Integrity node monitors workspace bounds
   - Validates planned trajectories before execution
   - Detects unexpected obstacles

3. **Weight Verification**
   - Compares estimated vs actual weight
   - Configurable tolerance threshold
   - Logs discrepancies for system improvement

4. **Gripper Safety**
   - Force feedback monitoring
   - Object detection confirmation
   - Configurable grasp force limits

## Hardware Interfaces

### UR5e Robot
- Interface: UR RTDE or ros2_control
- Topics: /control/joint_states, /control/execute_trajectory
- Lifecycle managed for safe startup/shutdown

### RGBD Camera
- Type: RealSense D435/D455 or similar
- Interface: RealSense ROS2 wrapper or custom driver
- Output: RGB images, depth images, organized point clouds

### Arduino Gripper Controller
- Interface: Serial over USB (/dev/ttyACM0)
- Baud Rate: 115200
- Protocol:
  - RX: `G<angle>\n` (gripper command)
  - TX: `W<weight>\n` (weight sensor reading)
- Components:
  - Servo motor (PWM control)
  - Force/weight sensor (analog input)

## Configuration Priority

1. **System Controller** - Define sorting bins and weight ranges
2. **Perception** - Camera calibration and detection thresholds
3. **Planning** - Workspace limits and safety parameters
4. **Control** - Robot IP, motion speeds, pick/place heights
5. **Motion Control** - Gripper calibration and force thresholds

## Extension Points

The system is designed to be extended:

1. **Add more sorting criteria** - Modify sort_node.cpp
2. **Integrate ML models** - Update weight_estimation_node.cpp
3. **Support multiple grippers** - Extend motion_control_module
4. **Add bin fullness detection** - New node in perception_module
5. **Implement learning** - Log data and train improved estimators
6. **Multi-robot coordination** - Add coordination layer above supervisor

## Performance Considerations

- **Camera frame rate**: 30 Hz (configurable)
- **Joint state publishing**: 50 Hz
- **Force sensor sampling**: 20 Hz
- **Environment checks**: 10 Hz
- **Planning time**: ~5 seconds (MoveIt2 default)

## Dependencies

### ROS2 Packages:
- rclcpp, rclcpp_lifecycle, rclcpp_action
- sensor_msgs, geometry_msgs, trajectory_msgs
- moveit_msgs, moveit_ros_planning_interface
- cv_bridge, OpenCV

### External Libraries:
- OpenCV (3.4+ or 4.x)
- MoveIt2 (optional but recommended)
- PCL - Point Cloud Library (optional)
- Serial library (for Arduino communication)

## Build Order

Due to dependencies, packages must be built in order:
1. sort_interfaces (no dependencies)
2. All other modules (depend on sort_interfaces)

The provided `build_all.sh` handles this automatically with `colcon build`.
