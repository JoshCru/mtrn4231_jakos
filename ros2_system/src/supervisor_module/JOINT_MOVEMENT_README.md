# Joint Movement Controller for UR5e

## Overview

This package provides a C++ service-based interface for controlling the UR5e robot's joint movements using MoveIt. It's designed to be called by the supervisor node or any other node that needs to command the robot.

## Joint Numbering Convention

**IMPORTANT:** This system uses a specific joint numbering:

```
Joint 6 = shoulder_pan_joint (base rotation)
Joint 1 = shoulder_lift_joint
Joint 2 = elbow_joint
Joint 3 = wrist_1_joint
Joint 4 = wrist_2_joint
Joint 5 = wrist_3_joint
```

**Joint order in arrays:** `[Joint6, Joint1, Joint2, Joint3, Joint4, Joint5]`

## Components

### 1. `joint_movement_controller` (C++ Node)

**Purpose:** Provides a service to move the robot to specified joint positions using MoveIt.

**Service:** `/move_to_joint_position` (type: `sort_interfaces/srv/MoveToJointPosition`)

**Features:**
- Uses MoveIt action client for trajectory planning and execution
- Tight tolerances (±0.01 rad ≈ ±0.57°) for accurate positioning
- Detailed logging of target positions in both radians and degrees
- Error handling and status reporting

### 2. `joint_movement_client_example` (Python Example)

**Purpose:** Shows how to use the joint movement service from Python (e.g., from supervisor).

**Features:**
- Demonstrates service client setup
- Shows how to convert degrees to radians
- Includes example pickup sequence

## Usage

### Step 1: Launch the System

First, launch the UR5e with MoveIt:

```bash
# Terminal 1: Source workspace
source ~/Documents/mtrn4231_jakos/ros2_system/install/setup.bash

# Launch the complete system (driver + MoveIt with gripper)
ros2 launch motion_control_module ur5e_real_with_gripper.launch.py
```

### Step 2: Start the Joint Movement Controller

```bash
# Terminal 2: Source workspace
source ~/Documents/mtrn4231_jakos/ros2_system/install/setup.bash

# Run the joint movement controller
ros2 run supervisor_module joint_movement_controller
```

### Step 3: Test with Example Client

```bash
# Terminal 3: Source workspace
source ~/Documents/mtrn4231_jakos/ros2_system/install/setup.bash

# Run the example client
ros2 run supervisor_module joint_movement_client_example
```

## Service Interface

### Service Definition

```
# Request
string position_name           # Name of the position (e.g., "home", "pick_up")
float64[] joint_positions      # 6 joint positions in radians [J6, J1, J2, J3, J4, J5]

# Response
bool success                   # True if movement was successful
string message                 # Status message or error description
```

### Example: Calling from Python

```python
from sort_interfaces.srv import MoveToJointPosition
import math

# Create client
move_client = self.create_client(MoveToJointPosition, '/move_to_joint_position')

# Define positions in degrees [J6, J1, J2, J3, J4, J5]
home_position_deg = [0.0, -75.0, 90.0, -105.0, -90.0, 0.0]

# Convert to radians
home_position_rad = [math.radians(d) for d in home_position_deg]

# Create and send request
request = MoveToJointPosition.Request()
request.position_name = "home"
request.joint_positions = home_position_rad

future = move_client.call_async(request)
# Wait for response...
```

### Example: Calling from C++

```cpp
#include "sort_interfaces/srv/move_to_joint_position.hpp"

// Create client
auto client = this->create_client<sort_interfaces::srv::MoveToJointPosition>(
    "/move_to_joint_position");

// Create request
auto request = std::make_shared<sort_interfaces::srv::MoveToJointPosition::Request>();
request->position_name = "home";
request->joint_positions = {0.0, -1.309, 1.571, -1.833, -1.571, 0.0};  // radians

// Call service
auto future = client->async_send_request(request);
// Wait for response...
```

## Predefined Positions

The example client includes these predefined positions (in degrees):

| Position Name | J6    | J1     | J2    | J3      | J4    | J5   |
|---------------|-------|--------|-------|---------|-------|------|
| Home          | 0.0   | -75.0  | 90.0  | -105.0  | -90.0 | 0.0  |
| Pick up       | 0.0   | -53.0  | 88.0  | -127.0  | -86.0 | 9.0  |
| Lift up       | -12.0 | -60.0  | 82.0  | -113.0  | -86.0 | 9.0  |
| Put down      | -12.0 | -53.0  | 88.0  | -126.0  | -86.0 | 9.0  |

## Integration with Supervisor

To use in your supervisor node:

```python
class SystemSupervisor(Node):
    def __init__(self):
        super().__init__('system_supervisor')

        # Create joint movement client
        self.move_client = self.create_client(
            MoveToJointPosition,
            '/move_to_joint_position'
        )

    def perform_pickup(self):
        """Example: Perform a pickup operation"""

        # Move to pick position
        self.move_to_joints("pick_up", [0.0, -53.0, 88.0, -127.0, -86.0, 9.0])

        # Close gripper (your gripper control code)
        # ...

        # Move to lift position
        self.move_to_joints("lift_up", [-12.0, -60.0, 82.0, -113.0, -86.0, 9.0])

    def move_to_joints(self, name, joint_positions_deg):
        """Helper function to move to joint position"""
        request = MoveToJointPosition.Request()
        request.position_name = name
        request.joint_positions = [math.radians(d) for d in joint_positions_deg]

        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result().success
```

## Troubleshooting

### Service not available
```bash
# Check if the controller is running
ros2 node list | grep joint_movement_controller

# Check if service exists
ros2 service list | grep move_to_joint_position
```

### Movement fails
- Check MoveIt logs for planning failures
- Verify joint positions are in radians
- Ensure positions are within robot's joint limits
- Check for collisions in RViz

### Check current joint states
```bash
# Use the diagnostic tool
ros2 run motion_control_module check_joint_states.py
```

## Files

- `src/joint_movement_controller.cpp` - C++ service node
- `supervisor_module/joint_movement_client_example.py` - Python example client
- `srv/MoveToJointPosition.srv` - Service definition (in sort_interfaces package)

## Dependencies

- `rclcpp`
- `rclcpp_action`
- `moveit_msgs`
- `sensor_msgs`
- `sort_interfaces`

All dependencies are listed in `package.xml` and will be installed automatically.
