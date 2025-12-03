# Resource Analysis for Modular Launch Scripts

This document analyzes all nodes, packages, scripts, and messages used by `start_persistent.sh` and `start_temporary.sh`, and verifies their existence in the `/ros2_system/` workspace.

---

## Summary

✅ **All resources used by the scripts exist in the ros2_system workspace**

---

## 1. start_persistent.sh - Resources Used

### Launch Files
| Resource | Type | Location | Status |
|----------|------|----------|--------|
| `ur_robot_driver/ur_control.launch.py` | Launch File | External Package (ur_robot_driver) | ✅ External (from apt) |
| `motion_control_module/ur5e_moveit_with_gripper.launch.py` | Launch File | `/ros2_system/src/motion_control_module/launch/` | ✅ Found |

### URDF/Xacro Files
| Resource | Type | Location | Status |
|----------|------|----------|--------|
| `ur5e_with_end_effector.urdf.xacro` | URDF Xacro | `/ros2_system/src/motion_control_module/urdf/` | ✅ Found |

### Nodes (Executables)
| Node | Package | Source File | Status |
|------|---------|-------------|--------|
| `gripper_controller_node` | control_module | `/ros2_system/src/control_module/src/gripper_controller_node.cpp` | ✅ Found |
| `cartesian_controller_node` | motion_control_module | `/ros2_system/src/motion_control_module/src/cartesian_controller_node.cpp` | ✅ Found |

### Python Scripts
| Script | Location | Status |
|--------|----------|--------|
| `safety_boundary_collision.py` | `/ros2_system/src/motion_control_module/scripts/` | ✅ Found |

### ROS2 Commands Used
| Command | Purpose | Status |
|---------|---------|--------|
| `ros2 control list_controllers` | Check active controllers | ✅ Standard ROS2 CLI |
| `ros2 param list /move_group` | Check MoveIt parameters | ✅ Standard ROS2 CLI |
| `ros2 lifecycle set /gripper_controller_node configure` | Configure lifecycle node | ✅ Standard ROS2 CLI |
| `ros2 lifecycle set /gripper_controller_node activate` | Activate lifecycle node | ✅ Standard ROS2 CLI |

---

## 2. start_temporary.sh - Resources Used

### Nodes (Executables)
| Node | Package | Source File | Status |
|------|---------|-------------|--------|
| `go_home` | motion_control_module | `/ros2_system/src/motion_control_module/scripts/go_home.py` | ✅ Found |
| `simulated_perception_node` | supervisor_module | `/ros2_system/src/supervisor_module/supervisor_module/simulated_perception_node.py` | ✅ Found |
| `weight_detector` | weight_detection_module | `/ros2_system/src/weight_detection_module/src/weight_detector.cpp` | ✅ Found |
| `sorting_brain_node` | supervisor_module | `/ros2_system/src/supervisor_module/supervisor_module/sorting_brain_node.py` | ✅ Found |
| `simple_pick_and_weigh_node` | motion_control_module | `/ros2_system/src/motion_control_module/src/simple_pick_and_weigh_node.cpp` | ✅ Found |

### Python Scripts
| Script | Location | Status |
|--------|----------|--------|
| `check_simulated_positions.py` | `/ros2_system/src/motion_control_module/scripts/` | ✅ Found |
| `plot_weight.sh` | `/ros2_system/` | ✅ Found |

### Message Types Used
| Message Type | Package | Status |
|--------------|---------|--------|
| `std_msgs/msg/String` | std_msgs | ✅ Standard ROS2 Message |

### ROS2 Commands Used
| Command | Purpose | Status |
|---------|---------|--------|
| `ros2 topic pub --once /sorting/command std_msgs/msg/String "data: 'start'"` | Start sorting | ✅ Standard ROS2 CLI |

---

## 3. ROS2 Packages in Workspace

All packages referenced by the scripts exist in `/ros2_system/src/`:

| Package Name | Used By | Status |
|--------------|---------|--------|
| `control_module` | Persistent (gripper_controller_node) | ✅ Present |
| `motion_control_module` | Both scripts (multiple nodes/scripts) | ✅ Present |
| `supervisor_module` | Temporary (perception, sorting brain) | ✅ Present |
| `weight_detection_module` | Temporary (weight_detector) | ✅ Present |
| `perception_module` | Not used in scripts | ⚠️ Available but unused |
| `planning_module` | Not used in scripts | ⚠️ Available but unused |
| `sort_interfaces` | Used indirectly by nodes | ✅ Present |
| `util_arduino_serial` | Not used in scripts | ⚠️ Available but unused |

---

## 4. External Dependencies

The following resources are external to the ros2_system workspace:

| Resource | Type | Source | Status |
|----------|------|--------|--------|
| `ur_robot_driver` | ROS2 Package | apt package (ros-humble-ur-robot-driver) | ✅ External dependency |
| `std_msgs` | ROS2 Package | Standard ROS2 (ros-humble-std-msgs) | ✅ External dependency |
| `moveit` | ROS2 Package | apt packages (ros-humble-moveit-*) | ✅ External dependency |

---

## 5. Node Parameters Used

### gripper_controller_node (Persistent)
| Parameter | Type | Values Used |
|-----------|------|-------------|
| `simulation_mode` | bool | `true` (sim), `false` (real) |

### cartesian_controller_node (Persistent)
| Parameter | Type | Values Used |
|-----------|------|-------------|
| `use_fake_hardware` | bool | `true` (sim), `false` (real) |

### simulated_perception_node (Temporary)
| Parameter | Type | Values Used |
|-----------|------|-------------|
| `num_objects` | int | `4` |
| `publish_rate` | float | `5.0` |
| `randomize_positions` | bool | `true` |

### simple_pick_and_weigh_node (Temporary)
| Parameter | Type | Values Used |
|-----------|------|-------------|
| `grip_weight` | int | `100` (default), user-configurable |
| `initial_positioning` | bool | `true` |

---

## 6. Unused Resources in ros2_system

The following packages/nodes exist in ros2_system but are **not** used by the modular launch scripts:

### Unused Packages
| Package | Location | Notes |
|---------|----------|-------|
| `perception_module` | `/ros2_system/src/perception_module/` | Contains real perception (YOLO-based), could be used with `--real-perception` flag (not yet implemented) |
| `planning_module` | `/ros2_system/src/planning_module/` | Contains verification and integrity nodes |
| `util_arduino_serial` | `/ros2_system/src/util_arduino_serial/` | Arduino communication utilities |

### Unused Nodes from Used Packages
| Node | Package | Source File | Notes |
|------|---------|-------------|-------|
| `brain_node` | supervisor_module | `src/supervisor_module/src/brain_node.cpp` | C++ version of sorting brain |
| `pick_operation_node` | control_module | `src/control_module/src/pick_operation_node.cpp` | Pick action server |
| `place_operation_node` | control_module | `src/control_module/src/place_operation_node.cpp` | Place action server |
| `robot_driver_node` | control_module | `src/control_module/src/robot_driver_node.cpp` | Custom robot driver |
| `verification_node` | planning_module | `src/planning_module/src/verification_node.cpp` | Weight verification |
| `integrity_node` | planning_module | `src/planning_module/src/integrity_node.cpp` | System integrity checks |
| `weight_detector_py.py` | weight_detection_module | Python version of weight detector |
| `object_detect_yolo.py` | perception_module | YOLO-based object detection |
| `system_dashboard.py` | supervisor_module | System dashboard UI |

### Unused Launch Files
| Launch File | Package | Location | Notes |
|-------------|---------|----------|-------|
| `sorting_system.launch.py` | supervisor_module | `src/supervisor_module/launch/` | All-in-one launch file |

### Unused Custom Messages/Services/Actions
All custom messages, services, and actions in `sort_interfaces` are used **indirectly** by the nodes that are launched. The scripts themselves don't directly publish/subscribe to these, but the nodes they launch do.

#### Messages (in sort_interfaces)
- `TargetArea.msg`
- `EnvironmentStatus.msg`
- `DetectedObjects.msg`
- `WeightEstimate.msg`
- `ForceFeedback.msg`
- `SortDecision.msg`
- `BoundingBox.msg`

#### Services (in sort_interfaces)
- `MoveToCartesian.srv`
- `CalibrateBaseline.srv`
- `SystemCommand.srv`
- `CalibrateGripper.srv`
- `MoveToJointPosition.srv`
- `ValidateWorkspace.srv`
- `GripperControl.srv`

#### Actions (in sort_interfaces)
- `PlaceObject.action`
- `VerifyWeight.action`
- `PlanTrajectory.action`
- `PickObject.action`
- `MoveToCartesian.action`

---

## 7. Real Perception Implementation Status

The `--real-perception` flag in `start_temporary.sh` is **not yet implemented**.

### Current Status
```bash
# Line 141-153 in start_temporary.sh
if [ "$SIM_PERCEPTION" = true ]; then
    # Simulated perception is implemented ✅
    ros2 run supervisor_module simulated_perception_node ...
else
    # Real perception is NOT implemented ❌
    echo "  WARNING: Real perception not yet configured!"
fi
```

### Available for Real Perception
| Resource | Location | Status |
|----------|----------|--------|
| `object_detect_yolo.py` | `/ros2_system/src/perception_module/perception_module/` | ✅ Available |

### To Implement Real Perception
You would need to add something like:
```bash
ros2 run perception_module object_detect_yolo &
```

---

## 8. Verification Checklist

✅ All launch files exist
✅ All URDF/Xacro files exist
✅ All C++ nodes are compiled
✅ All Python nodes are installed
✅ All scripts are executable
✅ All ROS2 standard messages available
✅ All custom interfaces compiled
✅ External dependencies documented
⚠️ Real perception not yet implemented
⚠️ Some packages unused but available

---

## 9. Dependencies Graph

```
start_persistent.sh
├── ur_robot_driver (external)
│   └── ur_control.launch.py
├── motion_control_module
│   ├── ur5e_moveit_with_gripper.launch.py
│   ├── ur5e_with_end_effector.urdf.xacro
│   ├── cartesian_controller_node
│   └── safety_boundary_collision.py
└── control_module
    └── gripper_controller_node

start_temporary.sh
├── motion_control_module
│   ├── go_home
│   ├── simple_pick_and_weigh_node
│   └── check_simulated_positions.py
├── supervisor_module
│   ├── simulated_perception_node
│   └── sorting_brain_node
├── weight_detection_module
│   └── weight_detector
└── ros2_system
    └── plot_weight.sh
```

---

## 10. Recommendations

1. **Real Perception Implementation**: Implement the real perception mode using the existing `object_detect_yolo.py` from `perception_module`

2. **Consider Using Unused Nodes**:
   - `verification_node` for weight verification
   - `integrity_node` for system health checks
   - `system_dashboard.py` for UI control

3. **Package Cleanup**: Consider if unused packages should be:
   - Removed from the workspace
   - Documented as "future use"
   - Integrated into the modular launch system

4. **Documentation**: Update README.md to note that real perception is not yet implemented

---

**Generated:** 2025-12-03
**Workspace:** `/home/mtrn/Documents/mtrn4231_jakos/ros2_system`
**Scripts Analyzed:** `start_persistent.sh`, `start_temporary.sh`
