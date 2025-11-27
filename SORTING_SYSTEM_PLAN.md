# Weight Sorting System - Implementation Plan

## Overview

This plan describes the implementation of a closed-loop weight sorting system for the UR5e robot. The system picks weights from a picking area, calibrates them, and places them in a sorted line in the placing area.

## Current System State

### What Already Exists

| Component | Status | Notes |
|-----------|--------|-------|
| `CartesianControllerNode` | ✅ Complete | `/motion_control/move_to_cartesian` service |
| `GripperControllerNode` | ✅ Complete | Lifecycle node with simulation mode, `/motion_control/gripper_control` service |
| `BrainNode` | ✅ Partial | Central orchestrator, can send movement commands, monitors nodes |
| `PickOperationNode` | ⚠️ Skeleton | Has state machine but movements are simulated (TODOs) |
| `PlaceOperationNode` | ⚠️ Skeleton | Has state machine but movements are simulated (TODOs) |
| `pick_and_place_demo.py` | ✅ Reference | Contains correct Z heights and area definitions |

### Defined Zones (from pick_and_place_demo.py)
```
Picking Area:  X: -787mm to -420mm, Y: 50mm to -252mm
Placing Area:  X: -787mm to -420mm, Y: -252mm to -391mm

Z Heights (tool0 frame):
  - Home Z: 371mm
  - Descend Z: 212mm
  - Pickup/Place Z: 182mm

Default Orientation: rx=2.221, ry=2.221, rz=0.0
```

### Topic/Service Interface Summary

| Interface | Type | Purpose |
|-----------|------|---------|
| `/motion_control/move_to_cartesian` | Service | Move robot to position |
| `/motion_control/gripper_control` | Service | Open/close gripper |
| `/motion_control/use_gripper_tip` | Service | Toggle gripper_tip vs tool0 frame |
| `/perception/detected_objects` | Topic | Objects detected by Kevin's perception |
| `/recognition/estimated_weights` | Topic | Weight estimates from Asad's calibration |
| `/control/pick_object` | Action | Pick operation |
| `/control/place_object` | Action | Place operation |

---

## Implementation Tasks

### Phase 1: Fix Pick and Place Operations (Update existing nodes)

**Task 1.1: Update PickOperationNode to use CartesianController**

Current state: `pick_operation_node.cpp` has skeleton code with `sleep()` calls simulating motion.

Changes needed:
1. Add service client for `/motion_control/move_to_cartesian`
2. Replace `move_to_approach_position()` → call cartesian service to move to (target_x, target_y, Z_DESCEND)
3. Replace `move_to_grasp_position()` → call cartesian service to move to (target_x, target_y, Z_PICKUP)
4. Replace `lift_object()` → call cartesian service to move back to Z_DESCEND
5. Use gripper service instead of topic for more reliable control

**Task 1.2: Update PlaceOperationNode to use CartesianController**

Current state: `place_operation_node.cpp` has skeleton code with `sleep()` calls.

Changes needed:
1. Add service client for `/motion_control/move_to_cartesian`
2. Replace `move_to_approach_position()` → call cartesian service
3. Replace `lower_to_place_position()` → call cartesian service to Z_PLACE
4. Replace `retreat_from_object()` → call cartesian service back to Z_DESCEND

---

### Phase 2: Create Sorting Brain Node (New Python node)

**File: `ros2_system/src/supervisor_module/scripts/sorting_brain_node.py`**

This is the "brain" that orchestrates the full sorting cycle.

**State Machine:**
```
IDLE → WAITING_FOR_DETECTION → PICKING → WEIGHING → DECIDING_PLACEMENT →
  → (if rearrangement needed) REARRANGING → PLACING → return to IDLE
```

**Key Responsibilities:**
1. Subscribe to `/perception/detected_objects` (from Kevin)
2. Select a weight to pick based on position in picking area
3. Call `/control/pick_object` action
4. Wait for weight estimate from `/recognition/estimated_weights` (from Asad)
5. Decide placement position based on weight sorting algorithm
6. Check if rearrangement is needed (shift existing weights)
7. Call `/control/place_object` action
8. Maintain memory of placed weights and their positions

**Sorting Algorithm:**
- Weights placed left-to-right in ascending order
- If new weight needs to be inserted in the middle:
  1. Drop current weight temporarily
  2. Move heavier weights right to make space
  3. Return to pick up the dropped weight
  4. Place in correct position

**Memory Structure:**
```python
placed_weights = [
    {"position": (x, y), "weight": 150.0, "id": 1},
    {"position": (x, y), "weight": 200.0, "id": 2},
    ...
]
```

---

### Phase 3: Perception Integration (Interface with Kevin's node)

**Expected Interface from Kevin:**
- Topic: `/perception/detected_objects`
- Message: `DetectedObjects` containing array of `BoundingBox`

**Required Transforms:**
Kevin's perception likely outputs in camera frame. Need:
1. Camera-to-base calibration (TF or manual)
2. Function to convert bounding box center to robot coordinates (x_mm, y_mm)

**Interface Adapter (if needed):**
Create `perception_adapter_node.py` that:
1. Subscribes to Kevin's output
2. Transforms to robot base_link frame
3. Filters objects within picking area bounds
4. Publishes on `/sorting/available_weights` with robot coordinates

---

### Phase 4: Weight Calibration Integration (Interface with Asad's node)

**Expected Interface from Asad:**
- Topic: `/recognition/estimated_weights`
- Message: `WeightEstimate` with fields: `object_id`, `estimated_weight`, `confidence`

**Integration Points:**
1. After picking and lifting to Z_DESCEND, sorting_brain waits for weight estimate
2. Timeout handling if no estimate received
3. Weight estimate used to determine sort position

**If Asad's node isn't ready:**
- Use `force_feedback` from gripper as fallback
- GripperControllerNode already publishes `/motion_control/force_feedback` with `measured_weight`

---

### Phase 5: Simulation Testing

**Setup for Fake Hardware Testing:**

1. **Modify `setupFakeur5e.sh`** to launch full system:
   ```bash
   # After moveit launches, add:
   ros2 launch supervisor_module sorting_system.launch.py simulation:=true
   ```

2. **Create `sorting_system.launch.py`:**
   - Launch `sorting_brain_node`
   - Launch `pick_operation_node`
   - Launch `place_operation_node`
   - Launch `gripper_controller_node` with `simulation_mode:=true`

3. **Simulated Perception Node:**
   Create `simulated_perception_node.py` that publishes fake detected objects at predefined positions in the picking area for testing.

4. **Simulated Weight Estimation:**
   Use gripper's built-in simulation mode which returns random weights (140-160g) when gripper is closed.

---

## File Changes Summary

### New Files to Create

| File | Purpose |
|------|---------|
| `supervisor_module/scripts/sorting_brain_node.py` | Main sorting orchestrator |
| `supervisor_module/scripts/simulated_perception_node.py` | Fake perception for testing |
| `supervisor_module/launch/sorting_system.launch.py` | Launch all sorting nodes |

### Files to Modify

| File | Changes |
|------|---------|
| `control_module/src/pick_operation_node.cpp` | Add cartesian service calls |
| `control_module/src/place_operation_node.cpp` | Add cartesian service calls |
| `4231_scripts/setupFakeur5e.sh` | Launch sorting system |

---

## Node Communication Diagram

```
                    ┌─────────────────────┐
                    │   sorting_brain     │
                    │   (orchestrator)    │
                    └─────────────────────┘
                            │
        ┌───────────────────┼───────────────────┐
        │                   │                   │
        ▼                   ▼                   ▼
┌───────────────┐   ┌───────────────┐   ┌───────────────┐
│  perception   │   │    Asad's     │   │   motion      │
│   (Kevin)     │   │  calibration  │   │   control     │
└───────────────┘   └───────────────┘   └───────────────┘
        │                   │                   │
        │                   │                   ▼
        │                   │           ┌───────────────┐
        │                   │           │   gripper     │
        │                   │           │  controller   │
        │                   │           └───────────────┘
        │                   │
        ▼                   ▼
/perception/          /recognition/
detected_objects      estimated_weights
```

---

## Immediate Next Steps

1. **Start with sorting_brain_node.py** - The main orchestrator
2. **Update pick_operation_node.cpp** - Connect to cartesian controller
3. **Update place_operation_node.cpp** - Connect to cartesian controller
4. **Create simulated_perception_node.py** - For testing without camera
5. **Create launch file** - To start all nodes together
6. **Test in simulation** - Using setupFakeur5e.sh

---

## Questions for Team

1. **Kevin (Perception):** What coordinate frame does your detection output use? Do you provide object center coordinates or just bounding boxes?

2. **Asad (Calibration):** What topic name will you publish weight estimates on? How long after the robot holds still should we expect the estimate?

3. **General:** What spacing should we use between placed weights in the placing area? (Suggest 40-50mm)
