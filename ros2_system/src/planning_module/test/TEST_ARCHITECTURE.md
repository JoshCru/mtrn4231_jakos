# Test Architecture Diagram

## Overall Test System

```
┌─────────────────────────────────────────────────────────────────────┐
│                         TEST RUNNER (run_tests.sh)                  │
│  1. Build workspace                                                 │
│  2. Launch planning nodes                                           │
│  3. Execute pytest tests                                            │
│  4. Collect results                                                 │
│  5. Cleanup                                                         │
└─────────────────────────────────────────────────────────────────────┘
                                  │
                    ┌─────────────┴─────────────┐
                    ▼                           ▼
        ┌───────────────────┐       ┌───────────────────┐
        │   PLANNING NODES  │       │   TEST NODES      │
        │   (Production)    │       │   (Test Code)     │
        └───────────────────┘       └───────────────────┘
                    │                           │
        ┌───────────┼───────────┐              │
        ▼           ▼           ▼              ▼
    ┌────────┐ ┌────────┐ ┌────────┐    ┌──────────┐
    │sort_   │ │verify_ │ │integ_  │    │Test Node │
    │node    │ │node    │ │node    │    │Publishers│
    └────────┘ └────────┘ └────────┘    │Subscribers│
                                         │Clients   │
                                         └──────────┘
```

---

## Test Node Architecture

### Sort Node Test

```
┌──────────────────────────────────────────────────────────────┐
│                    SortNodeTestNode                          │
├──────────────────────────────────────────────────────────────┤
│  Publishers:                                                 │
│    • /recognition/estimated_weights (WeightEstimate)         │
│    • /system/target_areas (TargetArea)                       │
│                                                               │
│  Subscribers:                                                │
│    • /planning/sort_decisions (SortDecision)                 │
│                                                               │
│  Test Logic:                                                 │
│    1. Publish target areas (bins)                            │
│    2. Publish weight estimate                                │
│    3. Wait for sort decision                                 │
│    4. Assert correct bin selected                            │
└──────────────────────────────────────────────────────────────┘
                         │
                         ▼
        ┌────────────────────────────────┐
        │         sort_node              │
        │  Receives: WeightEstimate      │
        │  Receives: TargetArea          │
        │  Publishes: SortDecision       │
        └────────────────────────────────┘
```

**Test Flow:**
```
Test Node                              Sort Node
    │                                      │
    ├─── Publish TargetArea(id=1) ──────>│
    ├─── Publish TargetArea(id=2) ──────>│
    │                                     Store target areas
    │                                      │
    ├─── Publish WeightEstimate(100g) ──>│
    │                                     Match to bin
    │                                     Compute decision
    │<─── Publish SortDecision(bin=2) ────┤
    │                                      │
   Assert decision.target_area_id == 2
```

---

### Verification Node Test

```
┌──────────────────────────────────────────────────────────────┐
│              VerificationNodeTestNode                        │
├──────────────────────────────────────────────────────────────┤
│  Publishers:                                                 │
│    • /recognition/estimated_weights (WeightEstimate)         │
│    • /motion_control/force_feedback (ForceFeedback)          │
│                                                               │
│  Action Client:                                              │
│    • /planning/verify_weight (VerifyWeight)                  │
│                                                               │
│  Test Logic:                                                 │
│    1. Send VerifyWeight goal (estimated: 100g, tol: 10%)     │
│    2. Publish force feedback (actual: 105g)                  │
│    3. Wait for result                                        │
│    4. Assert verified == True (within tolerance)             │
└──────────────────────────────────────────────────────────────┘
                         │
                         ▼
        ┌────────────────────────────────┐
        │     verification_node          │
        │  Action Server: VerifyWeight   │
        │  Receives: ForceFeedback       │
        │  Returns: verified, error%     │
        └────────────────────────────────┘
```

**Test Flow (Action Server):**
```
Test Node                              Verification Node
    │                                      │
    ├─── send_goal(estimated=100g) ─────>│
    │<─── goal_accepted ───────────────────┤
    │                                     Wait for measurements
    │                                      │
    ├─── ForceFeedback(105g) ────────────>│
    ├─── ForceFeedback(105g) ────────────>│
    ├─── ForceFeedback(105g) ────────────>│
    │                                     Apply filter
    │                                     Compute error%
    │                                     error = |105-100|/100 = 5%
    │                                     5% < 10% → verified=True
    │<─── result(verified=True, error=5%) ─┤
    │                                      │
   Assert result.verified == True
```

---

### Integrity Node Test

```
┌──────────────────────────────────────────────────────────────┐
│               IntegrityNodeTestNode                          │
├──────────────────────────────────────────────────────────────┤
│  Publishers:                                                 │
│    • /camera/pointcloud (PointCloud2)                        │
│    • /system/status (String)                                 │
│                                                               │
│  Subscribers:                                                │
│    • /planning/environment_status (EnvironmentStatus)        │
│                                                               │
│  Service Client:                                             │
│    • /planning/validate_workspace (ValidateWorkspace)        │
│                                                               │
│  Test Logic:                                                 │
│    1. Create waypoint poses                                  │
│    2. Call ValidateWorkspace service                         │
│    3. Wait for response                                      │
│    4. Assert is_safe == True/False based on bounds           │
└──────────────────────────────────────────────────────────────┘
                         │
                         ▼
        ┌────────────────────────────────┐
        │      integrity_node            │
        │  Service: ValidateWorkspace    │
        │  Publishes: EnvironmentStatus  │
        │  Check: Workspace bounds       │
        └────────────────────────────────┘
```

**Test Flow (Service Call):**
```
Test Node                              Integrity Node
    │                                      │
    ├─── call_service(waypoints=[        │
    │      Pose(x=0.3, y=0.2, z=0.3),     │
    │      Pose(x=0.8, y=0.0, z=0.3)      │ ← x=0.8 > max(0.6)
    │    ]) ────────────────────────────>│
    │                                     Check each waypoint:
    │                                     - (0.3, 0.2, 0.3) ✓ valid
    │                                     - (0.8, 0.0, 0.3) ✗ x out of bounds
    │<─── response(is_safe=False, ────────┤
    │      warnings=["X out of bounds"])   │
    │                                      │
   Assert response.is_safe == False
```

**Test Flow (Periodic Status):**
```
Test Node                              Integrity Node
    │                                      │
    │                                     Timer (10 Hz)
    │<─── EnvironmentStatus ───────────────┤
    │     (is_safe, workspace_clear, ...)  │
    │                                      │
    │     Wait 2 seconds                   │
    │<─── EnvironmentStatus ───────────────┤
    │<─── EnvironmentStatus ───────────────┤
    │<─── EnvironmentStatus ───────────────┤
    │     ...                              │
    │                                      │
   Assert ~20 messages received (10 Hz × 2s)
```

---

### MoveIt2 Interface Node Test

```
┌──────────────────────────────────────────────────────────────┐
│           MoveIt2InterfaceTestNode                           │
├──────────────────────────────────────────────────────────────┤
│  Action Clients:                                             │
│    • /planning/plan_pick (PlanTrajectory)                    │
│    • /planning/plan_place (PlanTrajectory)                   │
│                                                               │
│  Subscribers:                                                │
│    • /planning/trajectory (DisplayTrajectory)                │
│                                                               │
│  Test Logic:                                                 │
│    1. Send PlanTrajectory goal (target pose)                 │
│    2. Wait for goal acceptance                               │
│    3. Wait for result                                        │
│    4. Assert response received (stub returns success=False)  │
└──────────────────────────────────────────────────────────────┘
                         │
                         ▼
        ┌────────────────────────────────┐
        │   moveit2_interface_node       │
        │  Action: /planning/plan_pick   │
        │  Action: /planning/plan_place  │
        │  (Stub implementation)         │
        └────────────────────────────────┘
```

**Test Flow:**
```
Test Node                              MoveIt2 Interface Node
    │                                      │
    ├─── send_goal(                       │
    │      target_pose=(0.3, 0.2, 0.15),  │
    │      planning_group="ur_manipulator"│
    │    ) ────────────────────────────>│
    │<─── goal_accepted ───────────────────┤
    │                                     (Stub: no actual planning)
    │<─── result(success=False, ──────────┤
    │      message="Not implemented")      │
    │                                      │
   Assert result is not None
   (Note: stub returns success=False - this is expected)
```

---

## Test Execution Flow

```
┌─────────────────────────────────────────────────────────────┐
│  1. TEST INITIALIZATION (setUpClass)                        │
│     • rclpy.init()                                          │
│     • Initialize ROS2 for all tests                         │
└─────────────────────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  FOR EACH TEST:                                             │
│                                                              │
│  2. TEST SETUP (setUp)                                      │
│     • Create test node                                      │
│     • Create publishers/subscribers/clients                 │
│     • Wait for connections (0.5s)                           │
│                                                              │
│  3. TEST EXECUTION (test_xxx)                               │
│     • Publish test inputs                                   │
│     • Wait for outputs (with timeout)                       │
│     • Assert expected behavior                              │
│                                                              │
│  4. TEST TEARDOWN (tearDown)                                │
│     • Destroy test node                                     │
│     • Clean up resources                                    │
└─────────────────────────────────────────────────────────────┘
                         │
                         ▼
┌─────────────────────────────────────────────────────────────┐
│  5. TEST SHUTDOWN (tearDownClass)                           │
│     • rclpy.shutdown()                                      │
│     • Final cleanup                                         │
└─────────────────────────────────────────────────────────────┘
```

---

## Message Flow Patterns

### Pattern 1: Pub/Sub (Sort Node)

```
┌──────────┐                    ┌──────────┐
│   Test   │ ─── Publish ──────>│   Node   │
│   Node   │                    │          │
│          │<─── Publish ────── │          │
└──────────┘                    └──────────┘

Characteristics:
• Asynchronous
• Fire-and-forget
• Multiple subscribers possible
• No guaranteed delivery
```

### Pattern 2: Action (Verification Node)

```
┌──────────┐                    ┌──────────┐
│   Test   │ ─── Goal ────────>│   Node   │
│   Node   │<─── Accepted ───── │          │
│          │<─── Feedback ───── │          │
│          │<─── Feedback ───── │          │
│          │<─── Result ─────── │          │
└──────────┘                    └──────────┘

Characteristics:
• Asynchronous (non-blocking)
• Goal, Feedback, Result
• Can be canceled
• Progress updates via feedback
```

### Pattern 3: Service (Integrity Node)

```
┌──────────┐                    ┌──────────┐
│   Test   │ ─── Request ─────>│   Node   │
│   Node   │                    │          │
│          │<─── Response ───── │          │
└──────────┘                    └──────────┘

Characteristics:
• Synchronous (blocking)
• Request-response pair
• Guaranteed response
• One client at a time
```

---

## Test Assertion Patterns

### 1. Value Assertions

```python
# Exact match
self.assertEqual(decision.target_area_id, 2)

# Range check
self.assertGreater(result.error_percentage, 10.0)
self.assertLess(result.error_percentage, 15.0)

# Approximate
self.assertAlmostEqual(weight, 100.0, delta=1.0)
```

### 2. Boolean Assertions

```python
# True/False
self.assertTrue(result.verified)
self.assertFalse(response.is_safe)

# Existence
self.assertIsNotNone(goal_handle)
self.assertIsInstance(status.is_safe, bool)
```

### 3. Collection Assertions

```python
# Size checks
self.assertEqual(len(decisions), 3)
self.assertGreater(len(warnings), 0)

# Membership
self.assertIn("out of bounds", response.warnings)
```

---

## Timing Diagram

```
Time (seconds)
0.0   Test starts
      │
0.0   │ setUpClass: rclpy.init()
      │
0.0   │ setUp: Create test node
      │ Create publishers/subscribers
0.5   │ Wait for connections
      │
0.5   │ Test execution begins
      │ Publish input messages
      │
0.6   │ Node processes messages
      │
0.7   │ Receive output messages
      │
0.7   │ Assert results
      │
0.7   │ tearDown: Destroy node
      │
0.7   │ Next test setUp
      │
...   │ (repeat for each test)
      │
45s   │ All tests complete
      │
45s   │ tearDownClass: rclpy.shutdown()
      │
45s   Test ends
```

---

## Error Handling Flow

```
┌─────────────────────┐
│  Test Execution     │
└─────────┬───────────┘
          │
          ▼
    ┌─────────┐
    │ Timeout?│────── Yes ──────> FAIL (timeout)
    └────┬────┘
         │ No
         ▼
    ┌─────────┐
    │ Exception?│──── Yes ──────> FAIL (error)
    └────┬────┘
         │ No
         ▼
    ┌─────────┐
    │ Assertion│──── Failed ────> FAIL (assertion)
    └────┬────┘
         │ Passed
         ▼
       PASS
```

---

## Test Coverage Map

```
┌────────────────────────────────────────────────────────┐
│                   PLANNING MODULE                      │
├────────────────────────────────────────────────────────┤
│                                                        │
│  ┌──────────────┐  ┌──────────────┐                  │
│  │  sort_node   │  │  verify_node │                  │
│  │  ✓ 7 tests   │  │  ✓ 10 tests  │                  │
│  │              │  │              │                  │
│  │ Topics: ✓    │  │ Topics: ✓    │                  │
│  │ Logic: ✓     │  │ Actions: ✓   │                  │
│  │ Bounds: ✓    │  │ Service: ✓   │                  │
│  └──────────────┘  └──────────────┘                  │
│                                                        │
│  ┌──────────────┐  ┌──────────────┐                  │
│  │integ_node    │  │moveit2_node  │                  │
│  │ ✓ 13 tests   │  │ ✓ 12 tests   │                  │
│  │              │  │              │                  │
│  │ Service: ✓   │  │ Actions: ✓   │                  │
│  │ Topics: ✓    │  │ Requests: ✓  │                  │
│  │ Bounds: ✓    │  │ Edges: ✓     │                  │
│  └──────────────┘  └──────────────┘                  │
│                                                        │
│  Total: 42 tests                                      │
│  Coverage: All interfaces tested                      │
└────────────────────────────────────────────────────────┘
```

---

## Summary

**Test Architecture:**
- ✅ Test nodes communicate with planning nodes via ROS2
- ✅ All message patterns tested (pub/sub, action, service)
- ✅ Comprehensive timing and timeout handling
- ✅ Proper setup/teardown for isolation
- ✅ 42 tests cover all node interfaces

**Key Patterns:**
- Integration tests verify real communication
- Tests are isolated (each has own setup/teardown)
- Timeouts prevent hanging tests
- Assertions verify expected behavior
