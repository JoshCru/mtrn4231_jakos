# Cleanup Plan for ros2_system

This document identifies which files can be safely removed from ros2_system based on usage in all .sh scripts.

**NOTE: perception_module is KEPT intact - Kevin is still working on it**

---

## Files TO KEEP (Used by shell scripts)

### control_module
✅ **KEEP:**
- `src/gripper_controller_node.cpp` - Used by multiple scripts
- `config/gripper_params.yaml` - Likely used by gripper_controller_node
- `include/simple_serial.hpp` - Dependency for gripper

❌ **REMOVE:**
- `src/gripper_button_interface.cpp` - NOT used in any script
- `src/pick_operation_node.cpp` - NOT used in any script
- `src/place_operation_node.cpp` - NOT used in any script
- `src/robot_driver_node.cpp` - NOT used in any script
- `config/control.yaml` - NOT referenced
- `launch/control.launch.py` - NOT used
- `launch/gripper_hardware.launch.py` - NOT used
- `launch/gripper_hardware_with_interface.launch.py` - NOT used
- `launch/gripper_sim.launch.py` - NOT used
- `launch/gripper_sim_with_interface.launch.py` - NOT used
- `test/test_gripper_hardware.py` - Test file
- `test/test_gripper_simulation.py` - Test file
- `urdf/gripper.urdf` - NOT used (we use xacro version)

### motion_control_module
✅ **KEEP:**
- `src/cartesian_controller_node.cpp` - Used by multiple scripts
- `src/simple_pick_and_weigh_node.cpp` - Used by runSimplePickAndWeigh.sh
- `scripts/go_home.py` - Used by multiple scripts
- `scripts/check_simulated_positions.py` - Used by hybrid/real modular scripts
- `scripts/safety_boundary_collision.py` - Used by multiple scripts
- `launch/ur5e_moveit_with_gripper.launch.py` - Used by multiple scripts
- `urdf/ur5e_with_end_effector.urdf.xacro` - Used by multiple scripts
- `urdf/end_effector.urdf.xacro` - Dependency of ur5e_with_end_effector.urdf.xacro
- `srdf/ur5e_with_gripper.srdf.xacro` - Used by MoveIt launch file
- `config/ur_controllers_with_home.yaml` - Likely used by launch files

❌ **REMOVE:**
- `scripts/simple_pick_and_weigh.py` - Python version NOT used (C++ version is used)
- `launch/ur5e_real_with_gripper.launch.py` - NOT used in any script
- `urdf/gripper.urdf` - NOT used (we use xacro in end_effector)

### perception_module
✅ **KEEP EVERYTHING** - Kevin is working on this

### planning_module
❌ **REMOVE ENTIRE PACKAGE:**
- `src/integrity_node.cpp` - NOT used in any script
- `src/verification_node.cpp` - NOT used in any script
- `src/moveit2_interface_node.cpp` - NOT used in any script
- `src/sort_node.cpp` - NOT used in any script
- `config/planning.yaml` - NOT used
- `launch/planning.launch.py` - NOT used

### supervisor_module
✅ **KEEP:**
- `supervisor_module/simulated_perception_node.py` - Used by multiple scripts
- `supervisor_module/sorting_brain_node.py` - Used by multiple scripts
- `supervisor_module/system_dashboard.py` - Used by launchDashboard.sh
- `supervisor_module/__init__.py` - Python package requirement
- `setup.py` - Python package requirement

❌ **REMOVE:**
- `include/supervisor_module/brain_node.hpp` - C++ version NOT used
- `config/system_controller.yaml` - NOT referenced
- `launch/sorting_system.launch.py` - NOT used (scripts use modular approach)

### util_arduino_serial
❌ **REMOVE ENTIRE PACKAGE:**
- `util_arduino_serial/util_arduino_serial.py` - NOT used in any script
- `util_arduino_serial/__init__.py`
- `setup.py`
- `test/test_copyright.py`
- `test/test_flake8.py`
- `test/test_pep257.py`

### weight_detection_module
✅ **KEEP:**
- `src/weight_detector.cpp` - Used by multiple scripts
- `weight_detection_module/__init__.py` - Python package requirement
- `setup.py` - Package requirement

❌ **REMOVE:**
- `weight_detection_module/old.py` - Old/unused code
- `weight_detection_module/weight_detector_py.py` - Python version used only in test scripts (keep if you want tests)
- `launch/weight_detection.launch.py` - NOT used

### sort_interfaces
✅ **KEEP ENTIRE PACKAGE** - Used indirectly by all nodes for communication

---

## Summary of Removals

### Complete Packages to Remove:
1. **planning_module** (entire directory)
2. **util_arduino_serial** (entire directory)

### Files to Remove from control_module:
```
control_module/
├── src/
│   ├── gripper_button_interface.cpp          ❌
│   ├── pick_operation_node.cpp                ❌
│   ├── place_operation_node.cpp               ❌
│   └── robot_driver_node.cpp                  ❌
├── config/
│   └── control.yaml                           ❌
├── launch/
│   ├── control.launch.py                      ❌
│   ├── gripper_hardware.launch.py             ❌
│   ├── gripper_hardware_with_interface.launch.py ❌
│   ├── gripper_sim.launch.py                  ❌
│   └── gripper_sim_with_interface.launch.py   ❌
├── test/
│   ├── test_gripper_hardware.py               ❌
│   └── test_gripper_simulation.py             ❌
└── urdf/
    └── gripper.urdf                           ❌
```

### Files to Remove from motion_control_module:
```
motion_control_module/
├── scripts/
│   └── simple_pick_and_weigh.py               ❌
├── launch/
│   └── ur5e_real_with_gripper.launch.py       ❌
└── urdf/
    └── gripper.urdf                           ❌
```

### Files to Remove from supervisor_module:
```
supervisor_module/
├── include/supervisor_module/
│   └── brain_node.hpp                         ❌
├── config/
│   └── system_controller.yaml                 ❌
└── launch/
    └── sorting_system.launch.py               ❌
```

### Files to Remove from weight_detection_module:
```
weight_detection_module/
├── weight_detection_module/
│   ├── old.py                                 ❌
│   └── weight_detector_py.py                  ❌ (optional - used in tests)
└── launch/
    └── weight_detection.launch.py             ❌
```

---

## Impact Analysis

### Files that will be removed: ~30 files
### Complete packages removed: 2 (planning_module, util_arduino_serial)
### Packages preserved: 6 (control_module, motion_control_module, perception_module, supervisor_module, weight_detection_module, sort_interfaces)

### Space savings estimate:
- planning_module: ~4 source files + configs
- util_arduino_serial: ~1 source file + package files
- Unused launch files: ~10 files
- Unused nodes: ~6 files
- Test files: ~5 files
- **Total:** ~30 files

---

## Recommendations

1. **Create a backup branch first** before removing anything
2. **Keep test files** if you want to preserve testing capability
3. **Keep weight_detector_py.py** if test scripts are important
4. **Document removed functionality** in case it's needed later

---

## Safety Notes

⚠️ **Before removing:**
1. Verify no CMakeLists.txt references these files
2. Check package.xml dependencies
3. Ensure no Python imports reference removed modules
4. Test build after removal: `colcon build`

---

**Generated:** 2025-12-03
**Status:** PENDING USER APPROVAL
