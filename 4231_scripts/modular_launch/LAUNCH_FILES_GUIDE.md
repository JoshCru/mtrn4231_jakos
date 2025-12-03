# ROS 2 Launch Files Guide

## Overview

The modular shell scripts have been converted to **proper ROS 2 launch files** located in `/ros2_system/launch/`. This is the recommended way to launch ROS 2 systems.

### Benefits of Launch Files over Shell Scripts:
✅ Better dependency management
✅ Automatic node lifecycle management
✅ Parameter configuration in one place
✅ Event-driven execution (wait for nodes to be ready)
✅ Standard ROS 2 practice
✅ Easier to debug and maintain

---

## Launch Files Available

### 1. `persistent_nodes.launch.py`
Launches long-running infrastructure nodes:
- UR Driver
- MoveIt
- Safety Visualizer
- Gripper Controller (lifecycle node)
- Cartesian Controller

### 2. `temporary_nodes.launch.py`
Launches task-specific temporary nodes:
- Go Home
- Perception (Simulated/Real)
- Weight Detection (Real/Simulated/None)
- Position Check
- Application Mode (Sorting/Simple Pick & Weigh)

---

## Quick Start

### Basic Workflow

**1. Launch persistent nodes:**
```bash
ros2 launch persistent_nodes.launch.py
```

**2. Launch temporary nodes:**
```bash
ros2 launch temporary_nodes.launch.py mode:=sorting sim_perception:=true
```

---

## Persistent Nodes Launch File

### Location
```
/home/mtrn/Documents/mtrn4231_jakos/ros2_system/launch/persistent_nodes.launch.py
```

### Usage
```bash
ros2 launch persistent_nodes.launch.py [OPTIONS]
```

### Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `sim_robot` | bool | false | Use simulated robot (fake hardware) |
| `robot_ip` | string | 192.168.0.100 | Robot IP address |
| `launch_rviz` | bool | true | Launch RViz with MoveIt |
| `launch_safety` | bool | true | Launch Safety Visualizer |

### Examples

**Real robot with RViz:**
```bash
ros2 launch persistent_nodes.launch.py robot_ip:=192.168.0.100
```

**Simulated robot without RViz:**
```bash
ros2 launch persistent_nodes.launch.py sim_robot:=true launch_rviz:=false
```

**Real robot without safety visualizer:**
```bash
ros2 launch persistent_nodes.launch.py launch_safety:=false
```

---

## Temporary Nodes Launch File

### Location
```
/home/mtrn/Documents/mtrn4231_jakos/ros2_system/launch/temporary_nodes.launch.py
```

### Usage
```bash
ros2 launch temporary_nodes.launch.py [OPTIONS]
```

### Options

| Option | Type | Default | Description |
|--------|------|---------|-------------|
| `mode` | string | sorting | Application mode: `sorting` or `simple` |
| `sim_perception` | bool | false | Use simulated perception |
| `real_weight` | bool | false | Use real weight detection |
| `sim_weight` | bool | true | Use simulated weight detection |
| `go_home` | bool | false | Run go_home action first |
| `position_check` | bool | false | Run position check (for sim perception) |
| `autorun` | bool | false | Auto-start application |
| `grip_weight` | int | 100 | Grip weight for simple mode (grams) |

### Examples

**Sorting with simulated perception:**
```bash
ros2 launch temporary_nodes.launch.py \
    mode:=sorting \
    sim_perception:=true \
    position_check:=true \
    autorun:=true
```

**Sorting with real perception and weight:**
```bash
ros2 launch temporary_nodes.launch.py \
    mode:=sorting \
    real_weight:=true \
    go_home:=true
```

**Simple pick and weigh:**
```bash
ros2 launch temporary_nodes.launch.py \
    mode:=simple \
    real_weight:=true \
    go_home:=true \
    grip_weight:=150
```

---

## Common Use Cases

### 1. Full Simulation (Sorting System)
```bash
# Terminal 1: Persistent nodes
ros2 launch persistent_nodes.launch.py sim_robot:=true

# Terminal 2: Temporary nodes
ros2 launch temporary_nodes.launch.py \
    mode:=sorting \
    sim_perception:=true \
    autorun:=true
```

### 2. Hybrid Mode (Real Robot + Simulated Perception)
```bash
# Terminal 1: Persistent nodes
ros2 launch persistent_nodes.launch.py robot_ip:=192.168.0.100

# Terminal 2: Temporary nodes
ros2 launch temporary_nodes.launch.py \
    mode:=sorting \
    sim_perception:=true \
    sim_weight:=true \
    go_home:=true \
    position_check:=true \
    autorun:=true
```

### 3. Full Real System
```bash
# Terminal 1: Persistent nodes
ros2 launch persistent_nodes.launch.py robot_ip:=192.168.0.100

# Terminal 2: Temporary nodes
ros2 launch temporary_nodes.launch.py \
    mode:=sorting \
    real_weight:=true \
    go_home:=true
```

### 4. Simple Pick and Weigh
```bash
# Terminal 1: Persistent nodes
ros2 launch persistent_nodes.launch.py robot_ip:=192.168.0.100

# Terminal 2: Simple mode
ros2 launch temporary_nodes.launch.py \
    mode:=simple \
    real_weight:=true \
    go_home:=true \
    grip_weight:=100
```

---

## Advantages Over Shell Scripts

### 1. **Better Timing Control**
Launch files use `TimerAction` to wait for nodes to be ready:
```python
# Wait 10 seconds for UR driver before launching MoveIt
moveit_launch = TimerAction(
    period=10.0,
    actions=[IncludeLaunchDescription(...)]
)
```

### 2. **Lifecycle Node Management**
Automatic lifecycle transitions for gripper controller:
```python
# Launch lifecycle node
LifecycleNode(package='control_package', ...)

# Auto-configure
ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/gripper_controller_node', 'configure'])

# Auto-activate
ExecuteProcess(cmd=['ros2', 'lifecycle', 'set', '/gripper_controller_node', 'activate'])
```

### 3. **Conditional Execution**
Only launch nodes when needed:
```python
if sim_perception.lower() == 'true':
    actions.append(Node(...))
```

### 4. **Parameter Management**
Parameters configured in one place:
```python
Node(
    package='supervisor_package',
    executable='simulated_perception_node',
    parameters=[{
        'num_objects': 4,
        'publish_rate': 5.0,
        'randomize_positions': True
    }]
)
```

### 5. **Event Handlers**
React to node events:
```python
RegisterEventHandler(
    OnProcessStart(
        target_action=ur_driver_launch,
        on_start=[LogInfo(msg='UR5e Driver started.')]
    )
)
```

---

## Shell Scripts Still Available

The original shell scripts are still available in:
```
/home/mtrn/Documents/mtrn4231_jakos/4231_scripts/modular_launch/
├── start_persistent.sh
└── start_temporary.sh
```

Use them if you prefer shell scripts or need quick testing.

---

## Migration from Shell Scripts

### Old Shell Script Approach
```bash
./start_persistent.sh --real-robot --robot-ip 192.168.0.100
./start_temporary.sh --mode sorting --sim-perception --position-check --autorun
```

### New Launch File Approach
```bash
ros2 launch persistent_nodes.launch.py robot_ip:=192.168.0.100
ros2 launch temporary_nodes.launch.py mode:=sorting sim_perception:=true position_check:=true autorun:=true
```

---

## Debugging Launch Files

### View launch file arguments:
```bash
ros2 launch persistent_nodes.launch.py --show-args
ros2 launch temporary_nodes.launch.py --show-args
```

### Print launch description:
```bash
ros2 launch persistent_nodes.launch.py --print-description
```

### Launch with debug output:
```bash
ros2 launch persistent_nodes.launch.py --debug
```

---

## Best Practices

1. **Always source workspace first:**
   ```bash
   source /opt/ros/humble/setup.bash
   source ~/Documents/mtrn4231_jakos/ros2_system/install/setup.bash
   ```

2. **Use separate terminals** for persistent and temporary nodes for better control

3. **Check node status:**
   ```bash
   ros2 node list
   ros2 lifecycle list /gripper_controller_node
   ```

4. **Monitor topics:**
   ```bash
   ros2 topic list
   ros2 topic echo /sorting/command
   ```

5. **Stop cleanly** with `Ctrl+C` in each terminal

---

## Troubleshooting

### Launch file not found
```bash
# Rebuild workspace
cd ~/Documents/mtrn4231_jakos/ros2_system
colcon build --symlink-install
source install/setup.bash
```

### Nodes not starting
```bash
# Check if persistent nodes are running first
ros2 node list

# Verify UR driver is connected (real robot)
ros2 control list_controllers
```

### Timing issues
- Increase `TimerAction` periods in launch file if nodes start too quickly
- Default: UR driver (10s), MoveIt (20s), Controllers (22-28s)

---

## Summary

**Use ROS 2 Launch Files** (`persistent_nodes.launch.py` and `temporary_nodes.launch.py`) as the primary method to launch your system. They provide better control, timing, and integrate properly with the ROS 2 ecosystem.

The shell scripts remain available as a fallback or for quick testing.

---

**Created:** 2025-12-03
**Location:** `/home/mtrn/Documents/mtrn4231_jakos/ros2_system/launch/`
