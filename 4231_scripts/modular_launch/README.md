# Modular Launch System

A flexible, modular approach to launching your ROS2 robotic sorting system. This system separates persistent (long-running) nodes from temporary (one-off) nodes for better control and flexibility.

## Overview

### Two-Script System

1. **`start_persistent.sh`** - Launches long-running infrastructure nodes
   - UR5e Driver
   - MoveIt
   - Safety Visualizer
   - Gripper Controller
   - Cartesian Controller

2. **`start_temporary.sh`** - Launches task-specific temporary nodes
   - Go Home action
   - Perception (Simulated or Real)
   - Weight Detection (Simulated or Real)
   - Position Check
   - Application Mode (Sorting System or Simple Pick & Weigh)

## Quick Start

### Basic Workflow

1. **Start persistent nodes first:**
   ```bash
   cd 4231_scripts/modular_launch
   ./start_persistent.sh --real-robot --robot-ip 192.168.0.100
   ```

2. **Then start temporary nodes:**
   ```bash
   ./start_temporary.sh --mode sorting --sim-perception --position-check --autorun
   ```

3. **To stop:**
   - Press `Ctrl+C` in each terminal to cleanly shutdown

## Persistent Nodes Script

### Usage
```bash
./start_persistent.sh [OPTIONS]
```

### Options

| Option | Description |
|--------|-------------|
| `--sim-robot` | Use simulated robot (fake hardware) |
| `--real-robot` | Use real robot (default) |
| `--robot-ip IP` | Robot IP address (default: 192.168.0.100) |
| `--no-rviz` | Don't launch RViz with MoveIt |
| `--no-safety` | Don't launch Safety Visualizer |
| `--step` | Pause before each node launch (for debugging) |
| `--help` | Show help message |

### Examples

**Real robot with RViz:**
```bash
./start_persistent.sh --real-robot --robot-ip 192.168.0.100
```

**Simulated robot without RViz:**
```bash
./start_persistent.sh --sim-robot --no-rviz
```

**Real robot without safety visualizer:**
```bash
./start_persistent.sh --real-robot --no-safety
```

## Temporary Nodes Script

### Usage
```bash
./start_temporary.sh [OPTIONS]
```

### Options

| Option | Description |
|--------|-------------|
| `--mode MODE` | Application mode: `sorting` or `simple` (default: sorting) |
| `--sim-perception` | Use simulated perception |
| `--real-perception` | Use real perception (camera-based) |
| `--sim-weight` | Use simulated weight detection (default) |
| `--real-weight` | Use real weight detection |
| `--no-weight` | Skip weight detection entirely |
| `--go-home` | Run go_home action before other nodes |
| `--position-check` | Run position check (for simulated perception) |
| `--autorun` | Auto-start application after launch |
| `--grip-weight GRAMS` | Weight for gripper angle (simple mode only, default: 100) |
| `--help` | Show help message |

### Application Modes

#### Sorting Mode (`--mode sorting`)
Launches the full sorting system with the sorting brain node. This mode:
- Runs the sorting brain node
- Can use simulated or real perception
- Can use simulated or real weight detection
- Supports autorun to automatically start sorting

#### Simple Pick & Weigh Mode (`--mode simple`)
Runs a single pick and weigh cycle. This mode:
- Runs the simple pick and weigh node (blocking)
- Requires real weight detection (`--real-weight`)
- Can optionally use simulated perception for testing
- Supports custom grip weight parameter

### Examples

**Sorting with simulated perception:**
```bash
./start_temporary.sh --mode sorting --sim-perception --position-check --autorun
```

**Sorting with real perception and weight:**
```bash
./start_temporary.sh --mode sorting --real-perception --real-weight --go-home
```

**Simple pick and weigh with real weight:**
```bash
./start_temporary.sh --mode simple --real-weight --go-home --grip-weight 150
```

**Simple pick and weigh with simulated perception (testing):**
```bash
./start_temporary.sh --mode simple --sim-perception --sim-weight --go-home
```

## Common Use Cases

### 1. Full Simulation (Sorting System)
```bash
# Terminal 1: Start persistent nodes in simulation
./start_persistent.sh --sim-robot

# Terminal 2: Start sorting with simulated perception
./start_temporary.sh --mode sorting --sim-perception --autorun
```

### 2. Hybrid Mode (Real Robot + Simulated Perception)
```bash
# Terminal 1: Start persistent nodes on real robot
./start_persistent.sh --real-robot --robot-ip 192.168.0.100

# Terminal 2: Start sorting with simulated perception and position check
./start_temporary.sh --mode sorting --sim-perception --go-home --position-check --autorun
```

### 3. Full Real System (Sorting System)
```bash
# Terminal 1: Start persistent nodes on real robot
./start_persistent.sh --real-robot --robot-ip 192.168.0.100

# Terminal 2: Start sorting with real perception and weight
./start_temporary.sh --mode sorting --real-perception --real-weight --go-home
```

### 4. Simple Pick and Weigh (Real System)
```bash
# Terminal 1: Start persistent nodes on real robot
./start_persistent.sh --real-robot --robot-ip 192.168.0.100

# Terminal 2: Run simple pick and weigh
./start_temporary.sh --mode simple --real-weight --go-home --grip-weight 100
```

### 5. Testing Position Calibration
```bash
# Terminal 1: Start persistent nodes (sim or real)
./start_persistent.sh --sim-robot

# Terminal 2: Just run position check with simulated perception
./start_temporary.sh --mode sorting --sim-perception --go-home --position-check
```

## Configuration Matrix

### Robot Mode
| Script | Simulation | Real |
|--------|-----------|------|
| Persistent | `--sim-robot` | `--real-robot` |

### Perception Mode
| Script | Simulation | Real |
|--------|-----------|------|
| Temporary | `--sim-perception` | `--real-perception` |

### Weight Detection Mode
| Script | Simulation | Real | Disabled |
|--------|-----------|------|----------|
| Temporary | `--sim-weight` | `--real-weight` | `--no-weight` |

### Application Mode
| Mode | Flag | Description |
|------|------|-------------|
| Sorting System | `--mode sorting` | Full sorting system with sorting brain |
| Simple Pick & Weigh | `--mode simple` | Single pick and weigh cycle |

## Tips

1. **Always start persistent nodes first** - These provide the infrastructure needed by temporary nodes

2. **Use step mode for debugging** - Add `--step` to pause before each node launch

3. **Position check is recommended** - When using simulated perception, run `--position-check` to verify robot positions

4. **Simulated perception requires position check** - The first time you use simulated perception, run the position check to ensure coordinates are correct

5. **Simple mode requires real weight** - The simple pick and weigh mode is designed for real weight detection

6. **Clean shutdown** - Always use `Ctrl+C` to cleanly shutdown nodes rather than killing terminals

## Troubleshooting

### Persistent nodes won't start
- Check that no other instances are running: `ros2 node list`
- Ensure ROS2 workspace is built: `cd ~/Documents/mtrn4231_jakos/ros2_system && colcon build`

### Robot won't connect (Real mode)
- Verify robot IP address is correct
- Ensure `ros.urp` is loaded on teach pendant
- Check network connection to robot

### Weight detection not working
- Verify weight sensor is connected and powered
- Check topic: `ros2 topic echo /weight/raw`
- Restart weight detection node

### Sorting brain not responding
- Check sorting command topic: `ros2 topic pub --once /sorting/command std_msgs/msg/String "data: 'start'"`
- Verify all required nodes are running: `ros2 node list`
- Check logs for errors

## Node Overview

### Persistent Nodes
| Node | Purpose | Required |
|------|---------|----------|
| UR Driver | Robot control | Yes |
| MoveIt | Motion planning | Yes |
| Safety Visualizer | Collision detection visualization | Optional |
| Gripper Controller | Gripper control | Yes |
| Cartesian Controller | Cartesian motion control | Yes |

### Temporary Nodes
| Node | Purpose | Modes |
|------|---------|-------|
| Go Home | Move robot to home position | Both |
| Simulated Perception | Simulated object detection | Both |
| Weight Detection | Real weight measurement | Both (real mode) |
| Position Check | Verify simulated positions | Sorting (with sim perception) |
| Sorting Brain | Sorting system control | Sorting |
| Simple Pick & Weigh | Single cycle pick and weigh | Simple |

## Advanced Usage

### Custom grip weight for simple mode
```bash
./start_temporary.sh --mode simple --real-weight --grip-weight 250
```

### Running without safety visualizer (less resource intensive)
```bash
./start_persistent.sh --real-robot --no-safety
```

### Testing with step-by-step launch
```bash
./start_persistent.sh --sim-robot --step
```

### Sorting with manual control (no autorun)
```bash
# Launch nodes
./start_temporary.sh --mode sorting --sim-perception --position-check

# Then manually start sorting when ready
ros2 topic pub --once /sorting/command std_msgs/msg/String "data: 'start'"
```

## Migration from Old Scripts

### Old: `runHybridModular.sh`
**New equivalent:**
```bash
# Terminal 1
./start_persistent.sh --real-robot --robot-ip 192.168.0.100

# Terminal 2
./start_temporary.sh --mode sorting --sim-perception --position-check --autorun
```

### Old: `runSimplePickAndWeigh.sh`
**New equivalent:**
```bash
# Terminal 1
./start_persistent.sh --real-robot --robot-ip 192.168.0.100

# Terminal 2
./start_temporary.sh --mode simple --real-weight --go-home --grip-weight 100
```

### Old: `runSimulationModular.sh`
**New equivalent:**
```bash
# Terminal 1
./start_persistent.sh --sim-robot

# Terminal 2
./start_temporary.sh --mode sorting --sim-perception --autorun
```

---

**Note:** These scripts are designed to work together. Always start the persistent nodes before launching temporary nodes.
