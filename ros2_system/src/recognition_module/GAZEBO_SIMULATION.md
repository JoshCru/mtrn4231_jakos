# Gazebo Simulation Guide

This guide explains how to use the Gazebo simulation for testing the recognition node with drag-and-drop objects.

## Overview

The simulation provides:
- **Gazebo world** with a workspace table and depth camera
- **Three object types** with accurate masses (50g, 100g, 200g)
- **Drag-and-drop** capability to manually place objects
- **Automatic point cloud generation** from simulated depth camera
- **Scripts to spawn objects** programmatically

---

## Quick Start

### 1. Build the Package

```bash
cd ~/Documents/mtrn4231_jakos/ros2_system
colcon build --packages-select recognition_module
source install/setup.bash
```

### 2. Launch the Simulation

```bash
ros2 launch recognition_module gazebo_simulation.launch.py
```

This will:
- Start Gazebo with the weight sorting world
- Launch the recognition node
- Set up the simulated depth camera

### 3. Spawn Objects

In a new terminal:

```bash
source ~/Documents/mtrn4231_jakos/ros2_system/install/setup.bash

# Spawn test set (one of each type)
ros2 run recognition_module spawn_objects.py --type test

# OR spawn random objects
ros2 run recognition_module spawn_objects.py --type random --count 5

# OR spawn a specific type at a specific position
ros2 run recognition_module spawn_objects.py --type bolt_50g --x 0.2 --y 0.1
```

### 4. Monitor Recognition Results

In another terminal:

```bash
# Watch weight estimates
ros2 topic echo /recognition/estimated_weights

# Monitor point cloud (check it's publishing)
ros2 topic hz /camera/pointcloud
```

---

## Simulation Components

### World Setup

- **Table**: 1.2m × 1.2m workspace at z=0
- **Camera**: Mounted 0.8m above table, looking down
- **Field of View**: ~60° (1.047 radians)
- **Resolution**: 640×480
- **Update Rate**: 10 Hz

### Object Models

#### bolt_50g
- **Mass**: 50 grams
- **Shape**: Cylinder (r=1cm, h=2cm)
- **Color**: Light gray
- **Material**: Stainless steel (8000 kg/m³)

#### part_100g
- **Mass**: 100 grams
- **Shape**: Cylinder (r=1.5cm, h=1.77cm)
- **Color**: Medium gray
- **Material**: Stainless steel

#### part_200g
- **Mass**: 200 grams
- **Shape**: Cylinder (r=2cm, h=2cm)
- **Color**: Dark gray
- **Material**: Stainless steel

---

## Using the Simulation

### Drag and Drop Objects

1. Launch the simulation (see Quick Start)
2. Spawn some objects
3. In Gazebo GUI:
   - Click the "move" tool (arrow icon) or press `T`
   - Click on an object to select it
   - Drag to move it around
   - Release to drop

The recognition node will continuously detect and estimate weights!

### Spawn Objects Programmatically

```bash
# Test set - one of each type in a row
ros2 run recognition_module spawn_objects.py --type test

# Random set - 10 random objects
ros2 run recognition_module spawn_objects.py --type random --count 10

# Single object at specific position
ros2 run recognition_module spawn_objects.py \
  --type part_100g \
  --x 0.3 \
  --y -0.2 \
  --z 0.05

# Single object at random position
ros2 run recognition_module spawn_objects.py --type bolt_50g
```

### Visualize in RViz

```bash
rviz2
```

Add displays:
1. **PointCloud2** → Topic: `/camera/pointcloud`
2. **TF** → Show transforms
3. **Marker** → For visualizing detected objects (if you add this later)

Set Fixed Frame to `camera_link`

---

## Recording Test Data

### Record a Bag File

```bash
# Start simulation and spawn objects first, then:

# Terminal 1: Record for 30 seconds
ros2 run recognition_module record_bag.py record --duration 30 --name gazebo_test_01

# OR use the shell script
ros2 run recognition_module record_test_data.sh
```

See [BAG_RECORDING.md](BAG_RECORDING.md) for detailed bag recording guide.

---

## Launch File Options

### Basic Launch

```bash
ros2 launch recognition_module gazebo_simulation.launch.py
```

### Launch Options

```bash
# Launch without GUI (headless, for automated testing)
ros2 launch recognition_module gazebo_simulation.launch.py gui:=false

# Use real time (not simulation time)
ros2 launch recognition_module gazebo_simulation.launch.py use_sim_time:=false

# Specify custom world file
ros2 launch recognition_module gazebo_simulation.launch.py \
  world:=/path/to/custom.world

# Verbose output
ros2 launch recognition_module gazebo_simulation.launch.py verbose:=true
```

---

## Testing Scenarios

### Scenario 1: Single Object Weight Check

1. Launch simulation
2. Spawn one object of known mass:
   ```bash
   ros2 run recognition_module spawn_objects.py --type part_100g --x 0 --y 0
   ```
3. Check weight estimate:
   ```bash
   ros2 topic echo /recognition/estimated_weights
   ```
4. Verify estimated weight is close to 100g

### Scenario 2: Multiple Objects

1. Launch simulation
2. Spawn test set:
   ```bash
   ros2 run recognition_module spawn_objects.py --type test
   ```
3. Verify 3 separate objects detected with weights ~50g, ~100g, ~200g

### Scenario 3: Drag and Drop Test

1. Launch simulation and spawn test set
2. Use Gazebo GUI to move objects around
3. Watch recognition output update in real-time
4. Verify weights remain consistent as objects move

### Scenario 4: Clustering Test

1. Launch simulation
2. Spawn multiple objects close together:
   ```bash
   ros2 run recognition_module spawn_objects.py --type bolt_50g --x 0.1 --y 0.1
   ros2 run recognition_module spawn_objects.py --type bolt_50g --x 0.15 --y 0.1
   ```
3. Check if they're detected as separate objects or merged
4. Adjust `cluster_tolerance` in `config/recognition.yaml` if needed

---

## Troubleshooting

### Gazebo won't start
- **Check**: Is Gazebo installed?
  ```bash
  gazebo --version
  ```
- **Install if needed**:
  ```bash
  sudo apt install gazebo ros-humble-gazebo-ros-pkgs
  ```

### No point cloud published
- **Check topic**:
  ```bash
  ros2 topic list | grep pointcloud
  ros2 topic hz /camera/pointcloud
  ```
- **Check Gazebo plugins loaded**: Look for messages in Gazebo output about camera plugin

### Objects fall through table
- This shouldn't happen with the provided models
- Check physics engine is running in Gazebo
- Verify table collision is configured

### Recognition node not detecting objects
- **Check workspace bounds** in `config/recognition.yaml`
- Objects must be within the workspace (±0.6m in X/Y, 0-0.6m in Z)
- **Check cluster parameters**:
  - `min_cluster_size`: Try reducing if small objects aren't detected
  - `cluster_tolerance`: Adjust if objects are being merged/split incorrectly

### Models not found
- **Rebuild and install**:
  ```bash
  cd ~/Documents/mtrn4231_jakos/ros2_system
  colcon build --packages-select recognition_module
  source install/setup.bash
  ```
- **Check environment**:
  ```bash
  echo $GAZEBO_MODEL_PATH
  ```

### Spawn script fails
- **Check Gazebo is running**
- **Wait a few seconds** after launching Gazebo before spawning
- **Check model paths** in spawn_objects.py

---

## Advanced Usage

### Custom Objects

To add your own objects:

1. Create a model directory: `models/my_object/`
2. Add `model.config` and `model.sdf`
3. Set correct mass in the `<inertial>` block
4. Calculate inertia tensor for your shape
5. Update spawn script to include your object type

### Multiple Cameras

To add more cameras, edit `worlds/weight_sorting.world` and duplicate the camera model with a new name and position.

### Physics Parameters

Adjust in `worlds/weight_sorting.world`:
- `max_step_size`: Simulation time step
- `real_time_factor`: Speed multiplier
- Contact parameters in model SDFs

---

## Performance Tips

1. **Reduce point cloud resolution** if simulation is slow (edit camera resolution in world file)
2. **Lower update rate** from 10 Hz to 5 Hz if needed
3. **Run headless** (gui:=false) for automated tests
4. **Limit number of objects** - more than 10-15 may slow down
5. **Use voxel downsampling** in recognition node (already configured)

---

## Next Steps

- See [BAG_RECORDING.md](BAG_RECORDING.md) for recording test data
- See [TESTING.md](TESTING.md) for running integration tests
- See [README.md](README.md) for recognition node details
