# ROS2 Bag Recording Guide

This guide explains how to record and use ROS2 bag files for testing the recognition and planning nodes.

## What Gets Recorded?

The bag recording scripts capture all essential data for testing:

### Recognition Topics
- `/camera/pointcloud` - Point cloud data from depth camera
- `/camera/image_raw` - RGB camera image
- `/camera/depth/image_raw` - Depth image
- `/camera/camera_info` - Camera calibration
- `/camera/depth/camera_info` - Depth camera calibration
- `/recognition/estimated_weights` - Weight estimates from recognition node

### Transform Topics
- `/tf` - Dynamic transforms
- `/tf_static` - Static transforms

### Planning Topics (optional)
- `/planning/trajectory` - Planned trajectories
- `/planning/sorted_objects` - Sorting decisions
- `/robot/joint_states` - Robot joint positions

---

## Recording Bags

### Method 1: Using the Python Script (Recommended)

#### Basic Recording
```bash
# Record with default settings (auto-generated name, recognition topics)
cd ~/Documents/mtrn4231_jakos/ros2_system/src/recognition_module
python3 scripts/record_bag.py record

# Record for 30 seconds
python3 scripts/record_bag.py record --duration 30

# Record with custom name
python3 scripts/record_bag.py record --name my_test_scenario

# Record all topics (recognition + planning + transforms)
python3 scripts/record_bag.py record --topics all
```

#### List Recorded Bags
```bash
python3 scripts/record_bag.py list
```

### Method 2: Using the Shell Script

```bash
# Simple recording with all recognition topics
cd ~/Documents/mtrn4231_jakos/ros2_system/src/recognition_module
./scripts/record_test_data.sh
```

Press `Ctrl+C` to stop recording.

### Method 3: Manual ros2 bag record

```bash
# Record specific topics manually
ros2 bag record \
  /camera/pointcloud \
  /recognition/estimated_weights \
  /tf /tf_static
```

---

## Playing Back Bags

### Using the Launch File

```bash
# Play a specific bag with recognition node
ros2 launch recognition_module playback_test.launch.py \
  bag_file:=~/Documents/mtrn4231_jakos/test_bags/recognition_test_20241027_120000

# Play at half speed
ros2 launch recognition_module playback_test.launch.py \
  bag_file:=path/to/bag \
  rate:=0.5

# Loop playback
ros2 launch recognition_module playback_test.launch.py \
  bag_file:=path/to/bag \
  loop:=true

# Play bag only (without recognition node)
ros2 launch recognition_module playback_test.launch.py \
  bag_file:=path/to/bag \
  run_recognition:=false
```

### Manual Playback

```bash
# Basic playback
ros2 bag play path/to/bag

# Play with clock (needed for use_sim_time)
ros2 bag play path/to/bag --clock

# Play at 2x speed
ros2 bag play path/to/bag --rate 2.0

# Loop playback
ros2 bag play path/to/bag --loop

# Start paused
ros2 bag play path/to/bag --start-paused
```

---

## Testing Workflow

### 1. Record Test Data from Simulation

```bash
# Terminal 1: Start Gazebo simulation
ros2 launch recognition_module gazebo_simulation.launch.py

# Terminal 2: Spawn test objects
cd ~/Documents/mtrn4231_jakos/ros2_system/src/recognition_module
python3 scripts/spawn_objects.py --type test

# Terminal 3: Record data
python3 scripts/record_bag.py record --name sim_test_01 --duration 30
```

### 2. Test Recognition Node with Recorded Data

```bash
# Play back and test
ros2 launch recognition_module playback_test.launch.py \
  bag_file:=~/Documents/mtrn4231_jakos/test_bags/sim_test_01
```

### 3. Monitor Results

```bash
# In another terminal, monitor weight estimates
ros2 topic echo /recognition/estimated_weights

# Or use RViz to visualize
rviz2
```

---

## Bag Storage Location

By default, bags are stored in:
```
~/Documents/mtrn4231_jakos/test_bags/
```

You can change this with the `--bag-dir` option:
```bash
python3 scripts/record_bag.py record --bag-dir /path/to/custom/dir
```

---

## Tips

1. **Recording Duration**: For testing, 20-30 seconds is usually sufficient
2. **Naming Convention**: Use descriptive names like `sim_test_01`, `real_camera_test_01`, etc.
3. **Bag Size**: Point cloud bags can be large (100-500 MB for 30 seconds)
4. **Use Simulation Time**: Always use `use_sim_time:=true` when playing back bags
5. **Check Topics**: Before recording, verify topics are publishing:
   ```bash
   ros2 topic list
   ros2 topic hz /camera/pointcloud
   ```

---

## Troubleshooting

### No Data Recorded
- Check that the camera/simulation is running
- Verify topics are publishing: `ros2 topic list`
- Check topic names match

### Playback Issues
- Ensure `use_sim_time:=true` is set for all nodes
- Check bag file path is correct
- Verify bag isn't corrupted: `ros2 bag info path/to/bag`

### Recognition Node Not Processing
- Verify point cloud topic name matches: `/camera/pointcloud`
- Check recognition node logs for errors
- Ensure bag playback is using `--clock` flag
