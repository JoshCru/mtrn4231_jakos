# Testing Recognition Module with Interactive Simulation

## Overview

This guide shows how to test the recognition_node with the interactive weight simulation to see real-time object detection and weight estimation.

## Quick Test

### Terminal 1: Start Recognition Node
```bash
cd /home/joshc/mtrn4231_jakos/ros2_system
source install/setup.bash
ros2 run recognition_module recognition_node
```

### Terminal 2: Start Interactive Simulation
```bash
cd /home/joshc/mtrn4231_jakos/ros2_system/src/recognition_module/scripts
source /home/joshc/mtrn4231_jakos/ros2_system/install/setup.bash
python3 simple_weight_simulation.py --mode publish --objects 3 --rate 1.0
```

### What You'll See

The recognition_node will print detailed output like:

```
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
Received point cloud with 7500 points
🔍 Filtering workspace...
  ✓ Workspace filtered: 7200 points remaining
  ✓ Downsampled: 1800 points
🔍 Locating objects using Euclidean clustering...
  ✓ Found 3 object(s) in scene

📦 Processing detected objects:

  [Object 1/3]
    ⚙ Analyzing cluster with 450 points...
    📏 Bounding box dimensions:
       Width  (X): 2.5 cm
       Height (Y): 2.5 cm
       Depth  (Z): 1.0 cm
    📊 Volume: 0.000006 m³ (6.25 cm³)

    ✓ OBJECT DETECTED:
      ID:         0
      Weight:     50.00 g (estimated)
      Size:       2.5 × 2.5 × 1.0 cm
      Position:   (0.300, 0.150, 0.100) m
      Confidence: 85.0%

  [Object 2/3]
    ⚙ Analyzing cluster with 550 points...
    📏 Bounding box dimensions:
       Width  (X): 3.0 cm
       Height (Y): 3.0 cm
       Depth  (Z): 1.5 cm
    📊 Volume: 0.000013 m³ (13.50 cm³)

    ✓ OBJECT DETECTED:
      ID:         1
      Weight:     100.00 g (estimated)
      Size:       3.0 × 3.0 × 1.5 cm
      Position:   (0.350, -0.100, 0.120) m
      Confidence: 90.0%

  [Object 3/3]
    ⚙ Analyzing cluster with 750 points...
    📏 Bounding box dimensions:
       Width  (X): 4.0 cm
       Height (Y): 4.0 cm
       Depth  (Z): 1.5 cm
    📊 Volume: 0.000024 m³ (24.00 cm³)

    ✓ OBJECT DETECTED:
      ID:         2
      Weight:     200.00 g (estimated)
      Size:       4.0 × 4.0 × 1.5 cm
      Position:   (0.250, 0.000, 0.115) m
      Confidence: 95.0%

✓ Recognition complete
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
```

## Interactive Testing

Once both nodes are running:

1. **In the simulation window**, use the controls to move objects:
   - Click an object to select it
   - Use sliders to move it in X/Y/Z
   - Or use keyboard: Arrow keys, +/-

2. **Watch the recognition_node terminal** to see:
   - Objects being located and analyzed
   - Updated positions as you move objects
   - Size and weight estimates
   - Confidence levels

3. **Try different scenarios:**
   - Move objects closer together
   - Move objects to workspace edges
   - Add/remove objects
   - Change heights

## Monitoring ROS Topics

### View Published Weight Estimates
```bash
ros2 topic echo /recognition/estimated_weights
```

### Check Point Cloud Data
```bash
ros2 topic hz /camera/pointcloud
ros2 topic info /camera/pointcloud
```

### List All Topics
```bash
ros2 topic list
```

## Expected Behavior

### When Objects Are Stationary
- Recognition node processes point cloud
- Detects all objects in workspace
- Publishes weight estimates at 1 Hz

### When You Move Objects
- New point clouds published immediately
- Recognition re-processes scene
- Updated object positions and IDs
- Real-time feedback in logs

### When You Add/Remove Objects
- Cluster count changes
- Object IDs reassigned
- Recognition adapts to new scene

## Troubleshooting

### No Objects Detected

**Check:**
- Objects are in workspace bounds (-0.6 to 0.6 m)
- Point cloud is being published: `ros2 topic hz /camera/pointcloud`
- Recognition node is subscribed: `ros2 node info recognition_node`

**Fix:**
- Move objects within workspace
- Adjust workspace parameters in config/recognition.yaml

### Wrong Weight Estimates

**Check:**
- Material density parameter (8000 kg/m³ for stainless steel)
- Object dimensions match specifications
- Volume calculation method

**Adjust:**
- Edit config/recognition.yaml
- Restart recognition_node

### Low Confidence

**Check:**
- min_cluster_size parameter (default 100 points)
- cluster_tolerance parameter (default 0.02 m)

**Fix:**
- Lower min_cluster_size for smaller objects
- Increase cluster_tolerance if objects are being split

### Objects Not Moving in Recognition Output

**Check:**
- Simulation is in `--mode publish` (not display)
- Both terminals have workspace sourced
- ROS2 communication is working

**Verify:**
```bash
ros2 topic list | grep pointcloud  # Should show /camera/pointcloud
ros2 topic hz /camera/pointcloud   # Should show ~1 Hz
```

## Advanced Testing

### High-Speed Updates
```bash
# Increase publishing rate to 5 Hz
python3 simple_weight_simulation.py --mode publish --objects 3 --rate 5.0
```

### Test with Different Object Counts
```bash
# Single object
python3 simple_weight_simulation.py --mode publish --objects 1

# All four objects
python3 simple_weight_simulation.py --mode publish --objects 4
```

### Custom Configuration
Edit `/home/joshc/mtrn4231_jakos/ros2_system/src/recognition_module/config/recognition.yaml`:

```yaml
recognition_node:
  ros__parameters:
    material_density: 8000.0        # kg/m³
    min_cluster_size: 50            # Lower for smaller objects
    max_cluster_size: 10000
    cluster_tolerance: 0.015         # 1.5cm - tighter clustering
    voxel_size: 0.003               # 3mm - finer downsampling
    workspace_min_z: 0.01           # Ignore table surface
```

Then restart recognition_node:
```bash
ros2 run recognition_module recognition_node
```

## Performance Tips

### Reduce CPU Usage
- Lower simulation publish rate
- Increase voxel_size (faster downsampling)
- Increase min_cluster_size

### Improve Accuracy
- Lower voxel_size (finer detail)
- Adjust cluster_tolerance
- Lower confidence_threshold to see all detections

### Better Visualization
- Keep simulation window visible
- Monitor recognition terminal for detailed logs
- Use rviz2 to visualize point clouds (optional)

## Integration with Full System

To test with the complete perception + recognition pipeline:

### Terminal 1: Perception Module
```bash
ros2 launch perception_module perception.launch.py
```

### Terminal 2: Recognition Module
```bash
ros2 run recognition_module recognition_node
```

### Terminal 3: Simulation (if not using real camera)
```bash
python3 simple_weight_simulation.py --mode publish --objects 3
```

## Success Criteria

✅ **Working correctly if:**
- Recognition node prints object detection messages
- Object count matches simulation
- Sizes approximately match specifications:
  - 50g: ~2.5×2.5×1 cm
  - 100g: ~3×3×1.5 cm
  - 200g: ~4×4×1.5 cm
  - 150g: ~3.5×3.5×1.2 cm
- Positions update when you move objects in simulation
- Confidence levels are reasonable (>50%)

## Summary

This testing setup allows you to:
1. Interactively position objects via GUI
2. See real-time recognition output
3. Validate weight estimation algorithms
4. Test edge cases and failure modes
5. Tune parameters for optimal performance

The system demonstrates the complete pipeline:
```
Simulation → Point Cloud → Recognition → Weight Estimates
```

Happy testing! 🎉
