# Recognition Module - Quick Start

## 🚀 Get Started in 3 Steps

### 1. Build

```bash
cd /path/to/ros2_system
colcon build --packages-select recognition_module sort_interfaces
source install/setup.bash
```

### 2. Launch with Mock Camera (Testing)

```bash
ros2 launch recognition_module recognition_with_mock_camera.launch.py
```

### 3. See Results

```bash
# In another terminal
ros2 topic echo /recognition/estimated_weights
```

You should see weight estimates like:

```yaml
object_id: 1
estimated_weight: 52.3  # grams
confidence: 0.85
pose:
  position: {x: 0.3, y: 0.15, z: 0.1}
volume: 0.0000065  # m³
```

---

## 📊 What It Does

- **Input:** Point cloud from RGBD camera (`/camera/pointcloud`)
- **Processing:** Clusters objects, estimates volume
- **Output:** Weight estimates (`/recognition/estimated_weights`)
- **Method:** Weight = Volume × Density (stainless steel)

---

## 🔧 Common Commands

### Launch with Real Camera
```bash
ros2 launch recognition_module recognition.launch.py
```

### Launch with Mock Camera (Testing)
```bash
ros2 launch recognition_module recognition_with_mock_camera.launch.py
```

### Monitor Weight Estimates
```bash
ros2 topic echo /recognition/estimated_weights
```

### Monitor Point Cloud
```bash
ros2 run rviz2 rviz2
# Add PointCloud2 display for /camera/pointcloud
```

### Run Tests
```bash
# Terminal 1
ros2 launch recognition_module recognition_with_mock_camera.launch.py

# Terminal 2
colcon test --packages-select recognition_module
colcon test-result --verbose
```

---

## ⚙️ Configuration

Edit `config/recognition.yaml`:

```yaml
recognition_node:
  ros__parameters:
    material_density: 8000.0       # kg/m³ (stainless steel)
    min_cluster_size: 100          # Minimum points per object
    cluster_tolerance: 0.02        # 2cm clustering distance
    voxel_size: 0.005              # 5mm downsampling
```

---

## 🎯 Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `material_density` | 8000.0 | kg/m³ (stainless steel) |
| `min_cluster_size` | 100 | Minimum points for valid object |
| `cluster_tolerance` | 0.02 | Max distance between points (m) |
| `voxel_size` | 0.005 | Downsampling resolution (m) |
| `confidence_threshold` | 0.5 | Min confidence to publish |

---

## 📦 What's Included

| File | Purpose |
|------|---------|
| `recognition_node` | Main node (real camera) |
| `mock_camera_node` | Simulated camera (testing) |
| `recognition.launch.py` | Launch for real camera |
| `recognition_with_mock_camera.launch.py` | Launch for testing |
| `recognition.yaml` | Configuration |
| `test_recognition_node.py` | Integration tests |

---

## 🐛 Troubleshooting

### No weight estimates?

1. **Check point cloud:**
   ```bash
   ros2 topic echo /camera/pointcloud
   ```

2. **Check if nodes running:**
   ```bash
   ros2 node list
   # Should show: /recognition_node, /mock_camera_node
   ```

3. **Lower confidence threshold:**
   ```yaml
   confidence_threshold: 0.3  # Instead of 0.5
   ```

### Inaccurate weights?

1. **Check material density:**
   ```yaml
   material_density: 8000.0  # Stainless steel
   ```

2. **Adjust clustering:**
   ```yaml
   cluster_tolerance: 0.015  # Tighter clustering
   ```

---

## 🔗 Integration with Planning Module

```bash
# Terminal 1: Recognition module
ros2 launch recognition_module recognition_with_mock_camera.launch.py

# Terminal 2: Planning module
ros2 launch planning_module planning.launch.py

# Weight estimates flow to planning for sorting decisions!
```

---

## 📖 More Info

- **Full docs:** `README.md`
- **PCL docs:** https://pointclouds.org/
- **ROS2 docs:** https://docs.ros.org/

---

## Summary

**To test the recognition module:**

```bash
# Build
colcon build --packages-select recognition_module sort_interfaces
source install/setup.bash

# Launch
ros2 launch recognition_module recognition_with_mock_camera.launch.py

# Monitor (in another terminal)
ros2 topic echo /recognition/estimated_weights
```

That's it! The mock camera will simulate 3 objects, and the recognition node will estimate their weights based on volume.
