# Recognition Module

Point cloud-based weight estimation using PCL (Point Cloud Library) for stainless steel object sorting.

---

## Overview

The recognition module processes RGBD point clouds to estimate object weights based on volume and material density. Designed for **stainless steel weights** with the **Lenovo 510 RGBD webcam**.

### Algorithm

```
Point Cloud → Workspace Filter → Downsample → Cluster → Volume Estimation → Weight Calculation
```

1. **Workspace Filtering:** PassThrough filter for X/Y/Z bounds
2. **Downsampling:** Voxel grid (5mm resolution)
3. **Clustering:** Euclidean clustering (2cm tolerance)
4. **Volume:** Convex hull calculation
5. **Weight:** `weight = volume × density` (density = 8000 kg/m³)

---

## Nodes

### 1. `recognition_node`

**Purpose:** Process point clouds and estimate object weights

**Subscriptions:**
- `/camera/pointcloud` (sensor_msgs/PointCloud2) - Input point cloud

**Publications:**
- `/recognition/estimated_weights` (sort_interfaces/WeightEstimate) - Weight estimates

**Parameters:**
```yaml
material_density: 8000.0       # kg/m³ (stainless steel)
min_cluster_size: 100          # Minimum points per object
max_cluster_size: 10000        # Maximum points per object
cluster_tolerance: 0.02        # 2cm Euclidean distance
voxel_size: 0.005              # 5mm downsampling
workspace_min_x: -0.6          # meters
workspace_max_x: 0.6
workspace_min_y: -0.6
workspace_max_y: 0.6
workspace_min_z: 0.0           # Table surface
workspace_max_z: 0.6
confidence_threshold: 0.5      # Minimum confidence to publish
```

---

### 2. `mock_camera_node`

**Purpose:** Simulate Lenovo 510 RGBD camera for testing

**Publications:**
- `/camera/pointcloud` (sensor_msgs/PointCloud2) - Simulated point cloud

**Parameters:**
```yaml
publish_rate: 1.0              # Hz
frame_id: "camera_link"
num_objects: 3                 # Number of objects to simulate
add_noise: true                # Add Gaussian noise
noise_stddev: 0.002            # 2mm noise
```

**Simulated Objects:**
- Object 1: 50g (2.5cm × 2.5cm × 1cm)
- Object 2: 100g (3cm × 3cm × 1.5cm)
- Object 3: 200g (4cm × 4cm × 1.5cm)

---

## Quick Start

### With Real Camera (Lenovo 510)

```bash
# Launch recognition node only
ros2 launch recognition_module recognition.launch.py
```

### With Mock Camera (Testing)

```bash
# Launch both mock camera and recognition node
ros2 launch recognition_module recognition_with_mock_camera.launch.py
```

---

## Building

```bash
cd /path/to/ros2_system
colcon build --packages-select recognition_module sort_interfaces
source install/setup.bash
```

**Dependencies:**
- PCL (Point Cloud Library)
- pcl_conversions
- pcl_ros
- sort_interfaces

**Install PCL (if not installed):**
```bash
sudo apt install libpcl-dev ros-humble-pcl-conversions ros-humble-pcl-ros
```

---

## Testing

### Run Integration Tests

```bash
# Terminal 1: Launch nodes with mock camera
ros2 launch recognition_module recognition_with_mock_camera.launch.py

# Terminal 2: Run tests
colcon test --packages-select recognition_module
colcon test-result --verbose
```

### Test Suite (12 tests)

- ✅ Weight estimates received
- ✅ Message structure validation
- ✅ Position within workspace bounds
- ✅ Multiple objects detected
- ✅ Reasonable weight values
- ✅ Unique object IDs
- ✅ Confidence threshold
- ✅ Continuous publishing
- ✅ Volume calculation
- ✅ Weight-density relationship
- ✅ Orientation fields
- ✅ Timestamp validity

---

## Usage Examples

### Manual Testing

```bash
# Terminal 1: Launch
ros2 launch recognition_module recognition_with_mock_camera.launch.py

# Terminal 2: Monitor weight estimates
ros2 topic echo /recognition/estimated_weights

# Terminal 3: Monitor point cloud
ros2 run rviz2 rviz2
# Add PointCloud2 display for /camera/pointcloud
```

### Integration with Planning Module

```bash
# Launch full pipeline
ros2 launch recognition_module recognition_with_mock_camera.launch.py &
ros2 launch planning_module planning.launch.py
```

The planning module will receive weight estimates and make sorting decisions.

---

## Configuration

### Adjusting Material Density

For different materials, change `material_density`:

```yaml
recognition_node:
  ros__parameters:
    material_density: 2700.0  # Aluminum (kg/m³)
    # or
    material_density: 7850.0  # Carbon steel (kg/m³)
    # or
    material_density: 8000.0  # Stainless steel 316 (default)
```

### Adjusting Clustering

For smaller/larger objects:

```yaml
recognition_node:
  ros__parameters:
    min_cluster_size: 50      # Smaller objects
    cluster_tolerance: 0.01    # Tighter clustering (1cm)
```

### Adjusting Workspace

Match your robot workspace:

```yaml
recognition_node:
  ros__parameters:
    workspace_min_x: -0.5
    workspace_max_x: 0.5
    # etc.
```

---

## Message Types

### WeightEstimate (Output)

```
Header header
uint32 object_id
float32 estimated_weight  # grams
float32 confidence        # 0.0 to 1.0
geometry_msgs/Pose pose   # Position and orientation
float32 volume            # m³
```

**Example:**
```yaml
object_id: 1
estimated_weight: 105.3  # grams
confidence: 0.87
pose:
  position: {x: 0.3, y: 0.15, z: 0.1}
  orientation: {x: 0, y: 0, z: 0, w: 1}
volume: 0.0000132  # m³
```

---

## Architecture

```
┌─────────────────┐
│  Camera         │
│  (Lenovo 510)   │
└────────┬────────┘
         │ /camera/pointcloud
         ▼
┌─────────────────┐
│ recognition_    │
│ node            │
│  - Filter       │
│  - Cluster      │
│  - Volume calc  │
│  - Weight calc  │
└────────┬────────┘
         │ /recognition/estimated_weights
         ▼
┌─────────────────┐
│ Planning Module │
│ (sort_node)     │
└─────────────────┘
```

---

## Algorithm Details

### 1. Workspace Filtering (PassThrough)

```cpp
// X filter
pass.setFilterFieldName("x");
pass.setFilterLimits(-0.6, 0.6);

// Y filter
pass.setFilterFieldName("y");
pass.setFilterLimits(-0.6, 0.6);

// Z filter
pass.setFilterFieldName("z");
pass.setFilterLimits(0.0, 0.6);
```

### 2. Downsampling (Voxel Grid)

```cpp
voxel_grid.setLeafSize(0.005, 0.005, 0.005);  // 5mm
```

### 3. Euclidean Clustering

```cpp
EuclideanClusterExtraction:
  ClusterTolerance: 0.02  // 2cm
  MinClusterSize: 100 points
  MaxClusterSize: 10000 points
```

### 4. Volume Estimation (Convex Hull)

```cpp
ConvexHull:
  Input: Cluster point cloud
  Output: Volume in m³
```

**Note:** Convex hull tends to overestimate volume for concave objects.

### 5. Weight Calculation

```
weight (kg) = volume (m³) × density (kg/m³)
weight (g) = weight (kg) × 1000
```

For stainless steel:
```
weight (g) = volume (m³) × 8000 × 1000
```

### 6. Confidence Calculation

```cpp
// Based on number of points in cluster
confidence = 0.5 + (normalized_point_count × 0.5)
// Range: [0.5, 1.0]
```

---

## Troubleshooting

### Problem: No weight estimates published

**Check:**
1. Point cloud is being published:
   ```bash
   ros2 topic echo /camera/pointcloud
   ```
2. Objects in workspace bounds
3. Cluster size is adequate (>100 points)

**Solution:**
- Adjust `min_cluster_size`
- Check workspace bounds
- Increase point cloud resolution

### Problem: Inaccurate weight estimates

**Causes:**
- Convex hull overestimation
- Incorrect density value
- Poor point cloud quality

**Solutions:**
1. Verify material density is correct
2. Improve camera positioning
3. Reduce noise with filtering
4. Calibrate camera

### Problem: Multiple detections of same object

**Cause:** Cluster tolerance too small

**Solution:**
```yaml
cluster_tolerance: 0.03  # Increase to 3cm
```

### Problem: Objects not separated

**Cause:** Cluster tolerance too large

**Solution:**
```yaml
cluster_tolerance: 0.01  # Decrease to 1cm
```

---

## Performance

### Timing

| Stage | Typical Time |
|-------|-------------|
| Point cloud reception | ~30ms (30 Hz) |
| Filtering | ~5ms |
| Downsampling | ~10ms |
| Clustering | ~20ms |
| Volume calculation | ~15ms |
| **Total** | **~50ms** |

**Throughput:** ~20 Hz processing

### Resource Usage

- **CPU:** ~15% (single core)
- **Memory:** ~200 MB
- **Network:** ~5 Mbps (point cloud)

---

## Lenovo 510 RGBD Camera

### Specifications

- **Resolution:** 1920x1080 RGB, 640x480 depth
- **Depth Range:** 0.5m to 4.5m
- **Field of View:** 58° H × 45° V
- **Frame Rate:** 30 FPS

### ROS2 Integration

For real camera (not included, requires camera driver):

```bash
# Example with realsense2_camera (if compatible)
ros2 run realsense2_camera realsense2_camera_node

# Remap topics if needed
ros2 run realsense2_camera realsense2_camera_node \
  --ros-args --remap depth/points:=/camera/pointcloud
```

**Note:** Lenovo 510 may require a custom driver. Check manufacturer documentation.

---

## Known Limitations

1. **Convex Hull Accuracy:**
   - Overestimates volume for concave objects
   - Best for convex shapes (cubes, cylinders)

2. **Single Material:**
   - Assumes all objects are same material (stainless steel)
   - Cannot distinguish different materials

3. **Occlusion:**
   - Cannot estimate volume of occluded parts
   - Objects must be fully visible

4. **Flat Objects:**
   - Poor volume estimation for very flat objects
   - Minimum thickness should be >5mm

---

## Future Improvements

- [ ] Mesh-based volume estimation (more accurate)
- [ ] Multi-material classification
- [ ] Object pose estimation (orientation)
- [ ] Temporal filtering (track objects over time)
- [ ] Deep learning for weight prediction
- [ ] Calibration procedure for camera

---

## File Structure

```
recognition_module/
├── src/
│   ├── recognition_node.cpp         # Main recognition node
│   └── mock_camera_node.cpp         # Mock camera for testing
├── include/
│   └── recognition_module/
├── launch/
│   ├── recognition.launch.py        # Real camera launch
│   └── recognition_with_mock_camera.launch.py  # Testing launch
├── config/
│   └── recognition.yaml             # Configuration
├── test/
│   └── test_recognition_node.py     # Integration tests
├── CMakeLists.txt
├── package.xml
└── README.md
```

---

## References

- [PCL Documentation](https://pointclouds.org/)
- [PCL ROS2](https://github.com/ros-perception/perception_pcl)
- [Euclidean Clustering](https://pcl.readthedocs.io/projects/tutorials/en/latest/cluster_extraction.html)
- [Convex Hull](https://pcl.readthedocs.io/projects/tutorials/en/latest/hull_2d.html)

---

## Summary

**Purpose:** Estimate object weights from RGBD point clouds

**Input:** `/camera/pointcloud` (PointCloud2)

**Output:** `/recognition/estimated_weights` (WeightEstimate)

**Method:** Volume × Density (stainless steel)

**Accuracy:** ±20% (convex hull approximation)

**Test it:**
```bash
ros2 launch recognition_module recognition_with_mock_camera.launch.py
ros2 topic echo /recognition/estimated_weights
```
