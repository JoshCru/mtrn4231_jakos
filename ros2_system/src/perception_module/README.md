# Perception Module

This package performs real-time object detection of **red cylindrical weights** using YOLOv8, extracts precise **3D coordinates** using Intel RealSense depth, transforms them into the **UR5e base frame**, and publishes both coordinates and weight estimation to downstream robot modules.

---

## 1. System Overview

### Processing Pipeline
1. RealSense D435 streams RGB + aligned depth.
2. YOLOv8 detects red cylindrical weights.
3. Circle-refinement detects the red top knob for more accurate centre pixel.
4. Depth + RealSense intrinsics → **3D camera frame point**.
5. TF2 static transform → **base_link coordinates**.
6. Node publishes:
   - `/object_coords` (3D position in `base_link`, meters)
   - `/object_weight` (UR-base coordinates in mm + estimated weight in grams)
7. Live OpenCV windows display:
   - YOLO detections  
   - Top/table sampling points for height estimation

---

## 2. ROS2 Topics

### Subscribed Topics
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RGB frame |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | Depth (aligned) |
| `/camera/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | Intrinsics |

### Published Topics
| Topic          | Type                      | Description |
|----------------|---------------------------|-------------|
| `object_coords`| `geometry_msgs/PointStamped` | 3D position in `base_link` (m) |
| `object_weight`| `std_msgs/String`         | Combined UR-base coordinates and weight in grams, encoded as `x_mm,y_mm,z_mm,weight_g` |
| TF Frames      | `TransformStamped`        | `base_link → yolo_object_i` (debug only) |

The `object_weight` string is formatted as:

```text
x_mm,y_mm,z_mm,weight_g
```
For example:
```text
210.5,-145.2,85.0,200
```

---

## 3. Calculations & Coordinate Transforms

### 3.1 Pixel → Depth → 3D (Camera Frame)

Depth patch median (smoothing):
```python
depth_m = median(depth_patch) * 0.001
```

RealSense deprojection:

```python
Xo, Yo, Zo = rs2_deproject_pixel_to_point(intrinsics, (u, v), depth_m)
```

Convert from optical frame → camera_link:

```python
Xc = Zo
Yc = -Xo
Zc = -Yo
```

### 3.2 Camera Frame → UR5e Base Frame
Static TF:

```python
ros2 run tf2_ros static_transform_publisher 1.30938 0.0206053 0.670571 -0.398486 0.00254305 0.917119 0.00974536 base_link camera_link
```

Apply transform:

```python
pt_base = do_transform_point(pt_cam, tf)
```

Coordinates in base_link (meters) are published.

---

## 4. Weight Estimation Logic
Object height is computed from depth:

```ini
height_m = depth_table - depth_top
```
Threshold → weight:

| Height (m) | Weight |
| ---------- | ------ |
| ≥ 0.040    | 500 g  |
| ≥ 0.030    | 200 g  |
| ≥ 0.025    | 100 g  |
| ≥ 0.020    | 50 g   |
| ≥ 0.015    | 20 g   |
| < 0.015    | 10 g   |

These thresholds must be tuned using real measurements.

---

## 5. Visual Outputs
OpenCV Windows:
- YOLO Detection
  Bounding boxes, centre points, top-circle radius

- Height Sampling Pixels
  Shows which pixels were used to calculate height

TF Tree in RViz:

```cpp
base_link
   └── camera_link (static)
         └── yolo_object_0
         └── yolo_object_1
         └── ...
```

---

## 6. How to Run
### A) Recommended (Launch File)
```nginx
ros2 launch perception_module object_detect.launch.py
```
Your launch file automatically:

- Starts RealSense

- Publishes static TF

- Launches YOLO node

### B) Manual Run
1. Start RealSense:
```go
ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_color:=true enable_depth:=true pointcloud.enable:=true
```
2. Publish TF:

Replace X Y Z QX QY QZ QW with the measured transform from base_link → camera_link
```arduino
ros2 run tf2_ros static_transform_publisher 1.30938 0.0206053 0.670571 -0.398486 0.00254305 0.917119 0.00974536 base_link camera_link
```
3. Run YOLO Node:
```bash
colcon build
source install/setup.bash
ros2 run perception_module object_detect_yolo \
  --ros-args \
  -p yolo_weights:=/home/mtrn/mtrn4231_jakos/ros2_system/src/perception_module/best.pt \
  -p target_class_name:=red_object
```

---

## 7. Package Structure
```arduino
perception_module/
├── perception_module/
│   ├── object_detect_yolo.py
│   ├── object_detect_yolo2.py
├── launch/
│   ├── object_detect.launch.py
├── red_object.v3/        # YOLO dataset
├── runs/detect/trainXX/  # Trained YOLO weights
├── resource/
├── test/
├── best.pt               # YOLO trained weights
├── package.xml
├── setup.py
└── README.md
```

---

## 8. Known Issues & Assumptions
- Depth noise affects height estimation.

- Shiny metal knob requires top_offset_px tuning.

- YOLO model must be trained only on red weights.

- Static TF must accurately represent real camera mounting.

- Table must be planar for correct Z estimation.

- Requires depth alignment: align_depth.enable:=true.

---

## 9. Author
Kevin Lloyd Lazaro

UNSW Sydney
