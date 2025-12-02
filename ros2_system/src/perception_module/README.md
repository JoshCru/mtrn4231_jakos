# Perception Module – YOLO + RealSense + UR5e Coordinate Projection  
**UNSW – MTRN4231 Robotics Project**

This package performs real-time object detection of red cylindrical weights using YOLOv8, extracts precise 3D coordinates using Intel RealSense depth data, transforms them into the UR5e robot’s base frame, and publishes the estimated weight and coordinates for downstream robot control.

---

## 1. System Overview

Pipeline:

1. RGB + aligned depth frames received from Intel RealSense D435.
2. YOLOv8 detects red cylindrical weights.
3. HSV-based top-circle refinement improves centre accuracy.
4. RealSense depth + intrinsics convert pixel → 3D point.
5. TF2 static transform converts camera frame → `base_link`.
6. Publishes:
   - 3D coordinates (`PointStamped`)
   - Estimated weight (`String`)
7. Displays YOLO detections + sampling pixels.

---

## 2. ROS2 Topics

### Subscribed
| Topic | Type | Description |
|-------|------|-------------|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RGB image |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | Depth image |
| `/camera/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | Camera intrinsics |

### Published
| Topic | Type | Description |
|--------|-------|-------------|
| `object_coords` | `geometry_msgs/PointStamped` | Object 3D position in `base_link` |
| `object_weight` | `std_msgs/String` | Estimated weight label |
| TF Frames | `TransformStamped` | `base_link → yolo_object_i` |

---

## 3. Calculations & Transformations

### 3.1 Pixel → Depth → Camera Coordinates
Depth smoothing:

depth_m = median(depth_patch) * 0.001

makefile
Copy code

Deprojection:

Xo, Yo, Zo = rs2_deproject_pixel_to_point(intrinsics, (u,v), depth_m)

sql
Copy code

Convert optical frame → camera_link:

Xc = Zo
Yc = -Xo
Zc = -Yo

powershell
Copy code

### 3.2 Camera → base_link Transform

Static TF launch:

ros2 run tf2_ros static_transform_publisher <x y z qx qy qz qw> base_link camera_link

css
Copy code

Transform:

pt_base = do_transform_point(pt_cam, tf)

yaml
Copy code

Coordinates then published in metres.

---

## 4. Weight Estimation Logic

Depth difference between top sample pixel and table sample pixel:

height_m = depth_table - depth_top

yaml
Copy code

Threshold → label mapping:

| Height (m) | Weight |
|------------|---------|
| ≥ 0.090 | 500 g |
| ≥ 0.070 | 200 g |
| ≥ 0.055 | 100 g |
| ≥ 0.040 | 50 g |
| ≥ 0.030 | 20 g |
| < 0.030 | 10 g |

---

## 5. Visual Outputs

OpenCV windows:
- **"YOLO Detection"** – bounding boxes, centres, radius overlay  
- **"Height Sampling Pixels"** – shows top & table sampling positions

TF tree (RViz):

base_link
↑ camera_link (static)
↑ yolo_object_0...N

yaml
Copy code

---

## 6. How to Run

### A. Using Launch File (Recommended)

ros2 launch perception_module object_detect.launch.py

mathematica
Copy code

### B. Manual Run

**1. Launch RealSense**

ros2 launch realsense2_camera rs_launch.py
align_depth.enable:=true enable_color:=true enable_depth:=true pointcloud.enable:=true

markdown
Copy code

**2. Publish TF**

ros2 run tf2_ros static_transform_publisher
1.30938 0.0206053 0.670571
-0.398486 0.00254305 0.917119 0.00974536
base_link camera_link

markdown
Copy code

**3. Run YOLO node**

colcon build
source install/setup.bash

ros2 run perception_module object_detect_yolo
--ros-args
-p yolo_weights:=/runs/detect/train/weights/best.pt
-p target_class_name:=red_object

yaml
Copy code

---

## 7. Package Structure

perception_module/
├── perception_module/
│ ├── object_detect_yolo.py
├── launch/
│ ├── object_detect.launch.py
├── params/
├── rviz/
├── setup.py
├── package.xml
└── README.md

yaml
Copy code

---

## 8. Known Issues & Assumptions

- Weight estimation depends on depth accuracy.
- Shiny metal knob requires offset tuning (`top_offset_px`).
- Static TF must match real camera mounting.
- YOLO model must be trained only on red weights.
- Depth alignment is required (`align_depth.enable:=true`).
- Table must appear flat in transformation — large TF errors will tilt it.

---

## 9. Author

**Kevin Lloyd Lazaro**  
MTRN4231 Robotics Project – UNSW Sydney  
Perception & Computer Vision Lead
