# Perception Module – YOLO + RealSense Depth + UR5e Coordinate Projection  
**University of New South Wales – MTRN4231 Robotics Project**

This package performs real-time object detection of **red cylindrical weights** using YOLOv8, extracts **precise 3D coordinates** using Intel RealSense depth data, and transforms those coordinates into the **UR5e robot's base frame**.  
It also performs **height-based weight estimation** and publishes both the coordinates and the estimated weight to other ROS2 nodes.

---

## 📌 **1. System Overview**

The system performs the following pipeline:

1. Receive RGB + aligned depth frames from Intel RealSense D435.
2. Run YOLOv8 on the RGB frame to detect red weights.
3. Refine the 2D detection using HSV circle-finding for the top knob.
4. Use RealSense intrinsics + depth to compute the 3D camera-frame point.
5. Transform `camera_link → base_link` using TF2 static transform.
6. Publish:
   - 3D point in **base_link** (`geometry_msgs/PointStamped`)
   - Estimated weight (`std_msgs/String`)
7. Visualise detection results in RViz or OpenCV windows.

---

## 📡 **2. ROS2 Inputs & Outputs**

### **Subscribed Topics**
| Topic | Type | Description |
|------|------|-------------|
| `/camera/camera/color/image_raw` | `sensor_msgs/Image` | RGB image for YOLO detection |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/Image` | Depth image aligned to colour |
| `/camera/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/CameraInfo` | RealSense intrinsics |

### **Published Topics**
| Topic | Type | Description |
|------|------|-------------|
| `object_coords` | `geometry_msgs/PointStamped` | 3D object coordinate in **base_link** |
| `object_weight` | `std_msgs/String` | Estimated weight label (`"200 g"`, `"50 g"`, etc.) |
| TF frames | `tf2_msgs/TFMessage` | `base_link → yolo_object_i` transforms |

---

## 🔧 **3. How the Calculations Work**

### **3.1 Pixel → Depth → 3D Camera Coordinates**

From YOLO detection, we get a pixel `(u, v)`.

Depth image gives `depth_mm`.

We convert to metres:



depth_m = depth_mm * 0.001


We then use RealSense's deprojection:



Xo, Yo, Zo = rs2_deproject_pixel_to_point(intrinsics, (u,v), depth_m)


This gives coordinates in the **optical** frame.

Convert optical → `camera_link`:



Xc = Zo
Yc = -Xo
Zc = -Yo


### **3.2 Camera Frame → Robot Base Frame (TF Transform)**

A static transform is launched:



ros2 run tf2_ros static_transform_publisher x y z qx qy qz qw base_link camera_link


We apply:



pt_base = do_transform_point(pt_camera, transform)


This gives coordinates in **base_link**, which the UR5e understands.

---

## 🧮 **4. Weight Estimation Logic**

We measure height using depth at two points:

- **Top of object**  
- **Table surface beneath object**

Height:



height = depth_table - depth_top


Weights are classified by threshold ranges:

| Height Range (m) | Weight |
|------------------|--------|
| ≥ 0.090 | **500 g** |
| ≥ 0.070 | **200 g** |
| ≥ 0.055 | **100 g** |
| ≥ 0.040 | **50 g** |
| ≥ 0.030 | **20 g** |
| < 0.030 | **10 g** |

*(These must be tuned for real weights.)*

---

## 🖼️ **5. Visualisations**

### OpenCV Windows:
- `"YOLO Detection"` – RGB image with bbox + circular top marker
- `"Height Sampling Pixels"` – shows where height measurement was taken

### RViz:
TF tree:


      base_link
           ↑
   camera_link (static)
           ↑
   yolo_object_0,1,2...


---

## 🚀 **6. How to Run Everything**

### **Option A — Using the Launch File (Recommended)**

Create launch file:  
`perception_module/launch/object_detect.launch.py`

Run:



ros2 launch perception_module object_detect.launch.py


This automatically:

✔ launches the RealSense  
✔ publishes static TF  
✔ launches YOLO detection node  

### **Option B — Manual Run (Old way)**

1. Launch RealSense:



ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_color:=true enable_depth:=true pointcloud.enable:=true


2. Publish TF:



ros2 run tf2_ros static_transform_publisher
1.30938 0.0206053 0.670571
-0.398486 0.00254305 0.917119 0.00974536
base_link camera_link


3. Run node:



colcon build
source install/setup.bash

ros2 run perception_module object_detect_yolo
--ros-args
-p yolo_weights:=/runs/detect/train/weights/best.pt
-p target_class_name:=red_object


---

## ⚠️ **7. Known Limitations & Notes**

- Height estimation depends on clean depth data — noise can affect results.
- Reflective surfaces (shiny top knobs) require pixel offset tuning.
- YOLO must be trained only on red cylinders for optimal results.
- Accuracy depends on:
  - camera calibration  
  - TF static transform accuracy  
  - lighting consistency  
- Depth alignment **must** be enabled (`align_depth.enable:=true`).

---

## 📁 **8. Package Structure**



perception_module/
│── perception_module/
│ ├── object_detect_yolo.py
│── launch/
│ ├── object_detect.launch.py
│── params/
│── rviz/
│── setup.py
│── package.xml
│── README.md ← you are here


---

## 👤 **Author**
**Kevin Lloyd Lazaro**  
UNSW Sydney – MTRN4231 Robotics Project  
Perception & Computer Vision Lead

---
