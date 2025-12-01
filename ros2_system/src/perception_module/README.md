# Perception Module – YOLO-Based Object Detection & 3D Projection  
**(ROS2 + RealSense + UR5e coordinate transforms + weight estimation)**

This package implements the perception pipeline used to detect, localise, and classify cylindrical red weights for the UR5e sorting task. The node performs:

- YOLO detection on the RGB stream  
- Red-circle refinement to pinpoint the **top centre** of the cylindrical weight  
- Depth extraction and reprojection into **3D camera coordinates**  
- Transformation into **UR5e base frame (`base_link`)**  
- Height-based **weight estimation** (10g–500g)  
- Publishing coordinates + metadata to other nodes  
- Optional live visualisation for debugging  

Main node:  
`object_detect_yolo`  
Launch file:  
`object_detect.launch.py`

---

# 1. System Inputs

## 1.1 Subscribed Topics

| Topic | Type | Description |
|-------|-------|-------------|
| `/camera/camera/color/image_raw` | `sensor_msgs/msg/Image` | RGB image used for YOLO + circle detection |
| `/camera/camera/aligned_depth_to_color/image_raw` | `sensor_msgs/msg/Image` | Depth image aligned to colour (mm) |
| `/camera/camera/aligned_depth_to_color/camera_info` | `sensor_msgs/msg/CameraInfo` | Intrinsics (fx, fy, cx, cy, distortion), required for depth → 3D |

### Assumptions
- RealSense is launched with **aligned depth**:  
  `align_depth.enable:=true`
- Depth is in **millimetres** (`uint16`)
- Frame naming conventions: `camera_link` → RealSense origin

---

## 1.2 TF Frames (Required)

A static transform must exist:

base_link ← camera_link

arduino
Copy code

Set using:

'''bash
ros2 run tf2_ros static_transform_publisher \
  1.30938 0.0206053 0.670571 \
  -0.398486 0.00254305 0.917119 0.00974536 \
  base_link camera_link
Translation units: meters

Quaternion format: (qx, qy, qz, qw)

1.3 Parameters
Parameter	Type	Default	Description
yolo_weights	string	~/.../best.pt	Path to YOLO model file
conf_thres	float	0.25	YOLO confidence threshold
target_class_name	string	""	If set, only this class is processed
debug_view	bool	True	Enable OpenCV visualisation
top_offset_px	int	8	Pixel offset from bbox top to avoid the shiny knob
table_offset_px	int	10	Pixel offset below bbox to measure table depth
patch_half_size	int	3	Size of the median depth patch

2. System Outputs
2.1 Published Topics
Topic	Type	Description
object_coords	geometry_msgs/msg/PointStamped	3D coordinates of object in base_link (meters)
object_weight	std_msgs/msg/String	CSV-style string: "class,confidence,weight_label"

Example message
Copy code
red_object,0.87,200 g
3D Coordinate Frame Details
Published frame: base_link

Format (meters):

ini
Copy code
x = forward from base  
y = left from base  
z = up
An additional +0.04 m Z-offset is applied for safe gripper clearance.

2.2 TF Outputs
For visualisation, each object produces:

css
Copy code
base_link → yolo_object_<i>
Zero rotation is used, only the translated position.

2.3 Debug Windows
If debug_view = True, two windows appear:

YOLO Detection
Blue bounding boxes

Green circle = fitted top of cylinder

Red dot = pixel used for depth projection

Text = class, confidence, weight estimate

Height Sampling Pixels
Shows:

TOP point (depth_top sample)

TABLE point (depth_table sample)

Press q to cleanly exit.

3. Processing Pipeline & Calculations
This section explains the core logic used in the system.

3.1 YOLO Detection → Pixel Refinement
YOLO returns bounding boxes:

Copy code
x1, y1, x2, y2
We extract the ROI and run HSV-based red detection (two red hue ranges).
The largest contour is taken, and a circle is fitted:

powershell
Copy code
(cx, cy), r = cv2.minEnclosingCircle(contour)
The refined pixel coordinates are:

ini
Copy code
u = x1 + cx
v = y1 + cy
Fallback (if contour fails):

ini
Copy code
u = (x1 + x2)/2  
v = (y1 + y2)/2
3.2 Pixel → Depth → 3D Camera Coordinates
Depth patch median (robust):

ini
Copy code
depth_val_m = median(depth_patch) * 0.001
RealSense deprojection:

python
Copy code
Xo, Yo, Zo = rs.rs2_deproject_pixel_to_point(intr, (u, v), depth_val_m)
RealSense optical → camera_link frame:

ini
Copy code
Xc = Zo
Yc = -Xo
Zc = -Yo
Empirical correction:

makefile
Copy code
Xc += -0.038
Final:

ini
Copy code
p_cam = [Xc, Yc, Zc]
3.3 Transform to UR5e Base Frame (base_link)
Using TF2:

python
Copy code
pt_base = do_transform_point(pt_cam, transform)
This gives:

scss
Copy code
x_b, y_b, z_b (meters)
We add Z offset:

makefile
Copy code
z_b += 0.04
3.4 Height Estimation + Weight Classification
Two vertical sample points:

ini
Copy code
v_top   = y1 + top_offset_px
v_table = y2 + table_offset_px
Depths:

ini
Copy code
depth_top   = median(depth_patch_at_top)
depth_table = median(depth_patch_table)
Object height:

ini
Copy code
height_m = depth_table - depth_top
Mapping:

Height Range	Weight
> 9.0 cm	500 g
7–9 cm	200 g
5.5–7 cm	100 g
4–5.5 cm	50 g
3–4 cm	20 g
< 3 cm	10 g

These thresholds must be tuned per physical set.

4. TF Diagram
pgsql
Copy code
                +-------------------+
                |  YOLO Detection   |
                |  Pixel (u, v)     |
                +---------+---------+
                          |
                          v
           +----------------------------------+
           | Depth Reprojection (camera_link) |
           |   RealSense intrinsics           |
           +----------------+-----------------+
                            |
                            v
        +------------------------------------------------+
        |   TF Transform: camera_link → base_link        |
        |   (static_transform_publisher + tf2 buffer)    |
        +--------------------------+---------------------+
                                   |
                                   v
                     +-------------------------------+
                     | 3D Object Position (base_link) |
                     | + weight estimation            |
                     +-------------------------------+
5. How to Run
5.1 With the Launch File (recommended)
In perception_module/launch/object_detect.launch.py, you likely start:

RealSense

Static TF

Perception node

Then run:

bash
Copy code
ros2 launch perception_module object_detect.launch.py
5.2 Manual Run (for debugging)
1. Start RealSense
bash
Copy code
ros2 launch realsense2_camera rs_launch.py \
  align_depth.enable:=true \
  enable_color:=true \
  enable_depth:=true \
  pointcloud.enable:=true
2. Publish static transform
bash
Copy code
ros2 run tf2_ros static_transform_publisher \
  1.30938 0.0206053 0.670571 \
  -0.398486 0.00254305 0.917119 0.00974536 \
  base_link camera_link
3. Build & run node
bash
Copy code
colcon build
source install/setup.bash
Run:

bash
Copy code
ros2 run perception_module object_detect_yolo \
  --ros-args \
  -p yolo_weights:=/absolute/path/to/best.pt \
  -p target_class_name:=red_object
6. Known Limitations & Assumptions
Depth noise can cause height misclassification.

HSV thresholds must be tuned per lighting environment.

Camera offset (-0.038 m) is environment-specific.

Table must be flat, non-reflective, and clearly visible in depth.

YOLO accuracy depends on training quality.

No multi-object tracking — each frame is independent.

If TF is not available, the program will skip all detections.

7. File Structure
arduino
Copy code
perception_module/
│
├── object_detect_yolo.py
├── launch/
│   └── object_detect.launch.py
├── params/
├── rviz/
├── models/
├── README.md   ← (this file)
└── setup.py
8. Example Output (Terminal)
arduino
Copy code
[INFO] Estimated height: 72.5 mm, estimated weight: 200 g
[INFO] [YOLO] red_object (0.87)
UR Base (mm): X=-589.74, Y=-134.00, Z=150.00
The arm module then picks the object based on this data.
