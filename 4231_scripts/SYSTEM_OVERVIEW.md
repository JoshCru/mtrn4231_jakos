# System Overview: UR5e + RealSense Integration

## Component Architecture

```
┌─────────────────────────────────────────────────────────────────┐
│                         RViz Visualization                       │
│  ┌──────────────┐  ┌──────────────┐  ┌──────────────────────┐  │
│  │ Robot Model  │  │ MotionPlanning│ │ PointCloud2 Display │  │
│  │              │  │   Interface   │  │  (Depth Camera)     │  │
│  └──────────────┘  └──────────────┘  └──────────────────────┘  │
└─────────────────────────────────────────────────────────────────┘
                              ▲
                              │ Subscribes to topics
                              │
        ┌─────────────────────┼─────────────────────────┐
        │                     │                         │
        ▼                     ▼                         ▼
┌──────────────┐    ┌──────────────────┐    ┌─────────────────────┐
│ Robot Driver │    │  MoveIt/MoveIt2  │    │  RealSense Camera   │
│              │    │                  │    │                     │
│ Publishes:   │    │ Publishes:       │    │ Publishes:          │
│ - joint_states│   │ - planned_path   │    │ - /camera/color/... │
│ - /tf         │    │ - /tf            │    │ - /camera/depth/... │
│               │    │                  │    │ - /camera/.../points│
└───────┬───────┘    └──────────────────┘    └──────────┬──────────┘
        │                                               │
        │ Controls UR5e                                 │ USB 3.0
        ▼                                               ▼
┌──────────────┐                              ┌─────────────────────┐
│  UR5e Robot  │                              │ Intel RealSense D435│
│ 192.168.0.100│                              │  (Depth Camera)     │
└──────────────┘                              └─────────────────────┘
```

## TF (Transform) Tree

```
world
  └── base_link (robot base)
       ├── base_link_inertia
       │    └── shoulder_link
       │         └── upper_arm_link
       │              └── forearm_link
       │                   └── wrist_1_link
       │                        └── wrist_2_link
       │                             └── wrist_3_link
       │                                  └── flange
       │                                       └── tool0
       └── camera_link (hand-eye calibration)
            └── camera_depth_frame
                 └── camera_depth_optical_frame
                      └── camera_color_frame
                           └── camera_color_optical_frame
```

**Key Transform:**
- `base_link` → `camera_link` is published by `static_transform_publisher`
- Translation: [1.277, 0.018, 0.674] meters
- This places the camera ~1.3m in front of the robot, ~0.7m high

## Data Flow

```
┌─────────────────┐
│ RealSense D435  │
└────────┬────────┘
         │
         ▼
┌─────────────────────────────────────┐
│   RealSense Camera Node             │
│   - Captures RGB + Depth            │
│   - Generates PointCloud2           │
│   - Aligns depth to color           │
└────────┬────────────────────────────┘
         │
         ├─► /camera/color/image_raw (RGB image)
         ├─► /camera/depth/image_rect_raw (Depth image)
         ├─► /camera/depth/color/points (PointCloud2) ◄─┐
         ├─► /camera/color/camera_info                  │
         └─► /camera/depth/camera_info                  │
                                                         │
                                                         │ Displayed in RViz
┌────────────────────────────────────┐                  │
│  Static Transform Publisher        │                  │
│  - Publishes base_link → camera_link│                 │
└────────┬───────────────────────────┘                  │
         │                                               │
         └─► /tf_static ─────────────────────────────────┘
                                  (Transforms pointcloud
                                   to robot frame)
```

## Topics Published

### Robot Topics (from ur_robot_driver)
- `/joint_states` - Current joint positions/velocities
- `/tf` - Dynamic transforms (robot movement)
- `/robot_description` - URDF model

### Camera Topics (from realsense2_camera)
- `/camera/color/image_raw` - RGB image (640x480 @ 30fps)
- `/camera/depth/image_rect_raw` - Depth image (848x480 @ 30fps)
- `/camera/depth/color/points` - PointCloud2 (aligned, colored)
- `/camera/color/camera_info` - Camera intrinsics
- `/camera/depth/camera_info` - Depth camera intrinsics

### Planning Topics (from MoveIt)
- `/display_planned_path` - Visualized planned trajectory
- `/planning_scene` - Collision objects and environment

### Static Transforms (from static_transform_publisher)
- `/tf_static` - base_link to camera_link transform

## Coordinate Frames

### Base Link (Robot Base)
- Origin: Center of robot base
- X: Forward (toward front of robot)
- Y: Left
- Z: Up

### Camera Link (RealSense)
- Origin: Camera optical center
- X: Right (when looking from camera)
- Y: Down
- Z: Forward (camera viewing direction)

**The hand-eye calibration transform rotates and translates between these frames.**

## Launch Sequence

```
1. Robot Driver (ur_robot_driver)
   └─► Connects to UR5e at 192.168.0.100
   └─► Publishes joint states and TF
        │
        ▼ Wait 10 seconds
        │
2. MoveIt + RViz
   └─► Loads robot model
   └─► Starts motion planning
   └─► Opens RViz window
        │
        ▼ Wait 5 seconds
        │
3. Camera + Transform
   └─► Publishes base_link → camera_link transform
   └─► Starts RealSense camera
   └─► Publishes pointcloud to /camera/depth/color/points
        │
        ▼
4. User adds PointCloud2 to RViz manually
   └─► Subscribes to /camera/depth/color/points
   └─► Renders pointcloud in robot's workspace
```

## Why This Works

1. **Static Transform** tells RViz where the camera is relative to the robot
2. **PointCloud2** messages include header with frame_id (`camera_color_optical_frame`)
3. **RViz** uses TF tree to transform pointcloud from camera frame to `base_link`
4. **Result**: Pointcloud appears in correct position relative to robot in 3D view

## Camera Calibration Explained

The static transform values come from hand-eye calibration:
```
Position: [1.27677, 0.0175114, 0.673798]
          [  X   ,     Y    ,    Z    ]
          [forward, left   , up      ]  meters from base_link

Rotation: [-0.414096, -0.019425, 0.910018, 0.00376407]
          [   qx    ,    qy   ,    qz   ,    qw      ]  quaternion
```

This means:
- Camera is mounted ~1.28m in front of robot base
- Camera is ~0.67m above robot base
- Camera is slightly (0.02m) to the left
- Camera is tilted to look at workspace

## Use Cases

With this setup, you can:
1. **See robot workspace** - Pointcloud shows objects in camera view
2. **Plan around obstacles** - Add pointcloud as collision objects (future)
3. **Visual servoing** - Use camera feedback for precision tasks
4. **Pick and place** - Detect objects in pointcloud for grasping
5. **Quality inspection** - Use 3D data for measurement
