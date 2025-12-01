README_YOLO

A: Steps to run the node - Completely

colcon build
source install/setup.bash
ros2 launch perception_module object_detect.launch.py

B: Commands to train a model based on annotated images:

python3 -m venv ~/venvs/yolo
source ~/venvs/yolo/bin/activate
pip install --upgrade pip
pip install ultralytics opencv-python
#Choose ONE torch build (GPU if you have CUDA, else CPU):
#GPU example (adjust for your CUDA):
pip install torch torchvision --index-url https://download.pytorch.org/whl/cu121
#OR CPU-only:
#pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu

yolo detect train \
  data=/src/object_detect/dataset/data.yaml \
  model=yolov8n.pt \
  epochs=50 \
  imgsz=960 \
  batch=16

C: Steps to run the node - Separately

1. Launch the realsense2_camera stream

ros2 launch realsense2_camera rs_launch.py align_depth.enable:=true enable_color:=true enable_depth:=true pointcloud.enable:=true

2. Run the ros2 transform - In another terminal

ros2 run tf2_ros static_transform_publisher 1.30938 0.0206053 0.670571   -0.398486 0.00254305 0.917119 0.00974536 base_link camera_link

3. Build, Source and run the file - In another terminal

colcon build
source install/setup.bash
ros2 run perception_module object_detect_yolo \ --ros-args \ -p yolo_weights:=/runs/detect/weights/best.pt \ -p target_class_name:=red_object

4. Run Rviz - In another terminal

rviz2





