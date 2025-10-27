#!/bin/bash
# Script to record ROS2 bag files for testing recognition and planning nodes
# Records all essential topics for testing

# Set bag directory
BAG_DIR="${HOME}/Documents/mtrn4231_jakos/test_bags"
mkdir -p "$BAG_DIR"

# Generate timestamp for bag name
TIMESTAMP=$(date +"%Y%m%d_%H%M%S")
BAG_NAME="recognition_test_${TIMESTAMP}"

echo "======================================"
echo "Recording ROS2 Bag for Recognition Testing"
echo "======================================"
echo "Bag name: $BAG_NAME"
echo "Location: $BAG_DIR/$BAG_NAME"
echo ""
echo "Recording topics:"
echo "  - /camera/pointcloud          (Point cloud from camera)"
echo "  - /camera/image_raw           (RGB image)"
echo "  - /camera/depth/image_raw     (Depth image)"
echo "  - /camera/camera_info         (Camera calibration)"
echo "  - /recognition/estimated_weights (Weight estimates)"
echo "  - /tf                         (Transforms)"
echo "  - /tf_static                  (Static transforms)"
echo ""
echo "Press Ctrl+C to stop recording"
echo "======================================"
echo ""

# Record bag with specified topics
ros2 bag record \
  -o "$BAG_DIR/$BAG_NAME" \
  /camera/pointcloud \
  /camera/image_raw \
  /camera/depth/image_raw \
  /camera/camera_info \
  /camera/depth/camera_info \
  /recognition/estimated_weights \
  /tf \
  /tf_static

echo ""
echo "======================================"
echo "Recording complete!"
echo "Bag saved to: $BAG_DIR/$BAG_NAME"
echo "======================================"
