#!/bin/bash

echo "==============================================="
echo "Testing RealSense Camera Setup"
echo "==============================================="
echo ""

echo "1. Checking if camera topics exist..."
echo "-------------------------------------------"
timeout 2 ros2 topic list | grep camera || echo "❌ No camera topics found"
echo ""

echo "2. Checking camera transform..."
echo "-------------------------------------------"
timeout 2 ros2 run tf2_ros tf2_echo base_link camera_link 2>/dev/null && echo "✅ Transform published" || echo "❌ No transform found"
echo ""

echo "3. Checking pointcloud publishing rate..."
echo "-------------------------------------------"
timeout 5 ros2 topic hz /camera/depth/color/points 2>/dev/null || echo "❌ No pointcloud data"
echo ""

echo "4. Checking robot topics..."
echo "-------------------------------------------"
timeout 2 ros2 topic list | grep -E "(joint_states|robot_description)" && echo "✅ Robot driver running" || echo "❌ Robot driver not found"
echo ""

echo "5. Checking RViz..."
echo "-------------------------------------------"
timeout 2 ros2 node list | grep rviz && echo "✅ RViz is running" || echo "❌ RViz not running"
echo ""

echo "==============================================="
echo "Summary of Available Camera Topics:"
echo "==============================================="
timeout 2 ros2 topic list | grep camera
echo ""

echo "To view pointcloud info:"
echo "  ros2 topic info /camera/depth/color/points"
echo ""
echo "To echo pointcloud data:"
echo "  ros2 topic echo /camera/depth/color/points --no-arr"
echo ""
