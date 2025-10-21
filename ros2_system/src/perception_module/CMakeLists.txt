cmake_minimum_required(VERSION 3.8)
project(perception_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(cv_bridge REQUIRED)
find_package(sort_interfaces REQUIRED)
find_package(OpenCV REQUIRED)

# rgbd_camera_node executable
add_executable(rgbd_camera_node src/rgbd_camera_node.cpp)
ament_target_dependencies(rgbd_camera_node
  rclcpp
  sensor_msgs
)

# opencv_processor_node executable
add_executable(opencv_processor_node src/opencv_processor_node.cpp)
ament_target_dependencies(opencv_processor_node
  rclcpp
  sensor_msgs
  cv_bridge
  sort_interfaces
)
target_link_libraries(opencv_processor_node ${OpenCV_LIBS})

# pointcloud_processor_node executable
add_executable(pointcloud_processor_node src/pointcloud_processor_node.cpp)
ament_target_dependencies(pointcloud_processor_node
  rclcpp
  sensor_msgs
  geometry_msgs
  visualization_msgs
  sort_interfaces
)

# Install targets
install(TARGETS
  rgbd_camera_node
  opencv_processor_node
  pointcloud_processor_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files and config
install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
