cmake_minimum_required(VERSION 3.8)
project(recognition_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sort_interfaces REQUIRED)

# Find PCL
find_package(PCL REQUIRED COMPONENTS
  common
  filters
  segmentation
  surface
)
find_package(pcl_conversions REQUIRED)

# Include directories
include_directories(
  include
  ${PCL_INCLUDE_DIRS}
)

link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

# recognition_node executable
add_executable(recognition_node src/recognition_node.cpp)
ament_target_dependencies(recognition_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  sort_interfaces
  pcl_conversions
)
target_link_libraries(recognition_node
  ${PCL_LIBRARIES}
)

# mock_camera_node executable
add_executable(mock_camera_node src/mock_camera_node.cpp)
ament_target_dependencies(mock_camera_node
  rclcpp
  sensor_msgs
  pcl_conversions
)
target_link_libraries(mock_camera_node
  ${PCL_LIBRARIES}
)

# Install targets
install(TARGETS
  recognition_node
  mock_camera_node
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
  find_package(ament_cmake_pytest REQUIRED)

  # Linting
  ament_lint_auto_find_test_dependencies()

  # Integration tests
  ament_add_pytest_test(test_recognition_node test/test_recognition_node.py
    TIMEOUT 60
  )
endif()

ament_package()
