cmake_minimum_required(VERSION 3.8)
project(motion_control_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sort_interfaces REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# joint_movement_controller executable (C++)
add_executable(joint_movement_controller src/joint_movement_controller.cpp)
ament_target_dependencies(joint_movement_controller
  rclcpp
  rclcpp_action
  std_msgs
  sensor_msgs
  geometry_msgs
  moveit_msgs
  sort_interfaces
)

# safety_boundary_node executable (C++)
add_executable(safety_boundary_node src/safety_boundary_node.cpp)
ament_target_dependencies(safety_boundary_node
  rclcpp
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
)

# Install C++ targets
install(TARGETS
  joint_movement_controller
  safety_boundary_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, config, urdf, scripts, srdf, and meshes
install(DIRECTORY
  launch
  config
  urdf
  scripts
  srdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
