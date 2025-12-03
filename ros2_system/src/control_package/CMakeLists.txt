cmake_minimum_required(VERSION 3.8)
project(control_package)

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
find_package(trajectory_msgs REQUIRED)
find_package(control_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sort_interfaces REQUIRED)

# gripper_controller_node executable (lifecycle node)
add_executable(gripper_controller_node src/gripper_controller_node.cpp)
target_include_directories(gripper_controller_node PRIVATE
  ${CMAKE_CURRENT_SOURCE_DIR}/include
)
ament_target_dependencies(gripper_controller_node
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  sort_interfaces
)

# Install targets
install(TARGETS
  gripper_controller_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install launch files, config, urdf, and meshes
install(DIRECTORY
  launch
  config
  urdf
  meshes
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
