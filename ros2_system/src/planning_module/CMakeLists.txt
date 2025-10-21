cmake_minimum_required(VERSION 3.8)
project(planning_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(sort_interfaces REQUIRED)

# sort_node executable
add_executable(sort_node src/sort_node.cpp)
ament_target_dependencies(sort_node
  rclcpp
  std_msgs
  geometry_msgs
  sort_interfaces
)

# verification_node executable
add_executable(verification_node src/verification_node.cpp)
ament_target_dependencies(verification_node
  rclcpp
  rclcpp_action
  std_msgs
  sort_interfaces
)

# integrity_node executable
add_executable(integrity_node src/integrity_node.cpp)
ament_target_dependencies(integrity_node
  rclcpp
  sensor_msgs
  geometry_msgs
  std_msgs
  sort_interfaces
)

# moveit2_interface_node executable
add_executable(moveit2_interface_node src/moveit2_interface_node.cpp)
ament_target_dependencies(moveit2_interface_node
  rclcpp
  rclcpp_action
  geometry_msgs
  moveit_msgs
  moveit_ros_planning_interface
  sort_interfaces
)

# Install targets
install(TARGETS
  sort_node
  verification_node
  integrity_node
  moveit2_interface_node
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
