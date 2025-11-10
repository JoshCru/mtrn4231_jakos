cmake_minimum_required(VERSION 3.8)
project(supervisor_module)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++17 standard
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclpy REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(sort_interfaces REQUIRED)

# system_controller_node executable (C++)
add_executable(system_controller_node src/system_controller_node.cpp)
ament_target_dependencies(system_controller_node
  rclcpp
  std_msgs
  sort_interfaces
)

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

# Install C++ targets
install(TARGETS
  system_controller_node
  joint_movement_controller
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  supervisor_module/system_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME system_dashboard
)

install(PROGRAMS
  supervisor_module/joint_movement_client_example.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME joint_movement_client_example
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
