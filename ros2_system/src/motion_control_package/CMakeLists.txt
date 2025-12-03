cmake_minimum_required(VERSION 3.8)
project(motion_control_package)

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
find_package(std_srvs REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)

# cartesian_controller_node executable (C++)
add_executable(cartesian_controller_node src/cartesian_controller_node.cpp)
ament_target_dependencies(cartesian_controller_node
  rclcpp
  rclcpp_action
  std_msgs
  std_srvs
  sensor_msgs
  geometry_msgs
  trajectory_msgs
  control_msgs
  moveit_msgs
  sort_interfaces
)

# simple_pick_and_weigh_node executable (C++)
add_executable(simple_pick_and_weigh_node src/simple_pick_and_weigh_node.cpp)
ament_target_dependencies(simple_pick_and_weigh_node
  rclcpp
  std_msgs
  sort_interfaces
)

# Install C++ targets
install(TARGETS
  cartesian_controller_node
  simple_pick_and_weigh_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python scripts as executables
install(PROGRAMS
  scripts/go_home.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME go_home
)

install(PROGRAMS
  scripts/simple_pick_and_weigh.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simple_pick_and_weigh
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
