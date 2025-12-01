cmake_minimum_required(VERSION 3.8)
project(supervisor_module)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclpy REQUIRED)

# Install Python module
ament_python_install_package(${PROJECT_NAME})

# Install Python executables
install(PROGRAMS
  supervisor_module/sorting_brain_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME sorting_brain_node
)

install(PROGRAMS
  supervisor_module/simulated_perception_node.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME simulated_perception_node
)

install(PROGRAMS
  supervisor_module/system_dashboard.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME system_dashboard
)

# Install launch files
install(DIRECTORY
  launch
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

# Install config
install(DIRECTORY
  config
  DESTINATION share/${PROJECT_NAME}
  OPTIONAL
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
