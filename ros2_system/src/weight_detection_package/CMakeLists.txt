cmake_minimum_required(VERSION 3.8)
project(weight_detection_package)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(ament_cmake_python REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sort_interfaces REQUIRED)
find_package(Eigen3 REQUIRED)

# C++ executable (default)
add_executable(weight_detector src/weight_detector.cpp)
target_include_directories(weight_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
  ${EIGEN3_INCLUDE_DIR}
)
target_compile_features(weight_detector PUBLIC cxx_std_17)
ament_target_dependencies(weight_detector
  rclcpp
  sensor_msgs
  std_msgs
  sort_interfaces
)

# Install C++ executable
install(TARGETS weight_detector
  DESTINATION lib/${PROJECT_NAME}
)

# Install Python modules
ament_python_install_package(${PROJECT_NAME})

# Install Python executable (alternative implementation)
install(PROGRAMS
  weight_detection_package/weight_detector_py.py
  DESTINATION lib/${PROJECT_NAME}
  RENAME weight_detector_py
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
