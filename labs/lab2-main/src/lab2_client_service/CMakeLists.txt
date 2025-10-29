cmake_minimum_required(VERSION 3.5)
project(lab2_client_service)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(example_interfaces REQUIRED)

add_executable(int_addition_server src/int_addition_server.cpp)
add_executable(int_addition_client src/int_addition_client.cpp)

ament_target_dependencies(int_addition_server rclcpp example_interfaces)
ament_target_dependencies(int_addition_client rclcpp example_interfaces)

install(TARGETS
  int_addition_server
  int_addition_client
  DESTINATION lib/${PROJECT_NAME})

ament_package()
