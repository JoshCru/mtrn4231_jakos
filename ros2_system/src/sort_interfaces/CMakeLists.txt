cmake_minimum_required(VERSION 3.8)
project(sort_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# Generate interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
  # Messages
  "msg/BoundingBox.msg"
  "msg/DetectedObjects.msg"
  "msg/TargetArea.msg"
  "msg/WeightEstimate.msg"
  "msg/SortDecision.msg"
  "msg/EnvironmentStatus.msg"
  "msg/ForceFeedback.msg"
  # Services
  "srv/SystemCommand.srv"
  "srv/ValidateWorkspace.srv"
  "srv/CalibrateGripper.srv"
  "srv/GripperControl.srv"
  "srv/MoveToJointPosition.srv"
  # Actions
  "action/PickObject.action"
  "action/PlaceObject.action"
  "action/PlanTrajectory.action"
  "action/VerifyWeight.action"
  DEPENDENCIES
    std_msgs
    geometry_msgs
    moveit_msgs
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
