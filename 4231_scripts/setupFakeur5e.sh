
#!/bin/bash

# Get the directory where this script is located
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PROJECT_ROOT="$(cd "$SCRIPT_DIR/.." && pwd)"

# Workspace paths (relative to project root)
UR_DRIVER_WORKSPACE="$HOME/4231/ros_ur_driver/install/setup.bash"
LOCAL_WORKSPACE="$PROJECT_ROOT/ros2_system/install/setup.bash"

# Launch file paths (relative to script directory)
UR_CONTROL_LAUNCH="$SCRIPT_DIR/ur_control_fixed.launch.py"
UR_MOVEIT_LAUNCH="$SCRIPT_DIR/ur_moveit_fixed.launch.py"

# Source both workspaces
source "$UR_DRIVER_WORKSPACE"
source "$LOCAL_WORKSPACE"

gnome-terminal -t "DriverServer" -- bash -c "source '$UR_DRIVER_WORKSPACE' && source '$LOCAL_WORKSPACE' && ros2 launch '$UR_CONTROL_LAUNCH' ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=true launch_rviz:=false description_file:=ur5e_with_end_effector.urdf.xacro description_package:=motion_control_module; exec bash"

sleep 5

gnome-terminal -t "MoveitServer" -- bash -c "source '$UR_DRIVER_WORKSPACE' && source '$LOCAL_WORKSPACE' && ros2 launch '$UR_MOVEIT_LAUNCH' ur_type:=ur5e launch_rviz:=true use_fake_hardware:=true description_file:=ur5e_with_end_effector.urdf.xacro description_package:=motion_control_module; exec bash"





