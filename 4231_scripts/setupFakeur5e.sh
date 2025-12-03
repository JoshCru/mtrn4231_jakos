#!/bin/bash

# Stop any existing ROS 2 daemons to avoid conflicts
ros2 daemon stop 2>/dev/null
sleep 1

# Set FastDDS environment variables to fix buffer overflow issues
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_profile.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Create FastDDS profile with increased buffer sizes if it doesn't exist
if [ ! -f /tmp/fastdds_profile.xml ]; then
    cat > /tmp/fastdds_profile.xml << 'FASTDDS_EOF'
<?xml version="1.0" encoding="UTF-8" ?>
<profiles xmlns="http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles">
    <transport_descriptors>
        <transport_descriptor>
            <transport_id>CustomUdpTransport</transport_id>
            <type>UDPv4</type>
            <sendBufferSize>1048576</sendBufferSize>
            <receiveBufferSize>4194304</receiveBufferSize>
            <maxMessageSize>65500</maxMessageSize>
        </transport_descriptor>
    </transport_descriptors>

    <participant profile_name="default_participant" is_default_profile="true">
        <rtps>
            <userTransports>
                <transport_id>CustomUdpTransport</transport_id>
            </userTransports>
            <useBuiltinTransports>false</useBuiltinTransports>
        </rtps>
    </participant>

    <data_writer profile_name="default_writer" is_default_profile="true">
        <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    </data_writer>

    <data_reader profile_name="default_reader" is_default_profile="true">
        <historyMemoryPolicy>PREALLOCATED_WITH_REALLOC</historyMemoryPolicy>
    </data_reader>
</profiles>
FASTDDS_EOF
fi

source ../ros2_system/install/setup.bash

ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=yyy.yyy.yyy.yyy initial_joint_controller:=joint_trajectory_controller use_fake_hardware:=true launch_rviz:=false description_file:=ur5e_with_end_effector.urdf.xacro description_package:=motion_control_package &

sleep 5

ros2 launch motion_control_package ur5e_moveit_with_gripper.launch.py ur_type:=ur5e launch_rviz:=true use_fake_hardware:=true &

echo "Waiting for MoveIt to initialize..."
sleep 5

# Wait for robot_description_semantic parameter to be available
echo "Waiting for robot_description_semantic parameter..."
timeout 30 bash -c 'until ros2 param list /move_group 2>/dev/null | grep -q robot_description_semantic; do sleep 1; done' || echo "Warning: robot_description_semantic not found, continuing anyway..."

sleep 2

python3 ../ros2_system/install/motion_control_package/share/motion_control_package/scripts/safety_boundary_visualizer.py &

sleep 2

python3 ../ros2_system/install/motion_control_package/share/motion_control_package/scripts/safety_boundary_collision.py &
