#!/bin/bash

# Move robot to Cartesian position using the new CartesianRobot class
# Usage: ./move_cartesian.sh <x> <y> <z> <rx> <ry> <rz>
# Position in mm, rotation in radians

if [ "$#" -ne 6 ]; then
    echo "Usage: $0 <x> <y> <z> <rx> <ry> <rz>"
    echo ""
    echo "Arguments:"
    echo "  x, y, z  : Position in mm (relative to base_link)"
    echo "  rx, ry, rz: Rotation vector in radians (UR axis-angle format)"
    echo ""
    echo "Safety Boundaries:"
    echo "  x >= -300mm, y >= -300mm, 0mm <= z <= 655mm"
    echo ""
    echo "Example (gripper facing down at 100mm, 400mm, 300mm):"
    echo "  $0 100 400 300 -1.571 0.0 0.0"
    echo ""
    echo "Common rotation vectors:"
    echo "  Gripper down: rx=-1.571, ry=0.0, rz=0.0"
    echo "  Gripper forward: rx=0.0, ry=0.0, rz=0.0"
    exit 1
fi

# Set FastDDS environment variables to fix buffer overflow issues
export FASTRTPS_DEFAULT_PROFILES_FILE=/tmp/fastdds_profile.xml
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

# Create FastDDS profile if it doesn't exist
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

echo "Moving to: x=$1mm, y=$2mm, z=$3mm, rx=$4rad, ry=$5rad, rz=$6rad"

python3 ../ros2_system/install/motion_control_module/share/motion_control_module/scripts/move_cartesian_simple.py $1 $2 $3 $4 $5 $6
