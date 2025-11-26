#!/bin/bash

# Run the cartesian robot test script
# This will execute a sequence of pre-defined safe movements

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

echo "Starting Cartesian Robot Test..."
echo "This will execute several test movements"
echo ""

python3 ../ros2_system/install/motion_control_module/share/motion_control_module/scripts/test_cartesian_robot.py
