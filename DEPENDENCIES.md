# System Dependencies and Setup

This document lists all dependencies required to run the UR5e weight sorting system.

## System Requirements

- **OS**: Ubuntu 22.04 LTS
- **ROS**: ROS 2 Humble
- **Python**: 3.10+
- **Robot**: Universal Robots UR5e

## ROS 2 Installation

### 1. Install ROS 2 Humble

```bash
# Set locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Add ROS 2 repository
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install ROS 2 Humble Desktop
sudo apt update
sudo apt install ros-humble-desktop
```

### 2. Install Required ROS 2 Packages

```bash
# Core ROS 2 packages
sudo apt install -y \
    ros-humble-ur \
    ros-humble-ur-robot-driver \
    ros-humble-ur-description \
    ros-humble-ur-moveit-config \
    ros-humble-moveit \
    ros-humble-moveit-ros-planning-interface \
    ros-humble-moveit-ros-move-group \
    ros-humble-rviz2 \
    ros-humble-xacro \
    ros-humble-joint-state-publisher \
    ros-humble-robot-state-publisher \
    ros-humble-controller-manager \
    ros-humble-joint-trajectory-controller \
    ros-humble-scaled-joint-trajectory-controller

# Additional dependencies
sudo apt install -y \
    python3-colcon-common-extensions \
    python3-rosdep \
    python3-vcstool
```

### 3. Install Python Dependencies

```bash
# Tkinter for dashboard GUI
sudo apt install -y python3-tk

# Other Python packages
pip3 install \
    numpy \
    scipy \
    pyyaml
```

## ROS 2 Workspace Setup

### 1. Clone and Build

```bash
# Navigate to workspace
cd ~/mtrn4231_jakos/ros2_system

# Source ROS 2
source /opt/ros/humble/setup.bash

# Initialize rosdep (first time only)
sudo rosdep init
rosdep update

# Install workspace dependencies
rosdep install --from-paths src --ignore-src -r -y

# Build the workspace
colcon build

# Source the workspace
source install/setup.bash
```

### 2. Custom Message Interfaces

The system uses custom messages defined in `sort_interfaces`:

**Topics:**
- `DetectedObjects` - Bounding boxes from perception
- `WeightEstimate` - Weight measurements from calibration
- `BoundingBox` - Object location and ID

**Services:**
- `MoveToCartesian` - Cartesian movement control
- `GripperControl` - Gripper open/close

**Actions:**
- `PickObject` - Pick operation
- `PlaceObject` - Place operation

These are automatically built when you run `colcon build`.

## Package Dependencies

### supervisor_module

**ROS Dependencies:**
- rclcpp
- rclpy
- std_msgs
- geometry_msgs
- visualization_msgs (for RViz markers)
- moveit_msgs
- sort_interfaces

**System Dependencies:**
- python3-tk (for dashboard GUI)

**Purpose:**
- `sorting_brain_node` - Main orchestrator
- `simulated_perception_node` - Simulated weight detection
- `system_dashboard` - Tkinter UI control panel

### motion_control_module

**ROS Dependencies:**
- rclcpp
- rclcpp_action
- moveit_ros_planning_interface
- ur_robot_driver
- ur_moveit_config
- ur_description
- sort_interfaces
- tf_transformations
- shape_msgs

**Purpose:**
- `cartesian_controller_node` - Cartesian motion control
- `go_home` - Home position utility
- Safety boundary visualization

## External Team Dependencies

### Kevin's Perception Module (recognition_module)

**Required topic:** `/perception/detected_objects`

**Message type:** `DetectedObjects`

**Must provide:**
- Bounding boxes for objects in picking area
- Object IDs
- Coordinates in millimeters relative to base_link

### Asad's Weight Detection Module (weight_detection_module)

**Published topic:** `/recognition/estimated_mass`

**Message type:** `Int32` (weight in grams)

**Values:** [0, 20, 50, 100, 200, 500]

**Description:**
- Uses UR5e joint torques to estimate object mass
- Includes Kalman filtering and physics-based estimation
- Provides matplotlib visualization of torque data

**Integration:**
- The module is fully integrated and launched automatically in real robot mode
- In simulation mode, `simulated_perception_node` publishes to the same topic with assumed perfect values

## Build Instructions

### Full Clean Build

```bash
cd ~/mtrn4231_jakos/ros2_system

# Clean previous build
rm -rf build/ install/ log/

# Source ROS 2
source /opt/ros/humble/setup.bash

# Build all packages
colcon build

# Source workspace
source install/setup.bash
```

### Build Specific Package

```bash
# Build only supervisor_module
colcon build --packages-select supervisor_module

# Build only motion_control_module
colcon build --packages-select motion_control_module

# Build custom interfaces
colcon build --packages-select sort_interfaces
```

## Network Configuration

### For Real Robot

**Robot IP:** `192.168.0.100` (default in scripts)

**Network Setup:**
1. Connect computer to robot via Ethernet
2. Configure static IP on same subnet (e.g., 192.168.0.10)
3. Verify connection: `ping 192.168.0.100`

**Robot Configuration:**
- Must be in **remote control mode**
- External control program installed on UR5e
- Safety parameters configured

### For Simulation

No network configuration needed - uses fake hardware.

## Troubleshooting

### Missing Dependencies

```bash
# Check for missing dependencies
rosdep check --from-paths src --ignore-src

# Install missing dependencies
rosdep install --from-paths src --ignore-src -r -y
```

### Tkinter Not Found

```bash
# Install tkinter
sudo apt install python3-tk

# Verify installation
python3 -c "import tkinter; print('Tkinter OK')"
```

### MoveIt Not Found

```bash
# Install MoveIt
sudo apt install ros-humble-moveit

# Verify installation
ros2 pkg list | grep moveit
```

### UR Driver Not Found

```bash
# Install UR packages
sudo apt install ros-humble-ur ros-humble-ur-robot-driver

# Verify installation
ros2 pkg list | grep ur_
```

## Quick Start Checklist

- [ ] ROS 2 Humble installed
- [ ] UR packages installed (`ros-humble-ur`, `ros-humble-ur-robot-driver`)
- [ ] MoveIt installed (`ros-humble-moveit`)
- [ ] Python3-tk installed (`sudo apt install python3-tk`)
- [ ] Workspace built (`colcon build`)
- [ ] Workspace sourced (`source install/setup.bash`)
- [ ] (Optional) Kevin's perception nodes ready
- [ ] (Optional) Asad's calibration nodes ready
- [ ] (Optional) Robot powered on and in remote control mode

## Running the System

See [SYSTEM_MODES.md](SYSTEM_MODES.md) for detailed instructions on running the system in different modes (simulation, hybrid, real robot).

## For New Team Members

### Minimal Setup (Simulation Only)

```bash
# Install ROS 2 Humble
sudo apt install ros-humble-desktop

# Install required packages
sudo apt install -y \
    ros-humble-ur \
    ros-humble-ur-robot-driver \
    ros-humble-moveit \
    python3-tk \
    python3-colcon-common-extensions

# Clone and build
cd ~/mtrn4231_jakos/ros2_system
source /opt/ros/humble/setup.bash
colcon build
source install/setup.bash

# Run simulation
cd ../4231_scripts
./runSimulation.sh
```

### Full Setup (Real Robot)

Follow all steps above, plus:
- Configure network for robot IP 192.168.0.100
- Ensure Kevin's and Asad's nodes are installed and running
- Verify robot is in remote control mode
- Run: `./runRealRobot.sh`
