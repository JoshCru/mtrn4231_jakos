# System Dashboard Installation Guide

## Quick Start

### 1. Install Dependencies

**Ubuntu/Debian:**
```bash
sudo apt-get update
sudo apt-get install python3-pyqt5
```

**macOS:**
```bash
brew install pyqt5
# Or using pip:
pip3 install PyQt5
```

### 2. Build the Package

```bash
cd ~/ros2_ws  # Or your workspace directory
colcon build --packages-select supervisor_module
source install/setup.bash
```

### 3. Launch the Dashboard

**Option A: Dashboard Only**
```bash
# Make sure system_controller_node is already running
ros2 run supervisor_module system_dashboard
```

**Option B: Full Supervisor (Controller + Dashboard)**
```bash
ros2 launch supervisor_module full_supervisor.launch.py
```

**Option C: Dashboard with Auto-Start**
```bash
ros2 launch supervisor_module full_supervisor.launch.py auto_start:=true
```

## Verification

### Check Installation
```bash
# Verify package is built
ros2 pkg list | grep supervisor_module

# List executables
ros2 pkg executables supervisor_module
# Should show:
# supervisor_module system_controller_node
# supervisor_module system_dashboard

# List launch files
ros2 launch supervisor_module <TAB><TAB>
# Should show: dashboard.launch.py, full_supervisor.launch.py, supervisor.launch.py
```

### Test Dashboard
```bash
# Terminal 1: Start system controller
ros2 run supervisor_module system_controller_node

# Terminal 2: Start dashboard
ros2 run supervisor_module system_dashboard
```

You should see a window titled "MTRN4231 Sort-by-Weight Robot - System Dashboard"

## Troubleshooting

### Error: "No module named 'PyQt5'"
```bash
# Ubuntu/Debian
sudo apt-get install python3-pyqt5

# macOS/pip
pip3 install PyQt5
```

### Error: "system_dashboard not found"
```bash
# Rebuild the package
cd ~/ros2_ws
colcon build --packages-select supervisor_module --symlink-install
source install/setup.bash
```

### Error: "Permission denied"
```bash
# Make sure the Python file is executable
chmod +x ~/ros2_ws/src/supervisor_module/supervisor_module/system_dashboard.py
```

### Dashboard Won't Connect to Services
```bash
# Verify system_controller_node is running
ros2 node list | grep system_controller

# Check available services
ros2 service list | grep system

# If not running, start it:
ros2 run supervisor_module system_controller_node
```

### Qt Platform Plugin Error (Linux)
```bash
# Install Qt platform plugins
sudo apt-get install qt5-default

# Or set environment variable
export QT_QPA_PLATFORM=xcb
```

## Integration with Full System

To integrate the dashboard with your full robot system:

1. **Add to your main launch file:**
```python
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

supervisor_launch = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([
        get_package_share_directory('supervisor_module'),
        '/launch/full_supervisor.launch.py'
    ]),
    launch_arguments={'auto_start': 'false'}.items()
)
```

2. **Or launch separately:**
```bash
# Terminal 1: Launch your robot system
ros2 launch your_package your_launch_file.launch.py

# Terminal 2: Launch dashboard
ros2 run supervisor_module system_dashboard
```

## Development Mode

For development with live code updates:

```bash
# Build with symlink install
cd ~/ros2_ws
colcon build --packages-select supervisor_module --symlink-install
source install/setup.bash

# Now you can edit Python files without rebuilding
# Just restart the node to see changes
```

## Next Steps

1. Read [DASHBOARD_README.md](DASHBOARD_README.md) for detailed usage instructions
2. Configure your system in `config/supervisor.yaml`
3. Customize the dashboard by editing `supervisor_module/system_dashboard.py`
4. Add custom widgets for your specific needs

## System Requirements

- ROS2 (Humble, Iron, or Rolling)
- Python 3.8+
- PyQt5 5.x
- Linux or macOS (Windows may work but untested)

## Support

For issues:
1. Check the troubleshooting section above
2. Verify all dependencies are installed
3. Check ROS2 logs: `ros2 topic echo /rosout`
4. Review dashboard logs in the "System Logs" tab
