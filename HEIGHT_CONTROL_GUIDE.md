# UR5e Height Control & End-Effector Visualization Guide

## Quick Answer: Where to Adjust Z-Height

Open `ros2_system/src/motion_control_module/scripts/square_movement_safe.py` and find **line 33**:

```python
# =================================================================
# ADJUST HEIGHT HERE - Control how high the robot moves
# =================================================================
# Options: 'low', 'medium', 'high'
# - 'low': Close to ground but safe (shoulder_lift ~ -1.8)
# - 'medium': Mid-height workspace (shoulder_lift ~ -1.5)
# - 'high': Higher up (shoulder_lift ~ -1.2)
# =================================================================
self.height_mode = 'medium'  # <<< CHANGE THIS
# =================================================================
```

**Just change `'medium'` to `'low'` or `'high'`!**

---

## Understanding Height Control

### How the UR5e Height System Works

The robot's height is **NOT** controlled by a simple z-coordinate. Instead, it's controlled by **joint angles**, specifically:

1. **shoulder_lift_joint** (Joint 2) - Primary height control
   - More negative = Lower position
   - Less negative = Higher position
   - **NEVER positive** or robot goes upside down!

2. **elbow_joint** (Joint 3) - Secondary height control
   - Works together with shoulder_lift
   - Helps avoid floor collisions

### Height Mode Settings

| Mode | Shoulder Lift | Elbow | Description |
|------|---------------|-------|-------------|
| `'low'` | -1.8 rad | -1.8 rad | Lowest safe height |
| `'medium'` | -1.5 rad | -1.6 rad | Balanced height (default) |
| `'high'` | -1.2 rad | -1.4 rad | Highest safe height |

### Safety Limits (HARD CODED - DO NOT EXCEED!)

```python
SAFETY_LIMITS = {
    'shoulder_lift_min': -2.0,   # Most downward
    'shoulder_lift_max': -1.0,   # Most upward
    'elbow_min': -2.5,           # Most bent
    'elbow_max': -1.0,           # Least bent
}
```

**Why these limits?**
- Keeps the first link (shoulder) above ground
- Prevents robot from flipping upside down
- Avoids singularities and unreachable positions
- Prevents end-effector from hitting the floor

---

## Files Created

### 1. Safe Square Movement with Height Control
**File:** `ros2_system/src/motion_control_module/scripts/square_movement_safe.py`

**Features:**
- ✅ Adjustable height modes: low, medium, high
- ✅ Safety verification for all positions
- ✅ Prevents upside-down configurations
- ✅ Keeps robot above ground
- ✅ Clear console output with status

### 2. End-Effector Position Visualizer GUI
**File:** `ros2_system/src/motion_control_module/scripts/end_effector_visualizer.py`

**Features:**
- ✅ Real-time position display (X, Y, Z in meters)
- ✅ Real-time orientation display (Roll, Pitch, Yaw in degrees)
- ✅ Modern dark-themed GUI
- ✅ Uses TF2 for accurate position tracking
- ✅ Updates 10 times per second

---

## Usage Instructions

### Step 1: Start the Fake UR5e (if not running)

```bash
cd ~/Documents/mtrn4231_jakos/4231_scripts
./setupFakeur5e.sh
```

Wait ~10 seconds for both terminals to start.

### Step 2: Launch the End-Effector Visualizer

Open a new terminal:

```bash
cd ~/Documents/mtrn4231_jakos
python3 ros2_system/src/motion_control_module/scripts/end_effector_visualizer.py
```

A GUI window will open showing:
- **Position**: X, Y, Z coordinates in meters
- **Orientation**: Roll, Pitch, Yaw angles in degrees
- **Status**: Connection status and current position

### Step 3: Run Square Movement (Optional)

In another terminal (visualizer stays open):

```bash
cd ~/Documents/mtrn4231_jakos
python3 ros2_system/src/motion_control_module/scripts/square_movement_safe.py
```

**Watch the visualizer update in real-time as the robot moves!**

---

## Customization

### Change Height

Edit `square_movement_safe.py` line 33:

```python
self.height_mode = 'low'     # Lower height
self.height_mode = 'medium'  # Default
self.height_mode = 'high'    # Higher height
```

### Change Square Size

Edit `square_movement_safe.py` line 37:

```python
self.square_size = 0.15  # 15cm square (default)
self.square_size = 0.20  # 20cm square (larger)
self.square_size = 0.10  # 10cm square (smaller)
```

### Advanced: Custom Height Configuration

You can add your own height mode in the `height_configs` dictionary (line 59):

```python
self.height_configs = {
    'low': {
        'shoulder_lift': -1.8,
        'elbow': -1.8,
        'description': 'Low height'
    },
    'medium': {
        'shoulder_lift': -1.5,
        'elbow': -1.6,
        'description': 'Medium height'
    },
    'high': {
        'shoulder_lift': -1.2,
        'elbow': -1.4,
        'description': 'High height'
    },
    'custom': {  # Add your own!
        'shoulder_lift': -1.65,  # Between medium and low
        'elbow': -1.7,
        'description': 'Custom height for my task'
    }
}
```

Then set:
```python
self.height_mode = 'custom'
```

---

## Understanding the Visualizer Output

### Position Display

```
X: +0.450  → Distance forward/backward from robot base (meters)
Y: -0.120  → Distance left/right from robot base (meters)
Z: +0.380  → Height above ground (meters)
```

- **X positive** = forward
- **X negative** = backward
- **Y positive** = left
- **Y negative** = right
- **Z positive** = above base
- **Z negative** = below base (DANGER!)

### Orientation Display

```
Roll:  +15.3°  → Rotation around X-axis (tilt left/right)
Pitch: -45.2°  → Rotation around Y-axis (tilt forward/back)
Yaw:   +90.0°  → Rotation around Z-axis (turn left/right)
```

---

## Monitoring Tips

### Watch Position Change

While square movement runs, watch the GUI:
- **X and Y** values change as robot traces the square
- **Z** value stays constant (height maintained!)
- **Pitch** changes as arm configuration adjusts

### Command Line Monitoring

```bash
# Watch joint positions
ros2 topic echo /joint_states

# Watch end-effector TF
ros2 run tf2_ros tf2_echo base_link tool0

# Check trajectory controller
ros2 topic echo /joint_trajectory_controller/controller_state
```

---

## Troubleshooting

### GUI doesn't open
```bash
# Check if tkinter is installed
python3 -c "import tkinter; print('OK')"

# If error, install:
sudo apt install python3-tk
```

### Position shows (0, 0, 0)
- Wait a few seconds for TF to initialize
- Make sure fake UR5e is running
- Check that `/joint_states` topic has data:
  ```bash
  ros2 topic echo /joint_states --once
  ```

### Robot hitting floor in RViz
- Change height mode from `'low'` to `'medium'` or `'high'`
- The safety limits should prevent actual floor collisions

### Robot going upside down
- This should NEVER happen with `square_movement_safe.py`
- All positions are verified against safety limits
- If it happens, there's a bug - please report it!

---

## Example Workflow

1. **Start robot**: `./4231_scripts/setupFakeur5e.sh`
2. **Open visualizer**: `python3 .../end_effector_visualizer.py`
3. **Adjust height**: Edit `square_movement_safe.py` to set height mode
4. **Run movement**: `python3 .../square_movement_safe.py`
5. **Watch**: Observe position changes in GUI and RViz simultaneously

---

## Technical Details

### Coordinate System
- Origin: Robot base center
- X-axis: Points forward
- Y-axis: Points left
- Z-axis: Points up

### Joint Numbering
1. shoulder_pan_joint (base rotation)
2. shoulder_lift_joint (height control)
3. elbow_joint (height control)
4. wrist_1_joint (orientation)
5. wrist_2_joint (orientation)
6. wrist_3_joint (orientation)

### TF Frames
- `base_link`: Robot base
- `tool0`: End-effector frame
- Visualizer reads: `base_link → tool0` transform
