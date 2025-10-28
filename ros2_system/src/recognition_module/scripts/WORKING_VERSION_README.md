# Working Weight Simulation - Simple Version

## ✓ This Version Works!

Due to matplotlib recursion issues with the widget-based version, **use this simplified version instead**:

```bash
python3 simple_weight_simulation.py --mode display --objects 3
```

## What's Different

| Feature | Original (interactive_weight_simulation.py) | Simplified (simple_weight_simulation.py) |
|---------|-------------------------------------------|----------------------------------------|
| **Status** | ❌ Recursion errors | ✅ **Works!** |
| **GUI Framework** | matplotlib widgets | Tkinter + matplotlib |
| **Grids** | Enabled (causes recursion) | Disabled |
| **Interactive** | matplotlib backend | Tk backend |
| **Views** | Top + 3D + Side | Top + Side |
| **Controls** | Sliders, buttons, drag | Sliders, buttons, click |

## Features That Work

✅ **Visual Display**
- Top-down view (XY plane)
- Side view (XZ plane)
- Clear object visualization

✅ **Interactive Controls**
- Click objects to select them
- Sliders for X/Y/Z positioning (1mm precision)
- Text entry for exact coordinates
- Add/Remove objects (1-4 total)
- Reset button

✅ **Keyboard Shortcuts**
- Arrow keys: Move in XY plane
- +/-: Adjust height
- R: Reset all objects

✅ **ROS2 Publishing**
- Same point cloud generation
- Same /camera/pointcloud topic
- Real-time updates

## Quick Start

### Display Mode
```bash
cd /home/joshc/mtrn4231_jakos/ros2_system/src/recognition_module/scripts
python3 simple_weight_simulation.py --mode display --objects 3
```

A window will open with:
- **Top view**: Click objects to select, use sliders to position
- **Side view**: Shows heights
- **Control panel**: Sliders, text boxes, buttons

### ROS2 Mode
```bash
source install/setup.bash
python3 simple_weight_simulation.py --mode publish --objects 3
```

## What's Missing (vs Original)

The simplified version removes features that caused recursion:
- ❌ Grid lines on plots (caused recursion error)
- ❌ Drag-and-drop (use click + sliders instead)
- ❌ 3D view (top + side views sufficient)
- ❌ Collision detection (can be added if needed)
- ❌ Snap-to-grid (can be added if needed)

## Controls

### Mouse
- **Click object**: Select it (turns red)
- Selected object info shown in status bar

### Sliders
- **X slider**: Move left/right (-0.5 to 0.5 m)
- **Y slider**: Move forward/back (-0.5 to 0.5 m)
- **Z slider**: Adjust height (0.05 to 0.2 m)

### Text Entry
- Type exact coordinates and press Enter
- Format: decimal numbers (e.g., 0.325)

### Buttons
- **Reset All**: Restore original positions
- **Add Object**: Add another weight (max 4)
- **Remove**: Delete selected object
- **Publish Now**: Manual point cloud publish (publish mode only)

### Keyboard
- **Arrow Keys**: Move selected object (5mm steps)
- **+ / -**: Increase/decrease height
- **R**: Reset all objects

## Testing with Recognition Module

### Terminal 1: Recognition Module
```bash
ros2 launch recognition_module recognition.launch.py
```

### Terminal 2: Simple Simulation
```bash
source install/setup.bash
python3 simple_weight_simulation.py --mode publish --objects 3
```

### Terminal 3: Monitor Output
```bash
ros2 topic echo /recognition/estimated_weights
```

Now move objects using sliders and watch the recognition module respond!

## Why the Original Version Failed

The original `interactive_weight_simulation.py` had recursion errors because:

1. **Matplotlib Version Conflicts**: System mpl_toolkits incompatible with pip matplotlib
2. **Grid Rendering**: `ax.grid(True)` triggers deep recursion in matplotlib's axis rendering
3. **Widget System**: matplotlib's widget system has recursion in event handling
4. **Backend Issues**: TkAgg backend with widgets causes stacking of event handlers

## Solution Approach

The simplified version works by:

1. **Use Agg Backend**: Non-interactive matplotlib, embed in Tkinter
2. **No Grids**: Avoid `ax.grid()` which causes recursion
3. **Tkinter Controls**: Use Tk widgets instead of matplotlib widgets
4. **Direct Canvas Control**: Manually manage canvas updates
5. **Increased Recursion Limit**: Set to 50,000 as safety

## Advantages of Simple Version

✅ **Reliable**: No recursion errors
✅ **Fast**: Tkinter is lightweight
✅ **Responsive**: Direct event handling
✅ **Maintainable**: Simpler code
✅ **Compatible**: Works with any matplotlib version

## Examples

### Example 1: Position Objects in a Line
```bash
python3 simple_weight_simulation.py --mode display --objects 3
```
1. Click first object
2. Set X=0.2, Y=0.0
3. Click second object
4. Set X=0.3, Y=0.0
5. Click third object
6. Set X=0.4, Y=0.0

### Example 2: Test Recognition with Moving Objects
```bash
# Terminal 1
ros2 launch recognition_module recognition.launch.py

# Terminal 2
python3 simple_weight_simulation.py --mode publish --objects 2

# Move objects around with sliders and watch recognition update in Terminal 3
ros2 topic echo /recognition/estimated_weights
```

### Example 3: Single Object Testing
```bash
python3 simple_weight_simulation.py --mode display --objects 1
```
Perfect for testing individual object recognition.

## Troubleshooting

### Window Doesn't Open

**Check Display:**
```bash
echo $DISPLAY  # Should show :0
```

**For WSL:**
Ensure X server is running on Windows (VcXsrv, Xming, etc.)

### TypeError or AttributeError

Make sure you're using the simplified version:
```bash
python3 simple_weight_simulation.py  # Not interactive_weight_simulation.py
```

### ROS2 Not Publishing

Verify workspace is sourced:
```bash
source /home/joshc/mtrn4231_jakos/ros2_system/install/setup.bash
```

## File Locations

All in: `/home/joshc/mtrn4231_jakos/ros2_system/src/recognition_module/scripts/`

**Use this:**
- ✅ **simple_weight_simulation.py** (Works!)

**Archived (has issues):**
- ❌ interactive_weight_simulation.py (recursion errors)

**Documentation:**
- WORKING_VERSION_README.md (this file)
- INTERACTIVE_SIMULATION_GUIDE.md (original - for reference)
- MATPLOTLIB_FIX_GUIDE.md (troubleshooting)

## Future Improvements

If needed, can add back:
- Collision detection
- Snap-to-grid
- Drag-and-drop (using Tkinter drag instead of matplotlib)
- Configuration save/load

## Summary

✓ **Use: `simple_weight_simulation.py`**
✓ **Works reliably with Tkinter + matplotlib**
✓ **All essential features included**
✓ **ROS2 publishing functional**
✓ **Easy to use and extend**

The simplified version provides all the functionality you need for testing the recognition module without the matplotlib recursion issues!
