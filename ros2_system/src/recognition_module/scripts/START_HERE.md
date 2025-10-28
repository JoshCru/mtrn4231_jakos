# Recognition Module - Quick Start

## Testing the Complete System

### Step 1: Open Two Terminals

### Terminal 1: Start Recognition Node
```bash
cd /home/joshc/mtrn4231_jakos/ros2_system
source install/setup.bash
ros2 run recognition_module recognition_node
```

### Terminal 2: Start Interactive Simulation
```bash
cd /home/joshc/mtrn4231_jakos/ros2_system/src/recognition_module/scripts
source /home/joshc/mtrn4231_jakos/ros2_system/install/setup.bash
python3 simple_weight_simulation.py --mode publish --objects 3 --rate 1.0
```

## What You'll See

**Terminal 1** will show clean, concise recognition output:
```
Detected 3 object(s)
  Object 0: 50.0g, 2.5×2.5×1.0cm @ (0.30, 0.15, 0.10)
  Object 1: 100.0g, 3.0×3.0×1.5cm @ (0.35, -0.10, 0.12)
  Object 2: 200.0g, 4.0×4.0×1.5cm @ (0.25, 0.00, 0.12)
```
Format: `Object ID: weight, size @ position`
- Real-time updates as you move objects in simulation

**Terminal 2** will open a GUI window where you can:
- Click objects to select them
- Use sliders to move objects around
- See changes reflected in Terminal 1

## Quick Tips

- **Move objects**: Click object → use sliders or arrow keys
- **Add objects**: Click "+ Add Object" button (max 4)
- **Reset**: Press 'R' or click "Reset All"
- **See updates**: Watch Terminal 1 for recognition output

## Documentation

- **TEST_GUIDE.md** - Comprehensive testing guide
- **WORKING_VERSION_README.md** - Simulation documentation
- **simple_weight_simulation.py** - The working simulator

## Troubleshooting

If simulation doesn't open:
```bash
echo $DISPLAY  # Should show :0
```

If no objects detected:
- Check objects are in workspace bounds
- Verify: `ros2 topic hz /camera/pointcloud`

---

**Everything is ready to test! Just run the two commands above.** 🚀
