#!/usr/bin/env python3
"""
Simplified Interactive Weight Simulation - Works around matplotlib issues

This is a simplified version that avoids problematic matplotlib features
while still providing interactive positioning capabilities.
"""

import argparse
import numpy as np
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple

# Aggressive matplotlib configuration to avoid recursion
sys.setrecursionlimit(50000)

import matplotlib
matplotlib.use('Agg')  # Use non-interactive backend first
import matplotlib.pyplot as plt
from matplotlib.patches import Rectangle
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import tkinter as tk
from tkinter import ttk


@dataclass
class WeightObject:
    """Represents a stainless steel weight object"""
    weight_grams: float
    width: float
    height: float
    depth: float
    x: float
    y: float
    z: float
    color: Tuple[int, int, int]
    name: str


class SimpleWeightSimulation:
    """Simplified interactive simulation using Tkinter + matplotlib"""

    WEIGHT_SPECS = [
        {'grams': 50, 'width': 0.025, 'height': 0.025, 'depth': 0.01,
         'x': 0.3, 'y': 0.15, 'z': 0.1, 'color': (192, 192, 192)},
        {'grams': 100, 'width': 0.03, 'height': 0.03, 'depth': 0.015,
         'x': 0.35, 'y': -0.1, 'z': 0.12, 'color': (180, 180, 180)},
        {'grams': 200, 'width': 0.04, 'height': 0.04, 'depth': 0.015,
         'x': 0.25, 'y': 0.0, 'z': 0.115, 'color': (170, 170, 170)},
        {'grams': 150, 'width': 0.035, 'height': 0.035, 'depth': 0.012,
         'x': 0.4, 'y': 0.2, 'z': 0.11, 'color': (175, 175, 175)}
    ]

    def __init__(self, mode='display', num_objects=3, publish_rate=1.0):
        self.mode = mode
        self.publish_rate = publish_rate
        self.objects = []
        self.selected_idx = 0
        self.snap_to_grid = False
        self.grid_size = 0.005

        # Initialize objects
        for i in range(min(num_objects, len(self.WEIGHT_SPECS))):
            spec = self.WEIGHT_SPECS[i]
            obj = WeightObject(
                spec['grams'], spec['width'], spec['height'], spec['depth'],
                spec['x'], spec['y'], spec['z'], spec['color'],
                f"{spec['grams']}g"
            )
            self.objects.append(obj)

        # ROS2 setup if needed
        self.ros_node = None
        if mode == 'publish':
            self._init_ros2()

        # Create GUI
        self._create_gui()

    def _init_ros2(self):
        """Initialize ROS2 (same as before)"""
        try:
            import rclpy
            from rclpy.node import Node
            from sensor_msgs.msg import PointCloud2, PointField
            import struct

            self.rclpy = rclpy
            self.PointCloud2 = PointCloud2
            self.PointField = PointField
            self.struct = struct

            rclpy.init()

            class WeightPublisher(Node):
                def __init__(self):
                    super().__init__('simple_weight_simulation')
                    self.publisher_ = self.create_publisher(PointCloud2, '/camera/pointcloud', 10)

            self.ros_node = WeightPublisher()
            print("✓ ROS2 initialized - Publishing to /camera/pointcloud")
        except ImportError:
            print("⚠ ROS2 not available - display mode only")
            self.mode = 'display'

    def _create_gui(self):
        """Create Tkinter GUI"""
        self.root = tk.Tk()
        self.root.title("Weight Simulation - Simplified")
        self.root.geometry("1200x800")

        # Create main frame
        main_frame = ttk.Frame(self.root)
        main_frame.pack(fill=tk.BOTH, expand=True)

        # Create matplotlib figure with minimal features
        self.fig, (self.ax_top, self.ax_side) = plt.subplots(1, 2, figsize=(12, 6))

        # Configure axes without grid (to avoid recursion)
        self.ax_top.set_xlim(-0.6, 0.6)
        self.ax_top.set_ylim(-0.6, 0.6)
        self.ax_top.set_aspect('equal')
        self.ax_top.set_title('Top-Down View (Click to Select)', fontweight='bold')
        self.ax_top.set_xlabel('X (m)')
        self.ax_top.set_ylabel('Y (m)')
        # NO GRID - this causes recursion

        self.ax_side.set_xlim(-0.6, 0.6)
        self.ax_side.set_ylim(0, 0.25)
        self.ax_side.set_title('Side View', fontweight='bold')
        self.ax_side.set_xlabel('X (m)')
        self.ax_side.set_ylabel('Z (m)')
        # NO GRID

        # Embed matplotlib in Tkinter
        self.canvas = FigureCanvasTkAgg(self.fig, master=main_frame)
        self.canvas.draw()
        self.canvas.get_tk_widget().pack(side=tk.TOP, fill=tk.BOTH, expand=True)

        # Mouse click event
        self.canvas.mpl_connect('button_press_event', self._on_click)

        # Control panel
        control_frame = ttk.Frame(main_frame)
        control_frame.pack(side=tk.BOTTOM, fill=tk.X, padx=10, pady=10)

        # Row 1: Object selection
        ttk.Label(control_frame, text="Selected:").grid(row=0, column=0, padx=5)
        self.object_var = tk.StringVar(value=self.objects[0].name if self.objects else "None")
        object_combo = ttk.Combobox(control_frame, textvariable=self.object_var, state='readonly', width=15)
        object_combo['values'] = [obj.name for obj in self.objects]
        object_combo.grid(row=0, column=1, padx=5)
        object_combo.bind('<<ComboboxSelected>>', self._on_object_select)

        # Row 2: X position
        ttk.Label(control_frame, text="X:").grid(row=1, column=0, padx=5)
        self.x_var = tk.DoubleVar(value=self.objects[0].x if self.objects else 0)
        self.x_scale = ttk.Scale(control_frame, from_=-0.5, to=0.5, orient=tk.HORIZONTAL,
                                variable=self.x_var, command=self._on_slider_change, length=300)
        self.x_scale.grid(row=1, column=1, columnspan=2, padx=5)
        self.x_entry = ttk.Entry(control_frame, textvariable=self.x_var, width=10)
        self.x_entry.grid(row=1, column=3, padx=5)
        self.x_entry.bind('<Return>', self._on_entry_change)

        # Row 3: Y position
        ttk.Label(control_frame, text="Y:").grid(row=2, column=0, padx=5)
        self.y_var = tk.DoubleVar(value=self.objects[0].y if self.objects else 0)
        self.y_scale = ttk.Scale(control_frame, from_=-0.5, to=0.5, orient=tk.HORIZONTAL,
                                variable=self.y_var, command=self._on_slider_change, length=300)
        self.y_scale.grid(row=2, column=1, columnspan=2, padx=5)
        self.y_entry = ttk.Entry(control_frame, textvariable=self.y_var, width=10)
        self.y_entry.grid(row=2, column=3, padx=5)
        self.y_entry.bind('<Return>', self._on_entry_change)

        # Row 4: Z position
        ttk.Label(control_frame, text="Z:").grid(row=3, column=0, padx=5)
        self.z_var = tk.DoubleVar(value=self.objects[0].z if self.objects else 0.1)
        self.z_scale = ttk.Scale(control_frame, from_=0.05, to=0.2, orient=tk.HORIZONTAL,
                                variable=self.z_var, command=self._on_slider_change, length=300)
        self.z_scale.grid(row=3, column=1, columnspan=2, padx=5)
        self.z_entry = ttk.Entry(control_frame, textvariable=self.z_var, width=10)
        self.z_entry.grid(row=3, column=3, padx=5)
        self.z_entry.bind('<Return>', self._on_entry_change)

        # Row 5: Buttons
        ttk.Button(control_frame, text="Reset All", command=self._reset).grid(row=4, column=0, padx=5, pady=5)
        ttk.Button(control_frame, text="Add Object", command=self._add_object).grid(row=4, column=1, padx=5, pady=5)
        ttk.Button(control_frame, text="Remove", command=self._remove_object).grid(row=4, column=2, padx=5, pady=5)

        if self.mode == 'publish':
            ttk.Button(control_frame, text="Publish Now", command=self._publish).grid(row=4, column=3, padx=5, pady=5)

        # Status label
        self.status_var = tk.StringVar(value=f"Mode: {self.mode.upper()} | Objects: {len(self.objects)}")
        ttk.Label(control_frame, textvariable=self.status_var, relief=tk.SUNKEN).grid(row=5, column=0, columnspan=4, sticky='ew', pady=5)

        # Initial draw
        self._update_display()

        # Keyboard bindings
        self.root.bind('<Left>', lambda e: self._move(-0.005, 0, 0))
        self.root.bind('<Right>', lambda e: self._move(0.005, 0, 0))
        self.root.bind('<Up>', lambda e: self._move(0, 0.005, 0))
        self.root.bind('<Down>', lambda e: self._move(0, -0.005, 0))
        self.root.bind('<plus>', lambda e: self._move(0, 0, 0.005))
        self.root.bind('<minus>', lambda e: self._move(0, 0, -0.005))
        self.root.bind('r', lambda e: self._reset())

    def _update_display(self):
        """Update matplotlib plots"""
        self.ax_top.clear()
        self.ax_side.clear()

        # Redraw table
        table_top = Rectangle((-0.5, -0.5), 1.0, 1.0, facecolor='saddlebrown', alpha=0.3, edgecolor='black')
        table_side = Rectangle((-0.5, 0), 1.0, 0.001, facecolor='saddlebrown', alpha=0.3, edgecolor='black')
        self.ax_top.add_patch(table_top)
        self.ax_side.add_patch(table_side)

        # Draw objects
        for i, obj in enumerate(self.objects):
            is_selected = (i == self.selected_idx)
            color = np.array(obj.color) / 255.0
            edge_color = 'red' if is_selected else 'black'
            edge_width = 3 if is_selected else 1

            # Top view
            rect_top = Rectangle(
                (obj.x - obj.width/2, obj.y - obj.height/2),
                obj.width, obj.height,
                facecolor=color, alpha=0.7, edgecolor=edge_color, linewidth=edge_width
            )
            self.ax_top.add_patch(rect_top)
            self.ax_top.text(obj.x, obj.y, obj.name, ha='center', va='center', fontsize=9, fontweight='bold')

            # Side view
            rect_side = Rectangle(
                (obj.x - obj.width/2, obj.z - obj.depth/2),
                obj.width, obj.depth,
                facecolor=color, alpha=0.7, edgecolor=edge_color, linewidth=edge_width
            )
            self.ax_side.add_patch(rect_side)

        # Reset axes (without grid to avoid recursion)
        self.ax_top.set_xlim(-0.6, 0.6)
        self.ax_top.set_ylim(-0.6, 0.6)
        self.ax_top.set_aspect('equal')
        self.ax_top.set_title('Top-Down View (Click to Select)', fontweight='bold')

        self.ax_side.set_xlim(-0.6, 0.6)
        self.ax_side.set_ylim(0, 0.25)
        self.ax_side.set_title('Side View', fontweight='bold')

        self.canvas.draw()

        # Update status
        if self.objects:
            obj = self.objects[self.selected_idx]
            self.status_var.set(f"Mode: {self.mode.upper()} | Objects: {len(self.objects)} | "
                              f"Selected: {obj.name} @ ({obj.x:.3f}, {obj.y:.3f}, {obj.z:.3f})")

    def _on_click(self, event):
        """Handle mouse click"""
        if event.inaxes != self.ax_top or not event.xdata:
            return

        # Find clicked object
        for i, obj in enumerate(self.objects):
            if (obj.x - obj.width/2 <= event.xdata <= obj.x + obj.width/2 and
                obj.y - obj.height/2 <= event.ydata <= obj.y + obj.height/2):
                self.selected_idx = i
                self.object_var.set(obj.name)
                self._sync_sliders()
                self._update_display()
                break

    def _on_object_select(self, event):
        """Handle object selection from dropdown"""
        name = self.object_var.get()
        for i, obj in enumerate(self.objects):
            if obj.name == name:
                self.selected_idx = i
                self._sync_sliders()
                self._update_display()
                break

    def _on_slider_change(self, value):
        """Handle slider change"""
        if not self.objects:
            return

        obj = self.objects[self.selected_idx]
        obj.x = self.x_var.get()
        obj.y = self.y_var.get()
        obj.z = self.z_var.get()

        self._update_display()
        if self.mode == 'publish':
            self._publish()

    def _on_entry_change(self, event):
        """Handle entry change"""
        self._on_slider_change(None)

    def _sync_sliders(self):
        """Sync sliders with selected object"""
        if not self.objects:
            return
        obj = self.objects[self.selected_idx]
        self.x_var.set(obj.x)
        self.y_var.set(obj.y)
        self.z_var.set(obj.z)

    def _move(self, dx, dy, dz):
        """Move selected object"""
        if not self.objects:
            return
        obj = self.objects[self.selected_idx]
        obj.x += dx
        obj.y += dy
        obj.z += dz
        self._sync_sliders()
        self._update_display()

    def _reset(self):
        """Reset to original positions"""
        for i, obj in enumerate(self.objects):
            spec = self.WEIGHT_SPECS[i]
            obj.x = spec['x']
            obj.y = spec['y']
            obj.z = spec['z']
        self._sync_sliders()
        self._update_display()

    def _add_object(self):
        """Add new object"""
        if len(self.objects) >= 4:
            return
        spec = self.WEIGHT_SPECS[len(self.objects)]
        obj = WeightObject(
            spec['grams'], spec['width'], spec['height'], spec['depth'],
            spec['x'], spec['y'], spec['z'], spec['color'], f"{spec['grams']}g"
        )
        self.objects.append(obj)
        # Update dropdown
        combo = self.root.nametowidget(self.x_scale.master).grid_slaves(row=0, column=1)[0]
        combo['values'] = [o.name for o in self.objects]
        self._update_display()

    def _remove_object(self):
        """Remove selected object"""
        if len(self.objects) <= 1:
            return
        del self.objects[self.selected_idx]
        self.selected_idx = min(self.selected_idx, len(self.objects) - 1)
        # Update dropdown
        combo = self.root.nametowidget(self.x_scale.master).grid_slaves(row=0, column=1)[0]
        combo['values'] = [o.name for o in self.objects]
        self.object_var.set(self.objects[self.selected_idx].name if self.objects else "None")
        self._sync_sliders()
        self._update_display()

    def _generate_pointcloud(self):
        """Generate point cloud"""
        points = []
        resolution = 0.002
        noise_stddev = 0.002

        for obj in self.objects:
            dx, dy, dz = obj.width/2, obj.height/2, obj.depth/2
            x_range = np.arange(-dx, dx + resolution, resolution)
            y_range = np.arange(-dy, dy + resolution, resolution)
            z_range = np.arange(-dz, dz + resolution, resolution)

            for x in x_range:
                for y in y_range:
                    for z in z_range:
                        noise = np.random.normal(0, noise_stddev, 3)
                        points.append([
                            obj.x + x + noise[0],
                            obj.y + y + noise[1],
                            obj.z + z + noise[2],
                            obj.color[0], obj.color[1], obj.color[2]
                        ])

        return np.array(points)

    def _publish(self):
        """Publish point cloud"""
        if self.mode != 'publish' or self.ros_node is None:
            return

        points = self._generate_pointcloud()
        msg = self.PointCloud2()
        msg.header.stamp = self.ros_node.get_clock().now().to_msg()
        msg.header.frame_id = 'camera_link'
        msg.height = 1
        msg.width = len(points)
        msg.fields = [
            self.PointField(name='x', offset=0, datatype=self.PointField.FLOAT32, count=1),
            self.PointField(name='y', offset=4, datatype=self.PointField.FLOAT32, count=1),
            self.PointField(name='z', offset=8, datatype=self.PointField.FLOAT32, count=1),
            self.PointField(name='rgb', offset=12, datatype=self.PointField.UINT32, count=1),
        ]
        msg.is_bigendian = False
        msg.point_step = 16
        msg.row_step = msg.point_step * msg.width
        msg.is_dense = True

        buffer = []
        for point in points:
            buffer.append(self.struct.pack('fff', point[0], point[1], point[2]))
            r, g, b = int(point[3]), int(point[4]), int(point[5])
            rgb = (r << 16) | (g << 8) | b
            buffer.append(self.struct.pack('I', rgb))

        msg.data = b''.join(buffer)
        self.ros_node.publisher_.publish(msg)

    def run(self):
        """Run the GUI"""
        print(f"\n✓ Simple Weight Simulation Started")
        print(f"  Mode: {self.mode.upper()}")
        print(f"  Objects: {len(self.objects)}")
        print(f"\nControls:")
        print(f"  - Click objects to select")
        print(f"  - Use sliders or type exact values")
        print(f"  - Arrow keys: Move in XY plane")
        print(f"  - +/-: Adjust height")
        print(f"  - R: Reset all\n")

        self.root.mainloop()

        if self.ros_node:
            self.ros_node.destroy_node()
            self.rclpy.shutdown()


def main():
    parser = argparse.ArgumentParser(description='Simple Interactive Weight Simulation')
    parser.add_argument('--mode', choices=['display', 'publish'], default='display')
    parser.add_argument('--objects', type=int, default=3, choices=[1, 2, 3, 4])
    parser.add_argument('--rate', type=float, default=1.0)
    args = parser.parse_args()

    try:
        sim = SimpleWeightSimulation(mode=args.mode, num_objects=args.objects, publish_rate=args.rate)
        sim.run()
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
        return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
