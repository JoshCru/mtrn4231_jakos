#!/usr/bin/env python3
"""
End Effector Position Map
Simple 2D/3D visualization showing end-effector as a point and workspace box.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import numpy as np
import sys
import threading

try:
    import tkinter as tk
    from tkinter import ttk
    GUI_AVAILABLE = True
except ImportError:
    print("ERROR: tkinter not available")
    GUI_AVAILABLE = False

try:
    from tf2_ros import TransformListener, Buffer
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    TF_AVAILABLE = True
except ImportError:
    TF_AVAILABLE = False


class EndEffectorTracker(Node):
    def __init__(self):
        super().__init__('end_effector_tracker')

        # Current position
        self.ee_x = 0.0
        self.ee_y = 0.0
        self.ee_z = 0.0

        # Position history for trail
        self.position_history = []
        self.max_history = 50  # Keep last 50 positions

        # Joint positions
        self.joint_positions = {}

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # TF listener for accurate position
        if TF_AVAILABLE:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.use_tf = True
            self.create_timer(0.1, self.update_from_tf)
        else:
            self.use_tf = False

        self.get_logger().info("End Effector Tracker initialized")

    def joint_state_callback(self, msg):
        """Store joint states"""
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position

        if not self.use_tf:
            self.compute_forward_kinematics()

    def update_from_tf(self):
        """Get position from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            self.ee_x = transform.transform.translation.x
            self.ee_y = transform.transform.translation.y
            self.ee_z = transform.transform.translation.z

            # Add to history
            self.position_history.append((self.ee_x, self.ee_y, self.ee_z))
            if len(self.position_history) > self.max_history:
                self.position_history.pop(0)

        except (LookupException, ConnectivityException, ExtrapolationException):
            pass

    def compute_forward_kinematics(self):
        """Simplified FK"""
        joints = self.joint_positions

        if 'shoulder_pan_joint' not in joints:
            return

        # UR5e link lengths
        d1 = 0.089159
        a2 = -0.425
        a3 = -0.39225
        d4 = 0.10915
        d5 = 0.09465
        d6 = 0.0823

        q1 = joints.get('shoulder_pan_joint', 0.0)
        q2 = joints.get('shoulder_lift_joint', 0.0)
        q3 = joints.get('elbow_joint', 0.0)

        r_reach = a2 * np.cos(q2) + a3 * np.cos(q2 + q3)
        z_reach = d1 + a2 * np.sin(q2) + a3 * np.sin(q2 + q3)

        wrist_extension = d4 + d5 + d6
        r_reach += wrist_extension * np.cos(q2 + q3)
        z_reach += wrist_extension * np.sin(q2 + q3)

        self.ee_x = r_reach * np.cos(q1)
        self.ee_y = r_reach * np.sin(q1)
        self.ee_z = z_reach

        # Add to history
        self.position_history.append((self.ee_x, self.ee_y, self.ee_z))
        if len(self.position_history) > self.max_history:
            self.position_history.pop(0)

    def get_position(self):
        """Get current position"""
        return self.ee_x, self.ee_y, self.ee_z

    def get_history(self):
        """Get position history"""
        return self.position_history.copy()


class MapGUI:
    def __init__(self, ros_node):
        self.ros_node = ros_node

        # Create window
        self.root = tk.Tk()
        self.root.title("End-Effector Position Map")
        self.root.geometry("900x700")
        self.root.configure(bg='#1e1e1e')

        # Main container
        main_frame = tk.Frame(self.root, bg='#1e1e1e')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Title
        title = tk.Label(
            main_frame,
            text="End-Effector Position Map",
            font=("Arial", 18, "bold"),
            bg='#1e1e1e',
            fg='#00ff00'
        )
        title.pack(pady=(0, 10))

        # View selector
        view_frame = tk.Frame(main_frame, bg='#1e1e1e')
        view_frame.pack(pady=(0, 10))

        tk.Label(
            view_frame,
            text="View:",
            font=("Arial", 12),
            bg='#1e1e1e',
            fg='#ffffff'
        ).pack(side=tk.LEFT, padx=(0, 10))

        self.view_mode = tk.StringVar(value="top")

        views = [("Top (X-Y)", "top"), ("Side (X-Z)", "side"), ("Front (Y-Z)", "front")]
        for text, mode in views:
            tk.Radiobutton(
                view_frame,
                text=text,
                variable=self.view_mode,
                value=mode,
                font=("Arial", 10),
                bg='#1e1e1e',
                fg='#ffffff',
                selectcolor='#333333',
                activebackground='#2e2e2e',
                activeforeground='#00ff00'
            ).pack(side=tk.LEFT, padx=5)

        # Canvas for drawing
        canvas_frame = tk.Frame(main_frame, bg='#000000', relief=tk.SUNKEN, bd=2)
        canvas_frame.pack(fill=tk.BOTH, expand=True)

        self.canvas = tk.Canvas(
            canvas_frame,
            bg='#0a0a0a',
            highlightthickness=0
        )
        self.canvas.pack(fill=tk.BOTH, expand=True, padx=5, pady=5)

        # Info panel
        info_frame = tk.Frame(main_frame, bg='#2e2e2e', relief=tk.RAISED, bd=1)
        info_frame.pack(fill=tk.X, pady=(10, 0))

        self.info_label = tk.Label(
            info_frame,
            text="Position: (0.000, 0.000, 0.000) m",
            font=("Courier", 11),
            bg='#2e2e2e',
            fg='#00ff00',
            anchor='w',
            padx=10,
            pady=8
        )
        self.info_label.pack(fill=tk.X)

        # Drawing parameters
        self.scale = 500  # pixels per meter
        self.center_x = 0
        self.center_y = 0

        # Square box parameters (for reference)
        self.box_size = 0.15  # 15cm square
        self.box_center_x = 0.3  # 30cm forward
        self.box_center_y = 0.0

        # Bind resize event
        self.canvas.bind('<Configure>', self.on_resize)

        # Start update loop
        self.update_map()

    def on_resize(self, event):
        """Handle canvas resize"""
        self.canvas_width = event.width
        self.canvas_height = event.height
        self.center_x = self.canvas_width / 2
        self.center_y = self.canvas_height / 2

    def world_to_canvas(self, x, y):
        """Convert world coordinates to canvas coordinates"""
        canvas_x = self.center_x + x * self.scale
        canvas_y = self.center_y - y * self.scale  # Flip Y
        return canvas_x, canvas_y

    def draw_grid(self):
        """Draw reference grid"""
        # Grid lines every 10cm
        grid_spacing = 0.1  # 10cm in meters

        # Get canvas dimensions
        width = self.canvas.winfo_width()
        height = self.canvas.winfo_height()

        # Draw vertical lines
        x = 0
        while True:
            cx, _ = self.world_to_canvas(x, 0)
            if cx > width:
                break
            self.canvas.create_line(cx, 0, cx, height, fill='#1a1a1a', width=1)
            x += grid_spacing

        x = -grid_spacing
        while True:
            cx, _ = self.world_to_canvas(x, 0)
            if cx < 0:
                break
            self.canvas.create_line(cx, 0, cx, height, fill='#1a1a1a', width=1)
            x -= grid_spacing

        # Draw horizontal lines
        y = 0
        while True:
            _, cy = self.world_to_canvas(0, y)
            if cy < 0:
                break
            self.canvas.create_line(0, cy, width, cy, fill='#1a1a1a', width=1)
            y += grid_spacing

        y = -grid_spacing
        while True:
            _, cy = self.world_to_canvas(0, y)
            if cy > height:
                break
            self.canvas.create_line(0, cy, width, cy, fill='#1a1a1a', width=1)
            y -= grid_spacing

    def draw_axes(self):
        """Draw coordinate axes"""
        # X-axis (red)
        cx0, cy0 = self.world_to_canvas(0, 0)
        cx1, cy1 = self.world_to_canvas(0.5, 0)
        self.canvas.create_line(cx0, cy0, cx1, cy1, fill='#ff0000', width=2, arrow=tk.LAST)
        self.canvas.create_text(cx1 + 15, cy1, text='X', fill='#ff0000', font=("Arial", 12, "bold"))

        # Y-axis (green)
        cx2, cy2 = self.world_to_canvas(0, 0.5)
        self.canvas.create_line(cx0, cy0, cx2, cy2, fill='#00ff00', width=2, arrow=tk.LAST)
        self.canvas.create_text(cx2, cy2 - 15, text='Y', fill='#00ff00', font=("Arial", 12, "bold"))

        # Origin
        self.canvas.create_oval(cx0-3, cy0-3, cx0+3, cy0+3, fill='#ffffff', outline='#ffffff')
        self.canvas.create_text(cx0 + 15, cy0 + 15, text='(0,0)', fill='#888888', font=("Arial", 9"))

    def draw_square_box(self):
        """Draw the square workspace box"""
        half_size = self.box_size / 2

        # Corners of the square
        corners = [
            (self.box_center_x + half_size, self.box_center_y - half_size),
            (self.box_center_x + half_size, self.box_center_y + half_size),
            (self.box_center_x - half_size, self.box_center_y + half_size),
            (self.box_center_x - half_size, self.box_center_y - half_size),
        ]

        # Convert to canvas coordinates
        canvas_corners = [self.world_to_canvas(x, y) for x, y in corners]

        # Draw box outline
        for i in range(4):
            x1, y1 = canvas_corners[i]
            x2, y2 = canvas_corners[(i + 1) % 4]
            self.canvas.create_line(x1, y1, x2, y2, fill='#0088ff', width=2)

        # Draw corner markers
        for cx, cy in canvas_corners:
            self.canvas.create_oval(cx-4, cy-4, cx+4, cy+4, fill='#0088ff', outline='#00aaff')

        # Label
        center_cx, center_cy = self.world_to_canvas(self.box_center_x, self.box_center_y)
        self.canvas.create_text(
            center_cx, center_cy,
            text=f"Square\n{int(self.box_size*100)}cm",
            fill='#0088ff',
            font=("Arial", 10, "bold")
        )

    def draw_end_effector(self):
        """Draw end-effector position and trail"""
        # Get current position
        ex, ey, ez = self.ros_node.get_position()

        # Select coordinates based on view
        view = self.view_mode.get()
        if view == "top":
            px, py = ex, ey
        elif view == "side":
            px, py = ex, ez
        else:  # front
            px, py = ey, ez

        # Draw trail
        history = self.ros_node.get_history()
        if len(history) > 1:
            for i in range(len(history) - 1):
                x1, y1, z1 = history[i]
                x2, y2, z2 = history[i + 1]

                if view == "top":
                    p1x, p1y = x1, y1
                    p2x, p2y = x2, y2
                elif view == "side":
                    p1x, p1y = x1, z1
                    p2x, p2y = x2, z2
                else:  # front
                    p1x, p1y = y1, z1
                    p2x, p2y = y2, z2

                c1x, c1y = self.world_to_canvas(p1x, p1y)
                c2x, c2y = self.world_to_canvas(p2x, p2y)

                # Fade trail
                alpha = int(255 * (i + 1) / len(history))
                color = f'#{alpha//4:02x}{alpha//2:02x}{alpha:02x}'

                self.canvas.create_line(c1x, c1y, c2x, c2y, fill=color, width=2)

        # Draw current position (big point)
        cx, cy = self.world_to_canvas(px, py)

        # Glow effect
        self.canvas.create_oval(cx-15, cy-15, cx+15, cy+15, fill='#00ff0030', outline='')
        self.canvas.create_oval(cx-10, cy-10, cx+10, cy+10, fill='#00ff0060', outline='')

        # Main point
        self.canvas.create_oval(cx-6, cy-6, cx+6, cy+6, fill='#00ff00', outline='#00ff00', width=2)

        # Position label
        self.canvas.create_text(
            cx, cy - 25,
            text=f"({px:.3f}, {py:.3f})",
            fill='#00ff00',
            font=("Courier", 10, "bold")
        )

    def update_map(self):
        """Update the map display"""
        # Clear canvas
        self.canvas.delete('all')

        # Draw elements
        self.draw_grid()
        self.draw_axes()
        self.draw_square_box()
        self.draw_end_effector()

        # Update info label
        ex, ey, ez = self.ros_node.get_position()
        self.info_label.config(
            text=f"Position: X={ex:+.3f}m  Y={ey:+.3f}m  Z={ez:+.3f}m  |  View: {self.view_mode.get().upper()}"
        )

        # Schedule next update
        self.root.after(50, self.update_map)  # 20 FPS

    def run(self):
        """Run the GUI"""
        self.root.mainloop()


def ros_spin_thread(node):
    """Spin ROS in separate thread"""
    rclpy.spin(node)


def main(args=None):
    if not GUI_AVAILABLE:
        print("ERROR: tkinter not available")
        return

    rclpy.init(args=args)

    # Create node
    node = EndEffectorTracker()

    # Start ROS thread
    ros_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    ros_thread.start()

    # Wait for initial data
    import time
    time.sleep(1.0)

    # Create and run GUI
    gui = MapGUI(node)

    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
