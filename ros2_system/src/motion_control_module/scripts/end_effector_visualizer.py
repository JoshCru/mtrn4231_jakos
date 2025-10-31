#!/usr/bin/env python3
"""
End Effector Position Visualizer GUI
Real-time visualization of the UR5e end-effector position in 3D space.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
import numpy as np
import sys

try:
    import tkinter as tk
    from tkinter import ttk
    import threading
    GUI_AVAILABLE = True
except ImportError:
    print("ERROR: tkinter not available")
    GUI_AVAILABLE = False

try:
    from tf2_ros import TransformListener, Buffer
    from tf2_ros import LookupException, ConnectivityException, ExtrapolationException
    TF_AVAILABLE = True
except ImportError:
    print("WARNING: tf2_ros not available, using forward kinematics")
    TF_AVAILABLE = False


class EndEffectorVisualizer(Node):
    def __init__(self):
        super().__init__('end_effector_visualizer')

        # End-effector position
        self.ee_position = {'x': 0.0, 'y': 0.0, 'z': 0.0}
        self.ee_orientation = {'roll': 0.0, 'pitch': 0.0, 'yaw': 0.0}
        self.joint_positions = {}

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Try to use TF for accurate position
        if TF_AVAILABLE:
            self.tf_buffer = Buffer()
            self.tf_listener = TransformListener(self.tf_buffer, self)
            self.use_tf = True
        else:
            self.use_tf = False

        # Timer to update TF
        self.create_timer(0.1, self.update_tf_position)

        self.get_logger().info("End Effector Visualizer initialized")
        self.get_logger().info(f"Using TF: {self.use_tf}")

    def joint_state_callback(self, msg):
        """Update joint positions"""
        for name, position in zip(msg.name, msg.position):
            self.joint_positions[name] = position

        # If not using TF, compute forward kinematics
        if not self.use_tf:
            self.compute_forward_kinematics()

    def update_tf_position(self):
        """Get end-effector position from TF"""
        if not self.use_tf:
            return

        try:
            # Look up transform from base to tool0 (end-effector)
            transform = self.tf_buffer.lookup_transform(
                'base_link',
                'tool0',
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.1)
            )

            # Extract position
            self.ee_position['x'] = transform.transform.translation.x
            self.ee_position['y'] = transform.transform.translation.y
            self.ee_position['z'] = transform.transform.translation.z

            # Extract orientation (convert quaternion to euler angles)
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            # Convert to euler angles
            roll, pitch, yaw = self.quaternion_to_euler(qx, qy, qz, qw)
            self.ee_orientation['roll'] = roll
            self.ee_orientation['pitch'] = pitch
            self.ee_orientation['yaw'] = yaw

        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            pass  # TF not available yet

    def quaternion_to_euler(self, x, y, z, w):
        """Convert quaternion to euler angles"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        return roll, pitch, yaw

    def compute_forward_kinematics(self):
        """Simplified forward kinematics (approximation)"""
        # This is a simplified version - not exact but good enough for visualization
        # Real FK would use DH parameters

        # Get joint angles
        joints = self.joint_positions

        if 'shoulder_pan_joint' not in joints:
            return

        # UR5e approximate link lengths (meters)
        d1 = 0.089159  # Shoulder height
        a2 = -0.425    # Upper arm length
        a3 = -0.39225  # Forearm length
        d4 = 0.10915   # Wrist 1 length
        d5 = 0.09465   # Wrist 2 length
        d6 = 0.0823    # Wrist 3 length

        q1 = joints.get('shoulder_pan_joint', 0.0)
        q2 = joints.get('shoulder_lift_joint', 0.0)
        q3 = joints.get('elbow_joint', 0.0)
        q4 = joints.get('wrist_1_joint', 0.0)
        q5 = joints.get('wrist_2_joint', 0.0)
        q6 = joints.get('wrist_3_joint', 0.0)

        # Simplified forward kinematics (approximate)
        # This gives approximate position of end-effector

        # Calculate position in the plane of the arm
        # First, calculate the reach in the arm's plane
        r_reach = a2 * np.cos(q2) + a3 * np.cos(q2 + q3)
        z_reach = d1 + a2 * np.sin(q2) + a3 * np.sin(q2 + q3)

        # Add wrist contribution (approximate)
        wrist_extension = d4 + d5 + d6
        r_reach += wrist_extension * np.cos(q2 + q3 + q4)
        z_reach += wrist_extension * np.sin(q2 + q3 + q4)

        # Rotate by shoulder pan angle
        self.ee_position['x'] = r_reach * np.cos(q1)
        self.ee_position['y'] = r_reach * np.sin(q1)
        self.ee_position['z'] = z_reach

        # Approximate orientation
        self.ee_orientation['roll'] = q5
        self.ee_orientation['pitch'] = q2 + q3 + q4
        self.ee_orientation['yaw'] = q1 + q6

    def get_position(self):
        """Get current end-effector position"""
        return self.ee_position.copy()

    def get_orientation(self):
        """Get current end-effector orientation"""
        return self.ee_orientation.copy()

    def get_joint_positions(self):
        """Get current joint positions"""
        return self.joint_positions.copy()


class VisualizerGUI:
    def __init__(self, ros_node):
        self.ros_node = ros_node

        # Create main window
        self.root = tk.Tk()
        self.root.title("UR5e End-Effector Position Visualizer")
        self.root.geometry("800x600")
        self.root.configure(bg='#2b2b2b')

        # Create main frame
        main_frame = tk.Frame(self.root, bg='#2b2b2b')
        main_frame.pack(fill=tk.BOTH, expand=True, padx=10, pady=10)

        # Title
        title_label = tk.Label(
            main_frame,
            text="UR5e End-Effector Position",
            font=("Arial", 20, "bold"),
            bg='#2b2b2b',
            fg='#00ff00'
        )
        title_label.pack(pady=(0, 20))

        # Position display
        pos_frame = tk.LabelFrame(
            main_frame,
            text="Position (meters)",
            font=("Arial", 14, "bold"),
            bg='#3b3b3b',
            fg='#00ff00',
            padx=20,
            pady=20
        )
        pos_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        self.pos_labels = {}
        for axis in ['X', 'Y', 'Z']:
            frame = tk.Frame(pos_frame, bg='#3b3b3b')
            frame.pack(fill=tk.X, pady=5)

            label = tk.Label(
                frame,
                text=f"{axis}:",
                font=("Arial", 16, "bold"),
                bg='#3b3b3b',
                fg='#ffffff',
                width=3
            )
            label.pack(side=tk.LEFT, padx=(0, 10))

            value_label = tk.Label(
                frame,
                text="0.000",
                font=("Courier", 20, "bold"),
                bg='#1b1b1b',
                fg='#00ff00',
                width=12,
                anchor='e',
                relief=tk.SUNKEN,
                padx=10,
                pady=5
            )
            value_label.pack(side=tk.LEFT, fill=tk.X, expand=True)

            self.pos_labels[axis.lower()] = value_label

        # Orientation display
        ori_frame = tk.LabelFrame(
            main_frame,
            text="Orientation (degrees)",
            font=("Arial", 14, "bold"),
            bg='#3b3b3b',
            fg='#00aaff',
            padx=20,
            pady=20
        )
        ori_frame.pack(fill=tk.BOTH, expand=True, pady=(0, 10))

        self.ori_labels = {}
        for axis in ['Roll', 'Pitch', 'Yaw']:
            frame = tk.Frame(ori_frame, bg='#3b3b3b')
            frame.pack(fill=tk.X, pady=5)

            label = tk.Label(
                frame,
                text=f"{axis}:",
                font=("Arial", 14, "bold"),
                bg='#3b3b3b',
                fg='#ffffff',
                width=6
            )
            label.pack(side=tk.LEFT, padx=(0, 10))

            value_label = tk.Label(
                frame,
                text="0.0",
                font=("Courier", 18, "bold"),
                bg='#1b1b1b',
                fg='#00aaff',
                width=12,
                anchor='e',
                relief=tk.SUNKEN,
                padx=10,
                pady=5
            )
            value_label.pack(side=tk.LEFT, fill=tk.X, expand=True)

            self.ori_labels[axis.lower()] = value_label

        # Status bar
        self.status_label = tk.Label(
            main_frame,
            text="Status: Waiting for data...",
            font=("Arial", 10),
            bg='#2b2b2b',
            fg='#888888',
            anchor='w'
        )
        self.status_label.pack(fill=tk.X, pady=(10, 0))

        # Update timer
        self.update_display()

    def update_display(self):
        """Update the display with current position"""
        try:
            # Get position
            pos = self.ros_node.get_position()
            for axis, label in self.pos_labels.items():
                value = pos[axis]
                label.config(text=f"{value:+.3f}")

            # Get orientation
            ori = self.ros_node.get_orientation()
            for axis, label in self.ori_labels.items():
                value = np.degrees(ori[axis])
                label.config(text=f"{value:+.1f}°")

            # Update status
            self.status_label.config(
                text=f"Status: Connected | Position: ({pos['x']:.3f}, {pos['y']:.3f}, {pos['z']:.3f})",
                fg='#00ff00'
            )

        except Exception as e:
            self.status_label.config(
                text=f"Status: Error - {str(e)}",
                fg='#ff0000'
            )

        # Schedule next update
        self.root.after(100, self.update_display)

    def run(self):
        """Run the GUI"""
        self.root.mainloop()


def ros_spin_thread(node):
    """Spin ROS node in a separate thread"""
    rclpy.spin(node)


def main(args=None):
    if not GUI_AVAILABLE:
        print("ERROR: tkinter not available")
        return

    rclpy.init(args=args)

    # Create ROS node
    node = EndEffectorVisualizer()

    # Start ROS spinning in a separate thread
    ros_thread = threading.Thread(target=ros_spin_thread, args=(node,), daemon=True)
    ros_thread.start()

    # Wait a bit for initial data
    import time
    time.sleep(1.0)

    # Create and run GUI (must be in main thread)
    gui = VisualizerGUI(node)

    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
