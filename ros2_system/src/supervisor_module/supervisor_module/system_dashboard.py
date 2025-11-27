#!/usr/bin/env python3
"""
Sorting System Dashboard - UI-based control and monitoring for the sorting brain.

This dashboard provides:
- Real-time state visualization
- Sorted weight display
- Start/Stop/Reset controls
- System status monitoring
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
import tkinter as tk
from tkinter import ttk, scrolledtext
from threading import Thread
import datetime

from std_msgs.msg import String, Int32
from sort_interfaces.msg import BoundingBox


class SortingDashboard(Node):
    """UI Dashboard for monitoring and controlling the sorting system."""

    def __init__(self):
        super().__init__('sorting_dashboard')

        # Publishers
        self.command_pub = self.create_publisher(String, '/sorting/command', 10)

        # Subscribers
        self.state_sub = self.create_subscription(
            String, '/sorting/state', self.state_callback, 10
        )
        self.status_sub = self.create_subscription(
            String, '/sorting/status', self.status_callback, 10
        )
        self.weight_sub = self.create_subscription(
            Int32, '/recognition/estimated_mass', self.weight_callback, 10
        )

        # State variables
        self.current_state = "UNKNOWN"
        self.sorted_weights = []
        self.current_weight = None
        self.last_status = ""
        self.last_gripper_command = "---"

        self.get_logger().info("Sorting Dashboard initialized")

    def state_callback(self, msg: String):
        """Update current state."""
        self.current_state = msg.data

    def status_callback(self, msg: String):
        """Update status log."""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.last_status = f"[{timestamp}] {msg.data}"

        # Extract gripper commands from status messages
        if "gripper" in msg.data.lower() or "opening" in msg.data.lower() or "closing" in msg.data.lower():
            if "Setting grip angle" in msg.data:
                # Extract weight from message like "Setting grip angle for 100g..."
                import re
                match = re.search(r'(\d+)g', msg.data)
                if match:
                    self.last_gripper_command = f"E {match.group(1)}"
            elif "Opening gripper" in msg.data:
                self.last_gripper_command = "W (Open - 5s wait)"
            elif "Closing gripper" in msg.data:
                self.last_gripper_command = "S (Close - 5s wait)"
            elif "opened" in msg.data.lower():
                self.last_gripper_command = "W Complete"
            elif "closed" in msg.data.lower():
                self.last_gripper_command = "S Complete"

    def weight_callback(self, msg: Int32):
        """Track measured weights."""
        self.current_weight = float(msg.data)

    def send_command(self, command: str):
        """Send command to sorting brain."""
        msg = String()
        msg.data = command
        self.command_pub.publish(msg)
        self.get_logger().info(f"Sent command: {command}")


class DashboardGUI:
    """Tkinter GUI for the sorting dashboard."""

    def __init__(self, node: SortingDashboard):
        self.node = node
        self.root = tk.Tk()
        self.root.title("UR5e Sorting System Dashboard")
        self.root.geometry("800x600")
        self.root.configure(bg='#2b2b2b')

        # Configure style
        self.style = ttk.Style()
        self.style.theme_use('clam')
        self.style.configure('TLabel', background='#2b2b2b', foreground='#ffffff', font=('Arial', 10))
        self.style.configure('Title.TLabel', font=('Arial', 16, 'bold'))
        self.style.configure('State.TLabel', font=('Arial', 14, 'bold'), foreground='#4CAF50')
        self.style.configure('TButton', font=('Arial', 10, 'bold'))

        self.setup_ui()
        self.update_display()

    def setup_ui(self):
        """Create the UI layout."""

        # Title
        title_frame = ttk.Frame(self.root)
        title_frame.pack(pady=10, fill=tk.X)

        title = ttk.Label(
            title_frame,
            text="🤖 UR5e Weight Sorting System",
            style='Title.TLabel'
        )
        title.pack()

        # Main content area
        content_frame = ttk.Frame(self.root)
        content_frame.pack(pady=10, padx=20, fill=tk.BOTH, expand=True)

        # Left panel - State and Controls
        left_panel = ttk.Frame(content_frame)
        left_panel.pack(side=tk.LEFT, fill=tk.BOTH, expand=True, padx=(0, 10))

        # Current State
        state_frame = ttk.LabelFrame(left_panel, text="Current State", padding=10)
        state_frame.pack(pady=5, fill=tk.X)

        self.state_label = ttk.Label(
            state_frame,
            text="IDLE",
            style='State.TLabel'
        )
        self.state_label.pack()

        # Current Weight
        weight_frame = ttk.LabelFrame(left_panel, text="Current Weight", padding=10)
        weight_frame.pack(pady=5, fill=tk.X)

        self.weight_label = ttk.Label(
            weight_frame,
            text="--- g",
            font=('Arial', 12)
        )
        self.weight_label.pack()

        # Gripper Status
        gripper_frame = ttk.LabelFrame(left_panel, text="Gripper Command", padding=10)
        gripper_frame.pack(pady=5, fill=tk.X)

        self.gripper_label = ttk.Label(
            gripper_frame,
            text="---",
            font=('Arial', 11, 'bold'),
            foreground='#FFA726'
        )
        self.gripper_label.pack()

        # Sorted Weights Display
        sorted_frame = ttk.LabelFrame(left_panel, text="Sorted Weights (Left → Right)", padding=10)
        sorted_frame.pack(pady=5, fill=tk.BOTH, expand=True)

        # Canvas for visual weight display
        self.sorted_canvas = tk.Canvas(
            sorted_frame,
            bg='#1e1e1e',
            height=100,
            highlightthickness=0
        )
        self.sorted_canvas.pack(fill=tk.BOTH, expand=True)

        # Sorted weights text
        self.sorted_text = ttk.Label(
            sorted_frame,
            text="No weights sorted yet",
            font=('Arial', 10)
        )
        self.sorted_text.pack(pady=5)

        # Control Buttons
        control_frame = ttk.LabelFrame(left_panel, text="Controls", padding=10)
        control_frame.pack(pady=5, fill=tk.X)

        btn_style = {'width': 15}

        self.start_btn = tk.Button(
            control_frame,
            text="▶ Start",
            bg='#4CAF50',
            fg='white',
            font=('Arial', 10, 'bold'),
            command=self.start_sorting,
            **btn_style
        )
        self.start_btn.pack(pady=5, padx=10)

        self.stop_btn = tk.Button(
            control_frame,
            text="⏸ Stop",
            bg='#FF9800',
            fg='white',
            font=('Arial', 10, 'bold'),
            command=self.stop_sorting,
            **btn_style
        )
        self.stop_btn.pack(pady=5, padx=10)

        self.reset_btn = tk.Button(
            control_frame,
            text="🔄 Reset",
            bg='#2196F3',
            fg='white',
            font=('Arial', 10, 'bold'),
            command=self.reset_system,
            **btn_style
        )
        self.reset_btn.pack(pady=5, padx=10)

        self.emergency_btn = tk.Button(
            control_frame,
            text="⚠ Emergency Stop",
            bg='#F44336',
            fg='white',
            font=('Arial', 10, 'bold'),
            command=self.emergency_stop,
            **btn_style
        )
        self.emergency_btn.pack(pady=5, padx=10)

        # Right panel - Status Log
        right_panel = ttk.Frame(content_frame)
        right_panel.pack(side=tk.RIGHT, fill=tk.BOTH, expand=True)

        log_frame = ttk.LabelFrame(right_panel, text="System Status Log", padding=10)
        log_frame.pack(fill=tk.BOTH, expand=True)

        self.status_log = scrolledtext.ScrolledText(
            log_frame,
            wrap=tk.WORD,
            bg='#1e1e1e',
            fg='#00ff00',
            font=('Courier', 9),
            height=20
        )
        self.status_log.pack(fill=tk.BOTH, expand=True)

        # Footer
        footer_frame = ttk.Frame(self.root)
        footer_frame.pack(pady=10, fill=tk.X)

        self.connection_label = ttk.Label(
            footer_frame,
            text="🟢 Connected to ROS2",
            font=('Arial', 9)
        )
        self.connection_label.pack()

    def update_display(self):
        """Update the display with current data from ROS2 node."""

        # Update state
        self.state_label.config(text=self.node.current_state)

        # Update gripper command
        self.gripper_label.config(text=self.node.last_gripper_command)

        # Color code state
        state_colors = {
            'IDLE': '#888888',
            'WAITING_FOR_DETECTION': '#2196F3',
            'PICKING': '#FF9800',
            'WEIGHING': '#9C27B0',
            'DECIDING_PLACEMENT': '#FFC107',
            'REARRANGING': '#FF5722',
            'PLACING': '#4CAF50',
            'ERROR': '#F44336'
        }
        color = state_colors.get(self.node.current_state, '#888888')
        self.state_label.config(foreground=color)

        # Update current weight
        if self.node.current_weight is not None:
            self.weight_label.config(text=f"{self.node.current_weight:.0f} g")
        else:
            self.weight_label.config(text="--- g")

        # Update sorted weights display
        # TODO: Subscribe to placed weights topic to get actual sorted list

        # Update status log
        if self.node.last_status:
            self.status_log.insert(tk.END, self.node.last_status + "\n")
            self.status_log.see(tk.END)
            # Keep log size manageable
            lines = int(self.status_log.index('end-1c').split('.')[0])
            if lines > 1000:
                self.status_log.delete('1.0', '500.0')

        # Schedule next update
        self.root.after(100, self.update_display)

    def start_sorting(self):
        """Send start command."""
        self.node.send_command("start")
        self.log_message("▶ START command sent")

    def stop_sorting(self):
        """Send stop command."""
        self.node.send_command("stop")
        self.log_message("⏸ STOP command sent")

    def reset_system(self):
        """Send reset command."""
        self.node.send_command("reset")
        self.log_message("🔄 RESET command sent")

    def emergency_stop(self):
        """Send emergency stop command."""
        self.node.send_command("emergency_stop")
        self.log_message("⚠ EMERGENCY STOP activated!")

    def log_message(self, message: str):
        """Add message to status log."""
        timestamp = datetime.datetime.now().strftime("%H:%M:%S")
        self.status_log.insert(tk.END, f"[{timestamp}] {message}\n")
        self.status_log.see(tk.END)

    def run(self):
        """Start the GUI main loop."""
        self.root.mainloop()


def main(args=None):
    """Main entry point."""
    rclpy.init(args=args)

    # Create ROS2 node
    dashboard_node = SortingDashboard()

    # Create GUI in main thread
    gui = DashboardGUI(dashboard_node)

    # Run ROS2 executor in background thread
    executor = MultiThreadedExecutor()
    executor.add_node(dashboard_node)

    def spin_node():
        try:
            executor.spin()
        except Exception as e:
            dashboard_node.get_logger().error(f"Executor error: {e}")

    ros_thread = Thread(target=spin_node, daemon=True)
    ros_thread.start()

    # Run GUI (blocks until window closed)
    try:
        gui.run()
    except KeyboardInterrupt:
        pass
    finally:
        dashboard_node.get_logger().info("Dashboard shutting down...")
        executor.shutdown()
        dashboard_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
