#!/usr/bin/env python3
"""
System Dashboard UI for MTRN4231 Sort-by-Weight Robot System
Provides comprehensive monitoring and control of all modules
"""

import sys
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QPushButton, QLabel, QTextEdit, QTableWidget, QTableWidgetItem,
    QTabWidget, QGridLayout, QProgressBar, QFrame, QSplitter
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QColor, QFont

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, Float32, Bool
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseArray
from sort_interfaces.msg import (
    DetectedObjects, WeightEstimate, SortDecision,
    ForceFeedback, TargetArea, EnvironmentStatus
)
from sort_interfaces.srv import SystemCommand, GripperControl, CalibrateGripper


class ROS2Thread(QThread):
    """Separate thread for ROS2 spinning"""
    def __init__(self, executor):
        super().__init__()
        self.executor = executor
        self._running = True

    def run(self):
        while self._running and rclpy.ok():
            self.executor.spin_once(timeout_sec=0.1)

    def stop(self):
        self._running = False


class SystemDashboardNode(Node):
    """ROS2 Node for dashboard backend"""

    # Signals for thread-safe UI updates
    system_status_signal = pyqtSignal(str)
    gripper_state_signal = pyqtSignal(float)
    force_feedback_signal = pyqtSignal(object)
    detected_objects_signal = pyqtSignal(object)
    weight_estimate_signal = pyqtSignal(object)
    sort_decision_signal = pyqtSignal(object)
    robot_status_signal = pyqtSignal(str)
    joint_states_signal = pyqtSignal(object)

    def __init__(self):
        super().__init__('system_dashboard_node')

        # Subscribers
        self.create_subscription(String, '/system/status', self.system_status_callback, 10)
        self.create_subscription(Float32, '/motion_control/gripper_state', self.gripper_state_callback, 10)
        self.create_subscription(ForceFeedback, '/motion_control/force_feedback', self.force_feedback_callback, 10)
        self.create_subscription(DetectedObjects, '/perception/detected_objects', self.detected_objects_callback, 10)
        self.create_subscription(WeightEstimate, '/recognition/estimated_weights', self.weight_estimate_callback, 10)
        self.create_subscription(SortDecision, '/planning/sort_decisions', self.sort_decision_callback, 10)
        self.create_subscription(String, '/control/robot_status', self.robot_status_callback, 10)
        self.create_subscription(JointState, '/control/joint_states', self.joint_states_callback, 10)

        # Service clients
        self.start_client = self.create_client(SystemCommand, '/system/start')
        self.stop_client = self.create_client(SystemCommand, '/system/stop')
        self.emergency_stop_client = self.create_client(SystemCommand, '/system/emergency_stop')
        self.gripper_control_client = self.create_client(GripperControl, '/motion_control/gripper_control')
        self.calibrate_gripper_client = self.create_client(CalibrateGripper, '/motion_control/calibrate_gripper')

        self.get_logger().info('System Dashboard Node initialized')

    def system_status_callback(self, msg):
        self.system_status_signal.emit(msg.data)

    def gripper_state_callback(self, msg):
        self.gripper_state_signal.emit(msg.data)

    def force_feedback_callback(self, msg):
        self.force_feedback_signal.emit(msg)

    def detected_objects_callback(self, msg):
        self.detected_objects_signal.emit(msg)

    def weight_estimate_callback(self, msg):
        self.weight_estimate_signal.emit(msg)

    def sort_decision_callback(self, msg):
        self.sort_decision_signal.emit(msg)

    def robot_status_callback(self, msg):
        self.robot_status_signal.emit(msg.data)

    def joint_states_callback(self, msg):
        self.joint_states_signal.emit(msg)


class SystemDashboard(QMainWindow):
    """Main Dashboard Window"""

    def __init__(self, ros_node):
        super().__init__()
        self.ros_node = ros_node

        # Connect signals
        self.ros_node.system_status_signal.connect(self.update_system_status)
        self.ros_node.gripper_state_signal.connect(self.update_gripper_state)
        self.ros_node.force_feedback_signal.connect(self.update_force_feedback)
        self.ros_node.detected_objects_signal.connect(self.update_detected_objects)
        self.ros_node.weight_estimate_signal.connect(self.update_weight_estimate)
        self.ros_node.sort_decision_signal.connect(self.update_sort_decision)
        self.ros_node.robot_status_signal.connect(self.update_robot_status)
        self.ros_node.joint_states_signal.connect(self.update_joint_states)

        # Initialize data
        self.system_state = "Unknown"
        self.gripper_position = 0.0
        self.measured_weight = 0.0
        self.object_detected = False
        self.num_detected_objects = 0
        self.estimated_weight = 0.0
        self.robot_status = "Unknown"

        self.init_ui()

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('MTRN4231 Sort-by-Weight Robot - System Dashboard')
        self.setGeometry(100, 100, 1400, 900)

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)

        # Main layout
        main_layout = QVBoxLayout(central_widget)

        # Title
        title = QLabel('Sort-by-Weight Robot System Control Dashboard')
        title_font = QFont()
        title_font.setPointSize(16)
        title_font.setBold(True)
        title.setFont(title_font)
        title.setAlignment(Qt.AlignCenter)
        main_layout.addWidget(title)

        # Create tabs
        tabs = QTabWidget()

        # Tab 1: System Control
        tabs.addTab(self.create_control_tab(), "System Control")

        # Tab 2: Module Status
        tabs.addTab(self.create_status_tab(), "Module Status")

        # Tab 3: Topic Monitor
        tabs.addTab(self.create_monitor_tab(), "Topic Monitor")

        # Tab 4: Logs
        tabs.addTab(self.create_logs_tab(), "System Logs")

        main_layout.addWidget(tabs)

        # Status bar
        self.statusBar().showMessage('Ready')

    def create_control_tab(self):
        """Create system control tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # System Control Group
        system_group = QGroupBox("System Control")
        system_layout = QGridLayout()

        # System status display
        self.system_status_label = QLabel("IDLE")
        self.system_status_label.setStyleSheet(
            "QLabel { background-color: gray; color: white; padding: 10px; "
            "font-size: 18px; font-weight: bold; border-radius: 5px; }")
        self.system_status_label.setAlignment(Qt.AlignCenter)
        system_layout.addWidget(QLabel("System Status:"), 0, 0)
        system_layout.addWidget(self.system_status_label, 0, 1, 1, 3)

        # Control buttons
        self.start_btn = QPushButton("START SYSTEM")
        self.start_btn.setStyleSheet("QPushButton { background-color: green; color: white; padding: 15px; font-size: 14px; }")
        self.start_btn.clicked.connect(self.start_system)
        system_layout.addWidget(self.start_btn, 1, 0)

        self.stop_btn = QPushButton("STOP SYSTEM")
        self.stop_btn.setStyleSheet("QPushButton { background-color: orange; color: white; padding: 15px; font-size: 14px; }")
        self.stop_btn.clicked.connect(self.stop_system)
        system_layout.addWidget(self.stop_btn, 1, 1)

        self.emergency_btn = QPushButton("EMERGENCY STOP")
        self.emergency_btn.setStyleSheet("QPushButton { background-color: red; color: white; padding: 15px; font-size: 14px; font-weight: bold; }")
        self.emergency_btn.clicked.connect(self.emergency_stop)
        system_layout.addWidget(self.emergency_btn, 1, 2, 1, 2)

        system_group.setLayout(system_layout)
        layout.addWidget(system_group)

        # Gripper Control Group
        gripper_group = QGroupBox("Gripper Control")
        gripper_layout = QGridLayout()

        # Gripper position
        gripper_layout.addWidget(QLabel("Position:"), 0, 0)
        self.gripper_position_label = QLabel("0.00")
        gripper_layout.addWidget(self.gripper_position_label, 0, 1)
        self.gripper_progress = QProgressBar()
        self.gripper_progress.setMaximum(100)
        gripper_layout.addWidget(self.gripper_progress, 0, 2, 1, 2)

        # Force feedback
        gripper_layout.addWidget(QLabel("Measured Weight:"), 1, 0)
        self.weight_label = QLabel("0.0 g")
        gripper_layout.addWidget(self.weight_label, 1, 1)

        gripper_layout.addWidget(QLabel("Object Detected:"), 1, 2)
        self.object_detected_label = QLabel("No")
        gripper_layout.addWidget(self.object_detected_label, 1, 3)

        # Gripper control buttons
        self.open_gripper_btn = QPushButton("Open Gripper")
        self.open_gripper_btn.clicked.connect(self.open_gripper)
        gripper_layout.addWidget(self.open_gripper_btn, 2, 0)

        self.close_gripper_btn = QPushButton("Close Gripper")
        self.close_gripper_btn.clicked.connect(self.close_gripper)
        gripper_layout.addWidget(self.close_gripper_btn, 2, 1)

        self.calibrate_gripper_btn = QPushButton("Calibrate Gripper")
        self.calibrate_gripper_btn.clicked.connect(self.calibrate_gripper)
        gripper_layout.addWidget(self.calibrate_gripper_btn, 2, 2, 1, 2)

        gripper_group.setLayout(gripper_layout)
        layout.addWidget(gripper_group)

        # Robot Status Group
        robot_group = QGroupBox("Robot Arm Status")
        robot_layout = QGridLayout()

        robot_layout.addWidget(QLabel("Status:"), 0, 0)
        self.robot_status_label = QLabel("Unknown")
        robot_layout.addWidget(self.robot_status_label, 0, 1, 1, 3)

        robot_layout.addWidget(QLabel("Joint States:"), 1, 0)
        self.joint_states_text = QTextEdit()
        self.joint_states_text.setMaximumHeight(100)
        self.joint_states_text.setReadOnly(True)
        robot_layout.addWidget(self.joint_states_text, 1, 1, 2, 3)

        robot_group.setLayout(robot_layout)
        layout.addWidget(robot_group)

        layout.addStretch()
        return widget

    def create_status_tab(self):
        """Create module status tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Module status table
        status_group = QGroupBox("Module Status Overview")
        status_layout = QVBoxLayout()

        self.module_table = QTableWidget()
        self.module_table.setColumnCount(3)
        self.module_table.setHorizontalHeaderLabels(["Module", "Status", "Last Update"])
        self.module_table.setRowCount(6)

        modules = ["Supervisor", "Perception", "Recognition", "Planning", "Control", "Gripper"]
        for i, module in enumerate(modules):
            self.module_table.setItem(i, 0, QTableWidgetItem(module))
            self.module_table.setItem(i, 1, QTableWidgetItem("Unknown"))
            self.module_table.setItem(i, 2, QTableWidgetItem("Never"))

        self.module_table.resizeColumnsToContents()
        status_layout.addWidget(self.module_table)
        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        # Perception data
        perception_group = QGroupBox("Perception Data")
        perception_layout = QGridLayout()

        perception_layout.addWidget(QLabel("Detected Objects:"), 0, 0)
        self.detected_objects_label = QLabel("0")
        perception_layout.addWidget(self.detected_objects_label, 0, 1)

        perception_group.setLayout(perception_layout)
        layout.addWidget(perception_group)

        # Recognition data
        recognition_group = QGroupBox("Recognition Data")
        recognition_layout = QGridLayout()

        recognition_layout.addWidget(QLabel("Estimated Weight:"), 0, 0)
        self.estimated_weight_label = QLabel("0.0 g")
        recognition_layout.addWidget(self.estimated_weight_label, 0, 1)

        recognition_group.setLayout(recognition_layout)
        layout.addWidget(recognition_group)

        # Planning data
        planning_group = QGroupBox("Planning Data")
        planning_layout = QGridLayout()

        planning_layout.addWidget(QLabel("Last Sort Decision:"), 0, 0)
        self.sort_decision_label = QLabel("None")
        planning_layout.addWidget(self.sort_decision_label, 0, 1, 1, 3)

        planning_group.setLayout(planning_layout)
        layout.addWidget(planning_group)

        layout.addStretch()
        return widget

    def create_monitor_tab(self):
        """Create topic monitor tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        monitor_group = QGroupBox("Topic Monitor")
        monitor_layout = QVBoxLayout()

        self.topic_monitor_text = QTextEdit()
        self.topic_monitor_text.setReadOnly(True)
        self.topic_monitor_text.setMaximumHeight(600)
        monitor_layout.addWidget(self.topic_monitor_text)

        clear_btn = QPushButton("Clear Monitor")
        clear_btn.clicked.connect(lambda: self.topic_monitor_text.clear())
        monitor_layout.addWidget(clear_btn)

        monitor_group.setLayout(monitor_layout)
        layout.addWidget(monitor_group)

        return widget

    def create_logs_tab(self):
        """Create system logs tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        logs_group = QGroupBox("System Logs")
        logs_layout = QVBoxLayout()

        self.logs_text = QTextEdit()
        self.logs_text.setReadOnly(True)
        logs_layout.addWidget(self.logs_text)

        clear_logs_btn = QPushButton("Clear Logs")
        clear_logs_btn.clicked.connect(lambda: self.logs_text.clear())
        logs_layout.addWidget(clear_logs_btn)

        logs_group.setLayout(logs_layout)
        layout.addWidget(logs_group)

        return widget

    # System control methods
    def start_system(self):
        """Start the system"""
        self.log_message("Starting system...")
        if self.ros_node.start_client.wait_for_service(timeout_sec=1.0):
            request = SystemCommand.Request()
            request.command = "start"
            future = self.ros_node.start_client.call_async(request)
            future.add_done_callback(lambda f: self.log_message(f"System start: {f.result().message}"))
        else:
            self.log_message("Start service not available!", level="ERROR")

    def stop_system(self):
        """Stop the system"""
        self.log_message("Stopping system...")
        if self.ros_node.stop_client.wait_for_service(timeout_sec=1.0):
            request = SystemCommand.Request()
            request.command = "stop"
            future = self.ros_node.stop_client.call_async(request)
            future.add_done_callback(lambda f: self.log_message(f"System stop: {f.result().message}"))
        else:
            self.log_message("Stop service not available!", level="ERROR")

    def emergency_stop(self):
        """Emergency stop"""
        self.log_message("EMERGENCY STOP ACTIVATED!", level="ERROR")
        if self.ros_node.emergency_stop_client.wait_for_service(timeout_sec=1.0):
            request = SystemCommand.Request()
            request.command = "emergency_stop"
            future = self.ros_node.emergency_stop_client.call_async(request)
            future.add_done_callback(lambda f: self.log_message(f"Emergency stop: {f.result().message}", level="ERROR"))
        else:
            self.log_message("Emergency stop service not available!", level="ERROR")

    def open_gripper(self):
        """Open gripper"""
        self.log_message("Opening gripper...")
        if self.ros_node.gripper_control_client.wait_for_service(timeout_sec=1.0):
            request = GripperControl.Request()
            request.command = "open"
            future = self.ros_node.gripper_control_client.call_async(request)
            future.add_done_callback(lambda f: self.log_message(f"Gripper: {f.result().message}"))
        else:
            self.log_message("Gripper control service not available!", level="WARN")

    def close_gripper(self):
        """Close gripper"""
        self.log_message("Closing gripper...")
        if self.ros_node.gripper_control_client.wait_for_service(timeout_sec=1.0):
            request = GripperControl.Request()
            request.command = "close"
            future = self.ros_node.gripper_control_client.call_async(request)
            future.add_done_callback(lambda f: self.log_message(f"Gripper: {f.result().message}"))
        else:
            self.log_message("Gripper control service not available!", level="WARN")

    def calibrate_gripper(self):
        """Calibrate gripper"""
        self.log_message("Calibrating gripper...")
        if self.ros_node.calibrate_gripper_client.wait_for_service(timeout_sec=1.0):
            request = CalibrateGripper.Request()
            request.tare_weight_sensor = True
            request.calibrate_position = False
            future = self.ros_node.calibrate_gripper_client.call_async(request)
            future.add_done_callback(lambda f: self.log_message(f"Calibration: {f.result().message}"))
        else:
            self.log_message("Calibration service not available!", level="WARN")

    # Update methods
    def update_system_status(self, status):
        """Update system status display"""
        self.system_state = status
        self.system_status_label.setText(status.upper())

        # Update color based on status
        if status == "running":
            self.system_status_label.setStyleSheet(
                "QLabel { background-color: green; color: white; padding: 10px; "
                "font-size: 18px; font-weight: bold; border-radius: 5px; }")
        elif status == "stopped":
            self.system_status_label.setStyleSheet(
                "QLabel { background-color: orange; color: white; padding: 10px; "
                "font-size: 18px; font-weight: bold; border-radius: 5px; }")
        elif status == "emergency_stopped":
            self.system_status_label.setStyleSheet(
                "QLabel { background-color: red; color: white; padding: 10px; "
                "font-size: 18px; font-weight: bold; border-radius: 5px; }")
        else:
            self.system_status_label.setStyleSheet(
                "QLabel { background-color: gray; color: white; padding: 10px; "
                "font-size: 18px; font-weight: bold; border-radius: 5px; }")

        self.update_module_status("Supervisor", "Active")

    def update_gripper_state(self, position):
        """Update gripper state"""
        self.gripper_position = position
        self.gripper_position_label.setText(f"{position:.2f}")
        self.gripper_progress.setValue(int(position * 100))
        self.update_module_status("Gripper", "Active")

    def update_force_feedback(self, msg):
        """Update force feedback"""
        self.measured_weight = msg.measured_weight
        self.object_detected = msg.object_detected
        self.weight_label.setText(f"{msg.measured_weight:.1f} g")
        self.object_detected_label.setText("Yes" if msg.object_detected else "No")

        if msg.object_detected:
            self.object_detected_label.setStyleSheet("QLabel { color: green; font-weight: bold; }")
        else:
            self.object_detected_label.setStyleSheet("QLabel { color: gray; }")

        self.monitor_log(f"Force: {msg.gripper_force:.2f}N, Weight: {msg.measured_weight:.1f}g")

    def update_detected_objects(self, msg):
        """Update detected objects"""
        self.num_detected_objects = msg.count
        self.detected_objects_label.setText(str(msg.count))
        self.monitor_log(f"Detected {msg.count} objects")
        self.update_module_status("Perception", "Active")

    def update_weight_estimate(self, msg):
        """Update weight estimate"""
        self.estimated_weight = msg.estimated_weight
        self.estimated_weight_label.setText(f"{msg.estimated_weight:.1f} g")
        self.monitor_log(f"Estimated weight: {msg.estimated_weight:.1f}g (confidence: {msg.confidence:.2f})")
        self.update_module_status("Recognition", "Active")

    def update_sort_decision(self, msg):
        """Update sort decision"""
        decision_text = f"Object {msg.object_id} -> Bin {msg.target_bin_id} ({msg.estimated_weight:.1f}g)"
        self.sort_decision_label.setText(decision_text)
        self.monitor_log(f"Sort decision: {decision_text}")
        self.update_module_status("Planning", "Active")

    def update_robot_status(self, status):
        """Update robot status"""
        self.robot_status = status
        self.robot_status_label.setText(status)
        self.update_module_status("Control", "Active")

    def update_joint_states(self, msg):
        """Update joint states"""
        if msg.position:
            joint_text = "\n".join([f"{name}: {pos:.3f}" for name, pos in zip(msg.name, msg.position)])
            self.joint_states_text.setPlainText(joint_text)

    def update_module_status(self, module, status):
        """Update module status in table"""
        from datetime import datetime
        current_time = datetime.now().strftime("%H:%M:%S")

        for row in range(self.module_table.rowCount()):
            if self.module_table.item(row, 0).text() == module:
                self.module_table.setItem(row, 1, QTableWidgetItem(status))
                self.module_table.setItem(row, 2, QTableWidgetItem(current_time))
                break

    def monitor_log(self, message):
        """Add message to topic monitor"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%H:%M:%S.%f")[:-3]
        self.topic_monitor_text.append(f"[{timestamp}] {message}")

        # Keep only last 100 lines
        if self.topic_monitor_text.document().blockCount() > 100:
            cursor = self.topic_monitor_text.textCursor()
            cursor.movePosition(cursor.Start)
            cursor.select(cursor.LineUnderCursor)
            cursor.removeSelectedText()
            cursor.deleteChar()

    def log_message(self, message, level="INFO"):
        """Add message to system logs"""
        from datetime import datetime
        timestamp = datetime.now().strftime("%Y-%m-%d %H:%M:%S")

        if level == "ERROR":
            color = "red"
        elif level == "WARN":
            color = "orange"
        else:
            color = "black"

        self.logs_text.append(f'<span style="color: {color};">[{timestamp}] [{level}] {message}</span>')
        self.statusBar().showMessage(message)


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    # Create QApplication
    app = QApplication(sys.argv)

    # Create ROS2 node
    ros_node = SystemDashboardNode()

    # Create executor
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    # Create ROS2 thread
    ros_thread = ROS2Thread(executor)
    ros_thread.start()

    # Create and show dashboard
    dashboard = SystemDashboard(ros_node)
    dashboard.show()

    # Run Qt event loop
    exit_code = app.exec_()

    # Cleanup
    ros_thread.stop()
    ros_thread.wait()
    ros_node.destroy_node()
    rclpy.shutdown()

    sys.exit(exit_code)


if __name__ == '__main__':
    main()
