#!/usr/bin/env python3
"""
Brain Dashboard - Enhanced UI for monitoring the Brain Node
Provides comprehensive visualization of system state, nodes, topics, and events
"""

import sys
import json
from datetime import datetime
from PyQt5.QtWidgets import (
    QApplication, QMainWindow, QWidget, QVBoxLayout, QHBoxLayout,
    QGroupBox, QPushButton, QLabel, QTextEdit, QTableWidget, QTableWidgetItem,
    QTabWidget, QGridLayout, QProgressBar, QFrame, QSplitter, QTreeWidget,
    QTreeWidgetItem, QHeaderView, QStatusBar, QLineEdit, QDoubleSpinBox,
    QRadioButton, QButtonGroup
)
from PyQt5.QtCore import QTimer, Qt, pyqtSignal, QThread
from PyQt5.QtGui import QColor, QFont, QPalette, QBrush

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String
from sort_interfaces.srv import SystemCommand


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


class SignalEmitter(QThread):
    """Signal emitter for thread-safe UI updates"""
    brain_status_signal = pyqtSignal(str)
    node_health_signal = pyqtSignal(str)
    topic_stats_signal = pyqtSignal(str)
    event_log_signal = pyqtSignal(str)


class BrainDashboardNode(Node):
    """ROS2 Node for Brain Dashboard"""

    def __init__(self, signal_emitter):
        super().__init__('brain_dashboard_node')
        self.signals = signal_emitter

        # Subscribe to brain node topics
        self.create_subscription(
            String, '/brain/status',
            self.brain_status_callback, 10)
        self.create_subscription(
            String, '/brain/node_health',
            self.node_health_callback, 10)
        self.create_subscription(
            String, '/brain/topic_stats',
            self.topic_stats_callback, 10)
        self.create_subscription(
            String, '/brain/event_log',
            self.event_log_callback, 10)

        # Service clients for brain commands
        self.brain_command_client = self.create_client(
            SystemCommand, '/brain/execute_command')
        self.brain_status_client = self.create_client(
            SystemCommand, '/brain/get_status')

        self.get_logger().info('Brain Dashboard Node initialized')

    def brain_status_callback(self, msg):
        self.signals.brain_status_signal.emit(msg.data)

    def node_health_callback(self, msg):
        self.signals.node_health_signal.emit(msg.data)

    def topic_stats_callback(self, msg):
        self.signals.topic_stats_signal.emit(msg.data)

    def event_log_callback(self, msg):
        self.signals.event_log_signal.emit(msg.data)


class BrainDashboard(QMainWindow):
    """Main Brain Dashboard Window"""

    def __init__(self, ros_node, signal_emitter):
        super().__init__()
        self.ros_node = ros_node
        self.signal_emitter = signal_emitter

        # Connect signals
        self.signal_emitter.brain_status_signal.connect(self.update_brain_status)
        self.signal_emitter.node_health_signal.connect(self.update_node_health)
        self.signal_emitter.topic_stats_signal.connect(self.update_topic_stats)
        self.signal_emitter.event_log_signal.connect(self.add_event_log)

        # Data storage
        self.current_state = "UNKNOWN"
        self.uptime = 0.0
        self.node_count = 0
        self.alive_count = 0
        self.topic_count = 0
        self.message_count = 0

        self.init_ui()

    def init_ui(self):
        """Initialize the user interface"""
        self.setWindowTitle('Brain Node - System Supervisor Dashboard')
        self.setGeometry(50, 50, 1600, 1000)

        # Set dark theme
        self.set_dark_theme()

        # Central widget
        central_widget = QWidget()
        self.setCentralWidget(central_widget)
        main_layout = QVBoxLayout(central_widget)

        # Title bar
        title_bar = self.create_title_bar()
        main_layout.addWidget(title_bar)

        # Main content with tabs
        tabs = QTabWidget()
        tabs.setStyleSheet("QTabWidget::pane { border: 1px solid #444; }")

        # Overview tab
        tabs.addTab(self.create_overview_tab(), "Overview")

        # Nodes tab
        tabs.addTab(self.create_nodes_tab(), "Node Monitor")

        # Topics tab
        tabs.addTab(self.create_topics_tab(), "Topic Monitor")

        # Events tab
        tabs.addTab(self.create_events_tab(), "Event Log")

        # Control tab
        tabs.addTab(self.create_control_tab(), "System Control")

        # Motion Control tab
        tabs.addTab(self.create_motion_control_tab(), "Motion Control")

        main_layout.addWidget(tabs)

        # Status bar
        self.statusBar().showMessage('Connecting to Brain Node...')
        self.statusBar().setStyleSheet("QStatusBar { background-color: #2d2d2d; color: #00ff00; }")

    def set_dark_theme(self):
        """Set dark theme for the application"""
        palette = QPalette()
        palette.setColor(QPalette.Window, QColor(53, 53, 53))
        palette.setColor(QPalette.WindowText, Qt.white)
        palette.setColor(QPalette.Base, QColor(35, 35, 35))
        palette.setColor(QPalette.AlternateBase, QColor(53, 53, 53))
        palette.setColor(QPalette.ToolTipBase, QColor(25, 25, 25))
        palette.setColor(QPalette.ToolTipText, Qt.white)
        palette.setColor(QPalette.Text, Qt.white)
        palette.setColor(QPalette.Button, QColor(53, 53, 53))
        palette.setColor(QPalette.ButtonText, Qt.white)
        palette.setColor(QPalette.BrightText, Qt.red)
        palette.setColor(QPalette.Link, QColor(42, 130, 218))
        palette.setColor(QPalette.Highlight, QColor(42, 130, 218))
        palette.setColor(QPalette.HighlightedText, QColor(35, 35, 35))
        self.setPalette(palette)

    def create_title_bar(self):
        """Create title bar with status indicators"""
        frame = QFrame()
        frame.setStyleSheet("QFrame { background-color: #1e1e1e; border-radius: 5px; padding: 10px; }")
        layout = QHBoxLayout(frame)

        # Title
        title = QLabel('BRAIN NODE - CENTRAL ORCHESTRATOR')
        title_font = QFont('Monospace', 18, QFont.Bold)
        title.setFont(title_font)
        title.setStyleSheet("color: #00ff00;")
        layout.addWidget(title)

        layout.addStretch()

        # Status indicator
        self.status_indicator = QLabel('OFFLINE')
        self.status_indicator.setStyleSheet(
            "QLabel { background-color: #ff0000; color: white; padding: 8px 15px; "
            "font-size: 14px; font-weight: bold; border-radius: 5px; }")
        layout.addWidget(self.status_indicator)

        # Uptime
        self.uptime_label = QLabel('Uptime: 0s')
        self.uptime_label.setStyleSheet("color: #aaaaaa; font-size: 12px;")
        layout.addWidget(self.uptime_label)

        return frame

    def create_overview_tab(self):
        """Create overview tab with key metrics"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Metrics grid
        metrics_frame = QFrame()
        metrics_frame.setStyleSheet("QFrame { background-color: #2d2d2d; border-radius: 5px; }")
        metrics_layout = QGridLayout(metrics_frame)

        # System State
        state_group = QGroupBox("System State")
        state_group.setStyleSheet("QGroupBox { font-weight: bold; color: #00ff00; }")
        state_layout = QVBoxLayout()
        self.state_label = QLabel("UNKNOWN")
        self.state_label.setStyleSheet(
            "QLabel { font-size: 24px; font-weight: bold; color: #ffffff; "
            "background-color: #666666; padding: 15px; border-radius: 5px; }")
        self.state_label.setAlignment(Qt.AlignCenter)
        state_layout.addWidget(self.state_label)
        state_group.setLayout(state_layout)
        metrics_layout.addWidget(state_group, 0, 0)

        # Node count
        nodes_group = QGroupBox("Active Nodes")
        nodes_group.setStyleSheet("QGroupBox { font-weight: bold; color: #00aaff; }")
        nodes_layout = QVBoxLayout()
        self.nodes_count_label = QLabel("0 / 0")
        self.nodes_count_label.setStyleSheet(
            "QLabel { font-size: 24px; font-weight: bold; color: #00aaff; }")
        self.nodes_count_label.setAlignment(Qt.AlignCenter)
        nodes_layout.addWidget(self.nodes_count_label)
        nodes_group.setLayout(nodes_layout)
        metrics_layout.addWidget(nodes_group, 0, 1)

        # Topics monitored
        topics_group = QGroupBox("Topics Monitored")
        topics_group.setStyleSheet("QGroupBox { font-weight: bold; color: #ffaa00; }")
        topics_layout = QVBoxLayout()
        self.topics_count_label = QLabel("0")
        self.topics_count_label.setStyleSheet(
            "QLabel { font-size: 24px; font-weight: bold; color: #ffaa00; }")
        self.topics_count_label.setAlignment(Qt.AlignCenter)
        topics_layout.addWidget(self.topics_count_label)
        topics_group.setLayout(topics_layout)
        metrics_layout.addWidget(topics_group, 0, 2)

        # Messages logged
        messages_group = QGroupBox("Messages Logged")
        messages_group.setStyleSheet("QGroupBox { font-weight: bold; color: #ff00ff; }")
        messages_layout = QVBoxLayout()
        self.messages_count_label = QLabel("0")
        self.messages_count_label.setStyleSheet(
            "QLabel { font-size: 24px; font-weight: bold; color: #ff00ff; }")
        self.messages_count_label.setAlignment(Qt.AlignCenter)
        messages_layout.addWidget(self.messages_count_label)
        messages_group.setLayout(messages_layout)
        metrics_layout.addWidget(messages_group, 0, 3)

        layout.addWidget(metrics_frame)

        # Quick actions
        actions_group = QGroupBox("Quick Actions")
        actions_layout = QHBoxLayout()

        self.start_btn = QPushButton("START")
        self.start_btn.setStyleSheet(
            "QPushButton { background-color: #00aa00; color: white; padding: 15px 30px; "
            "font-size: 16px; font-weight: bold; border-radius: 5px; }"
            "QPushButton:hover { background-color: #00cc00; }")
        self.start_btn.clicked.connect(self.send_start_command)
        actions_layout.addWidget(self.start_btn)

        self.stop_btn = QPushButton("STOP")
        self.stop_btn.setStyleSheet(
            "QPushButton { background-color: #ff8800; color: white; padding: 15px 30px; "
            "font-size: 16px; font-weight: bold; border-radius: 5px; }"
            "QPushButton:hover { background-color: #ffaa00; }")
        self.stop_btn.clicked.connect(self.send_stop_command)
        actions_layout.addWidget(self.stop_btn)

        self.emergency_btn = QPushButton("EMERGENCY STOP")
        self.emergency_btn.setStyleSheet(
            "QPushButton { background-color: #ff0000; color: white; padding: 15px 30px; "
            "font-size: 16px; font-weight: bold; border-radius: 5px; }"
            "QPushButton:hover { background-color: #ff3333; }")
        self.emergency_btn.clicked.connect(self.send_emergency_stop)
        actions_layout.addWidget(self.emergency_btn)

        self.reset_btn = QPushButton("RESET")
        self.reset_btn.setStyleSheet(
            "QPushButton { background-color: #0066cc; color: white; padding: 15px 30px; "
            "font-size: 16px; font-weight: bold; border-radius: 5px; }"
            "QPushButton:hover { background-color: #0088ff; }")
        self.reset_btn.clicked.connect(self.send_reset_command)
        actions_layout.addWidget(self.reset_btn)

        actions_group.setLayout(actions_layout)
        layout.addWidget(actions_group)

        # Recent events summary
        events_group = QGroupBox("Recent Events")
        events_layout = QVBoxLayout()
        self.recent_events_text = QTextEdit()
        self.recent_events_text.setReadOnly(True)
        self.recent_events_text.setMaximumHeight(300)
        self.recent_events_text.setStyleSheet(
            "QTextEdit { background-color: #1e1e1e; color: #00ff00; font-family: monospace; }")
        events_layout.addWidget(self.recent_events_text)
        events_group.setLayout(events_layout)
        layout.addWidget(events_group)

        return widget

    def create_nodes_tab(self):
        """Create node monitoring tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Node tree
        nodes_group = QGroupBox("Discovered Nodes")
        nodes_layout = QVBoxLayout()

        self.nodes_tree = QTreeWidget()
        self.nodes_tree.setHeaderLabels(["Node Name", "Status", "Last Seen", "Message Count"])
        self.nodes_tree.header().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.nodes_tree.setStyleSheet(
            "QTreeWidget { background-color: #1e1e1e; color: #ffffff; }"
            "QTreeWidget::item:selected { background-color: #0066cc; }")
        nodes_layout.addWidget(self.nodes_tree)

        # Refresh button
        refresh_btn = QPushButton("Refresh Nodes")
        refresh_btn.clicked.connect(self.refresh_nodes)
        nodes_layout.addWidget(refresh_btn)

        nodes_group.setLayout(nodes_layout)
        layout.addWidget(nodes_group)

        return widget

    def create_topics_tab(self):
        """Create topic monitoring tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Topic table
        topics_group = QGroupBox("Monitored Topics")
        topics_layout = QVBoxLayout()

        self.topics_table = QTableWidget()
        self.topics_table.setColumnCount(4)
        self.topics_table.setHorizontalHeaderLabels(
            ["Topic", "Message Count", "Last Update", "Last Data"])
        self.topics_table.horizontalHeader().setSectionResizeMode(QHeaderView.ResizeToContents)
        self.topics_table.setStyleSheet(
            "QTableWidget { background-color: #1e1e1e; color: #ffffff; gridline-color: #444444; }"
            "QTableWidget::item:selected { background-color: #0066cc; }")
        topics_layout.addWidget(self.topics_table)

        topics_group.setLayout(topics_layout)
        layout.addWidget(topics_group)

        return widget

    def create_events_tab(self):
        """Create event log tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Event log
        events_group = QGroupBox("System Event Log")
        events_layout = QVBoxLayout()

        self.event_log_text = QTextEdit()
        self.event_log_text.setReadOnly(True)
        self.event_log_text.setStyleSheet(
            "QTextEdit { background-color: #1e1e1e; color: #00ff00; font-family: monospace; font-size: 11px; }")
        events_layout.addWidget(self.event_log_text)

        # Controls
        controls_layout = QHBoxLayout()
        clear_btn = QPushButton("Clear Log")
        clear_btn.clicked.connect(lambda: self.event_log_text.clear())
        controls_layout.addWidget(clear_btn)

        export_btn = QPushButton("Export Log")
        export_btn.clicked.connect(self.export_log)
        controls_layout.addWidget(export_btn)

        controls_layout.addStretch()
        events_layout.addLayout(controls_layout)

        events_group.setLayout(events_layout)
        layout.addWidget(events_group)

        return widget

    def create_control_tab(self):
        """Create system control tab"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # Brain commands
        commands_group = QGroupBox("Brain Node Commands")
        commands_layout = QGridLayout()

        # Command input
        commands_layout.addWidget(QLabel("Custom Command:"), 0, 0)
        self.command_input = QLineEdit()
        self.command_input.setPlaceholderText("Enter command (e.g., get_nodes, get_topics)")
        commands_layout.addWidget(self.command_input, 0, 1, 1, 2)

        send_btn = QPushButton("Send Command")
        send_btn.clicked.connect(self.send_custom_command)
        commands_layout.addWidget(send_btn, 0, 3)

        # Response area
        commands_layout.addWidget(QLabel("Response:"), 1, 0)
        self.command_response = QTextEdit()
        self.command_response.setReadOnly(True)
        self.command_response.setMaximumHeight(200)
        self.command_response.setStyleSheet(
            "QTextEdit { background-color: #1e1e1e; color: #00aaff; font-family: monospace; }")
        commands_layout.addWidget(self.command_response, 1, 1, 1, 3)

        commands_group.setLayout(commands_layout)
        layout.addWidget(commands_group)

        # System info
        info_group = QGroupBox("Get System Information")
        info_layout = QHBoxLayout()

        get_nodes_btn = QPushButton("Get All Nodes")
        get_nodes_btn.clicked.connect(lambda: self.send_brain_command("get_nodes"))
        info_layout.addWidget(get_nodes_btn)

        get_topics_btn = QPushButton("Get All Topics")
        get_topics_btn.clicked.connect(lambda: self.send_brain_command("get_topics"))
        info_layout.addWidget(get_topics_btn)

        get_status_btn = QPushButton("Get Full Status")
        get_status_btn.clicked.connect(self.get_full_status)
        info_layout.addWidget(get_status_btn)

        info_group.setLayout(info_layout)
        layout.addWidget(info_group)

        layout.addStretch()
        return widget

    def create_motion_control_tab(self):
        """Create motion control tab for cartesian movements"""
        widget = QWidget()
        layout = QVBoxLayout(widget)

        # === Quick Actions ===
        quick_group = QGroupBox("Quick Actions")
        quick_layout = QHBoxLayout()

        self.home_btn = QPushButton("GO HOME")
        self.home_btn.setStyleSheet(
            "QPushButton { background-color: #0066cc; color: white; padding: 20px 40px; "
            "font-size: 18px; font-weight: bold; border-radius: 8px; }"
            "QPushButton:hover { background-color: #0088ff; }"
            "QPushButton:disabled { background-color: #444444; }")
        self.home_btn.clicked.connect(self.send_go_home)
        quick_layout.addWidget(self.home_btn)

        quick_group.setLayout(quick_layout)
        layout.addWidget(quick_group)

        # === End-Effector Frame Selection ===
        frame_group = QGroupBox("End-Effector Frame")
        frame_layout = QHBoxLayout()

        self.frame_button_group = QButtonGroup()

        self.gripper_tip_radio = QRadioButton("Gripper Tip")
        self.gripper_tip_radio.setChecked(True)
        self.gripper_tip_radio.setStyleSheet("QRadioButton { font-size: 14px; color: #00ff00; }")
        self.frame_button_group.addButton(self.gripper_tip_radio)
        frame_layout.addWidget(self.gripper_tip_radio)

        self.tool0_radio = QRadioButton("Tool0 (Robot TCP)")
        self.tool0_radio.setStyleSheet("QRadioButton { font-size: 14px; color: #00aaff; }")
        self.frame_button_group.addButton(self.tool0_radio)
        frame_layout.addWidget(self.tool0_radio)

        self.apply_frame_btn = QPushButton("Apply Frame")
        self.apply_frame_btn.setStyleSheet(
            "QPushButton { background-color: #666666; color: white; padding: 10px 20px; }"
            "QPushButton:hover { background-color: #888888; }")
        self.apply_frame_btn.clicked.connect(self.apply_frame_selection)
        frame_layout.addWidget(self.apply_frame_btn)

        frame_layout.addStretch()
        frame_group.setLayout(frame_layout)
        layout.addWidget(frame_group)

        # === Cartesian Position Input ===
        position_group = QGroupBox("Cartesian Position (mm / radians)")
        position_layout = QGridLayout()

        # Position labels and spinboxes
        position_layout.addWidget(QLabel("X (mm):"), 0, 0)
        self.x_spinbox = QDoubleSpinBox()
        self.x_spinbox.setRange(-2000, 2000)
        self.x_spinbox.setValue(-588)
        self.x_spinbox.setDecimals(1)
        self.x_spinbox.setSingleStep(10)
        self.x_spinbox.setStyleSheet("QDoubleSpinBox { padding: 8px; font-size: 14px; }")
        position_layout.addWidget(self.x_spinbox, 0, 1)

        position_layout.addWidget(QLabel("Y (mm):"), 0, 2)
        self.y_spinbox = QDoubleSpinBox()
        self.y_spinbox.setRange(-2000, 2000)
        self.y_spinbox.setValue(-133)
        self.y_spinbox.setDecimals(1)
        self.y_spinbox.setSingleStep(10)
        self.y_spinbox.setStyleSheet("QDoubleSpinBox { padding: 8px; font-size: 14px; }")
        position_layout.addWidget(self.y_spinbox, 0, 3)

        position_layout.addWidget(QLabel("Z (mm):"), 0, 4)
        self.z_spinbox = QDoubleSpinBox()
        self.z_spinbox.setRange(0, 1000)
        self.z_spinbox.setValue(222)
        self.z_spinbox.setDecimals(1)
        self.z_spinbox.setSingleStep(10)
        self.z_spinbox.setStyleSheet("QDoubleSpinBox { padding: 8px; font-size: 14px; }")
        position_layout.addWidget(self.z_spinbox, 0, 5)

        # Rotation labels and spinboxes
        position_layout.addWidget(QLabel("RX (rad):"), 1, 0)
        self.rx_spinbox = QDoubleSpinBox()
        self.rx_spinbox.setRange(-4, 4)
        self.rx_spinbox.setValue(-2.221)
        self.rx_spinbox.setDecimals(3)
        self.rx_spinbox.setSingleStep(0.1)
        self.rx_spinbox.setStyleSheet("QDoubleSpinBox { padding: 8px; font-size: 14px; }")
        position_layout.addWidget(self.rx_spinbox, 1, 1)

        position_layout.addWidget(QLabel("RY (rad):"), 1, 2)
        self.ry_spinbox = QDoubleSpinBox()
        self.ry_spinbox.setRange(-4, 4)
        self.ry_spinbox.setValue(2.221)
        self.ry_spinbox.setDecimals(3)
        self.ry_spinbox.setSingleStep(0.1)
        self.ry_spinbox.setStyleSheet("QDoubleSpinBox { padding: 8px; font-size: 14px; }")
        position_layout.addWidget(self.ry_spinbox, 1, 3)

        position_layout.addWidget(QLabel("RZ (rad):"), 1, 4)
        self.rz_spinbox = QDoubleSpinBox()
        self.rz_spinbox.setRange(-4, 4)
        self.rz_spinbox.setValue(0.0)
        self.rz_spinbox.setDecimals(3)
        self.rz_spinbox.setSingleStep(0.1)
        self.rz_spinbox.setStyleSheet("QDoubleSpinBox { padding: 8px; font-size: 14px; }")
        position_layout.addWidget(self.rz_spinbox, 1, 5)

        position_group.setLayout(position_layout)
        layout.addWidget(position_group)

        # === Move Button ===
        move_layout = QHBoxLayout()

        self.move_btn = QPushButton("MOVE TO POSITION")
        self.move_btn.setStyleSheet(
            "QPushButton { background-color: #00aa00; color: white; padding: 20px 60px; "
            "font-size: 18px; font-weight: bold; border-radius: 8px; }"
            "QPushButton:hover { background-color: #00cc00; }"
            "QPushButton:disabled { background-color: #444444; }")
        self.move_btn.clicked.connect(self.send_move_command)
        move_layout.addWidget(self.move_btn)

        self.stop_motion_btn = QPushButton("STOP")
        self.stop_motion_btn.setStyleSheet(
            "QPushButton { background-color: #ff0000; color: white; padding: 20px 40px; "
            "font-size: 18px; font-weight: bold; border-radius: 8px; }"
            "QPushButton:hover { background-color: #ff3333; }")
        self.stop_motion_btn.clicked.connect(self.send_emergency_stop)
        move_layout.addWidget(self.stop_motion_btn)

        layout.addLayout(move_layout)

        # === Preset Positions ===
        presets_group = QGroupBox("Preset Positions")
        presets_layout = QGridLayout()

        preset_positions = [
            ("Home", -588, -133, 222),
            ("Above Home", -588, -133, 350),
            ("Left", -700, -133, 222),
            ("Right", -400, -133, 222),
            ("Forward", -588, -250, 222),
            ("Back", -588, -50, 222),
        ]

        for i, (name, x, y, z) in enumerate(preset_positions):
            btn = QPushButton(name)
            btn.setStyleSheet(
                "QPushButton { background-color: #444444; color: white; padding: 10px 15px; }"
                "QPushButton:hover { background-color: #666666; }")
            btn.clicked.connect(lambda checked, x=x, y=y, z=z: self.set_preset_position(x, y, z))
            presets_layout.addWidget(btn, i // 3, i % 3)

        presets_group.setLayout(presets_layout)
        layout.addWidget(presets_group)

        # === Motion Status ===
        status_group = QGroupBox("Motion Status")
        status_layout = QVBoxLayout()

        self.motion_status_text = QTextEdit()
        self.motion_status_text.setReadOnly(True)
        self.motion_status_text.setMaximumHeight(150)
        self.motion_status_text.setStyleSheet(
            "QTextEdit { background-color: #1e1e1e; color: #00ff00; font-family: monospace; }")
        self.motion_status_text.setPlainText("Ready for motion commands...")
        status_layout.addWidget(self.motion_status_text)

        status_group.setLayout(status_layout)
        layout.addWidget(status_group)

        layout.addStretch()
        return widget

    # === Motion Control Methods ===

    def send_go_home(self):
        """Send go home command"""
        self.motion_status_text.append("Sending GO HOME command...")
        self.home_btn.setEnabled(False)
        self.move_btn.setEnabled(False)
        self.send_brain_command("go_home")
        # Re-enable after delay
        QTimer.singleShot(3000, lambda: self.home_btn.setEnabled(True))
        QTimer.singleShot(3000, lambda: self.move_btn.setEnabled(True))

    def apply_frame_selection(self):
        """Apply the selected end-effector frame"""
        if self.gripper_tip_radio.isChecked():
            self.send_brain_command("use_gripper_tip")
            self.motion_status_text.append("Switched to gripper_tip frame")
        else:
            self.send_brain_command("use_tool0")
            self.motion_status_text.append("Switched to tool0 frame")

    def send_move_command(self):
        """Send move to cartesian position command"""
        x = self.x_spinbox.value()
        y = self.y_spinbox.value()
        z = self.z_spinbox.value()
        rx = self.rx_spinbox.value()
        ry = self.ry_spinbox.value()
        rz = self.rz_spinbox.value()

        command = f"move {x} {y} {z} {rx} {ry} {rz}"
        self.motion_status_text.append(f"Sending: {command}")
        self.move_btn.setEnabled(False)
        self.home_btn.setEnabled(False)
        self.send_brain_command(command)
        # Re-enable after delay
        QTimer.singleShot(3000, lambda: self.move_btn.setEnabled(True))
        QTimer.singleShot(3000, lambda: self.home_btn.setEnabled(True))

    def set_preset_position(self, x, y, z):
        """Set spinbox values to preset position"""
        self.x_spinbox.setValue(x)
        self.y_spinbox.setValue(y)
        self.z_spinbox.setValue(z)
        self.motion_status_text.append(f"Preset loaded: x={x}, y={y}, z={z}")

    # === Update Methods ===

    def update_brain_status(self, json_str):
        """Update dashboard from brain status"""
        try:
            data = json.loads(json_str)

            # Update state
            state = data.get('system_state', 'UNKNOWN')
            self.current_state = state
            self.state_label.setText(state)

            # Set state color
            state_colors = {
                'IDLE': '#666666',
                'INITIALIZING': '#ffaa00',
                'RUNNING': '#00aa00',
                'PAUSED': '#ff8800',
                'ERROR': '#ff0000',
                'EMERGENCY_STOP': '#ff0000',
                'SHUTDOWN': '#aa0000'
            }
            color = state_colors.get(state, '#666666')
            self.state_label.setStyleSheet(
                f"QLabel {{ font-size: 24px; font-weight: bold; color: #ffffff; "
                f"background-color: {color}; padding: 15px; border-radius: 5px; }}")

            # Update status indicator
            if state in ['RUNNING', 'INITIALIZING']:
                self.status_indicator.setText('ONLINE')
                self.status_indicator.setStyleSheet(
                    "QLabel { background-color: #00aa00; color: white; padding: 8px 15px; "
                    "font-size: 14px; font-weight: bold; border-radius: 5px; }")
            else:
                self.status_indicator.setText(state)
                self.status_indicator.setStyleSheet(
                    f"QLabel {{ background-color: {color}; color: white; padding: 8px 15px; "
                    "font-size: 14px; font-weight: bold; border-radius: 5px; }")

            # Update metrics
            self.uptime = data.get('uptime_seconds', 0)
            self.uptime_label.setText(f"Uptime: {int(self.uptime)}s")

            total_nodes = data.get('total_nodes', 0)
            alive_nodes = data.get('alive_nodes', 0)
            self.nodes_count_label.setText(f"{alive_nodes} / {total_nodes}")

            self.topics_count_label.setText(str(data.get('topics_monitored', 0)))
            self.messages_count_label.setText(str(data.get('total_messages_logged', 0)))

            self.statusBar().showMessage(f"Last update: {data.get('timestamp', 'unknown')}")

        except json.JSONDecodeError as e:
            self.statusBar().showMessage(f"JSON parse error: {e}")

    def update_node_health(self, json_str):
        """Update node health display"""
        try:
            data = json.loads(json_str)
            nodes = data.get('nodes', {})

            self.nodes_tree.clear()
            for node_name, node_info in nodes.items():
                item = QTreeWidgetItem([
                    node_name,
                    "ALIVE" if node_info.get('alive', False) else "DEAD",
                    node_info.get('last_seen', 'N/A'),
                    str(node_info.get('message_count', 0))
                ])

                # Color based on status
                if node_info.get('alive', False):
                    item.setForeground(1, QBrush(QColor(0, 170, 0)))
                else:
                    item.setForeground(1, QBrush(QColor(255, 0, 0)))

                self.nodes_tree.addTopLevelItem(item)

        except json.JSONDecodeError:
            pass

    def update_topic_stats(self, json_str):
        """Update topic statistics display"""
        try:
            data = json.loads(json_str)
            topics = data.get('topics', {})

            self.topics_table.setRowCount(len(topics))
            for i, (topic_name, topic_info) in enumerate(topics.items()):
                self.topics_table.setItem(i, 0, QTableWidgetItem(topic_name))
                self.topics_table.setItem(i, 1, QTableWidgetItem(
                    str(topic_info.get('message_count', 0))))
                self.topics_table.setItem(i, 2, QTableWidgetItem(
                    topic_info.get('last_message_time', 'N/A')))

                # Truncate last data for display
                last_data = str(topic_info.get('last_data', ''))
                if len(last_data) > 100:
                    last_data = last_data[:100] + '...'
                self.topics_table.setItem(i, 3, QTableWidgetItem(last_data))

        except json.JSONDecodeError:
            pass

    def add_event_log(self, json_str):
        """Add event to log"""
        try:
            data = json.loads(json_str)
            timestamp = data.get('timestamp', datetime.now().isoformat())
            event_type = data.get('type', 'INFO')
            message = data.get('message', str(data))

            # Color code by type
            type_colors = {
                'STARTUP': '#00ff00',
                'DISCOVERY': '#00aaff',
                'WARNING': '#ffaa00',
                'ERROR': '#ff0000',
                'COMMAND': '#ff00ff',
                'PERCEPTION': '#00ffaa',
                'PLANNING': '#aaff00',
                'STATE_CHANGE': '#ffff00',
                'state_change': '#ffff00'
            }
            color = type_colors.get(event_type, '#ffffff')

            log_entry = f'<span style="color: {color};">[{timestamp}] [{event_type}] {message}</span><br>'
            self.event_log_text.insertHtml(log_entry)

            # Also add to recent events (keep last 10)
            if self.recent_events_text.document().blockCount() > 10:
                cursor = self.recent_events_text.textCursor()
                cursor.movePosition(cursor.Start)
                cursor.select(cursor.LineUnderCursor)
                cursor.removeSelectedText()
                cursor.deleteChar()
            self.recent_events_text.insertHtml(log_entry)

        except json.JSONDecodeError:
            self.event_log_text.append(json_str)

    # === Command Methods ===

    def send_brain_command(self, command):
        """Send command to brain node"""
        if not self.ros_node.brain_command_client.wait_for_service(timeout_sec=1.0):
            self.command_response.setPlainText("Brain command service not available")
            return

        request = SystemCommand.Request()
        request.command = command
        future = self.ros_node.brain_command_client.call_async(request)
        future.add_done_callback(self.handle_command_response)

    def handle_command_response(self, future):
        """Handle response from brain command"""
        try:
            response = future.result()
            if response.success:
                # Try to pretty print JSON
                try:
                    data = json.loads(response.message)
                    self.command_response.setPlainText(json.dumps(data, indent=2))
                except json.JSONDecodeError:
                    self.command_response.setPlainText(response.message)
            else:
                self.command_response.setPlainText(f"Error: {response.message}")
        except Exception as e:
            self.command_response.setPlainText(f"Exception: {e}")

    def send_start_command(self):
        self.send_brain_command("start")
        self.add_event_log(json.dumps({
            'type': 'COMMAND',
            'message': 'Start command sent',
            'timestamp': datetime.now().isoformat()
        }))

    def send_stop_command(self):
        self.send_brain_command("stop")
        self.add_event_log(json.dumps({
            'type': 'COMMAND',
            'message': 'Stop command sent',
            'timestamp': datetime.now().isoformat()
        }))

    def send_emergency_stop(self):
        self.send_brain_command("emergency_stop")
        self.add_event_log(json.dumps({
            'type': 'COMMAND',
            'message': 'EMERGENCY STOP command sent',
            'timestamp': datetime.now().isoformat()
        }))

    def send_reset_command(self):
        self.send_brain_command("reset")
        self.add_event_log(json.dumps({
            'type': 'COMMAND',
            'message': 'Reset command sent',
            'timestamp': datetime.now().isoformat()
        }))

    def send_custom_command(self):
        command = self.command_input.text().strip()
        if command:
            self.send_brain_command(command)

    def get_full_status(self):
        """Get full system status"""
        if not self.ros_node.brain_status_client.wait_for_service(timeout_sec=1.0):
            self.command_response.setPlainText("Brain status service not available")
            return

        request = SystemCommand.Request()
        request.command = "status"
        future = self.ros_node.brain_status_client.call_async(request)
        future.add_done_callback(self.handle_command_response)

    def refresh_nodes(self):
        """Request node list refresh"""
        self.send_brain_command("get_nodes")

    def export_log(self):
        """Export event log to file"""
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"/tmp/brain_node_log_{timestamp}.txt"
        with open(filename, 'w') as f:
            f.write(self.event_log_text.toPlainText())
        self.statusBar().showMessage(f"Log exported to {filename}")


def main(args=None):
    """Main entry point"""
    rclpy.init(args=args)

    # Create QApplication
    app = QApplication(sys.argv)

    # Create signal emitter (must be QObject-based)
    signal_emitter = SignalEmitter()

    # Create ROS2 node with signal emitter
    ros_node = BrainDashboardNode(signal_emitter)

    # Create executor
    executor = MultiThreadedExecutor()
    executor.add_node(ros_node)

    # Create ROS2 thread
    ros_thread = ROS2Thread(executor)
    ros_thread.start()

    # Create and show dashboard
    dashboard = BrainDashboard(ros_node, signal_emitter)
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
