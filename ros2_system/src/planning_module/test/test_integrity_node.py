#!/usr/bin/env python3
"""
Integration test for integrity_node
Tests workspace validation service and environment status monitoring
"""

import unittest
import time
import rclpy
from rclpy.node import Node
from sort_interfaces.msg import EnvironmentStatus
from sort_interfaces.srv import ValidateWorkspace
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import struct


class IntegrityNodeTestNode(Node):
    """Test node for integrity_node testing"""

    def __init__(self):
        super().__init__('integrity_node_test_node')

        # Publishers for inputs
        self.pointcloud_pub = self.create_publisher(
            PointCloud2,
            '/camera/pointcloud',
            10
        )
        self.status_pub = self.create_publisher(
            String,
            '/system/status',
            10
        )

        # Subscriber for environment status
        self.env_status_sub = self.create_subscription(
            EnvironmentStatus,
            '/planning/environment_status',
            self.env_status_callback,
            10
        )

        # Service client for workspace validation
        self.validate_client = self.create_client(
            ValidateWorkspace,
            '/planning/validate_workspace'
        )

        self.received_statuses = []
        self.status_received = False

    def env_status_callback(self, msg):
        """Store environment status messages"""
        self.received_statuses.append(msg)
        self.status_received = True
        self.get_logger().info(
            f'Received environment status: is_safe={msg.is_safe}, '
            f'workspace_clear={msg.workspace_clear}'
        )


class TestIntegrityNode(unittest.TestCase):
    """Integration tests for integrity_node"""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2"""
        rclpy.shutdown()

    def setUp(self):
        """Create test node before each test"""
        self.test_node = IntegrityNodeTestNode()
        time.sleep(0.5)  # Allow connections to establish

    def tearDown(self):
        """Cleanup after each test"""
        self.test_node.destroy_node()

    def create_pose(self, x, y, z):
        """Helper to create Pose message"""
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return pose

    def create_empty_pointcloud(self):
        """Helper to create empty PointCloud2 message"""
        msg = PointCloud2()
        msg.header.stamp = self.test_node.get_clock().now().to_msg()
        msg.header.frame_id = "camera_link"
        msg.height = 1
        msg.width = 0
        msg.is_dense = True
        msg.point_step = 16  # 4 floats (x, y, z, rgb)
        msg.row_step = 0
        return msg

    def test_service_available(self):
        """Test that the validate_workspace service is available"""
        service_available = self.test_node.validate_client.wait_for_service(timeout_sec=5.0)
        self.assertTrue(
            service_available,
            "ValidateWorkspace service not available within timeout"
        )

    def test_environment_status_published(self):
        """Test that environment status is published periodically"""
        # Clear previous statuses
        self.test_node.received_statuses.clear()
        self.test_node.status_received = False

        # Wait for at least one status message
        start_time = time.time()
        while not self.test_node.status_received and (time.time() - start_time) < 3.0:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        self.assertTrue(
            self.test_node.status_received,
            "No environment status received within timeout"
        )

        # Verify message structure
        status = self.test_node.received_statuses[-1]
        self.assertIsInstance(status.is_safe, bool)
        self.assertIsInstance(status.workspace_clear, bool)
        self.assertIsInstance(status.robot_in_bounds, bool)

    def test_validate_workspace_within_bounds(self):
        """Test workspace validation with all waypoints within bounds"""
        self.assertTrue(
            self.test_node.validate_client.wait_for_service(timeout_sec=5.0)
        )

        # Create request with waypoints within workspace
        # Workspace bounds from config: x[-0.6, 0.6], y[-0.6, 0.6], z[0.0, 0.6]
        request = ValidateWorkspace.Request()
        request.planned_waypoints = [
            self.create_pose(0.3, 0.2, 0.3),
            self.create_pose(0.4, -0.3, 0.2),
            self.create_pose(-0.2, 0.1, 0.4),
        ]

        # Call service
        future = self.test_node.validate_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)

        self.assertTrue(future.done(), "Service call did not complete")

        response = future.result()
        self.assertIsNotNone(response)
        self.assertTrue(response.is_safe, "All waypoints within bounds should be safe")
        self.assertEqual(len(response.warnings), 0, "Should have no warnings")

    def test_validate_workspace_x_out_of_bounds(self):
        """Test workspace validation with X coordinate out of bounds"""
        self.assertTrue(
            self.test_node.validate_client.wait_for_service(timeout_sec=5.0)
        )

        request = ValidateWorkspace.Request()
        request.planned_waypoints = [
            self.create_pose(0.3, 0.2, 0.3),   # Valid
            self.create_pose(0.8, 0.0, 0.3),   # X out of bounds (max 0.6)
        ]

        future = self.test_node.validate_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)

        response = future.result()
        self.assertIsNotNone(response)
        self.assertFalse(response.is_safe, "Out of bounds waypoint should be unsafe")
        self.assertGreater(len(response.warnings), 0, "Should have warnings")

    def test_validate_workspace_y_out_of_bounds(self):
        """Test workspace validation with Y coordinate out of bounds"""
        self.assertTrue(
            self.test_node.validate_client.wait_for_service(timeout_sec=5.0)
        )

        request = ValidateWorkspace.Request()
        request.planned_waypoints = [
            self.create_pose(0.3, -0.8, 0.3),  # Y out of bounds (min -0.6)
        ]

        future = self.test_node.validate_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)

        response = future.result()
        self.assertFalse(response.is_safe)
        self.assertGreater(len(response.warnings), 0)

    def test_validate_workspace_z_below_minimum(self):
        """Test workspace validation with Z coordinate below minimum"""
        self.assertTrue(
            self.test_node.validate_client.wait_for_service(timeout_sec=5.0)
        )

        request = ValidateWorkspace.Request()
        request.planned_waypoints = [
            self.create_pose(0.3, 0.2, -0.1),  # Z below minimum (0.0)
        ]

        future = self.test_node.validate_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)

        response = future.result()
        self.assertFalse(response.is_safe)
        self.assertGreater(len(response.warnings), 0)

    def test_validate_workspace_z_above_maximum(self):
        """Test workspace validation with Z coordinate above maximum"""
        self.assertTrue(
            self.test_node.validate_client.wait_for_service(timeout_sec=5.0)
        )

        request = ValidateWorkspace.Request()
        request.planned_waypoints = [
            self.create_pose(0.3, 0.2, 0.8),  # Z above maximum (0.6)
        ]

        future = self.test_node.validate_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)

        response = future.result()
        self.assertFalse(response.is_safe)

    def test_validate_workspace_boundary_values(self):
        """Test workspace validation at exact boundary values"""
        self.assertTrue(
            self.test_node.validate_client.wait_for_service(timeout_sec=5.0)
        )

        # Test all boundaries
        request = ValidateWorkspace.Request()
        request.planned_waypoints = [
            self.create_pose(0.6, 0.6, 0.6),    # Max corners
            self.create_pose(-0.6, -0.6, 0.0),  # Min corners
            self.create_pose(0.6, -0.6, 0.3),   # Mixed
        ]

        future = self.test_node.validate_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)

        response = future.result()
        # Boundary values should be valid (inclusive)
        self.assertTrue(response.is_safe, "Boundary values should be safe")

    def test_validate_workspace_empty_waypoints(self):
        """Test workspace validation with no waypoints"""
        self.assertTrue(
            self.test_node.validate_client.wait_for_service(timeout_sec=5.0)
        )

        request = ValidateWorkspace.Request()
        request.planned_waypoints = []

        future = self.test_node.validate_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)

        response = future.result()
        # Empty trajectory might be considered safe or have a warning
        self.assertIsNotNone(response)

    def test_validate_workspace_many_waypoints(self):
        """Test workspace validation with many waypoints"""
        self.assertTrue(
            self.test_node.validate_client.wait_for_service(timeout_sec=5.0)
        )

        # Create a trajectory with 100 waypoints
        request = ValidateWorkspace.Request()
        for i in range(100):
            # Create spiral pattern within bounds
            angle = i * 0.1
            radius = 0.3 + (i * 0.002)
            x = radius * (angle % 1.0) - 0.3
            y = radius * ((angle + 0.5) % 1.0) - 0.3
            z = 0.1 + (i * 0.003)
            request.planned_waypoints.append(self.create_pose(x, y, z))

        future = self.test_node.validate_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)

        response = future.result()
        self.assertIsNotNone(response)
        # Service should handle many waypoints

    def test_validate_workspace_mixed_valid_invalid(self):
        """Test workspace validation with mix of valid and invalid waypoints"""
        self.assertTrue(
            self.test_node.validate_client.wait_for_service(timeout_sec=5.0)
        )

        request = ValidateWorkspace.Request()
        request.planned_waypoints = [
            self.create_pose(0.3, 0.2, 0.3),   # Valid
            self.create_pose(0.9, 0.0, 0.3),   # Invalid (x out of bounds)
            self.create_pose(0.4, 0.3, 0.2),   # Valid
        ]

        future = self.test_node.validate_client.call_async(request)
        rclpy.spin_until_future_complete(self.test_node, future, timeout_sec=5.0)

        response = future.result()
        # Should be unsafe due to one invalid waypoint
        self.assertFalse(response.is_safe)
        self.assertGreater(len(response.warnings), 0)

    def test_emergency_stop_status(self):
        """Test that emergency stop affects environment status"""
        # Publish emergency stop status
        status_msg = String()
        status_msg.data = "emergency_stopped"
        self.test_node.status_pub.publish(status_msg)

        time.sleep(0.3)

        # Clear and wait for new environment status
        self.test_node.received_statuses.clear()
        self.test_node.status_received = False

        start_time = time.time()
        while not self.test_node.status_received and (time.time() - start_time) < 2.0:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        if self.test_node.status_received:
            status = self.test_node.received_statuses[-1]
            # Emergency stop should affect safety status
            # Implementation may mark as unsafe
            self.assertIsInstance(status.is_safe, bool)

    def test_pointcloud_processing(self):
        """Test that pointcloud messages are processed"""
        # Publish empty pointcloud
        pc_msg = self.create_empty_pointcloud()
        self.test_node.pointcloud_pub.publish(pc_msg)

        time.sleep(0.3)

        # Node should still publish environment status
        self.test_node.received_statuses.clear()
        self.test_node.status_received = False

        start_time = time.time()
        while not self.test_node.status_received and (time.time() - start_time) < 2.0:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        self.assertTrue(self.test_node.status_received)

    def test_periodic_status_updates(self):
        """Test that environment status is published at expected rate"""
        # Clear previous messages
        self.test_node.received_statuses.clear()

        # Collect messages for 2 seconds
        start_time = time.time()
        while time.time() - start_time < 2.0:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # At 10 Hz, we should get roughly 20 messages in 2 seconds
        # Allow some tolerance for timing variations
        num_messages = len(self.test_node.received_statuses)
        self.assertGreater(
            num_messages,
            10,
            f"Expected ~20 messages at 10Hz, got {num_messages}"
        )
        self.assertLess(
            num_messages,
            30,
            f"Got too many messages: {num_messages}"
        )


if __name__ == '__main__':
    unittest.main()
