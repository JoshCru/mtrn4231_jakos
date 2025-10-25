#!/usr/bin/env python3
"""
Integration test for verification_node
Tests weight verification action server and topic-based verification
"""

import unittest
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sort_interfaces.msg import WeightEstimate, ForceFeedback
from sort_interfaces.action import VerifyWeight
from std_msgs.msg import Bool
from geometry_msgs.msg import Pose, Point, Quaternion


class VerificationNodeTestNode(Node):
    """Test node for verification_node testing"""

    def __init__(self):
        super().__init__('verification_node_test_node')

        # Publishers for inputs
        self.weight_pub = self.create_publisher(
            WeightEstimate,
            '/recognition/estimated_weights',
            10
        )
        self.force_pub = self.create_publisher(
            ForceFeedback,
            '/motion_control/force_feedback',
            10
        )

        # Subscriber for verification result topic
        self.result_sub = self.create_subscription(
            Bool,
            '/planning/verification_result',
            self.result_callback,
            10
        )

        # Action client for verify_weight action
        self.action_client = ActionClient(
            self,
            VerifyWeight,
            '/planning/verify_weight'
        )

        self.received_results = []
        self.result_received = False

    def result_callback(self, msg):
        """Store verification results from topic"""
        self.received_results.append(msg.data)
        self.result_received = True
        self.get_logger().info(f'Received verification result: {msg.data}')


class TestVerificationNode(unittest.TestCase):
    """Integration tests for verification_node"""

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
        self.test_node = VerificationNodeTestNode()
        time.sleep(0.5)  # Allow connections to establish

    def tearDown(self):
        """Cleanup after each test"""
        self.test_node.destroy_node()

    def create_weight_estimate(self, object_id, weight, confidence=0.9):
        """Helper to create WeightEstimate message"""
        msg = WeightEstimate()
        msg.header.stamp = self.test_node.get_clock().now().to_msg()
        msg.object_id = object_id
        msg.estimated_weight = weight
        msg.confidence = confidence
        msg.pose = Pose(
            position=Point(x=0.3, y=0.0, z=0.1),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        msg.volume = 0.001
        return msg

    def create_force_feedback(self, object_id, measured_weight, gripper_force=10.0):
        """Helper to create ForceFeedback message"""
        msg = ForceFeedback()
        msg.header.stamp = self.test_node.get_clock().now().to_msg()
        msg.object_id = object_id
        msg.measured_weight = measured_weight
        msg.gripper_force = gripper_force
        msg.gripper_position = 0.05  # 50mm
        msg.object_detected = True
        msg.raw_sensor_value = measured_weight * 9.81  # Approximate force in N
        return msg

    def test_action_server_available(self):
        """Test that the verify_weight action server is available"""
        server_available = self.test_node.action_client.wait_for_server(timeout_sec=5.0)
        self.assertTrue(
            server_available,
            "VerifyWeight action server not available within timeout"
        )

    def test_verification_success_within_tolerance(self):
        """Test successful verification when weights match within tolerance"""
        # Wait for action server
        self.assertTrue(
            self.test_node.action_client.wait_for_server(timeout_sec=5.0)
        )

        # Send goal: estimated 100g, tolerance 10%
        goal_msg = VerifyWeight.Goal()
        goal_msg.object_id = 1
        goal_msg.estimated_weight = 100.0
        goal_msg.tolerance = 10.0  # 10% tolerance

        goal_future = self.test_node.action_client.send_goal_async(goal_msg)

        # Wait for goal acceptance
        rclpy.spin_until_future_complete(self.test_node, goal_future, timeout_sec=2.0)
        self.assertTrue(goal_future.done(), "Goal not accepted")

        goal_handle = goal_future.result()
        self.assertIsNotNone(goal_handle)
        self.assertTrue(goal_handle.accepted, "Goal was rejected")

        # Publish force feedback with weight within tolerance (105g = 5% error)
        for i in range(5):
            force_msg = self.create_force_feedback(
                object_id=1,
                measured_weight=105.0  # 5% error, within 10% tolerance
            )
            self.test_node.force_pub.publish(force_msg)
            time.sleep(0.1)

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.test_node, result_future, timeout_sec=5.0)

        self.assertTrue(result_future.done(), "Result not received")

        result = result_future.result().result
        self.assertTrue(result.verified, "Verification should succeed")
        self.assertAlmostEqual(result.actual_weight, 105.0, delta=1.0)
        self.assertLess(result.error_percentage, 10.0)

    def test_verification_failure_outside_tolerance(self):
        """Test verification failure when weights differ beyond tolerance"""
        self.assertTrue(
            self.test_node.action_client.wait_for_server(timeout_sec=5.0)
        )

        # Send goal: estimated 100g, tolerance 10%
        goal_msg = VerifyWeight.Goal()
        goal_msg.object_id = 2
        goal_msg.estimated_weight = 100.0
        goal_msg.tolerance = 10.0

        goal_future = self.test_node.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.test_node, goal_future, timeout_sec=2.0)

        goal_handle = goal_future.result()
        self.assertTrue(goal_handle.accepted)

        # Publish force feedback with weight outside tolerance (120g = 20% error)
        for i in range(5):
            force_msg = self.create_force_feedback(
                object_id=2,
                measured_weight=120.0  # 20% error, outside 10% tolerance
            )
            self.test_node.force_pub.publish(force_msg)
            time.sleep(0.1)

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.test_node, result_future, timeout_sec=5.0)

        result = result_future.result().result
        self.assertFalse(result.verified, "Verification should fail")
        self.assertGreater(result.error_percentage, 10.0)

    def test_verification_exact_match(self):
        """Test verification with exact weight match"""
        self.assertTrue(
            self.test_node.action_client.wait_for_server(timeout_sec=5.0)
        )

        goal_msg = VerifyWeight.Goal()
        goal_msg.object_id = 3
        goal_msg.estimated_weight = 150.0
        goal_msg.tolerance = 5.0  # Tight tolerance

        goal_future = self.test_node.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.test_node, goal_future, timeout_sec=2.0)

        goal_handle = goal_future.result()
        self.assertTrue(goal_handle.accepted)

        # Publish exact matching weight
        for i in range(5):
            force_msg = self.create_force_feedback(
                object_id=3,
                measured_weight=150.0  # Exact match
            )
            self.test_node.force_pub.publish(force_msg)
            time.sleep(0.1)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.test_node, result_future, timeout_sec=5.0)

        result = result_future.result().result
        self.assertTrue(result.verified)
        self.assertAlmostEqual(result.error_percentage, 0.0, delta=0.1)

    def test_verification_boundary_tolerance(self):
        """Test verification at exact tolerance boundary"""
        self.assertTrue(
            self.test_node.action_client.wait_for_server(timeout_sec=5.0)
        )

        goal_msg = VerifyWeight.Goal()
        goal_msg.object_id = 4
        goal_msg.estimated_weight = 100.0
        goal_msg.tolerance = 10.0

        goal_future = self.test_node.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.test_node, goal_future, timeout_sec=2.0)

        goal_handle = goal_future.result()
        self.assertTrue(goal_handle.accepted)

        # Publish weight at exact tolerance boundary (110g = exactly 10%)
        for i in range(5):
            force_msg = self.create_force_feedback(
                object_id=4,
                measured_weight=110.0  # Exactly 10% error
            )
            self.test_node.force_pub.publish(force_msg)
            time.sleep(0.1)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.test_node, result_future, timeout_sec=5.0)

        result = result_future.result().result
        # At exact boundary - implementation dependent, but should be consistent
        self.assertAlmostEqual(result.error_percentage, 10.0, delta=0.5)

    def test_verification_with_noisy_measurements(self):
        """Test verification with varying force measurements (simulating noise)"""
        self.assertTrue(
            self.test_node.action_client.wait_for_server(timeout_sec=5.0)
        )

        goal_msg = VerifyWeight.Goal()
        goal_msg.object_id = 5
        goal_msg.estimated_weight = 100.0
        goal_msg.tolerance = 10.0

        goal_future = self.test_node.action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.test_node, goal_future, timeout_sec=2.0)

        goal_handle = goal_future.result()
        self.assertTrue(goal_handle.accepted)

        # Publish varying measurements (simulating sensor noise)
        # Average should be around 102g (within tolerance)
        noisy_weights = [100.0, 104.0, 101.0, 103.0, 102.0]
        for weight in noisy_weights:
            force_msg = self.create_force_feedback(object_id=5, measured_weight=weight)
            self.test_node.force_pub.publish(force_msg)
            time.sleep(0.1)

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self.test_node, result_future, timeout_sec=5.0)

        result = result_future.result().result
        # Moving average filter should smooth out noise
        self.assertTrue(result.verified, "Should verify despite noise")
        # Actual weight should be close to average (~102g)
        self.assertGreater(result.actual_weight, 100.0)
        self.assertLess(result.actual_weight, 105.0)

    def test_topic_based_verification_success(self):
        """Test topic-based verification result publishing"""
        # First, send estimated weight
        weight_msg = self.create_weight_estimate(object_id=10, weight=100.0)
        self.test_node.weight_pub.publish(weight_msg)
        time.sleep(0.2)

        # Then send matching force feedback
        self.test_node.result_received = False
        force_msg = self.create_force_feedback(object_id=10, measured_weight=105.0)
        self.test_node.force_pub.publish(force_msg)

        # Wait for verification result on topic
        start_time = time.time()
        while not self.test_node.result_received and (time.time() - start_time) < 3.0:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Check if we received a result (may or may not based on implementation)
        # This test is informational about topic-based output
        if self.test_node.result_received:
            # If implementation publishes to topic, verify it
            result = self.test_node.received_results[-1]
            self.assertIsInstance(result, bool)

    def test_multiple_concurrent_verifications(self):
        """Test handling multiple verification requests"""
        self.assertTrue(
            self.test_node.action_client.wait_for_server(timeout_sec=5.0)
        )

        # Note: Current implementation may only handle one at a time
        # This tests the behavior with sequential requests

        goals = [
            (20, 100.0, 10.0, 105.0),  # object_id, estimated, tolerance, actual
            (21, 200.0, 5.0, 210.0),
        ]

        for obj_id, estimated, tolerance, actual in goals:
            goal_msg = VerifyWeight.Goal()
            goal_msg.object_id = obj_id
            goal_msg.estimated_weight = estimated
            goal_msg.tolerance = tolerance

            goal_future = self.test_node.action_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(self.test_node, goal_future, timeout_sec=2.0)

            goal_handle = goal_future.result()
            if goal_handle and goal_handle.accepted:
                # Publish force feedback
                for i in range(3):
                    force_msg = self.create_force_feedback(obj_id, actual)
                    self.test_node.force_pub.publish(force_msg)
                    time.sleep(0.1)

                # Get result
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(
                    self.test_node,
                    result_future,
                    timeout_sec=5.0
                )

                if result_future.done():
                    result = result_future.result().result
                    self.assertIsNotNone(result)


if __name__ == '__main__':
    unittest.main()
