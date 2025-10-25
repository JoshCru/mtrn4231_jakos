#!/usr/bin/env python3
"""
Integration test for recognition_node
Tests weight estimation from point cloud data
"""

import unittest
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from sort_interfaces.msg import WeightEstimate
import struct


class RecognitionNodeTestNode(Node):
    """Test node for recognition_node testing"""

    def __init__(self):
        super().__init__('recognition_node_test_node')

        # Subscriber for weight estimates
        self.weight_sub = self.create_subscription(
            WeightEstimate,
            '/recognition/estimated_weights',
            self.weight_callback,
            10
        )

        self.received_weights = []
        self.weight_received = False

    def weight_callback(self, msg):
        """Store received weight estimates"""
        self.received_weights.append(msg)
        self.weight_received = True
        self.get_logger().info(
            f'Received weight estimate: object_id={msg.object_id}, '
            f'weight={msg.estimated_weight}g, confidence={msg.confidence}'
        )


class TestRecognitionNode(unittest.TestCase):
    """Integration tests for recognition_node"""

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
        self.test_node = RecognitionNodeTestNode()
        time.sleep(0.5)  # Allow connections to establish

    def tearDown(self):
        """Cleanup after each test"""
        self.test_node.destroy_node()

    def spin_until_weight_received(self, timeout_sec=10.0, num_weights=1):
        """Spin node until weight received or timeout"""
        start_time = time.time()
        while len(self.test_node.received_weights) < num_weights:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if time.time() - start_time > timeout_sec:
                return False
        return True

    def test_weight_estimates_received(self):
        """Test that weight estimates are published from mock camera"""
        # Clear previous weights
        self.test_node.received_weights.clear()
        self.test_node.weight_received = False

        # Wait for at least one weight estimate
        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=1),
            "No weight estimates received within timeout"
        )

        # Verify we got weight estimates
        self.assertGreater(
            len(self.test_node.received_weights),
            0,
            "Should receive at least one weight estimate"
        )

    def test_weight_estimate_structure(self):
        """Test that WeightEstimate messages have valid structure"""
        self.test_node.received_weights.clear()

        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=1)
        )

        weight = self.test_node.received_weights[0]

        # Check message fields
        self.assertIsInstance(weight.object_id, int)
        self.assertIsInstance(weight.estimated_weight, float)
        self.assertIsInstance(weight.confidence, float)
        self.assertIsInstance(weight.volume, float)

        # Check value ranges
        self.assertGreater(weight.estimated_weight, 0.0, "Weight should be positive")
        self.assertGreaterEqual(weight.confidence, 0.0, "Confidence should be >= 0")
        self.assertLessEqual(weight.confidence, 1.0, "Confidence should be <= 1")
        self.assertGreater(weight.volume, 0.0, "Volume should be positive")

    def test_weight_estimate_position(self):
        """Test that position (pose) is within workspace bounds"""
        self.test_node.received_weights.clear()

        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=1)
        )

        weight = self.test_node.received_weights[0]
        pos = weight.pose.position

        # Workspace bounds from config
        self.assertGreaterEqual(pos.x, -0.6, "X should be >= -0.6")
        self.assertLessEqual(pos.x, 0.6, "X should be <= 0.6")
        self.assertGreaterEqual(pos.y, -0.6, "Y should be >= -0.6")
        self.assertLessEqual(pos.y, 0.6, "Y should be <= 0.6")
        self.assertGreaterEqual(pos.z, 0.0, "Z should be >= 0.0")
        self.assertLessEqual(pos.z, 0.6, "Z should be <= 0.6")

    def test_multiple_objects_detected(self):
        """Test that multiple objects are detected from mock camera"""
        self.test_node.received_weights.clear()

        # Mock camera publishes 3 objects by default
        # Wait for at least 2 objects (allowing for some clustering variations)
        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=2),
            "Should detect at least 2 objects"
        )

        # Verify we got multiple weight estimates
        self.assertGreaterEqual(
            len(self.test_node.received_weights),
            2,
            "Should detect at least 2 objects"
        )

    def test_weight_estimate_reasonable_values(self):
        """Test that estimated weights are in reasonable ranges for stainless steel"""
        self.test_node.received_weights.clear()

        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=1)
        )

        for weight in self.test_node.received_weights:
            # Mock objects are 50g, 100g, 200g
            # Allow for convex hull approximation errors (can overestimate)
            self.assertGreater(
                weight.estimated_weight,
                10.0,
                f"Weight {weight.estimated_weight}g seems too small"
            )
            self.assertLess(
                weight.estimated_weight,
                500.0,
                f"Weight {weight.estimated_weight}g seems too large"
            )

    def test_unique_object_ids(self):
        """Test that each object gets a unique ID"""
        self.test_node.received_weights.clear()

        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=2)
        )

        # Collect object IDs
        object_ids = [w.object_id for w in self.test_node.received_weights]

        # Check that IDs are unique
        unique_ids = set(object_ids)
        self.assertEqual(
            len(unique_ids),
            len(object_ids),
            "Object IDs should be unique"
        )

    def test_confidence_above_threshold(self):
        """Test that published estimates meet confidence threshold"""
        self.test_node.received_weights.clear()

        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=1)
        )

        # Config threshold is 0.5
        for weight in self.test_node.received_weights:
            self.assertGreaterEqual(
                weight.confidence,
                0.5,
                f"Confidence {weight.confidence} should be >= 0.5"
            )

    def test_continuous_publishing(self):
        """Test that weight estimates are published continuously"""
        self.test_node.received_weights.clear()

        # Wait for first batch
        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=1)
        )

        first_count = len(self.test_node.received_weights)

        # Wait a bit longer for more estimates (mock camera publishes at 1 Hz)
        time.sleep(2.5)
        while self.test_node.weight_received:
            self.test_node.weight_received = False
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Should have more estimates
        self.assertGreater(
            len(self.test_node.received_weights),
            first_count,
            "Should receive more estimates over time"
        )

    def test_volume_calculation(self):
        """Test that volume is calculated and non-zero"""
        self.test_node.received_weights.clear()

        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=1)
        )

        for weight in self.test_node.received_weights:
            self.assertGreater(
                weight.volume,
                0.0,
                "Volume should be positive"
            )
            # Reasonable volume range for small objects (m³)
            self.assertLess(
                weight.volume,
                0.001,  # 1000 cm³
                f"Volume {weight.volume}m³ seems too large"
            )

    def test_weight_density_relationship(self):
        """Test that weight = volume × density relationship holds approximately"""
        self.test_node.received_weights.clear()

        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=1)
        )

        # Stainless steel density = 8000 kg/m³
        density = 8000.0

        for weight in self.test_node.received_weights:
            # Calculate expected weight from volume
            expected_weight_kg = weight.volume * density
            expected_weight_g = expected_weight_kg * 1000.0

            # Allow for convex hull approximation error (±50%)
            error_ratio = abs(weight.estimated_weight - expected_weight_g) / expected_weight_g

            self.assertLess(
                error_ratio,
                0.5,  # 50% error tolerance
                f"Weight-volume relationship seems off: "
                f"weight={weight.estimated_weight}g, "
                f"volume={weight.volume}m³, "
                f"expected_weight={expected_weight_g}g"
            )


    def test_orientation_field(self):
        """Test that orientation field is set in pose"""
        self.test_node.received_weights.clear()

        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=1)
        )

        weight = self.test_node.received_weights[0]
        quat = weight.pose.orientation

        # Check quaternion is normalized (w, x, y, z)
        magnitude = (quat.w**2 + quat.x**2 + quat.y**2 + quat.z**2)**0.5
        self.assertAlmostEqual(
            magnitude,
            1.0,
            places=5,
            msg="Quaternion should be normalized"
        )

    def test_timestamp_validity(self):
        """Test that timestamps are recent and valid"""
        self.test_node.received_weights.clear()

        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=1)
        )

        weight = self.test_node.received_weights[0]

        # Check header has a timestamp
        self.assertIsNotNone(weight.header.stamp)

        # Timestamp should be recent (within last 30 seconds)
        now = self.test_node.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(weight.header.stamp)
        time_diff = (now - msg_time).nanoseconds / 1e9  # Convert to seconds

        self.assertLess(
            abs(time_diff),
            30.0,
            f"Timestamp seems stale: {time_diff}s old"
        )

    def test_frame_id(self):
        """Test that header frame_id is set correctly"""
        self.test_node.received_weights.clear()

        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=1)
        )

        weight = self.test_node.received_weights[0]

        # Frame ID should be set (typically camera_link or similar)
        self.assertIsNotNone(weight.header.frame_id)
        self.assertIsInstance(weight.header.frame_id, str)

    def test_no_negative_weights(self):
        """Test that no negative weights are published"""
        self.test_node.received_weights.clear()

        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=1)
        )

        for weight in self.test_node.received_weights:
            self.assertGreater(
                weight.estimated_weight,
                0.0,
                "Weight should never be negative"
            )

    def test_position_not_nan(self):
        """Test that position coordinates are not NaN"""
        self.test_node.received_weights.clear()

        self.assertTrue(
            self.spin_until_weight_received(timeout_sec=15.0, num_weights=1)
        )

        import math
        for weight in self.test_node.received_weights:
            pos = weight.pose.position
            self.assertFalse(math.isnan(pos.x), "X coordinate is NaN")
            self.assertFalse(math.isnan(pos.y), "Y coordinate is NaN")
            self.assertFalse(math.isnan(pos.z), "Z coordinate is NaN")


if __name__ == '__main__':
    unittest.main()
