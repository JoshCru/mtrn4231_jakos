#!/usr/bin/env python3
"""
Integration test for sort_node
Tests the sorting decision logic with various weight estimation scenarios
"""

import unittest
import time
import rclpy
from rclpy.node import Node
from sort_interfaces.msg import WeightEstimate, SortDecision, TargetArea
from geometry_msgs.msg import Pose, Point, Quaternion


class SortNodeTestNode(Node):
    """Test node that publishes inputs and subscribes to outputs"""

    def __init__(self):
        super().__init__('sort_node_test_node')

        # Publishers for inputs
        self.weight_pub = self.create_publisher(
            WeightEstimate,
            '/recognition/estimated_weights',
            10
        )
        self.target_area_pub = self.create_publisher(
            TargetArea,
            '/system/target_areas',
            10
        )

        # Subscriber for output
        self.decision_sub = self.create_subscription(
            SortDecision,
            '/planning/sort_decisions',
            self.decision_callback,
            10
        )

        self.received_decisions = []
        self.decision_received = False

    def decision_callback(self, msg):
        """Store received sort decisions"""
        self.received_decisions.append(msg)
        self.decision_received = True
        self.get_logger().info(
            f'Received decision: object_id={msg.object_id}, '
            f'target_area_id={msg.target_area_id}, '
            f'reason={msg.reason}'
        )


class TestSortNode(unittest.TestCase):
    """Integration tests for sort_node"""

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
        self.test_node = SortNodeTestNode()
        # Allow time for subscriptions/publishers to establish
        time.sleep(0.5)

    def tearDown(self):
        """Cleanup after each test"""
        self.test_node.destroy_node()

    def spin_until_decision_received(self, timeout_sec=5.0):
        """Spin node until decision received or timeout"""
        start_time = time.time()
        while not self.test_node.decision_received:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)
            if time.time() - start_time > timeout_sec:
                return False
        return True

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
        msg.volume = 0.001  # 1000 mm³
        return msg

    def create_target_area(self, area_id, label, weight_min, weight_max):
        """Helper to create TargetArea message"""
        msg = TargetArea()
        msg.id = area_id
        msg.label = label
        msg.weight_min = weight_min
        msg.weight_max = weight_max
        msg.pose = Pose(
            position=Point(x=0.5, y=0.3 * area_id, z=0.0),
            orientation=Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        )
        return msg

    def test_weight_based_exact_match(self):
        """Test weight-based sorting with exact weight match"""
        # Setup target areas
        target_areas = [
            self.create_target_area(1, "light", 0.0, 50.0),
            self.create_target_area(2, "medium", 50.0, 150.0),
            self.create_target_area(3, "heavy", 150.0, 300.0),
        ]

        for area in target_areas:
            self.target_area_pub.publish(area)

        time.sleep(0.2)  # Let target areas be received

        # Publish weight estimate that should match "medium" bin
        weight_msg = self.create_weight_estimate(
            object_id=1,
            weight=100.0  # Should go to medium bin (50-150g)
        )
        self.test_node.decision_received = False
        self.weight_pub.publish(weight_msg)

        # Wait for decision
        self.assertTrue(
            self.spin_until_decision_received(),
            "No sort decision received within timeout"
        )

        # Verify decision
        decision = self.test_node.received_decisions[-1]
        self.assertEqual(decision.object_id, 1)
        self.assertEqual(decision.target_area_id, 2)  # Medium bin
        self.assertEqual(decision.estimated_weight, 100.0)

    def test_weight_based_boundary_lower(self):
        """Test weight at lower boundary of range"""
        # Setup target areas
        target_areas = [
            self.create_target_area(1, "light", 0.0, 50.0),
            self.create_target_area(2, "medium", 50.0, 150.0),
        ]

        for area in target_areas:
            self.target_area_pub.publish(area)

        time.sleep(0.2)

        # Publish weight at exact lower boundary
        weight_msg = self.create_weight_estimate(object_id=2, weight=50.0)
        self.test_node.decision_received = False
        self.weight_pub.publish(weight_msg)

        self.assertTrue(self.spin_until_decision_received())

        decision = self.test_node.received_decisions[-1]
        self.assertEqual(decision.object_id, 2)
        # Should match medium bin (50.0 is in range [50, 150])
        self.assertEqual(decision.target_area_id, 2)

    def test_weight_based_boundary_upper(self):
        """Test weight at upper boundary of range"""
        target_areas = [
            self.create_target_area(1, "light", 0.0, 50.0),
            self.create_target_area(2, "medium", 50.0, 150.0),
        ]

        for area in target_areas:
            self.target_area_pub.publish(area)

        time.sleep(0.2)

        # Publish weight at exact upper boundary
        weight_msg = self.create_weight_estimate(object_id=3, weight=150.0)
        self.test_node.decision_received = False
        self.weight_pub.publish(weight_msg)

        self.assertTrue(self.spin_until_decision_received())

        decision = self.test_node.received_decisions[-1]
        self.assertEqual(decision.object_id, 3)
        self.assertEqual(decision.target_area_id, 2)  # Medium bin

    def test_weight_out_of_range_fallback(self):
        """Test fallback to closest bin when weight out of all ranges"""
        target_areas = [
            self.create_target_area(1, "light", 0.0, 50.0),
            self.create_target_area(2, "medium", 50.0, 150.0),
        ]

        for area in target_areas:
            self.target_area_pub.publish(area)

        time.sleep(0.2)

        # Publish weight outside all ranges (should pick closest)
        weight_msg = self.create_weight_estimate(object_id=4, weight=200.0)
        self.test_node.decision_received = False
        self.weight_pub.publish(weight_msg)

        self.assertTrue(self.spin_until_decision_received())

        decision = self.test_node.received_decisions[-1]
        self.assertEqual(decision.object_id, 4)
        # Should fallback to closest bin (medium, upper bound 150)
        self.assertEqual(decision.target_area_id, 2)

    def test_multiple_objects_sequential(self):
        """Test sorting multiple objects in sequence"""
        target_areas = [
            self.create_target_area(1, "bin1", 0.0, 100.0),
            self.create_target_area(2, "bin2", 100.0, 200.0),
        ]

        for area in target_areas:
            self.target_area_pub.publish(area)

        time.sleep(0.2)

        # Publish multiple weight estimates
        weights = [
            (10, 25.0),   # object 10, weight 25g -> bin1
            (11, 150.0),  # object 11, weight 150g -> bin2
            (12, 75.0),   # object 12, weight 75g -> bin1
        ]

        for obj_id, weight in weights:
            weight_msg = self.create_weight_estimate(obj_id, weight)
            self.test_node.decision_received = False
            self.weight_pub.publish(weight_msg)
            self.assertTrue(self.spin_until_decision_received())

        # Verify we got 3 decisions
        self.assertGreaterEqual(len(self.test_node.received_decisions), 3)

        # Verify last 3 decisions
        recent_decisions = self.test_node.received_decisions[-3:]

        self.assertEqual(recent_decisions[0].object_id, 10)
        self.assertEqual(recent_decisions[0].target_area_id, 1)

        self.assertEqual(recent_decisions[1].object_id, 11)
        self.assertEqual(recent_decisions[1].target_area_id, 2)

        self.assertEqual(recent_decisions[2].object_id, 12)
        self.assertEqual(recent_decisions[2].target_area_id, 1)

    def test_low_confidence_handling(self):
        """Test that low confidence weights are still processed"""
        target_areas = [
            self.create_target_area(1, "bin1", 0.0, 100.0),
        ]

        for area in target_areas:
            self.target_area_pub.publish(area)

        time.sleep(0.2)

        # Publish weight with low confidence
        weight_msg = self.create_weight_estimate(
            object_id=20,
            weight=50.0,
            confidence=0.3  # Low confidence
        )
        self.test_node.decision_received = False
        self.weight_pub.publish(weight_msg)

        # Should still receive a decision
        self.assertTrue(self.spin_until_decision_received())

        decision = self.test_node.received_decisions[-1]
        self.assertEqual(decision.object_id, 20)
        # Note: Current implementation doesn't use confidence,
        # but future versions might filter or warn


if __name__ == '__main__':
    unittest.main()
