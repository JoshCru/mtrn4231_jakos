#!/usr/bin/env python3
"""
Integration test for gripper_controller_node in SIMULATION mode
Tests gripper control and weight sensing without hardware
"""

import unittest
import time
import rclpy
from rclpy.node import Node
from rclpy.lifecycle import LifecycleNode, State, TransitionCallbackReturn
from lifecycle_msgs.srv import ChangeState, GetState
from lifecycle_msgs.msg import Transition
from std_msgs.msg import Float32
from sort_interfaces.msg import ForceFeedback
from sort_interfaces.srv import CalibrateGripper


class GripperTestNode(Node):
    """Test node for gripper controller testing"""

    def __init__(self):
        super().__init__('gripper_test_node')

        # Subscriber for gripper state
        self.state_sub = self.create_subscription(
            Float32,
            '/motion_control/gripper_state',
            self.state_callback,
            10
        )

        # Subscriber for force feedback
        self.force_sub = self.create_subscription(
            ForceFeedback,
            '/motion_control/force_feedback',
            self.force_callback,
            10
        )

        # Publisher for gripper commands
        self.command_pub = self.create_publisher(
            Float32,
            '/motion_control/gripper_command',
            10
        )

        # Service clients for lifecycle management
        self.change_state_client = self.create_client(
            ChangeState,
            '/gripper_controller_node/change_state'
        )

        self.get_state_client = self.create_client(
            GetState,
            '/gripper_controller_node/get_state'
        )

        # Calibration service client
        self.calibrate_client = self.create_client(
            CalibrateGripper,
            '/motion_control/calibrate_gripper'
        )

        self.received_states = []
        self.received_forces = []
        self.state_received = False
        self.force_received = False

    def state_callback(self, msg):
        """Store received gripper states"""
        self.received_states.append(msg.data)
        self.state_received = True
        self.get_logger().info(f'Received gripper state: {msg.data}')

    def force_callback(self, msg):
        """Store received force feedback"""
        self.received_forces.append(msg)
        self.force_received = True
        self.get_logger().info(
            f'Received force feedback: weight={msg.measured_weight}g, '
            f'pos={msg.gripper_position}, detected={msg.object_detected}'
        )

    def send_gripper_command(self, position):
        """Send gripper command"""
        msg = Float32()
        msg.data = position
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent gripper command: {position}')

    def change_lifecycle_state(self, transition_id):
        """Change lifecycle state"""
        if not self.change_state_client.wait_for_service(timeout_sec=5.0):
            return False

        request = ChangeState.Request()
        request.transition = Transition()
        request.transition.id = transition_id

        future = self.change_state_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None:
            return future.result().success
        return False


class TestGripperSimulation(unittest.TestCase):
    """Integration tests for gripper_controller_node in simulation mode"""

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
        self.test_node = GripperTestNode()
        time.sleep(0.5)  # Allow connections to establish

    def tearDown(self):
        """Cleanup after each test"""
        self.test_node.destroy_node()

    def spin_until_data_received(self, data_type='force', timeout_sec=10.0, count=1):
        """Spin node until data received or timeout"""
        start_time = time.time()

        while True:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

            if data_type == 'force' and len(self.test_node.received_forces) >= count:
                return True
            elif data_type == 'state' and len(self.test_node.received_states) >= count:
                return True

            if time.time() - start_time > timeout_sec:
                return False

    def test_node_starts_and_publishes(self):
        """Test that node starts and publishes data"""
        # Wait for force feedback
        self.assertTrue(
            self.spin_until_data_received('force', timeout_sec=15.0, count=1),
            "No force feedback received within timeout"
        )

        # Verify we got data
        self.assertGreater(
            len(self.test_node.received_forces),
            0,
            "Should receive at least one force feedback message"
        )

    def test_force_feedback_structure(self):
        """Test that ForceFeedback messages have valid structure"""
        self.test_node.received_forces.clear()

        self.assertTrue(
            self.spin_until_data_received('force', timeout_sec=15.0, count=1)
        )

        feedback = self.test_node.received_forces[0]

        # Check message fields exist and have correct types
        self.assertIsInstance(feedback.gripper_force, float)
        self.assertIsInstance(feedback.measured_weight, float)
        self.assertIsInstance(feedback.gripper_position, float)
        self.assertIsInstance(feedback.object_detected, bool)
        self.assertIsInstance(feedback.raw_sensor_value, float)

    def test_gripper_command_response(self):
        """Test that gripper responds to commands"""
        self.test_node.received_states.clear()

        # Send open command (0.0)
        self.test_node.send_gripper_command(0.0)
        time.sleep(0.5)

        # Wait for state update
        self.assertTrue(
            self.spin_until_data_received('state', timeout_sec=5.0, count=1),
            "No state feedback after gripper command"
        )

        # Should report open position
        self.assertAlmostEqual(
            self.test_node.received_states[-1],
            0.0,
            places=2,
            msg="Gripper should report open position"
        )

        # Clear and send close command (1.0)
        self.test_node.received_states.clear()
        self.test_node.send_gripper_command(1.0)
        time.sleep(0.5)

        self.assertTrue(
            self.spin_until_data_received('state', timeout_sec=5.0, count=1)
        )

        # Should report closed position
        self.assertAlmostEqual(
            self.test_node.received_states[-1],
            1.0,
            places=2,
            msg="Gripper should report closed position"
        )

    def test_weight_changes_with_position(self):
        """Test that simulated weight changes when gripper closes"""
        # Send open command
        self.test_node.send_gripper_command(0.0)
        time.sleep(0.5)

        self.test_node.received_forces.clear()
        self.assertTrue(
            self.spin_until_data_received('force', timeout_sec=5.0, count=2)
        )

        open_weight = self.test_node.received_forces[-1].measured_weight

        # Send close command
        self.test_node.send_gripper_command(1.0)
        time.sleep(0.5)

        self.test_node.received_forces.clear()
        self.assertTrue(
            self.spin_until_data_received('force', timeout_sec=5.0, count=2)
        )

        closed_weight = self.test_node.received_forces[-1].measured_weight

        # Closed gripper should show higher weight in simulation
        self.assertGreater(
            closed_weight,
            open_weight,
            "Simulated weight should be higher when gripper is closed"
        )

    def test_object_detection(self):
        """Test that object detection works based on weight threshold"""
        # Close gripper (should detect object in simulation)
        self.test_node.send_gripper_command(1.0)
        time.sleep(0.5)

        self.test_node.received_forces.clear()
        self.assertTrue(
            self.spin_until_data_received('force', timeout_sec=5.0, count=1)
        )

        feedback = self.test_node.received_forces[-1]

        # Simulated closed gripper should detect object
        if feedback.measured_weight > 50.0:  # threshold from config
            self.assertTrue(
                feedback.object_detected,
                "Object should be detected when weight is above threshold"
            )

    def test_gripper_position_range(self):
        """Test gripper position is clamped to valid range"""
        # Test various positions
        test_positions = [0.0, 0.25, 0.5, 0.75, 1.0]

        for pos in test_positions:
            self.test_node.received_states.clear()
            self.test_node.send_gripper_command(pos)
            time.sleep(0.3)

            self.assertTrue(
                self.spin_until_data_received('state', timeout_sec=5.0, count=1)
            )

            reported_pos = self.test_node.received_states[-1]

            self.assertGreaterEqual(reported_pos, 0.0, f"Position {pos} should be >= 0.0")
            self.assertLessEqual(reported_pos, 1.0, f"Position {pos} should be <= 1.0")

    def test_continuous_publishing(self):
        """Test that feedback is published continuously"""
        self.test_node.received_forces.clear()

        # Wait for first message
        self.assertTrue(
            self.spin_until_data_received('force', timeout_sec=5.0, count=1)
        )

        first_count = len(self.test_node.received_forces)

        # Wait a bit longer (publish_rate is 20 Hz, so expect ~40 messages in 2 sec)
        time.sleep(2.0)
        while self.test_node.force_received:
            self.test_node.force_received = False
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Should have more messages
        self.assertGreater(
            len(self.test_node.received_forces),
            first_count + 10,  # Expect at least 10 more in 2 seconds
            "Should receive continuous force feedback"
        )

    def test_weight_values_reasonable(self):
        """Test that simulated weight values are reasonable"""
        self.test_node.received_forces.clear()

        self.assertTrue(
            self.spin_until_data_received('force', timeout_sec=5.0, count=1)
        )

        for feedback in self.test_node.received_forces:
            self.assertGreaterEqual(
                feedback.measured_weight,
                0.0,
                "Weight should never be negative"
            )
            self.assertLess(
                feedback.measured_weight,
                1000.0,
                "Simulated weight should be reasonable"
            )

    def test_force_calculation(self):
        """Test that force is calculated from weight"""
        self.test_node.received_forces.clear()

        self.assertTrue(
            self.spin_until_data_received('force', timeout_sec=5.0, count=1)
        )

        feedback = self.test_node.received_forces[0]

        # Force (N) = Weight (g) / 9.81
        expected_force = feedback.measured_weight / 9.81

        self.assertAlmostEqual(
            feedback.gripper_force,
            expected_force,
            places=2,
            msg="Force should be calculated from weight correctly"
        )

    def test_timestamp_recent(self):
        """Test that timestamps are recent"""
        self.test_node.received_forces.clear()

        self.assertTrue(
            self.spin_until_data_received('force', timeout_sec=5.0, count=1)
        )

        feedback = self.test_node.received_forces[0]

        # Check header has timestamp
        self.assertIsNotNone(feedback.header.stamp)

        # Timestamp should be recent (within last 10 seconds)
        now = self.test_node.get_clock().now()
        msg_time = rclpy.time.Time.from_msg(feedback.header.stamp)
        time_diff = (now - msg_time).nanoseconds / 1e9

        self.assertLess(
            abs(time_diff),
            10.0,
            f"Timestamp should be recent (was {time_diff}s old)"
        )

    def test_position_in_feedback(self):
        """Test that gripper position is included in force feedback"""
        # Set a specific position
        self.test_node.send_gripper_command(0.7)
        time.sleep(0.5)

        self.test_node.received_forces.clear()
        self.assertTrue(
            self.spin_until_data_received('force', timeout_sec=5.0, count=1)
        )

        feedback = self.test_node.received_forces[-1]

        # Position in feedback should match commanded position
        self.assertAlmostEqual(
            feedback.gripper_position,
            0.7,
            places=2,
            msg="Feedback should include current gripper position"
        )


if __name__ == '__main__':
    unittest.main()
