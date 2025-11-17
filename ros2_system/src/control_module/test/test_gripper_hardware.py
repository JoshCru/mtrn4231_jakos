#!/usr/bin/env python3
"""
Integration test for gripper_controller_node in HARDWARE mode
Tests gripper control with actual Teensy 4.1 hardware

NOTE: This test requires:
1. Teensy 4.1 connected via USB
2. teensy_gripper_controller.ino uploaded to Teensy
3. Servo motor connected to pin 9
4. Force sensor connected to pin A0 (optional for basic tests)
"""

import unittest
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from sort_interfaces.msg import ForceFeedback
from sort_interfaces.srv import CalibrateGripper


class GripperHardwareTestNode(Node):
    """Test node for gripper hardware testing"""

    def __init__(self):
        super().__init__('gripper_hardware_test_node')

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

        # Calibration service client
        self.calibrate_client = self.create_client(
            CalibrateGripper,
            '/motion_control/calibrate_gripper'
        )

        self.received_states = []
        self.received_forces = []

    def state_callback(self, msg):
        self.received_states.append(msg.data)
        self.get_logger().info(f'Hardware state: {msg.data}')

    def force_callback(self, msg):
        self.received_forces.append(msg)
        self.get_logger().info(
            f'Hardware force: weight={msg.measured_weight}g, '
            f'detected={msg.object_detected}'
        )

    def send_gripper_command(self, position):
        msg = Float32()
        msg.data = position
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent hardware command: {position}')

    def call_calibrate_service(self, tare=True, calibrate_pos=False):
        """Call calibration service"""
        if not self.calibrate_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().error('Calibration service not available')
            return None

        request = CalibrateGripper.Request()
        request.tare_weight_sensor = tare
        request.calibrate_position = calibrate_pos

        future = self.calibrate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None:
            return future.result()
        return None


class TestGripperHardware(unittest.TestCase):
    """Integration tests for gripper_controller_node with real hardware"""

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
        self.test_node = GripperHardwareTestNode()
        time.sleep(1.0)  # Allow Teensy to initialize

    def tearDown(self):
        """Cleanup after each test"""
        # Return gripper to safe position (open)
        self.test_node.send_gripper_command(0.0)
        time.sleep(1.0)
        self.test_node.destroy_node()

    def spin_and_wait(self, duration_sec=2.0):
        """Spin for a duration to collect messages"""
        start = time.time()
        while time.time() - start < duration_sec:
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

    def test_hardware_connection(self):
        """Test that hardware is connected and responding"""
        self.test_node.received_forces.clear()

        # Wait for force feedback
        self.spin_and_wait(3.0)

        # Should receive messages from hardware
        self.assertGreater(
            len(self.test_node.received_forces),
            0,
            "Should receive force feedback from Teensy. "
            "Check: 1) Teensy is connected, 2) Code is uploaded, 3) Serial port is correct"
        )

    def test_servo_movement(self):
        """Test that servo moves in response to commands"""
        print("\n*** VISUAL TEST: Watch the servo motor ***")
        print("The servo should move from OPEN to CLOSED positions")

        # Open gripper
        print("Moving to OPEN (0 degrees)...")
        self.test_node.send_gripper_command(0.0)
        self.spin_and_wait(2.0)

        # Close gripper
        print("Moving to CLOSED (180 degrees)...")
        self.test_node.send_gripper_command(1.0)
        self.spin_and_wait(2.0)

        # Mid position
        print("Moving to MID (90 degrees)...")
        self.test_node.send_gripper_command(0.5)
        self.spin_and_wait(2.0)

        # Return to open
        print("Returning to OPEN...")
        self.test_node.send_gripper_command(0.0)
        self.spin_and_wait(2.0)

        # This test always passes - it's for visual verification
        self.assertTrue(True, "Servo movement test completed")

    def test_force_sensor_readings(self):
        """Test that force sensor provides readings"""
        self.test_node.received_forces.clear()

        # Open gripper (no object)
        self.test_node.send_gripper_command(0.0)
        self.spin_and_wait(2.0)

        open_forces = [f.measured_weight for f in self.test_node.received_forces]

        self.assertGreater(
            len(open_forces),
            5,
            "Should receive multiple force readings"
        )

        # Readings should be consistent (low variance)
        if len(open_forces) > 1:
            import statistics
            std_dev = statistics.stdev(open_forces)
            self.assertLess(
                std_dev,
                50.0,
                f"Force readings should be stable (std dev: {std_dev}g)"
            )

    def test_weight_calibration(self):
        """Test weight sensor calibration (tare)"""
        print("\n*** CALIBRATION TEST ***")
        print("Ensure NO object is in the gripper")

        # Call calibration service
        result = self.test_node.call_calibrate_service(tare=True, calibrate_pos=False)

        self.assertIsNotNone(result, "Calibration service should respond")
        self.assertTrue(result.success, "Calibration should succeed")

        print(f"Calibration result: {result.message}")
        print(f"Zero offset: {result.zero_force_offset}")

        # After calibration, weight should be close to zero
        self.test_node.received_forces.clear()
        self.spin_and_wait(2.0)

        if len(self.test_node.received_forces) > 0:
            avg_weight = sum(f.measured_weight for f in self.test_node.received_forces) / len(self.test_node.received_forces)
            self.assertLess(
                abs(avg_weight),
                20.0,
                f"After calibration, weight should be near zero (got {avg_weight}g)"
            )

    def test_object_detection_manual(self):
        """Test object detection with manual intervention"""
        print("\n*** MANUAL TEST: Object Detection ***")
        print("Step 1: Ensure gripper is EMPTY")
        input("Press Enter when ready...")

        # Open gripper, check no object detected
        self.test_node.send_gripper_command(0.0)
        self.test_node.received_forces.clear()
        self.spin_and_wait(2.0)

        empty_detected = [f.object_detected for f in self.test_node.received_forces]
        if len(empty_detected) > 0:
            # Most readings should be False
            false_count = sum(1 for d in empty_detected if not d)
            self.assertGreater(
                false_count,
                len(empty_detected) * 0.8,
                "Empty gripper should mostly not detect object"
            )

        print("\nStep 2: PLACE an object in the gripper")
        print("Then close the gripper")
        input("Press Enter when ready...")

        # Close gripper
        self.test_node.send_gripper_command(1.0)
        self.test_node.received_forces.clear()
        self.spin_and_wait(3.0)

        object_detected = [f.object_detected for f in self.test_node.received_forces]
        weights = [f.measured_weight for f in self.test_node.received_forces]

        if len(object_detected) > 0:
            print(f"\nWeight readings: min={min(weights):.1f}g, max={max(weights):.1f}g, avg={sum(weights)/len(weights):.1f}g")
            print(f"Object detected: {sum(1 for d in object_detected if d)}/{len(object_detected)} readings")

            # At least some readings should detect object
            true_count = sum(1 for d in object_detected if d)
            self.assertGreater(
                true_count,
                0,
                "Should detect object when grasping"
            )

    def test_serial_communication(self):
        """Test that serial communication is working"""
        self.test_node.received_forces.clear()

        # Send command
        self.test_node.send_gripper_command(0.5)

        # Wait for response
        self.spin_and_wait(2.0)

        # Should receive feedback (proves serial communication works)
        self.assertGreater(
            len(self.test_node.received_forces),
            5,
            "Should receive force feedback from Teensy (serial communication working)"
        )

    def test_rapid_commands(self):
        """Test rapid gripper commands"""
        print("\n*** STRESS TEST: Rapid commands ***")

        positions = [0.0, 1.0, 0.5, 0.25, 0.75, 0.0]

        for pos in positions:
            self.test_node.send_gripper_command(pos)
            time.sleep(0.5)
            rclpy.spin_once(self.test_node, timeout_sec=0.1)

        # Wait for final position
        self.spin_and_wait(1.0)

        # Should still be receiving feedback
        self.test_node.received_forces.clear()
        self.spin_and_wait(1.0)

        self.assertGreater(
            len(self.test_node.received_forces),
            0,
            "Should continue receiving feedback after rapid commands"
        )


if __name__ == '__main__':
    print("="*60)
    print("HARDWARE TEST - Teensy 4.1 Gripper Controller")
    print("="*60)
    print("\nPre-flight checklist:")
    print("[ ] Teensy 4.1 is connected via USB")
    print("[ ] teensy_gripper_controller.ino is uploaded")
    print("[ ] Servo motor is connected to pin 9")
    print("[ ] Force sensor is connected to pin A0 (optional)")
    print("[ ] Gripper is in a safe position")
    print("\n" + "="*60 + "\n")

    response = input("Ready to run hardware tests? (yes/no): ")
    if response.lower() in ['yes', 'y']:
        unittest.main()
    else:
        print("Tests cancelled")
