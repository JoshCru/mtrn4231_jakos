#!/usr/bin/env python3
"""
Test Home Position

This script tests the go_home() function and demonstrates
constrained movements with the move() method.
"""

import rclpy
from rclpy.node import Node
from cartesian_robot import CartesianRobot
import sys


def main():
    rclpy.init()
    node = Node("test_home_position")

    try:
        # Initialize robot
        node.get_logger().info("Initializing Cartesian Robot...")
        robot = CartesianRobot(node, use_fake_hardware=True)
        node.get_logger().info("Robot ready!")

        # Test 1: Go to home position
        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("TEST 1: Moving to HOME position")
        node.get_logger().info("="*60)
        robot.go_home()

        # Test 2: Move to a few positions using constrained orientation
        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("TEST 2: Moving with constrained orientation")
        node.get_logger().info("End-effector will always face down (rx=3.14, ry=0.001, rz=0.001)")
        node.get_logger().info("="*60)

        # Small movements to test smooth paths
        test_positions = [
            (100, 400, 300, "Position 1"),
            (120, 420, 320, "Position 2 - small movement from Position 1"),
            (100, 400, 300, "Position 1 again - small movement back"),
        ]

        for x, y, z, description in test_positions:
            node.get_logger().info(f"\nMoving to {description}: x={x}, y={y}, z={z}")
            robot.move(x, y, z)

        # Test 3: Go back home
        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("TEST 3: Returning to HOME position")
        node.get_logger().info("="*60)
        robot.go_home()

        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("All tests completed successfully!")
        node.get_logger().info("="*60)

        return 0

    except Exception as e:
        node.get_logger().error(f"Test failed: {e}")
        import traceback
        node.get_logger().error(traceback.format_exc())
        return 1

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
