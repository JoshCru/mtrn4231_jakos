#!/usr/bin/env python3
"""
Test script for CartesianRobot class

Demonstrates safe Cartesian movements with boundary checking.
Run this after starting the fake UR5e system with ./setupFakeur5e.sh
"""

import rclpy
from rclpy.node import Node
from cartesian_robot import CartesianRobot
import sys


def main():
    rclpy.init()
    node = Node("test_cartesian_robot")

    # Initialize robot controller
    node.get_logger().info("Initializing Cartesian Robot...")
    robot = CartesianRobot(node, use_fake_hardware=True)
    node.get_logger().info("Robot ready!")

    try:
        # Define some safe test positions
        # Format: (x, y, z, rx, ry, rz)
        # Positions in mm, rotations in radians
        # Gripper down: rx=-1.571 (-π/2)

        positions = [
            (100, 400, 300, -1.571, 0.0, 0.0, "Position 1: x=100, y=400, z=300"),
            (200, 300, 400, -1.571, 0.0, 0.0, "Position 2: x=200, y=300, z=400"),
            (150, 350, 350, -1.571, 0.0, 0.0, "Position 3: x=150, y=350, z=350"),
        ]

        node.get_logger().info(f"=== Starting test sequence with {len(positions)} positions ===")

        for i, (x, y, z, rx, ry, rz, description) in enumerate(positions, 1):
            node.get_logger().info(f"\n{'='*60}")
            node.get_logger().info(f"Movement {i}/{len(positions)}: {description}")
            node.get_logger().info(f"{'='*60}")

            success = robot.move_to_cartesian(x, y, z, rx, ry, rz)

            if success:
                node.get_logger().info(f"✓ Movement {i} completed successfully")
            else:
                node.get_logger().error(f"✗ Movement {i} failed")
                break

        node.get_logger().info("\n" + "="*60)
        node.get_logger().info("Test sequence completed!")
        node.get_logger().info("="*60)

    except Exception as e:
        node.get_logger().error(f"Test failed with exception: {e}")
        import traceback
        node.get_logger().error(traceback.format_exc())
        return 1

    finally:
        node.destroy_node()
        rclpy.shutdown()

    return 0


if __name__ == "__main__":
    sys.exit(main())
