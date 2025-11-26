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
            (100,100,100, 3.14, 0.001, 0.001, "Position 1")
            # (500, 400, 100, 3.14, 0.001, 0.001, "Position 1"),
            # (500, 500, 100, 3.14, 0.001, 0.001, "Position 2"),
            # (500, 200, 100, 3.14, 0.001, 0.001, "Position 3"),
            # (500, 400, 100, 3.14, 0.001, 0.001, "Position 4"),
            # (600, 600, 100, 3.14, 0.001, 0.001, "Position 5"),
            # (600, 200, 100, 3.14, 0.001, 0.001, "Position 6"),
            
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
