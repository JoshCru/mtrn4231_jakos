#!/usr/bin/env python3
"""
Simple Cartesian Movement Script

Usage:
    python3 move_cartesian_simple.py <x> <y> <z> <rx> <ry> <rz>

Arguments:
    x, y, z:   Position in mm (relative to base_link)
    rx, ry, rz: Rotation vector in radians (UR axis-angle format)

Example:
    # Move gripper facing down to (100mm, 400mm, 300mm)
    python3 move_cartesian_simple.py 100 400 300 -1.571 0.0 0.0

Safety Boundaries:
    x >= -300mm, y >= -300mm, 0mm <= z <= 655mm
"""

import rclpy
from rclpy.node import Node
from cartesian_robot import CartesianRobot
import sys


def main():
    if len(sys.argv) != 7:
        print(__doc__)
        return 1

    # Parse arguments
    try:
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])
        rx = float(sys.argv[4])
        ry = float(sys.argv[5])
        rz = float(sys.argv[6])
    except ValueError:
        print("Error: All arguments must be numbers")
        print(__doc__)
        return 1

    # Initialize ROS2
    rclpy.init()
    node = Node("move_cartesian_simple")

    try:
        # Initialize robot
        node.get_logger().info("Initializing Cartesian Robot...")
        robot = CartesianRobot(node, use_fake_hardware=True)

        # Execute movement
        node.get_logger().info(f"Moving to: x={x}, y={y}, z={z}, rx={rx}, ry={ry}, rz={rz}")
        success = robot.move_to_cartesian(x, y, z, rx, ry, rz)

        if success:
            node.get_logger().info("Movement completed successfully!")
            return 0
        else:
            node.get_logger().error("Movement failed!")
            return 1

    except Exception as e:
        node.get_logger().error(f"Error: {e}")
        return 1

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    sys.exit(main())
