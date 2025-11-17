#!/usr/bin/env python3
"""
Cartesian Movement Client Example
Demonstrates how to move the UR5e using Cartesian coordinates
"""

import rclpy
from rclpy.node import Node
from sort_interfaces.srv import MoveToCartesian
import sys
import math


class CartesianMovementClient(Node):
    def __init__(self):
        super().__init__('cartesian_movement_client')
        self.client = self.create_client(MoveToCartesian, '/move_to_cartesian')

        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /move_to_cartesian service...')

        self.get_logger().info('Service available!')

    def send_request(self, x, y, z, rx=0.0, ry=3.14159, rz=0.0):
        """
        Send Cartesian movement request

        Args:
            x: X position in millimeters
            y: Y position in millimeters
            z: Z position in millimeters
            rx: Rotation vector X component in radians (default 0)
            ry: Rotation vector Y component in radians (default pi for downward)
            rz: Rotation vector Z component in radians (default 0)

        Returns:
            Service response
        """
        request = MoveToCartesian.Request()
        request.x = float(x)
        request.y = float(y)
        request.z = float(z)
        request.rx = float(rx)
        request.ry = float(ry)
        request.rz = float(rz)

        self.get_logger().info(f'Sending request:')
        self.get_logger().info(f'  Position: x={x:.2f}, y={y:.2f}, z={z:.2f} mm')
        self.get_logger().info(f'  Rotation vector: rx={rx:.4f}, ry={ry:.4f}, rz={rz:.4f} rad')

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main():
    rclpy.init()
    client = CartesianMovementClient()

    # Default home position values from user (UR rotation vector format)
    home_x = 131.75
    home_y = -589.21
    home_z = -28.29
    home_rx = 0.004
    home_ry = 3.147
    home_rz = 0.0

    if len(sys.argv) >= 4:
        # User provided coordinates
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])

        # Optional rotation vector parameters (radians)
        rx = float(sys.argv[4]) if len(sys.argv) > 4 else home_rx
        ry = float(sys.argv[5]) if len(sys.argv) > 5 else home_ry
        rz = float(sys.argv[6]) if len(sys.argv) > 6 else home_rz

        print(f"\nMoving to specified position: x={x}, y={y}, z={z}")
        response = client.send_request(x, y, z, rx, ry, rz)

    elif len(sys.argv) == 2 and sys.argv[1] == 'home':
        # Move to home position
        print(f"\nMoving to home position: x={home_x}, y={home_y}, z={home_z}")
        print(f"Rotation vector: rx={home_rx}, ry={home_ry}, rz={home_rz}")
        response = client.send_request(home_x, home_y, home_z, home_rx, home_ry, home_rz)

    elif len(sys.argv) == 2 and sys.argv[1] == 'demo':
        # Demo sequence
        print("\n=== Cartesian Movement Demo ===")
        print("Moving through a series of positions...")

        positions = [
            (home_x, home_y, home_z, "Home"),
            (home_x + 50, home_y, home_z, "50mm in +X"),
            (home_x + 50, home_y + 50, home_z, "50mm in +Y"),
            (home_x + 50, home_y + 50, home_z + 50, "50mm in +Z"),
            (home_x, home_y, home_z, "Back to home"),
        ]

        for x, y, z, desc in positions:
            print(f"\n--- Moving to: {desc} ---")
            response = client.send_request(x, y, z, home_rx, home_ry, home_rz)
            if response.success:
                print(f"Success: {response.message}")
            else:
                print(f"Failed: {response.message}")
                break

        print("\n=== Demo Complete ===")
        client.destroy_node()
        rclpy.shutdown()
        return

    else:
        # Interactive mode
        print("\n=== Cartesian Movement Controller ===")
        print(f"Home position: x={home_x}, y={home_y}, z={home_z} mm")
        print(f"Home rotation: rx={home_rx}, ry={home_ry}, rz={home_rz} rad")
        print("Wrist2 is constrained to -90 degrees")
        print("\nUsage:")
        print(f"  {sys.argv[0]} <x> <y> <z> [rx] [ry] [rz]")
        print(f"  {sys.argv[0]} home")
        print(f"  {sys.argv[0]} demo")
        print("\nCoordinates in mm, rotation vector in radians")
        print("\nExamples:")
        print(f"  {sys.argv[0]} 131.75 -589.21 -28.29")
        print(f"  {sys.argv[0]} 200.0 -500.0 50.0")
        print(f"  {sys.argv[0]} 150.0 -600.0 0.0 0.004 3.147 0.0")
        print("")

        # Interactive loop
        while True:
            try:
                user_input = input("Enter coordinates (x y z [rx ry rz]) or 'quit': ").strip()

                if user_input.lower() in ['quit', 'exit', 'q']:
                    break

                if user_input.lower() == 'home':
                    x, y, z = home_x, home_y, home_z
                    rx, ry, rz = home_rx, home_ry, home_rz
                else:
                    parts = user_input.split()
                    if len(parts) < 3:
                        print("Please provide at least x, y, z coordinates")
                        continue

                    x = float(parts[0])
                    y = float(parts[1])
                    z = float(parts[2])
                    rx = float(parts[3]) if len(parts) > 3 else home_rx
                    ry = float(parts[4]) if len(parts) > 4 else home_ry
                    rz = float(parts[5]) if len(parts) > 5 else home_rz

                response = client.send_request(x, y, z, rx, ry, rz)

                if response.success:
                    print(f"\nSuccess: {response.message}")
                    if response.joint_positions:
                        print("Joint positions (radians):")
                        joint_names = [
                            'shoulder_pan', 'shoulder_lift', 'elbow',
                            'wrist_1', 'wrist_2', 'wrist_3'
                        ]
                        for i, pos in enumerate(response.joint_positions):
                            if i < len(joint_names):
                                print(f"  {joint_names[i]}: {pos:.4f} rad ({math.degrees(pos):.2f} deg)")
                else:
                    print(f"\nFailed: {response.message}")

                print("")

            except ValueError as e:
                print(f"Invalid input: {e}")
            except KeyboardInterrupt:
                break

        client.destroy_node()
        rclpy.shutdown()
        return

    # Single command result
    if response.success:
        print(f"\nSuccess: {response.message}")
        if response.joint_positions:
            print("Computed joint positions (radians):")
            joint_names = [
                'shoulder_pan', 'shoulder_lift', 'elbow',
                'wrist_1', 'wrist_2', 'wrist_3'
            ]
            for i, pos in enumerate(response.joint_positions):
                if i < len(joint_names):
                    print(f"  {joint_names[i]}: {pos:.4f} rad ({math.degrees(pos):.2f} deg)")
    else:
        print(f"\nFailed: {response.message}")

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
