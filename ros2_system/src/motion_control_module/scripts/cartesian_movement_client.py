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

    def send_request(self, x, y, z, roll=0.0, pitch=-180.0, yaw=0.0):
        """
        Send Cartesian movement request

        Args:
            x: X position in millimeters
            y: Y position in millimeters
            z: Z position in millimeters
            roll: Roll angle in degrees (default 0)
            pitch: Pitch angle in degrees (default -180 for downward)
            yaw: Yaw angle in degrees (default 0)

        Returns:
            Service response
        """
        request = MoveToCartesian.Request()
        request.x = float(x)
        request.y = float(y)
        request.z = float(z)
        request.roll = float(roll)
        request.pitch = float(pitch)
        request.yaw = float(yaw)

        self.get_logger().info(f'Sending request:')
        self.get_logger().info(f'  Position: x={x:.2f}, y={y:.2f}, z={z:.2f} mm')
        self.get_logger().info(f'  Orientation: roll={roll:.2f}, pitch={pitch:.2f}, yaw={yaw:.2f} deg')

        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        return future.result()


def main():
    rclpy.init()
    client = CartesianMovementClient()

    # Default home position values from user
    home_x = 131.75
    home_y = -589.22
    home_z = -28.30

    if len(sys.argv) >= 4:
        # User provided coordinates
        x = float(sys.argv[1])
        y = float(sys.argv[2])
        z = float(sys.argv[3])

        # Optional orientation parameters
        roll = float(sys.argv[4]) if len(sys.argv) > 4 else 0.0
        pitch = float(sys.argv[5]) if len(sys.argv) > 5 else -180.0
        yaw = float(sys.argv[6]) if len(sys.argv) > 6 else 0.0

        print(f"\nMoving to specified position: x={x}, y={y}, z={z}")
        response = client.send_request(x, y, z, roll, pitch, yaw)

    elif len(sys.argv) == 2 and sys.argv[1] == 'home':
        # Move to home position
        print(f"\nMoving to home position: x={home_x}, y={home_y}, z={home_z}")
        response = client.send_request(home_x, home_y, home_z)

    elif len(sys.argv) == 2 and sys.argv[1] == 'demo':
        # Demo sequence
        print("\n=== Cartesian Movement Demo ===")
        print("Moving through a series of positions...")

        positions = [
            (home_x, home_y, home_z, "Home"),
            (home_x + 50, home_y, home_z, "50mm right"),
            (home_x + 50, home_y + 50, home_z, "50mm forward"),
            (home_x + 50, home_y + 50, home_z + 50, "50mm up"),
            (home_x, home_y, home_z, "Back to home"),
        ]

        for x, y, z, desc in positions:
            print(f"\n--- Moving to: {desc} ---")
            response = client.send_request(x, y, z)
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
        print("Wrist2 is constrained to -90 degrees")
        print("\nUsage:")
        print(f"  {sys.argv[0]} <x> <y> <z> [roll] [pitch] [yaw]")
        print(f"  {sys.argv[0]} home")
        print(f"  {sys.argv[0]} demo")
        print("\nExamples:")
        print(f"  {sys.argv[0]} 131.75 -589.22 -28.30")
        print(f"  {sys.argv[0]} 200.0 -500.0 50.0")
        print(f"  {sys.argv[0]} 150.0 -600.0 0.0 0.0 -180.0 45.0")
        print("")

        # Interactive loop
        while True:
            try:
                user_input = input("Enter coordinates (x y z) or 'quit': ").strip()

                if user_input.lower() in ['quit', 'exit', 'q']:
                    break

                if user_input.lower() == 'home':
                    x, y, z = home_x, home_y, home_z
                    roll, pitch, yaw = 0.0, -180.0, 0.0
                else:
                    parts = user_input.split()
                    if len(parts) < 3:
                        print("Please provide at least x, y, z coordinates")
                        continue

                    x = float(parts[0])
                    y = float(parts[1])
                    z = float(parts[2])
                    roll = float(parts[3]) if len(parts) > 3 else 0.0
                    pitch = float(parts[4]) if len(parts) > 4 else -180.0
                    yaw = float(parts[5]) if len(parts) > 5 else 0.0

                response = client.send_request(x, y, z, roll, pitch, yaw)

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
