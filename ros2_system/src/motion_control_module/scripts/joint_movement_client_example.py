#!/usr/bin/env python3
"""
Example client for the joint_movement_controller service
Shows how the supervisor can call joint movements
"""

import rclpy
from rclpy.node import Node
from sort_interfaces.srv import MoveToJointPosition
import math


class JointMovementClient(Node):
    def __init__(self):
        super().__init__('joint_movement_client_example')

        # Create service client
        self.move_client = self.create_client(
            MoveToJointPosition,
            '/move_to_joint_position'
        )

        while not self.move_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for /move_to_joint_position service...')

        self.get_logger().info('Connected to joint movement service!')

    def move_to_position(self, position_name, joint_positions_deg):
        """
        Move to a position given joint angles in degrees

        Args:
            position_name (str): Name of the position
            joint_positions_deg (list): 6 joint positions in degrees [J6, J1, J2, J3, J4, J5]
        """
        # Convert degrees to radians
        joint_positions_rad = [math.radians(d) for d in joint_positions_deg]

        # Create request
        request = MoveToJointPosition.Request()
        request.position_name = position_name
        request.joint_positions = joint_positions_rad

        self.get_logger().info(f"\n{'='*60}")
        self.get_logger().info(f"Requesting movement to: {position_name}")
        self.get_logger().info(f"Joint positions [J6, J1, J2, J3, J4, J5]:")
        self.get_logger().info(f"  Degrees: {joint_positions_deg}")
        self.get_logger().info(f"  Radians: {[f'{r:.4f}' for r in joint_positions_rad]}")
        self.get_logger().info(f"{'='*60}\n")

        # Call service
        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        response = future.result()
        if response.success:
            self.get_logger().info(f"✓ {response.message}")
            return True
        else:
            self.get_logger().error(f"✗ {response.message}")
            return False

    def run_example_sequence(self):
        """Run the example pickup sequence"""
        # Define positions (same as pickup_movement.py)
        # Format: [Joint6, Joint1, Joint2, Joint3, Joint4, Joint5]
        positions = [
            ('Home point',         [  0.0, -75.0,  90.0, -105.0, -90.0,  0.0]),
            ('Pick up point',      [  0.0, -53.0,  88.0, -127.0, -86.0,  9.0]),
            ('Lift up point',      [-12.0, -60.0,  82.0, -113.0, -86.0,  9.0]),
            ('Put down point',     [-12.0, -53.0,  88.0, -126.0, -86.0,  9.0]),
            ('Home point (return)', [  0.0, -75.0,  90.0, -105.0, -90.0,  0.0])
        ]

        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("Starting example pickup sequence via C++ service")
        self.get_logger().info("="*60)

        for position_name, joint_angles in positions:
            print(f"\nPress ENTER to move to '{position_name}'...")
            input()

            success = self.move_to_position(position_name, joint_angles)

            if not success:
                self.get_logger().error(f"Failed at {position_name}, stopping sequence")
                return False

        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("✓ Sequence completed successfully!")
        self.get_logger().info("="*60)
        return True


def main(args=None):
    rclpy.init(args=args)

    client = JointMovementClient()

    try:
        client.run_example_sequence()

    except KeyboardInterrupt:
        client.get_logger().info("Interrupted by user")
    except Exception as e:
        client.get_logger().error(f"Error: {str(e)}")
        import traceback
        client.get_logger().error(traceback.format_exc())
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
