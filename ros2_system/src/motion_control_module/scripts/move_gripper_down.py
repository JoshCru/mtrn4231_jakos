#!/usr/bin/env python3
"""
Simplified Cartesian Movement Client - Gripper always faces down
Only specify X, Y, Z positions - orientation is automatically set to face downward

ALL COORDINATES ARE WITH RESPECT TO BASE_LINK FRAME (robot base)
- X, Y, Z: Position in millimeters relative to robot base
- Orientation is automatically set to gripper facing downward (RX=-π/2, RY=0, RZ=0)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sort_interfaces.action import MoveToCartesian
import sys


class GripperDownClient(Node):
    def __init__(self):
        super().__init__('gripper_down_client')

        self._action_client = ActionClient(self, MoveToCartesian, '/move_to_cartesian_action')

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready!')

    def move_to(self, x_mm, y_mm, z_mm):
        """
        Move gripper to position with downward orientation

        Args:
            x_mm: X position in millimeters
            y_mm: Y position in millimeters
            z_mm: Z position in millimeters
        """
        goal_msg = MoveToCartesian.Goal()
        goal_msg.x = x_mm
        goal_msg.y = y_mm
        goal_msg.z = z_mm

        # Fixed orientation: gripper facing down
        # RX = -π/2 (pointing down), RY = 0, RZ = 0
        goal_msg.rx = -1.5708  # -90 degrees in radians
        goal_msg.ry = 0.0
        goal_msg.rz = 0.0

        self.get_logger().info(f'Moving gripper (down) to: X={x_mm:.1f}, Y={y_mm:.1f}, Z={z_mm:.1f} mm')

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return None

        self.get_logger().info('Goal accepted, executing...')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        return get_result_future.result()

    def feedback_callback(self, feedback_msg):
        """Handle feedback from action server"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[{feedback.progress*100:.0f}%] {feedback.status}')


def main():
    rclpy.init()

    if len(sys.argv) < 4:
        print("Usage: move_gripper_down.py <x_mm> <y_mm> <z_mm>")
        print("")
        print("Moves the gripper to the specified position with downward orientation")
        print("  x_mm, y_mm, z_mm: Position in millimeters")
        print("")
        print("Examples:")
        print("  move_gripper_down.py 100 400 800    # Move to X=100, Y=400, Z=800mm")
        print("  move_gripper_down.py 0 383 1100     # Move to X=0, Y=383, Z=1100mm")
        sys.exit(1)

    # Parse arguments
    x_mm = float(sys.argv[1])
    y_mm = float(sys.argv[2])
    z_mm = float(sys.argv[3])

    client = GripperDownClient()
    result = client.move_to(x_mm, y_mm, z_mm)

    if result:
        if result.result.success:
            print(f'\n✓ SUCCESS: {result.result.message}')
            if result.result.joint_positions:
                print(f'Final joint positions: {[f"{j:.3f}" for j in result.result.joint_positions]}')
        else:
            print(f'\n✗ FAILED: {result.result.message}')
    else:
        print('\n✗ Goal was rejected')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
