#!/usr/bin/env python3
"""
Action Client for Cartesian Movement
Uses action interface instead of service for long-running movements

ALL COORDINATES ARE WITH RESPECT TO BASE_LINK FRAME (robot base)
- X, Y, Z: Position in millimeters relative to robot base
- RX, RY, RZ: Rotation vector in radians (UR axis-angle format)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sort_interfaces.action import MoveToCartesian
import sys


class ActionCartesianClient(Node):
    def __init__(self):
        super().__init__('action_cartesian_client')

        self._action_client = ActionClient(self, MoveToCartesian, '/move_to_cartesian_action')

        self.get_logger().info('Waiting for action server...')
        self._action_client.wait_for_server()
        self.get_logger().info('Action server ready!')

    def send_goal(self, x, y, z, rx, ry, rz):
        """Send movement goal and wait for result"""
        goal_msg = MoveToCartesian.Goal()
        goal_msg.x = x
        goal_msg.y = y
        goal_msg.z = z
        goal_msg.rx = rx
        goal_msg.ry = ry
        goal_msg.rz = rz

        self.get_logger().info(f'Sending goal: x={x}, y={y}, z={z} mm')
        self.get_logger().info(f'             rx={rx}, ry={ry}, rz={rz} rad')

        send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )

        rclpy.spin_until_future_complete(self, send_goal_future)
        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return None

        self.get_logger().info('Goal accepted, waiting for result...')

        get_result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, get_result_future)

        return get_result_future.result()

    def feedback_callback(self, feedback_msg):
        """Handle feedback from action server"""
        feedback = feedback_msg.feedback
        self.get_logger().info(f'[{feedback.progress*100:.0f}%] {feedback.status}')


def main():
    rclpy.init()

    if len(sys.argv) < 7:
        print("Usage: action_cartesian_client.py <x> <y> <z> <rx> <ry> <rz>")
        print("  x, y, z: Position in mm")
        print("  rx, ry, rz: Rotation vector in radians")
        print("\nHome position example:")
        print("  action_cartesian_client.py -589.22 -131.78 371.73 2.22 2.22 0.004")
        sys.exit(1)

    # Parse arguments
    x = float(sys.argv[1])
    y = float(sys.argv[2])
    z = float(sys.argv[3])
    rx = float(sys.argv[4])
    ry = float(sys.argv[5])
    rz = float(sys.argv[6])

    client = ActionCartesianClient()

    result = client.send_goal(x, y, z, rx, ry, rz)

    if result:
        if result.result.success:
            print(f'\n✓ SUCCESS: {result.result.message}')
            if result.result.joint_positions:
                print(f'Final joint positions: {[f"{j:.3f}" for j in result.result.joint_positions]}')
        else:
            print(f'\n✗ FAILED: {result.result.message}')
    else:
        print('\n✗ No result received')

    client.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
