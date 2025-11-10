#!/usr/bin/env python3
"""
Simple diagnostic script to check current joint states
Displays both radians and degrees for easy comparison
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import math


class JointStateChecker(Node):
    def __init__(self):
        super().__init__('joint_state_checker')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.get_logger().info("Joint State Checker started. Listening to /joint_states...")
        self.get_logger().info("Press Ctrl+C to exit")

    def joint_state_callback(self, msg):
        """Display joint states"""
        print("\n" + "="*80)
        print("CURRENT JOINT STATES")
        print("="*80)

        # Joint order: [Joint6, Joint1, Joint2, Joint3, Joint4, Joint5]
        joint_order = [
            ('shoulder_pan_joint', 'Joint 6 (base)'),
            ('shoulder_lift_joint', 'Joint 1'),
            ('elbow_joint', 'Joint 2'),
            ('wrist_1_joint', 'Joint 3'),
            ('wrist_2_joint', 'Joint 4'),
            ('wrist_3_joint', 'Joint 5')
        ]

        for joint_name, label in joint_order:
            try:
                idx = msg.name.index(joint_name)
                pos_rad = msg.position[idx]
                pos_deg = math.degrees(pos_rad)
                print(f"{label:20s} ({joint_name:25s}): {pos_rad:8.4f} rad = {pos_deg:8.2f}°")
            except ValueError:
                print(f"{label:20s} ({joint_name:25s}): NOT FOUND")

        print("="*80)
        print("As array [J6, J1, J2, J3, J4, J5]:")

        positions_rad = []
        positions_deg = []
        for joint_name, _ in joint_order:
            try:
                idx = msg.name.index(joint_name)
                positions_rad.append(msg.position[idx])
                positions_deg.append(math.degrees(msg.position[idx]))
            except ValueError:
                positions_rad.append(float('nan'))
                positions_deg.append(float('nan'))

        print(f"Radians: [{', '.join(f'{p:8.4f}' for p in positions_rad)}]")
        print(f"Degrees: [{', '.join(f'{p:7.2f}°' for p in positions_deg)}]")
        print("="*80 + "\n")


def main(args=None):
    rclpy.init(args=args)
    node = JointStateChecker()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("\nShutting down...")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
