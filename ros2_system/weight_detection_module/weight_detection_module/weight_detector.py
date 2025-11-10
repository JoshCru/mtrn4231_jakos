#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState


class WeightDetector(Node):
    def __init__(self):
        super().__init__('weight_detector')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.get_logger().info('Weight detection module initialized')

    def joint_state_callback(self, msg):
        joint_torques = msg.effort
        print(joint_torques)


def main(args=None):
    rclpy.init(args=args)
    weight_detector = WeightDetector()
    
    try:
        rclpy.spin(weight_detector)
    except KeyboardInterrupt:
        pass
    finally:
        weight_detector.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
