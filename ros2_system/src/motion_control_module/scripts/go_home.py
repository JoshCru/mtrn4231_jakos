#!/usr/bin/env python3
"""
Go Home Script - Moves robot to a known home position via joint trajectory.

This is needed before Cartesian movements because the robot starts at
all-zeros joint position with fake hardware, which is not a valid
starting point for Cartesian path planning.

Home Joint Angles (radians):
  [0.0, -1.309, 1.571, -1.833, -1.571, 0.0]
  = [0°, -75°, 90°, -105°, -90°, 0°]
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
import sys

JOINT_NAMES = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Home position joint angles (radians)
HOME_JOINTS = [0.0, -1.309, 1.571, -1.833, -1.571, 0.0]


class GoHomeNode(Node):
    def __init__(self):
        super().__init__('go_home_node')

        self.action_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        self.get_logger().info('Waiting for joint trajectory action server...')
        if not self.action_client.wait_for_server(timeout_sec=30.0):
            self.get_logger().error('Action server not available!')
            return

        self.get_logger().info('Action server ready')

    def go_home(self, duration_sec=5.0):
        """Send robot to home position."""
        goal = FollowJointTrajectory.Goal()

        trajectory = JointTrajectory()
        trajectory.joint_names = JOINT_NAMES

        point = JointTrajectoryPoint()
        point.positions = HOME_JOINTS
        point.velocities = [0.0] * 6
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)

        trajectory.points.append(point)
        goal.trajectory = trajectory

        self.get_logger().info(f'Moving to home position over {duration_sec}s...')
        self.get_logger().info(f'Target: {HOME_JOINTS}')

        future = self.action_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, future)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected!')
            return False

        self.get_logger().info('Goal accepted, executing...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()
        if result.result.error_code == 0:
            self.get_logger().info('Successfully moved to home position!')
            return True
        else:
            self.get_logger().error(f'Failed with error code: {result.result.error_code}')
            return False


def main(args=None):
    rclpy.init(args=args)

    node = GoHomeNode()

    duration = 5.0
    if len(sys.argv) > 1:
        try:
            duration = float(sys.argv[1])
        except ValueError:
            pass

    success = node.go_home(duration)

    node.destroy_node()
    rclpy.shutdown()

    sys.exit(0 if success else 1)


if __name__ == '__main__':
    main()
