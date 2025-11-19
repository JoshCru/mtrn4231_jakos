#!/usr/bin/env python3
"""
Test ExecuteTrajectory action directly to see if execution works at all
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import math


def rotation_vector_to_quaternion(rx, ry, rz):
    """Convert UR rotation vector to quaternion"""
    angle = math.sqrt(rx*rx + ry*ry + rz*rz)
    if angle < 1e-10:
        return [0.0, 0.0, 0.0, 1.0]
    axis = [rx/angle, ry/angle, rz/angle]
    half_angle = angle / 2.0
    sin_half = math.sin(half_angle)
    cos_half = math.cos(half_angle)
    return [axis[0]*sin_half, axis[1]*sin_half, axis[2]*sin_half, cos_half]


class TestExecuteTrajectory(Node):
    def __init__(self):
        super().__init__('test_execute_trajectory')

        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states', lambda msg: setattr(self, 'current_joint_state', msg), 10)

        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for cartesian service...')

        while not self.execute_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for execute action...')

        self.get_logger().info('Ready! Waiting 2 seconds for joint state...')

    def test_movement(self, x, y, z, rx, ry, rz):
        """Test a Cartesian movement"""
        import time
        time.sleep(2)  # Wait for joint state

        if self.current_joint_state is None:
            self.get_logger().error('No joint state!')
            return False

        self.get_logger().info(f'Testing movement to x={x}, y={y}, z={z}')

        # Convert to meters and quaternion
        x_m, y_m, z_m = x/1000.0, y/1000.0, z/1000.0
        quat = rotation_vector_to_quaternion(rx, ry, rz)

        # Create Cartesian path request
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.header.stamp = self.get_clock().now().to_msg()
        req.start_state.joint_state = self.current_joint_state
        req.group_name = 'ur_manipulator'
        req.link_name = 'gripper_tip'

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x_m, y_m, z_m
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        req.waypoints = [pose]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = False

        # Plan path
        self.get_logger().info('Computing path...')
        path_result = self.cartesian_client.call(req)

        self.get_logger().info(f'Path: {path_result.fraction*100:.1f}%')

        if path_result.fraction < 0.5:
            self.get_logger().error(f'Path only {path_result.fraction*100:.1f}% complete')
            return False

        # Create and send execute goal
        self.get_logger().info('Sending execute goal...')
        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = path_result.solution.joint_trajectory

        # Send goal and wait
        goal_handle_future = self.execute_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, goal_handle_future)

        goal_handle = goal_handle_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal REJECTED')
            return False

        self.get_logger().info('Goal ACCEPTED! Waiting for result...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)

        result = result_future.result()
        error_code = result.result.error_code.val

        self.get_logger().info(f'Result error code: {error_code}')

        if error_code == 1:
            self.get_logger().info('SUCCESS!')
            return True
        else:
            self.get_logger().error(f'FAILED with code {error_code}')
            return False


def main():
    rclpy.init()
    node = TestExecuteTrajectory()

    # Test movement to a position
    success = node.test_movement(-112.9, 426.6, 430.9, 2.986, 0.122, 0.028)

    node.get_logger().info(f'Test result: {"SUCCESS" if success else "FAILED"}')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
