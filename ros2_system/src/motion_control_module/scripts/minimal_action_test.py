#!/usr/bin/env python3
"""
Minimal Action Test - Direct ExecuteTrajectory call without service wrapper
Tests if action execution works when called from main thread without service callbacks
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import math
import time


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


class MinimalActionTest(Node):
    def __init__(self):
        super().__init__('minimal_action_test')

        # Use correct QoS for joint_states
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states', self.joint_state_callback, qos)

        self.get_logger().info('Waiting for joint states with correct QoS...')
        timeout = 10.0
        start = time.time()
        while self.current_joint_state is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_joint_state is None:
            self.get_logger().error('Failed to receive joint states!')
            return

        self.get_logger().info('Received joint states!')

        # Wait for services/actions
        self.get_logger().info('Waiting for cartesian path service...')
        self.cartesian_client.wait_for_service()

        self.get_logger().info('Waiting for execute trajectory action server...')
        self.execute_client.wait_for_server()

        self.get_logger().info('All services ready!')

    def joint_state_callback(self, msg):
        """Store latest joint state"""
        self.current_joint_state = msg

    def execute_movement(self, x_mm, y_mm, z_mm, rx, ry, rz):
        """Execute a Cartesian movement - called from main thread"""
        self.get_logger().info(f'Planning movement to: x={x_mm}, y={y_mm}, z={z_mm} mm')

        if self.current_joint_state is None:
            self.get_logger().error('No joint state available')
            return False

        # Convert to meters
        x, y, z = x_mm/1000.0, y_mm/1000.0, z_mm/1000.0
        quat = rotation_vector_to_quaternion(rx, ry, rz)

        # Create Cartesian path request
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.header.stamp = self.get_clock().now().to_msg()
        req.start_state.joint_state = self.current_joint_state
        req.group_name = 'ur_manipulator'
        req.link_name = 'gripper_tip'

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        req.waypoints = [pose]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = False

        # Plan path - using spin_until_future_complete from MAIN thread (not callback)
        self.get_logger().info('Computing Cartesian path...')
        path_future = self.cartesian_client.call_async(req)

        # This should work fine since we're NOT in a service callback
        rclpy.spin_until_future_complete(self, path_future, timeout_sec=30.0)

        if not path_future.done():
            self.get_logger().error('Path planning timeout')
            return False

        path_result = path_future.result()

        if path_result is None:
            self.get_logger().error('Path planning failed - no result')
            return False

        self.get_logger().info(f'Path computed: {path_result.fraction*100:.1f}% of trajectory')

        if path_result.fraction < 0.01:
            self.get_logger().error(f'Only {path_result.fraction*100:.1f}% of path could be computed')
            return False

        # Execute trajectory
        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = path_result.solution.joint_trajectory

        self.get_logger().info(f'Sending trajectory with {len(goal.trajectory.joint_trajectory.points)} points to /execute_trajectory action...')

        goal_future = self.execute_client.send_goal_async(goal)

        self.get_logger().info('Waiting for goal to be accepted...')
        rclpy.spin_until_future_complete(self, goal_future, timeout_sec=10.0)

        if not goal_future.done():
            self.get_logger().error('Goal send timeout')
            return False

        goal_handle = goal_future.result()

        if goal_handle is None:
            self.get_logger().error('Goal handle is None')
            return False

        if not goal_handle.accepted:
            self.get_logger().error('Goal was REJECTED by action server')
            return False

        self.get_logger().info('Goal ACCEPTED! Waiting for execution result...')

        # Wait for result
        result_future = goal_handle.get_result_async()

        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)

        if not result_future.done():
            self.get_logger().error('Execution timeout')
            return False

        result = result_future.result()

        if result is None:
            self.get_logger().error('Result is None')
            return False

        self.get_logger().info(f'Execution completed with error code: {result.result.error_code.val}')

        if result.result.error_code.val == 1:  # SUCCESS
            self.get_logger().info('✓ Movement completed successfully!')
            return True
        else:
            self.get_logger().error(f'✗ Execution failed with error code: {result.result.error_code.val}')
            return False


def main():
    rclpy.init()

    node = MinimalActionTest()

    if node.current_joint_state is None:
        node.get_logger().error('Cannot proceed without joint states')
        node.destroy_node()
        rclpy.shutdown()
        return

    # Test movement - home position
    node.get_logger().info('\n' + '='*60)
    node.get_logger().info('Testing movement to home position')
    node.get_logger().info('='*60)

    success = node.execute_movement(
        x_mm=-589.22,
        y_mm=-131.78,
        z_mm=371.73,
        rx=2.22,
        ry=2.22,
        rz=0.004
    )

    if success:
        node.get_logger().info('\n✓✓✓ TEST PASSED - Robot moved successfully! ✓✓✓\n')
    else:
        node.get_logger().error('\n✗✗✗ TEST FAILED - Robot did not move ✗✗✗\n')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
