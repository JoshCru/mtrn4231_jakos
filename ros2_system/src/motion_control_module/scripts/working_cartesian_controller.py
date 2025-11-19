#!/usr/bin/env python3
"""
Working Cartesian Controller - uses callbacks instead of spin_until_future_complete
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from sort_interfaces.srv import MoveToCartesian
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
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


class WorkingCartesianController(Node):
    def __init__(self):
        super().__init__('working_cartesian_controller')

        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        self.service = self.create_service(MoveToCartesian, '/move_to_cartesian', self.handle_request)

        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states', lambda msg: setattr(self, 'current_joint_state', msg), 10)

        # Wait for services
        while not self.cartesian_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for cartesian path service...')

        while not self.execute_client.wait_for_server(timeout_sec=1.0):
            self.get_logger().info('Waiting for execute trajectory server...')

        self.get_logger().info('Controller ready!')

    def handle_request(self, request, response):
        """Handle sync - use callbacks to avoid spinning deadlock"""
        self.get_logger().info(f'Request: x={request.x}, y={request.y}, z={request.z}')

        if self.current_joint_state is None:
            response.success = False
            response.message = "No joint state"
            return response

        # Convert to pose
        x, y, z = request.x/1000.0, request.y/1000.0, request.z/1000.0
        quat = rotation_vector_to_quaternion(request.rx, request.ry, request.rz)

        # Create request
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.header.stamp = self.get_clock().now().to_msg()
        req.start_state.joint_state = self.current_joint_state
        req.group_name = 'ur_manipulator'
        req.link_name = 'gripper_tip'

        pose = PoseStamped().pose
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        req.waypoints = [pose]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = False

        # Store response to fill in from callbacks
        self._pending_response = response
        self._service_done = False

        # Call async and wait with callback
        self.get_logger().info('Planning path...')
        future = self.cartesian_client.call_async(req)
        future.add_done_callback(self._path_callback)

        # Wait for callback to finish
        import time
        timeout = 30.0
        start = time.time()
        while not self._service_done and (time.time() - start) < timeout:
            time.sleep(0.01)

        if not self._service_done:
            response.success = False
            response.message = "Timeout waiting for path planning"
            return response

        return self._pending_response

    def _path_callback(self, future):
        """Callback when path planning is done"""
        try:
            result = future.result()

            if result.fraction < 0.01:
                self._pending_response.success = False
                self._pending_response.message = f"Only {result.fraction*100:.1f}% of path"
                self._service_done = True
                return

            self.get_logger().info(f'Path: {result.fraction*100:.1f}%')

            # Execute trajectory
            goal = ExecuteTrajectory.Goal()
            goal.trajectory.joint_trajectory = result.solution.joint_trajectory

            self.get_logger().info('Executing...')
            self._exec_done = False
            exec_future = self.execute_client.send_goal_async(goal)
            exec_future.add_done_callback(self._exec_goal_callback)

        except Exception as e:
            self.get_logger().error(f'Path planning failed: {e}')
            self._pending_response.success = False
            self._pending_response.message = f"Planning failed: {e}"
            self._service_done = True

    def _exec_goal_callback(self, future):
        """Callback when goal is accepted"""
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            self._pending_response.success = False
            self._pending_response.message = "Goal rejected"
            self._service_done = True
            return

        self.get_logger().info('Goal accepted, waiting for result...')
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self._exec_result_callback)

    def _exec_result_callback(self, future):
        """Callback when execution is done"""
        result = future.result()
        self.get_logger().info(f'Execution complete: {result.result.error_code.val}')

        if result.result.error_code.val == 1:  # SUCCESS
            self._pending_response.success = True
            self._pending_response.message = "Done"
        else:
            self._pending_response.success = False
            self._pending_response.message = f"Execution failed: {result.result.error_code.val}"

        self._service_done = True


def main():
    rclpy.init()
    node = WorkingCartesianController()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
