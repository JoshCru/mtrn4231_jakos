#!/usr/bin/env python3
"""
Final Cartesian Controller - Minimal implementation
Uses direct service calls WITHOUT nested spinning
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from sort_interfaces.srv import MoveToCartesian
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import math
import threading


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


class FinalCartesianController(Node):
    def __init__(self):
        super().__init__('final_cartesian_controller')

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

        # Use separate thread for processing to avoid blocking executor
        self._executor_thread = None

    def handle_request(self, request, response):
        """Handle request in non-blocking way"""
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

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x, y, z
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        req.waypoints = [pose]
        req.max_step = 0.01
        req.jump_threshold = 0.0
        req.avoid_collisions = False

        # Blocking call - but this runs in service handler thread, not executor thread
        self.get_logger().info('Planning path...')
        path_future = self.cartesian_client.call_async(req)

        # Wait for path result - executor will process it in another thread
        import time
        timeout = 30.0
        start = time.time()
        while not path_future.done() and (time.time() - start) < timeout:
            time.sleep(0.01)

        if not path_future.done():
            response.success = False
            response.message = "Path planning timeout"
            return response

        path_result = path_future.result()

        if path_result.fraction < 0.01:
            response.success = False
            response.message = f"Only {path_result.fraction*100:.1f}% of path"
            return response

        self.get_logger().info(f'Path: {path_result.fraction*100:.1f}%')

        # Execute trajectory
        goal = ExecuteTrajectory.Goal()
        goal.trajectory.joint_trajectory = path_result.solution.joint_trajectory

        self.get_logger().info('Executing...')
        goal_future = self.execute_client.send_goal_async(goal)

        start = time.time()
        while not goal_future.done() and (time.time() - start) < 10.0:
            time.sleep(0.01)

        if not goal_future.done():
            response.success = False
            response.message = "Goal send timeout"
            return response

        goal_handle = goal_future.result()
        if not goal_handle or not goal_handle.accepted:
            response.success = False
            response.message = "Goal rejected"
            return response

        # Wait for execution result
        result_future = goal_handle.get_result_async()
        start = time.time()
        while not result_future.done() and (time.time() - start) < 120.0:
            time.sleep(0.01)

        if not result_future.done():
            response.success = False
            response.message = "Execution timeout"
            return response

        result = result_future.result()

        if result.result.error_code.val == 1:
            response.success = True
            response.message = "Done"
        else:
            response.success = False
            response.message = f"Failed: {result.result.error_code.val}"

        return response


def main():
    rclpy.init()
    node = FinalCartesianController()

    # Use MultiThreadedExecutor
    from rclpy.executors import MultiThreadedExecutor
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
