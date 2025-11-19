#!/usr/bin/env python3
"""
Action-based Cartesian Controller - Uses action interface instead of service
Fully async implementation without spin_until_future_complete

ALL COORDINATES ARE WITH RESPECT TO BASE_LINK FRAME (robot base)
The controller accepts positions in mm and UR rotation vectors in radians
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, ActionClient
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.action import ExecuteTrajectory
from sort_interfaces.action import MoveToCartesian
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


class ActionCartesianController(Node):
    def __init__(self):
        super().__init__('action_cartesian_controller')

        # Use correct QoS for joint_states
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        # Clients for MoveIt services/actions
        self.cartesian_client = self.create_client(GetCartesianPath, '/compute_cartesian_path')
        self.execute_client = ActionClient(self, ExecuteTrajectory, '/execute_trajectory')

        # Subscribe to joint states
        self.current_joint_state = None
        self.create_subscription(JointState, '/joint_states', lambda msg: setattr(self, 'current_joint_state', msg), qos)

        # Wait for joint states first
        self.get_logger().info('Waiting for joint states...')
        import time
        timeout = 10.0
        start = time.time()
        while self.current_joint_state is None and (time.time() - start) < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)

        if self.current_joint_state is None:
            self.get_logger().warn('No joint states received yet - controller will wait for them before executing')
        else:
            self.get_logger().info('Joint states received!')

        # Wait for services
        self.get_logger().info('Waiting for cartesian path service...')
        self.cartesian_client.wait_for_service()

        self.get_logger().info('Waiting for execute trajectory action server...')
        self.execute_client.wait_for_server()

        # Create action server with execute_callback
        self._action_server = ActionServer(
            self,
            MoveToCartesian,
            '/move_to_cartesian_action',
            execute_callback=self.execute_callback,
            callback_group=rclpy.callback_groups.ReentrantCallbackGroup()
        )

        self.get_logger().info('Action Cartesian Controller ready!')
        self.get_logger().info('Action server: /move_to_cartesian_action')

    async def execute_callback(self, goal_handle):
        """Execute callback for action - fully async, no blocking calls"""
        self.get_logger().info(f'Received goal: x={goal_handle.request.x}, y={goal_handle.request.y}, z={goal_handle.request.z}')

        # Create feedback message
        feedback_msg = MoveToCartesian.Feedback()
        result = MoveToCartesian.Result()

        try:
            feedback_msg.status = 'Checking joint states...'
            feedback_msg.progress = 0.0
            goal_handle.publish_feedback(feedback_msg)

            if self.current_joint_state is None:
                result.success = False
                result.message = "No joint state available"
                result.joint_positions = []
                goal_handle.abort()
                return result

            # Convert to meters
            x = goal_handle.request.x / 1000.0
            y = goal_handle.request.y / 1000.0
            z = goal_handle.request.z / 1000.0
            quat = rotation_vector_to_quaternion(goal_handle.request.rx, goal_handle.request.ry, goal_handle.request.rz)

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

            # Plan path
            feedback_msg.status = 'Planning Cartesian path...'
            feedback_msg.progress = 0.2
            goal_handle.publish_feedback(feedback_msg)

            self.get_logger().info('Computing Cartesian path...')
            path_future = self.cartesian_client.call_async(req)

            # Await the future instead of spinning
            path_result = await path_future

            if path_result is None or path_result.fraction < 0.01:
                result.success = False
                result.message = f"Only {path_result.fraction*100:.1f}% of path could be computed" if path_result else "Path planning failed"
                result.joint_positions = []
                goal_handle.abort()
                return result

            self.get_logger().info(f'Path computed: {path_result.fraction*100:.1f}% of trajectory')

            # Execute trajectory
            feedback_msg.status = f'Executing trajectory ({path_result.fraction*100:.1f}% planned)...'
            feedback_msg.progress = 0.5
            goal_handle.publish_feedback(feedback_msg)

            exec_goal = ExecuteTrajectory.Goal()
            exec_goal.trajectory.joint_trajectory = path_result.solution.joint_trajectory

            self.get_logger().info(f'Sending trajectory with {len(exec_goal.trajectory.joint_trajectory.points)} points...')

            # Send goal and await
            goal_future = self.execute_client.send_goal_async(exec_goal)
            exec_goal_handle = await goal_future

            if not exec_goal_handle or not exec_goal_handle.accepted:
                result.success = False
                result.message = "Goal rejected by trajectory executor"
                result.joint_positions = []
                goal_handle.abort()
                return result

            self.get_logger().info('Goal accepted, executing...')

            feedback_msg.status = 'Executing movement...'
            feedback_msg.progress = 0.7
            goal_handle.publish_feedback(feedback_msg)

            # Wait for execution result
            result_future = exec_goal_handle.get_result_async()
            exec_result = await result_future

            # Create final result
            if exec_result and exec_result.result.error_code.val == 1:  # SUCCESS
                self.get_logger().info('✓ Movement completed successfully!')
                result.success = True
                result.message = "Movement completed"
                result.joint_positions = list(self.current_joint_state.position) if self.current_joint_state else []

                feedback_msg.status = 'Completed'
                feedback_msg.progress = 1.0
                goal_handle.publish_feedback(feedback_msg)

                goal_handle.succeed()
            else:
                error_code = exec_result.result.error_code.val if exec_result else -1
                self.get_logger().error(f'Execution failed with error code: {error_code}')
                result.success = False
                result.message = f"Execution failed: error code {error_code}"
                result.joint_positions = []
                goal_handle.abort()

            return result

        except Exception as e:
            self.get_logger().error(f'Exception in execute_callback: {e}')
            result.success = False
            result.message = f"Exception: {str(e)}"
            result.joint_positions = []
            try:
                goal_handle.abort()
            except:
                pass
            return result


def main():
    rclpy.init()
    node = ActionCartesianController()

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
