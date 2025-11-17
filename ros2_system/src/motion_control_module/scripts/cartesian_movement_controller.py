#!/usr/bin/env python3
"""
Cartesian Movement Controller Node
Controls UR5e robot using Cartesian coordinates with constrained wrist2 at -90 degrees
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup, ExecuteTrajectory
from moveit_msgs.srv import GetCartesianPath
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MotionPlanRequest,
    PlanningOptions,
    RobotState
)
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from trajectory_msgs.msg import JointTrajectory
from sort_interfaces.srv import MoveToCartesian
from tf_transformations import quaternion_from_euler, quaternion_about_axis
import numpy as np
import math


def rotation_vector_to_quaternion(rx, ry, rz):
    """
    Convert UR rotation vector (axis-angle) to quaternion.

    The rotation vector represents the axis of rotation multiplied by
    the angle of rotation in radians.

    Args:
        rx, ry, rz: Rotation vector components in radians

    Returns:
        Quaternion as [x, y, z, w]
    """
    # Calculate the angle (magnitude of rotation vector)
    angle = math.sqrt(rx*rx + ry*ry + rz*rz)

    if angle < 1e-10:
        # No rotation, return identity quaternion
        return [0.0, 0.0, 0.0, 1.0]

    # Normalize to get axis
    axis = [rx/angle, ry/angle, rz/angle]

    # Convert axis-angle to quaternion
    half_angle = angle / 2.0
    sin_half = math.sin(half_angle)
    cos_half = math.cos(half_angle)

    qx = axis[0] * sin_half
    qy = axis[1] * sin_half
    qz = axis[2] * sin_half
    qw = cos_half

    return [qx, qy, qz, qw]


class CartesianMovementController(Node):
    def __init__(self):
        super().__init__('cartesian_movement_controller')

        # Cartesian path service client
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )

        # ExecuteTrajectory action client
        self.execute_trajectory_client = ActionClient(
            self, ExecuteTrajectory, '/execute_trajectory'
        )

        # Service for Cartesian movement
        self.cartesian_service = self.create_service(
            MoveToCartesian,
            '/move_to_cartesian',
            self.move_to_cartesian_callback
        )

        # Subscribe to joint states
        self.current_joint_state = None
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        # Robot configuration
        self.group_name = "ur_manipulator"
        self.planning_frame = "base_link"
        self.end_effector_link = "tool0"

        # Joint names in MoveIt order
        self.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Wrist2 constraint value (-90 degrees in radians)
        self.wrist2_constraint = -math.pi / 2  # -90 degrees

        self.get_logger().info('Cartesian Movement Controller initialized')
        self.get_logger().info(f'Wrist2 constrained to: {math.degrees(self.wrist2_constraint):.2f} degrees')
        self.get_logger().info('Using Cartesian path planning (straight-line motion)')

        # Wait for services/actions
        self.get_logger().info('Waiting for /compute_cartesian_path service...')
        if not self.cartesian_path_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Cartesian path service not available')
        else:
            self.get_logger().info('Cartesian path service connected')

        self.get_logger().info('Waiting for /execute_trajectory action server...')
        if not self.execute_trajectory_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('ExecuteTrajectory action server not available')
        else:
            self.get_logger().info('ExecuteTrajectory action server connected')

    def joint_state_callback(self, msg):
        """Store current joint state"""
        self.current_joint_state = msg

    def move_to_cartesian_callback(self, request, response):
        """Handle Cartesian movement request"""
        self.get_logger().info(
            f'Received Cartesian request: x={request.x:.2f}, y={request.y:.2f}, '
            f'z={request.z:.2f} mm'
        )
        self.get_logger().info(
            f'Rotation vector: rx={request.rx:.4f}, ry={request.ry:.4f}, rz={request.rz:.4f} rad'
        )

        # Convert mm to meters
        x_m = request.x / 1000.0
        y_m = request.y / 1000.0
        z_m = request.z / 1000.0

        # Convert UR rotation vector to quaternion
        quat = rotation_vector_to_quaternion(request.rx, request.ry, request.rz)

        # Create target pose
        target_pose = PoseStamped()
        target_pose.header.frame_id = self.planning_frame
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.pose.position.x = x_m
        target_pose.pose.position.y = y_m
        target_pose.pose.position.z = z_m
        target_pose.pose.orientation.x = quat[0]
        target_pose.pose.orientation.y = quat[1]
        target_pose.pose.orientation.z = quat[2]
        target_pose.pose.orientation.w = quat[3]

        self.get_logger().info(
            f'Target pose (meters): x={x_m:.4f}, y={y_m:.4f}, z={z_m:.4f}'
        )
        self.get_logger().info(
            f'Quaternion: x={quat[0]:.4f}, y={quat[1]:.4f}, z={quat[2]:.4f}, w={quat[3]:.4f}'
        )

        # Plan and execute movement
        success, message, joint_positions = self.plan_and_execute(target_pose)

        response.success = success
        response.message = message
        response.joint_positions = joint_positions

        return response

    def plan_and_execute(self, target_pose):
        """Plan and execute Cartesian path to target pose with wrist2 constraint"""

        if self.current_joint_state is None:
            return False, "No joint state available", []

        # Create GetCartesianPath request
        request = GetCartesianPath.Request()
        request.header.frame_id = self.planning_frame
        request.header.stamp = self.get_clock().now().to_msg()

        # Set start state from current joint state
        request.start_state.joint_state = self.current_joint_state

        # Group name
        request.group_name = self.group_name
        request.link_name = self.end_effector_link

        # Add waypoints (just the target pose for straight-line motion)
        request.waypoints = [target_pose.pose]

        # Cartesian path parameters
        request.max_step = 0.01  # 1cm resolution for path interpolation
        request.jump_threshold = 0.0  # Disable jump detection (0.0 = no check)
        request.avoid_collisions = True

        # Add wrist2 path constraint
        path_constraints = Constraints()
        path_constraints.name = "wrist2_constraint"

        wrist2_constraint = JointConstraint()
        wrist2_constraint.joint_name = 'wrist_2_joint'
        wrist2_constraint.position = self.wrist2_constraint
        wrist2_constraint.tolerance_above = 0.1  # ~6 degrees tolerance
        wrist2_constraint.tolerance_below = 0.1
        wrist2_constraint.weight = 1.0
        path_constraints.joint_constraints.append(wrist2_constraint)

        request.path_constraints = path_constraints

        # Call Cartesian path service
        self.get_logger().info('Computing Cartesian path with wrist2 constraint...')
        future = self.cartesian_path_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() is None:
            return False, "Cartesian path service call failed", []

        response = future.result()

        # Check fraction of path achieved
        fraction = response.fraction
        self.get_logger().info(f'Cartesian path computed: {fraction * 100:.1f}% of path achieved')

        if fraction < 0.95:  # Require at least 95% of path
            return False, f"Could only compute {fraction * 100:.1f}% of Cartesian path", []

        if len(response.solution.joint_trajectory.points) == 0:
            return False, "No trajectory points generated", []

        # Apply velocity/acceleration scaling to trajectory
        scaled_trajectory = self.scale_trajectory(
            response.solution.joint_trajectory,
            velocity_scale=0.3,
            acceleration_scale=0.3
        )

        # Execute the trajectory
        self.get_logger().info(f'Executing Cartesian path with {len(scaled_trajectory.points)} waypoints...')

        execute_goal = ExecuteTrajectory.Goal()
        execute_goal.trajectory.joint_trajectory = scaled_trajectory

        send_goal_future = self.execute_trajectory_client.send_goal_async(execute_goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            return False, "Trajectory execution rejected", []

        self.get_logger().info('Trajectory accepted, executing...')

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=120.0)

        result = result_future.result()
        if result is None:
            return False, "No execution result received", []

        error_code = result.result.error_code.val

        if error_code == 1:  # SUCCESS
            self.get_logger().info('Cartesian path executed successfully!')

            # Extract final joint positions
            joint_positions = []
            if scaled_trajectory.points:
                final_point = scaled_trajectory.points[-1]
                joint_positions = list(final_point.positions)

                self.get_logger().info('Final joint positions (radians):')
                for i, name in enumerate(scaled_trajectory.joint_names):
                    if i < len(joint_positions):
                        self.get_logger().info(f'  {name}: {joint_positions[i]:.4f} ({math.degrees(joint_positions[i]):.2f} deg)')

            return True, "Cartesian path executed successfully", joint_positions
        else:
            error_messages = {
                -1: "PLANNING_FAILED",
                -4: "CONTROL_FAILED",
                -6: "TIMED_OUT",
                -7: "PREEMPTED",
            }
            error_msg = error_messages.get(error_code, f"EXECUTION_ERROR_{error_code}")
            self.get_logger().error(f'Execution failed: {error_msg}')
            return False, f"Execution failed: {error_msg}", []

    def scale_trajectory(self, trajectory, velocity_scale=0.3, acceleration_scale=0.3):
        """
        Scale trajectory timing for velocity and acceleration limits.
        """
        from copy import deepcopy
        from trajectory_msgs.msg import JointTrajectoryPoint
        from builtin_interfaces.msg import Duration

        scaled = deepcopy(trajectory)

        if len(scaled.points) < 2:
            return scaled

        # Simple time scaling - multiply all times by 1/velocity_scale
        time_scale = 1.0 / velocity_scale

        for i, point in enumerate(scaled.points):
            # Scale time from start
            original_secs = point.time_from_start.sec + point.time_from_start.nanosec * 1e-9
            new_secs = original_secs * time_scale

            scaled.points[i].time_from_start.sec = int(new_secs)
            scaled.points[i].time_from_start.nanosec = int((new_secs % 1) * 1e9)

            # Scale velocities
            if point.velocities:
                scaled.points[i].velocities = [v * velocity_scale for v in point.velocities]

            # Scale accelerations
            if point.accelerations:
                scaled.points[i].accelerations = [a * acceleration_scale for a in point.accelerations]

        return scaled


def main(args=None):
    rclpy.init(args=args)

    node = CartesianMovementController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
