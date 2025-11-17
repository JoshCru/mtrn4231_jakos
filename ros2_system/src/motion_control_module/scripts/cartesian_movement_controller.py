#!/usr/bin/env python3
"""
Cartesian Movement Controller Node
Controls UR5e robot using Cartesian coordinates with constrained wrist2 at -90 degrees
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup
from moveit_msgs.msg import (
    Constraints,
    JointConstraint,
    PositionConstraint,
    OrientationConstraint,
    BoundingVolume,
    MotionPlanRequest,
    PlanningOptions
)
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from shape_msgs.msg import SolidPrimitive
from sensor_msgs.msg import JointState
from sort_interfaces.srv import MoveToCartesian
from tf_transformations import quaternion_from_euler
import numpy as np
import math


class CartesianMovementController(Node):
    def __init__(self):
        super().__init__('cartesian_movement_controller')

        # MoveGroup action client
        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')

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

        # Wait for action server
        self.get_logger().info('Waiting for MoveGroup action server...')
        if not self.move_group_client.wait_for_server(timeout_sec=10.0):
            self.get_logger().error('MoveGroup action server not available')
        else:
            self.get_logger().info('MoveGroup action server connected')

    def joint_state_callback(self, msg):
        """Store current joint state"""
        self.current_joint_state = msg

    def move_to_cartesian_callback(self, request, response):
        """Handle Cartesian movement request"""
        self.get_logger().info(
            f'Received Cartesian request: x={request.x:.2f}, y={request.y:.2f}, '
            f'z={request.z:.2f} mm'
        )

        # Convert mm to meters
        x_m = request.x / 1000.0
        y_m = request.y / 1000.0
        z_m = request.z / 1000.0

        # Convert orientation degrees to radians
        roll_rad = math.radians(request.roll)
        pitch_rad = math.radians(request.pitch)
        yaw_rad = math.radians(request.yaw)

        # Create quaternion from RPY
        quat = quaternion_from_euler(roll_rad, pitch_rad, yaw_rad)

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
            f'Orientation (deg): roll={request.roll:.2f}, pitch={request.pitch:.2f}, yaw={request.yaw:.2f}'
        )

        # Plan and execute movement
        success, message, joint_positions = self.plan_and_execute(target_pose)

        response.success = success
        response.message = message
        response.joint_positions = joint_positions

        return response

    def plan_and_execute(self, target_pose):
        """Plan and execute movement to target pose with wrist2 constraint"""

        if self.current_joint_state is None:
            return False, "No joint state available", []

        # Create motion plan request
        motion_plan_request = MotionPlanRequest()
        motion_plan_request.group_name = self.group_name
        motion_plan_request.num_planning_attempts = 10
        motion_plan_request.allowed_planning_time = 5.0
        motion_plan_request.max_velocity_scaling_factor = 0.3
        motion_plan_request.max_acceleration_scaling_factor = 0.3

        # Set start state from current joint state
        motion_plan_request.start_state.joint_state = self.current_joint_state

        # Create pose goal constraint
        goal_constraints = Constraints()
        goal_constraints.name = "cartesian_goal"

        # Position constraint
        position_constraint = PositionConstraint()
        position_constraint.header.frame_id = self.planning_frame
        position_constraint.link_name = self.end_effector_link
        position_constraint.target_point_offset.x = 0.0
        position_constraint.target_point_offset.y = 0.0
        position_constraint.target_point_offset.z = 0.0

        # Define tolerance region (small box around target)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.001]  # 1mm tolerance

        bounding_volume = BoundingVolume()
        bounding_volume.primitives.append(primitive)
        bounding_volume.primitive_poses.append(target_pose.pose)

        position_constraint.constraint_region = bounding_volume
        position_constraint.weight = 1.0
        goal_constraints.position_constraints.append(position_constraint)

        # Orientation constraint
        orientation_constraint = OrientationConstraint()
        orientation_constraint.header.frame_id = self.planning_frame
        orientation_constraint.link_name = self.end_effector_link
        orientation_constraint.orientation = target_pose.pose.orientation
        orientation_constraint.absolute_x_axis_tolerance = 0.01
        orientation_constraint.absolute_y_axis_tolerance = 0.01
        orientation_constraint.absolute_z_axis_tolerance = 0.01
        orientation_constraint.weight = 1.0
        goal_constraints.orientation_constraints.append(orientation_constraint)

        motion_plan_request.goal_constraints.append(goal_constraints)

        # Add wrist2 path constraint to keep it at -90 degrees during motion
        path_constraints = Constraints()
        path_constraints.name = "wrist2_constraint"

        wrist2_constraint = JointConstraint()
        wrist2_constraint.joint_name = 'wrist_2_joint'
        wrist2_constraint.position = self.wrist2_constraint
        wrist2_constraint.tolerance_above = 0.05  # ~3 degrees tolerance
        wrist2_constraint.tolerance_below = 0.05
        wrist2_constraint.weight = 1.0
        path_constraints.joint_constraints.append(wrist2_constraint)

        motion_plan_request.path_constraints = path_constraints

        # Create MoveGroup goal
        move_group_goal = MoveGroup.Goal()
        move_group_goal.request = motion_plan_request

        planning_options = PlanningOptions()
        planning_options.plan_only = False  # Plan and execute
        planning_options.look_around = False
        planning_options.replan = True
        planning_options.replan_attempts = 3
        move_group_goal.planning_options = planning_options

        # Send goal
        self.get_logger().info('Sending goal to MoveGroup with wrist2 constraint...')
        send_goal_future = self.move_group_client.send_goal_async(move_group_goal)
        rclpy.spin_until_future_complete(self, send_goal_future, timeout_sec=10.0)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected by MoveGroup')
            return False, "Goal rejected by MoveGroup", []

        self.get_logger().info('Goal accepted, waiting for result...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=60.0)

        result = result_future.result()

        if result is None:
            return False, "No result received", []

        move_group_result = result.result

        # Check error code
        error_code = move_group_result.error_code.val

        if error_code == 1:  # SUCCESS
            self.get_logger().info('Motion executed successfully!')

            # Extract final joint positions from planned trajectory
            joint_positions = []
            if move_group_result.planned_trajectory.joint_trajectory.points:
                final_point = move_group_result.planned_trajectory.joint_trajectory.points[-1]
                joint_positions = list(final_point.positions)

                self.get_logger().info('Final joint positions (radians):')
                for i, name in enumerate(move_group_result.planned_trajectory.joint_trajectory.joint_names):
                    if i < len(joint_positions):
                        self.get_logger().info(f'  {name}: {joint_positions[i]:.4f} ({math.degrees(joint_positions[i]):.2f} deg)')

            return True, "Motion executed successfully", joint_positions
        else:
            error_messages = {
                -1: "PLANNING_FAILED",
                -2: "INVALID_MOTION_PLAN",
                -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
                -4: "CONTROL_FAILED",
                -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
                -6: "TIMED_OUT",
                -7: "PREEMPTED",
                -10: "START_STATE_IN_COLLISION",
                -11: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
                -12: "GOAL_IN_COLLISION",
                -13: "GOAL_VIOLATES_PATH_CONSTRAINTS",
                -14: "GOAL_CONSTRAINTS_VIOLATED",
                -15: "INVALID_GROUP_NAME",
                -16: "INVALID_GOAL_CONSTRAINTS",
                -17: "INVALID_ROBOT_STATE",
                -18: "INVALID_LINK_NAME",
                -19: "INVALID_OBJECT_NAME",
                -21: "FRAME_TRANSFORM_FAILURE",
                -22: "COLLISION_CHECKING_UNAVAILABLE",
                -23: "ROBOT_STATE_STALE",
                -24: "SENSOR_INFO_STALE",
                -31: "NO_IK_SOLUTION",
            }
            error_msg = error_messages.get(error_code, f"UNKNOWN_ERROR_{error_code}")
            self.get_logger().error(f'Motion failed: {error_msg}')
            return False, f"Motion failed: {error_msg}", []


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
