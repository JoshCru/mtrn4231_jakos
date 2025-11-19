#!/usr/bin/env python3
"""
Move to Home Position using Joint-Space Planning
Uses MoveIt's standard planning (not Cartesian) which is more flexible for large movements
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveItErrorCodes
from moveit_msgs.action import MoveGroup
from rclpy.action import ActionClient
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


class MoveToHome(Node):
    def __init__(self):
        super().__init__('move_to_home')

        self.move_group_client = ActionClient(self, MoveGroup, '/move_action')

        self.get_logger().info('Waiting for move_action server...')
        self.move_group_client.wait_for_server()
        self.get_logger().info('Connected to move_action server')

    def move_to_pose(self, x_mm, y_mm, z_mm, rx, ry, rz):
        """Move to pose using joint-space planning (not Cartesian)"""

        # Convert to meters
        x = x_mm / 1000.0
        y = y_mm / 1000.0
        z = z_mm / 1000.0

        quat = rotation_vector_to_quaternion(rx, ry, rz)

        # Create pose goal
        pose = PoseStamped()
        pose.header.frame_id = 'base_link'
        pose.header.stamp = self.get_clock().now().to_msg()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.position.z = z
        pose.pose.orientation.x = quat[0]
        pose.pose.orientation.y = quat[1]
        pose.pose.orientation.z = quat[2]
        pose.pose.orientation.w = quat[3]

        # Create MoveGroup goal
        goal = MoveGroup.Goal()
        goal.request.group_name = 'ur_manipulator'
        goal.request.num_planning_attempts = 10
        goal.request.allowed_planning_time = 5.0
        goal.request.max_velocity_scaling_factor = 0.3
        goal.request.max_acceleration_scaling_factor = 0.3

        # Set pose target
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive

        constraints = Constraints()

        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header = pose.header
        pos_constraint.link_name = 'gripper_tip'

        # Create a small box around the target
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = [0.001, 0.001, 0.001]  # 1mm tolerance

        pos_constraint.constraint_region.primitives.append(box)
        pos_constraint.constraint_region.primitive_poses.append(pose.pose)
        pos_constraint.weight = 1.0

        constraints.position_constraints.append(pos_constraint)

        # Orientation constraint
        orient_constraint = OrientationConstraint()
        orient_constraint.header = pose.header
        orient_constraint.link_name = 'gripper_tip'
        orient_constraint.orientation = pose.pose.orientation
        orient_constraint.absolute_x_axis_tolerance = 0.01
        orient_constraint.absolute_y_axis_tolerance = 0.01
        orient_constraint.absolute_z_axis_tolerance = 0.01
        orient_constraint.weight = 1.0

        constraints.orientation_constraints.append(orient_constraint)

        goal.request.goal_constraints.append(constraints)

        self.get_logger().info(f'Planning movement to: ({x*1000:.1f}, {y*1000:.1f}, {z*1000:.1f}) mm')
        self.get_logger().info('Using joint-space planning (not Cartesian path)')

        # Send goal
        send_goal_future = self.move_group_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()

        if not goal_handle.accepted:
            self.get_logger().error('Goal rejected')
            return False

        self.get_logger().info('Goal accepted, executing...')

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)

        result = result_future.result()

        if result.result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info('✓ Movement completed successfully!')
            return True
        else:
            self.get_logger().error(f'✗ Movement failed with error code: {result.result.error_code.val}')
            return False


def main():
    rclpy.init()

    node = MoveToHome()

    # Home position from user
    home_x = -589.22
    home_y = -131.78
    home_z = 371.73
    home_rx = 2.22
    home_ry = 2.22
    home_rz = 0.004

    print(f'\nMoving to HOME position using joint-space planning')
    print(f'Target: X={home_x}mm, Y={home_y}mm, Z={home_z}mm')
    print(f'Orientation: RX={home_rx}, RY={home_ry}, RZ={home_rz} rad\n')

    success = node.move_to_pose(home_x, home_y, home_z, home_rx, home_ry, home_rz)

    if success:
        print('\n✓✓✓ Successfully moved to home position! ✓✓✓\n')
    else:
        print('\n✗✗✗ Failed to move to home position ✗✗✗\n')

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
