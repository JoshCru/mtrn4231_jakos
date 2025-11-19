#!/usr/bin/env python3
"""
Simple Cartesian Controller using moveit_commander
Much simpler than the service-based approach
"""

import rclpy
from rclpy.node import Node
from sort_interfaces.srv import MoveToCartesian
import math
import sys

# Import moveit_commander
try:
    import moveit_commander
except ImportError:
    print("ERROR: moveit_commander not found. Install with:")
    print("  sudo apt install ros-humble-moveit-commander")
    sys.exit(1)


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


class SimpleCartesianController(Node):
    def __init__(self):
        super().__init__('simple_cartesian_controller')

        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Create MoveGroupCommander for the arm
        self.move_group = moveit_commander.MoveGroupCommander("ur_manipulator")
        self.move_group.set_end_effector_link("gripper_tip")
        self.move_group.set_max_velocity_scaling_factor(0.3)
        self.move_group.set_max_acceleration_scaling_factor(0.3)

        # Service
        self.service = self.create_service(
            MoveToCartesian,
            '/move_to_cartesian',
            self.handle_request
        )

        self.get_logger().info('Simple Cartesian Controller ready')
        self.get_logger().info(f'Planning frame: {self.move_group.get_planning_frame()}')
        self.get_logger().info(f'End effector: {self.move_group.get_end_effector_link()}')

    def handle_request(self, request, response):
        """Handle Cartesian movement request"""
        self.get_logger().info(f'Moving to: x={request.x}, y={request.y}, z={request.z} mm')

        # Convert to meters
        x = request.x / 1000.0
        y = request.y / 1000.0
        z = request.z / 1000.0

        # Convert rotation vector to quaternion
        quat = rotation_vector_to_quaternion(request.rx, request.ry, request.rz)

        # Create pose
        pose_goal = self.move_group.get_current_pose().pose
        pose_goal.position.x = x
        pose_goal.position.y = y
        pose_goal.position.z = z
        pose_goal.orientation.x = quat[0]
        pose_goal.orientation.y = quat[1]
        pose_goal.orientation.z = quat[2]
        pose_goal.orientation.w = quat[3]

        # Plan Cartesian path
        waypoints = [pose_goal]
        (plan, fraction) = self.move_group.compute_cartesian_path(
            waypoints,
            0.01,  # 1cm step
            0.0    # jump threshold
        )

        self.get_logger().info(f'Cartesian path: {fraction*100:.1f}% computed')

        if fraction < 0.8:
            response.success = False
            response.message = f"Could only plan {fraction*100:.1f}% of path"
            return response

        # Execute
        self.get_logger().info('Executing trajectory...')
        success = self.move_group.execute(plan, wait=True)

        if success:
            self.get_logger().info('Movement completed!')
            response.success = True
            response.message = "Success"
        else:
            self.get_logger().error('Execution failed')
            response.success = False
            response.message = "Execution failed"

        return response


def main(args=None):
    rclpy.init(args=args)
    node = SimpleCartesianController()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        moveit_commander.roscpp_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
