#!/usr/bin/env python3
"""
Square Box Movement Script for UR5e
Moves the end effector in a square pattern while maintaining constant z-axis
and restricting joints to avoid singularities.

Joint Numbering Convention:
  Joint 6 = shoulder_pan_joint (base rotation)
  Joint 1 = shoulder_lift_joint
  Joint 2 = elbow_joint
  Joint 3 = wrist_1_joint
  Joint 4 = wrist_2_joint
  Joint 5 = wrist_3_joint

Joint order in arrays: [Joint6, Joint1, Joint2, Joint3, Joint4, Joint5]
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
from moveit_msgs.msg import RobotState, Constraints, JointConstraint
from moveit_msgs.srv import GetPositionIK
import sys
import time

try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState as MoveItRobotState
except ImportError:
    print("MoveItPy not available, trying alternative imports...")
    try:
        from moveit_msgs.action import MoveGroup
        from rclpy.action import ActionClient
    except ImportError:
        print("Error: Could not import required MoveIt libraries")
        sys.exit(1)


class SquareMovementNode(Node):
    def __init__(self):
        super().__init__('square_movement_node')

        # Movement parameters
        self.square_size = 0.15  # 15cm square
        self.z_height = 0.3  # Maintain constant z-height (30cm above base)
        self.move_speed = 0.1  # Velocity scaling factor

        # Starting position (center of square)
        self.center_x = 0.3  # 30cm in front of robot
        self.center_y = 0.0  # Centered

        # Joint limits to avoid singularities (in radians)
        # Restricting shoulder_lift and elbow to safe ranges
        # Joint mapping: Joint6 (base), Joint1, Joint2, Joint3, Joint4, Joint5
        self.joint_constraints = {
            'shoulder_pan_joint': (-3.14, 3.14),     # Joint 6 (base) - Full range
            'shoulder_lift_joint': (-2.0, -0.5),     # Joint 1 - Restricted to avoid upward pointing
            'elbow_joint': (-2.5, -0.5),             # Joint 2 - Restricted elbow range
            'wrist_1_joint': (-3.14, 3.14),          # Joint 3 - Full range
            'wrist_2_joint': (-3.14, 3.14),          # Joint 4 - Full range
            'wrist_3_joint': (-3.14, 3.14),          # Joint 5 - Full range
        }

        self.get_logger().info("Square Movement Node initialized")
        self.get_logger().info(f"Square size: {self.square_size}m, Z-height: {self.z_height}m")

    def generate_square_waypoints(self):
        """Generate waypoints for a square pattern"""
        half_size = self.square_size / 2.0

        # Define corners of the square (counter-clockwise from starting position)
        waypoints = [
            # Start at bottom-right corner
            (self.center_x + half_size, self.center_y - half_size, self.z_height),
            # Move to bottom-left
            (self.center_x + half_size, self.center_y + half_size, self.z_height),
            # Move to top-left
            (self.center_x - half_size, self.center_y + half_size, self.z_height),
            # Move to top-right
            (self.center_x - half_size, self.center_y - half_size, self.z_height),
            # Return to start
            (self.center_x + half_size, self.center_y - half_size, self.z_height),
        ]

        return waypoints

    def create_pose(self, x, y, z):
        """Create a Pose message with fixed orientation (pointing down)"""
        pose = Pose()
        pose.position.x = x
        pose.position.y = y
        pose.position.z = z

        # Orientation: end effector pointing down (standard for pick/place)
        # This is a rotation of 180 degrees around Y-axis
        pose.orientation.x = 1.0
        pose.orientation.y = 0.0
        pose.orientation.z = 0.0
        pose.orientation.w = 0.0

        return pose

    def run_square_movement(self):
        """Execute the square movement pattern"""
        self.get_logger().info("Starting square movement pattern...")

        # Generate waypoints
        waypoints = self.generate_square_waypoints()

        try:
            # Initialize MoveItPy
            self.get_logger().info("Initializing MoveIt...")
            moveit = MoveItPy(node_name="moveit_py_square")

            # Get the manipulator planning component
            ur_manipulator = moveit.get_planning_component("ur_manipulator")
            planning_scene_monitor = moveit.get_planning_scene_monitor()

            self.get_logger().info("MoveIt initialized successfully")

            # Move through each waypoint
            for i, (x, y, z) in enumerate(waypoints):
                self.get_logger().info(f"Moving to waypoint {i+1}/{len(waypoints)}: ({x:.3f}, {y:.3f}, {z:.3f})")

                # Set the goal pose
                pose_goal = self.create_pose(x, y, z)
                ur_manipulator.set_goal_state(pose_stamped_msg=PoseStamped(pose=pose_goal),
                                             pose_link="tool0")

                # Plan and execute
                plan_result = ur_manipulator.plan()

                if plan_result:
                    self.get_logger().info(f"Planning successful, executing...")
                    robot_trajectory = plan_result.trajectory
                    moveit.execute(robot_trajectory, controllers=[])
                    time.sleep(1.0)  # Brief pause at each corner
                else:
                    self.get_logger().warn(f"Failed to plan to waypoint {i+1}")

            self.get_logger().info("Square movement completed!")

        except Exception as e:
            self.get_logger().error(f"Error during movement: {str(e)}")
            import traceback
            self.get_logger().error(traceback.format_exc())
            return False

        return True


def main(args=None):
    rclpy.init(args=args)

    node = SquareMovementNode()

    # Give some time for connections to establish
    time.sleep(2.0)

    try:
        # Run the square movement
        success = node.run_square_movement()

        if success:
            node.get_logger().info("Mission accomplished!")
        else:
            node.get_logger().error("Mission failed!")

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
