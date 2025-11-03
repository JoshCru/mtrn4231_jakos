#!/usr/bin/env python3
"""
Simple Square Box Movement Script for UR5e using MoveIt
Moves the end effector in a square pattern while maintaining constant z-axis.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
import sys
import math

try:
    from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory, MoveItErrorCodes, Constraints, JointConstraint
    from moveit_msgs.srv import GetCartesianPath
    from moveit_msgs.action import MoveGroup
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    from rclpy.action import ActionClient
    IMPORTS_OK = True
except ImportError as e:
    print(f"Import error: {e}")
    IMPORTS_OK = False


class SimpleSquareMovement(Node):
    def __init__(self):
        super().__init__('simple_square_movement')

        # Movement parameters
        self.square_size = 0.15  # 15cm square
        self.z_height = 0.3  # Maintain constant z-height (30cm above base)

        # Starting position (center of square) - X positive, Y negative quadrant
        self.center_x = 0.3  # 30cm in front of robot (positive X)
        self.center_y = -0.15  # Negative Y quadrant

        # Create MoveIt action client
        self.move_group_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.current_joint_state = None
        self.planning_group = "ur_manipulator"

        self.get_logger().info("Simple Square Movement Node initialized with MoveIt")
        self.get_logger().info(f"Square size: {self.square_size}m, Z-height: {self.z_height}m")
        self.get_logger().info(f"Planning group: {self.planning_group}")

    def joint_state_callback(self, msg):
        """Store current joint states"""
        self.current_joint_state = msg

    def generate_square_waypoints(self):
        """Generate waypoints for a square pattern"""
        half_size = self.square_size / 2.0

        waypoints = []

        # Define corners of the square
        corners = [
            (self.center_x + half_size, self.center_y - half_size, self.z_height),
            (self.center_x + half_size, self.center_y + half_size, self.z_height),
            (self.center_x - half_size, self.center_y + half_size, self.z_height),
            (self.center_x - half_size, self.center_y - half_size, self.z_height),
            (self.center_x + half_size, self.center_y - half_size, self.z_height),
        ]

        for x, y, z in corners:
            pose = Pose()
            pose.position.x = x
            pose.position.y = y
            pose.position.z = z

            # End effector pointing down
            pose.orientation.x = 1.0
            pose.orientation.y = 0.0
            pose.orientation.z = 0.0
            pose.orientation.w = 0.0

            waypoints.append(pose)

        return waypoints

    def move_to_joint_positions(self, joint_positions, duration_sec=5.0):
        """Plan and execute movement to joint positions using MoveIt"""
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveIt action server not available!")
            return False

        # Create MoveGroup goal
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = self.planning_group
        goal_msg.request.num_planning_attempts = 10
        goal_msg.request.allowed_planning_time = 10.0
        goal_msg.request.max_velocity_scaling_factor = 0.2
        goal_msg.request.max_acceleration_scaling_factor = 0.2

        # Set joint constraints (target positions)
        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        constraints = Constraints()
        for i, (name, position) in enumerate(zip(joint_names, joint_positions)):
            joint_constraint = JointConstraint()
            joint_constraint.joint_name = name
            joint_constraint.position = position
            joint_constraint.tolerance_above = 0.1
            joint_constraint.tolerance_below = 0.1
            joint_constraint.weight = 1.0
            constraints.joint_constraints.append(joint_constraint)

        goal_msg.request.goal_constraints.append(constraints)
        goal_msg.planning_options.plan_only = False  # Plan and execute

        # Send goal
        self.get_logger().info(f"Planning and executing with MoveIt: {[f'{p:.3f}' for p in joint_positions]}")
        send_goal_future = self.move_group_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("MoveIt goal rejected!")
            return False

        self.get_logger().info("MoveIt goal accepted, executing...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=30.0)

        if result_future.result() is None:
            self.get_logger().error("No result received from MoveIt (timeout?)")
            return False

        result = result_future.result().result
        self.get_logger().info(f"MoveIt result error code: {result.error_code.val}")

        # Map common error codes
        error_names = {
            1: "SUCCESS",
            99999: "FAILURE",
            -1: "PLANNING_FAILED",
            -2: "INVALID_MOTION_PLAN",
            -3: "MOTION_PLAN_INVALIDATED_BY_ENVIRONMENT_CHANGE",
            -4: "CONTROL_FAILED",
            -5: "UNABLE_TO_AQUIRE_SENSOR_DATA",
            -7: "TIMED_OUT",
            -10: "PREEMPTED",
            -11: "START_STATE_IN_COLLISION",
            -12: "START_STATE_VIOLATES_PATH_CONSTRAINTS",
            -21: "INVALID_GROUP_NAME",
        }
        error_name = error_names.get(result.error_code.val, f"UNKNOWN({result.error_code.val})")

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info("Movement successful!")
            return True
        else:
            self.get_logger().error(f"Movement failed: {error_name}")
            return False

    def run_simple_square(self):
        """Execute a simple square movement using predefined joint positions"""
        self.get_logger().info("Starting simple square movement...")

        # Print current joint state for debugging
        if self.current_joint_state:
            self.get_logger().info("Current joint positions:")
            for name, pos in zip(self.current_joint_state.name, self.current_joint_state.position):
                self.get_logger().info(f"  {name}: {pos:.6f}")

        # Safe joint configurations that form a square pattern (approximately)
        # These are hand-tuned to avoid singularities
        # Format: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        # Skip the first "home" position since we're already there

        positions = [
            # Corner 1 (move in X+ Y- quadrant)
            [0.15, -1.25, 1.60, -1.90, -1.57, 0.0],

            # Corner 2
            [-0.15, -1.25, 1.60, -1.90, -1.57, 0.0],

            # Corner 3
            [-0.15, -1.45, 1.50, -1.60, -1.57, 0.0],

            # Corner 4
            [0.15, -1.45, 1.50, -1.60, -1.57, 0.0],

            # Return to Corner 1
            [0.15, -1.25, 1.60, -1.90, -1.57, 0.0],

            # Return home (safe position)
            [0.0, -1.57, 1.57, -1.57, -1.57, 0.0],
        ]

        self.get_logger().info(f"Moving through {len(positions)} positions...")

        for i, joint_pos in enumerate(positions):
            self.get_logger().info(f"\n{'='*50}")
            self.get_logger().info(f"Moving to position {i+1}/{len(positions)}")
            self.get_logger().info(f"{'='*50}")

            success = self.move_to_joint_positions(joint_pos, duration_sec=3.0)

            if not success:
                self.get_logger().error(f"Failed to reach position {i+1}")
                return False

            # Small pause at each waypoint
            import time
            time.sleep(1.0)

        self.get_logger().info("\n" + "="*50)
        self.get_logger().info("Square movement completed successfully!")
        self.get_logger().info("="*50)
        return True


def main(args=None):
    if not IMPORTS_OK:
        print("ERROR: Required imports failed. Make sure all dependencies are installed.")
        return

    rclpy.init(args=args)
    node = SimpleSquareMovement()

    # Wait for joint states
    node.get_logger().info("Waiting for joint states...")
    import time
    timeout = 10.0
    start_time = time.time()

    while node.current_joint_state is None and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.current_joint_state is None:
        node.get_logger().error("Timeout waiting for joint states!")
        node.get_logger().error("Make sure the UR5e and MoveIt are running (use setupRealur5e.sh or setupFakeur5e.sh)")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("Joint states received, starting movement...")

    try:
        success = node.run_simple_square()

        if success:
            node.get_logger().info("✓ Mission accomplished!")
        else:
            node.get_logger().error("✗ Mission failed!")

    except KeyboardInterrupt:
        node.get_logger().info("Interrupted by user")
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {str(e)}")
        import traceback
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
