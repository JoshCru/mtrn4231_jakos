#!/usr/bin/env python3
"""
Workspace Calibration Test Script
Tests the UR5e reachability in all 4 quadrants with the gripper attached
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import MoveItErrorCodes
from geometry_msgs.msg import Pose, Point, Quaternion
import sys

# Try to import moveit_py (ROS 2 Humble)
try:
    from moveit.planning import MoveItPy
    from moveit.core.robot_state import RobotState
except ImportError:
    print("ERROR: moveit_py not available. Using alternative approach.")
    MoveItPy = None

from rclpy.action import ActionClient
from moveit_msgs.action import MoveGroup


class WorkspaceCalibration(Node):
    def __init__(self):
        super().__init__('workspace_calibration')

        self.get_logger().info("Initializing Workspace Calibration Test...")

        # Define test positions in different quadrants (x, y, z in meters)
        # Base height is around 0.1m, max reach is about 0.85m
        self.test_positions = {
            # Quadrant 1: Front-Right (+X, +Y)
            "Q1_near": (0.3, 0.2, 0.3),
            "Q1_mid": (0.4, 0.3, 0.4),
            "Q1_far": (0.5, 0.4, 0.3),
            "Q1_high": (0.4, 0.3, 0.6),
            "Q1_low": (0.3, 0.2, 0.15),

            # Quadrant 2: Front-Left (+X, -Y)
            "Q2_near": (0.3, -0.2, 0.3),
            "Q2_mid": (0.4, -0.3, 0.4),
            "Q2_far": (0.5, -0.4, 0.3),
            "Q2_high": (0.4, -0.3, 0.6),
            "Q2_low": (0.3, -0.2, 0.15),

            # Quadrant 3: Back-Left (-X, -Y)
            "Q3_near": (-0.3, -0.2, 0.3),
            "Q3_mid": (-0.4, -0.3, 0.4),
            "Q3_far": (-0.5, -0.4, 0.3),
            "Q3_high": (-0.4, -0.3, 0.6),
            "Q3_low": (-0.3, -0.2, 0.15),

            # Quadrant 4: Back-Right (-X, +Y)
            "Q4_near": (-0.3, 0.2, 0.3),
            "Q4_mid": (-0.4, 0.3, 0.4),
            "Q4_far": (-0.5, 0.4, 0.3),
            "Q4_high": (-0.4, 0.3, 0.6),
            "Q4_low": (-0.3, 0.2, 0.15),

            # Central positions
            "center_mid": (0.0, 0.0, 0.4),
            "center_high": (0.0, 0.0, 0.7),
            "center_low": (0.0, 0.0, 0.2),
        }

        # Action client for move_group
        self._action_client = ActionClient(
            self,
            MoveGroup,
            '/move_action'
        )

        self.results = {}

    def create_pose_goal(self, x, y, z):
        """Create a pose goal with position and standard orientation (gripper pointing down)"""
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        # Gripper pointing down (standard orientation)
        pose.orientation = Quaternion(x=-0.707, y=0.0, z=0.0, w=0.707)
        return pose

    def test_position(self, name, position):
        """Test if a position is reachable"""
        x, y, z = position
        self.get_logger().info(f"\nTesting {name}: x={x:.2f}, y={y:.2f}, z={z:.2f}")

        # Create goal message
        goal_msg = MoveGroup.Goal()
        goal_msg.request.group_name = "ur_manipulator"
        goal_msg.request.num_planning_attempts = 5
        goal_msg.request.allowed_planning_time = 5.0
        goal_msg.request.max_velocity_scaling_factor = 0.1
        goal_msg.request.max_acceleration_scaling_factor = 0.1

        # Create pose constraint
        from moveit_msgs.msg import Constraints, PositionConstraint, OrientationConstraint
        from shape_msgs.msg import SolidPrimitive

        constraints = Constraints()

        # Position constraint
        pos_constraint = PositionConstraint()
        pos_constraint.header.frame_id = "base_link"
        pos_constraint.link_name = "tool0"

        # Define constraint region (small sphere around target)
        primitive = SolidPrimitive()
        primitive.type = SolidPrimitive.SPHERE
        primitive.dimensions = [0.01]  # 1cm tolerance

        pose = self.create_pose_goal(x, y, z)
        pos_constraint.constraint_region.primitives = [primitive]
        pos_constraint.constraint_region.primitive_poses = [pose]
        pos_constraint.weight = 1.0

        # Orientation constraint
        ori_constraint = OrientationConstraint()
        ori_constraint.header.frame_id = "base_link"
        ori_constraint.link_name = "tool0"
        ori_constraint.orientation = pose.orientation
        ori_constraint.absolute_x_axis_tolerance = 0.1
        ori_constraint.absolute_y_axis_tolerance = 0.1
        ori_constraint.absolute_z_axis_tolerance = 0.1
        ori_constraint.weight = 1.0

        constraints.position_constraints = [pos_constraint]
        constraints.orientation_constraints = [ori_constraint]
        goal_msg.request.goal_constraints = [constraints]

        # Plan only (don't execute for safety)
        goal_msg.planning_options.plan_only = True

        # Send goal
        if not self._action_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveGroup action server not available!")
            return False

        future = self._action_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().warn(f"  ❌ {name}: Goal rejected")
            return False

        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=10.0)

        result = result_future.result().result

        if result.error_code.val == MoveItErrorCodes.SUCCESS:
            self.get_logger().info(f"  ✓ {name}: REACHABLE")
            return True
        else:
            self.get_logger().warn(f"  ❌ {name}: NOT REACHABLE (error code: {result.error_code.val})")
            return False

    def run_calibration(self):
        """Run the full workspace calibration"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("Starting Workspace Calibration Test")
        self.get_logger().info("="*60)
        self.get_logger().info("NOTE: This will PLAN motions but NOT execute them")
        self.get_logger().info("="*60 + "\n")

        # Test each position
        for name, position in self.test_positions.items():
            success = self.test_position(name, position)
            self.results[name] = success
            # Small delay between tests
            self.get_logger().info("Waiting 1 second...")
            rclpy.spin_once(self, timeout_sec=1.0)

        # Print summary
        self.print_summary()

    def print_summary(self):
        """Print a summary of all test results"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("WORKSPACE CALIBRATION SUMMARY")
        self.get_logger().info("="*60)

        # Count by quadrant
        quadrants = {"Q1": [], "Q2": [], "Q3": [], "Q4": [], "center": []}

        for name, success in self.results.items():
            quadrant = name.split('_')[0]
            if quadrant in quadrants:
                quadrants[quadrant].append(success)

        for quadrant, results in quadrants.items():
            if results:
                total = len(results)
                successful = sum(results)
                percentage = (successful / total) * 100
                self.get_logger().info(f"{quadrant}: {successful}/{total} reachable ({percentage:.0f}%)")

        total_tests = len(self.results)
        total_success = sum(self.results.values())
        overall_percentage = (total_success / total_tests) * 100

        self.get_logger().info("-" * 60)
        self.get_logger().info(f"OVERALL: {total_success}/{total_tests} positions reachable ({overall_percentage:.0f}%)")
        self.get_logger().info("="*60)

        # List unreachable positions
        unreachable = [name for name, success in self.results.items() if not success]
        if unreachable:
            self.get_logger().info("\nUnreachable positions:")
            for name in unreachable:
                pos = self.test_positions[name]
                self.get_logger().info(f"  - {name}: ({pos[0]:.2f}, {pos[1]:.2f}, {pos[2]:.2f})")


def main(args=None):
    rclpy.init(args=args)

    calibration = WorkspaceCalibration()

    try:
        # Wait a moment for MoveIt to initialize
        calibration.get_logger().info("Waiting 3 seconds for MoveIt to initialize...")
        rclpy.spin_once(calibration, timeout_sec=3.0)

        # Run the calibration
        calibration.run_calibration()

    except KeyboardInterrupt:
        calibration.get_logger().info("\nCalibration interrupted by user")
    except Exception as e:
        calibration.get_logger().error(f"Error during calibration: {e}")
        import traceback
        traceback.print_exc()
    finally:
        calibration.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
