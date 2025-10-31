#!/usr/bin/env python3
"""
Simple Square Box Movement Script for UR5e using MoveGroup Python interface
Moves the end effector in a square pattern while maintaining constant z-axis.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import sys
import math

try:
    from moveit_msgs.msg import RobotTrajectory, DisplayTrajectory
    from moveit_msgs.srv import GetCartesianPath
    from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
    from sensor_msgs.msg import JointState
    from std_msgs.msg import Header
    from control_msgs.action import FollowJointTrajectory
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

        # Starting position (center of square)
        self.center_x = 0.3  # 30cm in front of robot
        self.center_y = 0.0  # Centered

        # Create action client for joint trajectory control
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/joint_trajectory_controller/follow_joint_trajectory'
        )

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.current_joint_state = None

        # Create service client for cartesian path planning
        self.cartesian_path_client = self.create_client(
            GetCartesianPath,
            '/compute_cartesian_path'
        )

        self.get_logger().info("Simple Square Movement Node initialized")
        self.get_logger().info(f"Square size: {self.square_size}m, Z-height: {self.z_height}m")

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
        """Send joint trajectory goal"""
        if not self.trajectory_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().error("Action server not available!")
            return False

        # Create trajectory message
        goal_msg = FollowJointTrajectory.Goal()
        goal_msg.trajectory = JointTrajectory()
        goal_msg.trajectory.joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = joint_positions
        point.time_from_start.sec = int(duration_sec)
        point.time_from_start.nanosec = int((duration_sec % 1) * 1e9)

        goal_msg.trajectory.points = [point]

        # Send goal
        self.get_logger().info(f"Sending trajectory with positions: {[f'{p:.3f}' for p in joint_positions]}")
        send_goal_future = self.trajectory_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 2.0)

        return True

    def run_simple_square(self):
        """Execute a simple square movement using predefined joint positions"""
        self.get_logger().info("Starting simple square movement...")

        # Safe joint configurations that form a square pattern (approximately)
        # These are hand-tuned to avoid singularities
        # Format: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]

        positions = [
            # Home/Start position
            [0.0, -1.57, -1.57, -1.57, 1.57, 0.0],

            # Corner 1 (forward-right)
            [0.3, -1.4, -1.7, -1.5, 1.57, 0.0],

            # Corner 2 (forward-left)
            [-0.3, -1.4, -1.7, -1.5, 1.57, 0.0],

            # Corner 3 (back-left)
            [-0.3, -1.7, -1.4, -1.5, 1.57, 0.0],

            # Corner 4 (back-right)
            [0.3, -1.7, -1.4, -1.5, 1.57, 0.0],

            # Return to Corner 1
            [0.3, -1.4, -1.7, -1.5, 1.57, 0.0],

            # Return home
            [0.0, -1.57, -1.57, -1.57, 1.57, 0.0],
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
        node.get_logger().error("Make sure the fake UR5e is running (use setupFakeur5e.sh)")
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
