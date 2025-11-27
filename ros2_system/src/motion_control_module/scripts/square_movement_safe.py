#!/usr/bin/env python3
"""
Safe Square Box Movement Script for UR5e with Height Control
Ensures the robot stays above ground and doesn't flip upside down.
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


class SafeSquareMovement(Node):
    def __init__(self):
        super().__init__('safe_square_movement')

        # =================================================================
        # ADJUST HEIGHT HERE - Control how high the robot moves
        # =================================================================
        # Options: 'low', 'medium', 'high'
        # - 'low': Close to ground but safe (shoulder_lift ~ -1.8)
        # - 'medium': Mid-height workspace (shoulder_lift ~ -1.5)
        # - 'high': Higher up (shoulder_lift ~ -1.2)
        # =================================================================
        self.height_mode = 'medium'  # <<< CHANGE THIS
        # =================================================================

        # Movement parameters
        self.square_size = 0.15  # 15cm square pattern

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

        # Define safe height configurations
        self.height_configs = {
            'low': {
                'shoulder_lift': -1.8,  # Lower position
                'elbow': -1.8,
                'description': 'Low height - close to ground but safe'
            },
            'medium': {
                'shoulder_lift': -1.5,  # Medium position (default)
                'elbow': -1.6,
                'description': 'Medium height - good balance'
            },
            'high': {
                'shoulder_lift': -1.2,  # Higher position
                'elbow': -1.4,
                'description': 'High height - maximum safe height'
            }
        }

        # Safety constraints to keep robot above ground
        # These are HARD LIMITS - do not exceed!
        self.SAFETY_LIMITS = {
            'shoulder_lift_min': -2.0,  # Most downward (toward ground)
            'shoulder_lift_max': -1.0,  # Most upward (never go positive!)
            'elbow_min': -2.5,          # Most bent
            'elbow_max': -1.0,          # Least bent
        }

        self.get_logger().info("="*60)
        self.get_logger().info("Safe Square Movement Node Initialized")
        self.get_logger().info("="*60)
        self.get_logger().info(f"Height mode: {self.height_mode}")
        self.get_logger().info(f"Description: {self.height_configs[self.height_mode]['description']}")
        self.get_logger().info(f"Square size: {self.square_size}m")
        self.get_logger().info("="*60)
        self.get_logger().info("")
        self.get_logger().info("SAFETY FEATURES:")
        self.get_logger().info("  ✓ First link (shoulder) stays above ground")
        self.get_logger().info("  ✓ Shoulder lift restricted to prevent upward pointing")
        self.get_logger().info("  ✓ Elbow restricted to safe ranges")
        self.get_logger().info("  ✓ All movements verified within safety limits")
        self.get_logger().info("="*60)

    def joint_state_callback(self, msg):
        """Store current joint states"""
        self.current_joint_state = msg

    def verify_joint_safety(self, joint_positions):
        """Verify that joint positions are within safety limits"""
        shoulder_lift = joint_positions[1]
        elbow = joint_positions[2]

        if shoulder_lift < self.SAFETY_LIMITS['shoulder_lift_min']:
            self.get_logger().error(f"UNSAFE: shoulder_lift {shoulder_lift:.3f} below minimum {self.SAFETY_LIMITS['shoulder_lift_min']}")
            return False

        if shoulder_lift > self.SAFETY_LIMITS['shoulder_lift_max']:
            self.get_logger().error(f"UNSAFE: shoulder_lift {shoulder_lift:.3f} above maximum {self.SAFETY_LIMITS['shoulder_lift_max']}")
            return False

        if elbow < self.SAFETY_LIMITS['elbow_min']:
            self.get_logger().error(f"UNSAFE: elbow {elbow:.3f} below minimum {self.SAFETY_LIMITS['elbow_min']}")
            return False

        if elbow > self.SAFETY_LIMITS['elbow_max']:
            self.get_logger().error(f"UNSAFE: elbow {elbow:.3f} above maximum {self.SAFETY_LIMITS['elbow_max']}")
            return False

        return True

    def generate_square_positions(self):
        """Generate safe joint positions for square pattern at selected height"""
        config = self.height_configs[self.height_mode]
        shoulder_lift = config['shoulder_lift']
        elbow = config['elbow']

        # These positions form a square pattern
        # Format: [shoulder_pan, shoulder_lift, elbow, wrist_1, wrist_2, wrist_3]
        positions = [
            # Home/Start position (center)
            [0.0, shoulder_lift, elbow, -1.57, 1.57, 0.0],

            # Corner 1 (forward-right)
            [0.35, shoulder_lift - 0.1, elbow - 0.1, -1.5, 1.57, 0.0],

            # Corner 2 (forward-left)
            [-0.35, shoulder_lift - 0.1, elbow - 0.1, -1.5, 1.57, 0.0],

            # Corner 3 (back-left)
            [-0.35, shoulder_lift + 0.15, elbow + 0.15, -1.5, 1.57, 0.0],

            # Corner 4 (back-right)
            [0.35, shoulder_lift + 0.15, elbow + 0.15, -1.5, 1.57, 0.0],

            # Return to Corner 1
            [0.35, shoulder_lift - 0.1, elbow - 0.1, -1.5, 1.57, 0.0],

            # Return home
            [0.0, shoulder_lift, elbow, -1.57, 1.57, 0.0],
        ]

        # Verify all positions are safe
        for i, pos in enumerate(positions):
            if not self.verify_joint_safety(pos):
                self.get_logger().error(f"Position {i} failed safety check!")
                return None

        self.get_logger().info("✓ All positions verified safe")
        return positions

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
        self.get_logger().info(f"Moving joints: [pan={joint_positions[0]:.2f}, lift={joint_positions[1]:.2f}, elbow={joint_positions[2]:.2f}, ...]")
        send_goal_future = self.trajectory_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal rejected!")
            return False

        self.get_logger().info("Goal accepted, executing...")
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 2.0)

        return True

    def run_safe_square(self):
        """Execute a safe square movement pattern"""
        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("STARTING SAFE SQUARE MOVEMENT")
        self.get_logger().info("="*60)

        # Generate safe positions
        positions = self.generate_square_positions()
        if positions is None:
            self.get_logger().error("Failed to generate safe positions!")
            return False

        self.get_logger().info(f"Moving through {len(positions)} waypoints...")
        self.get_logger().info("")

        # Execute movement
        for i, joint_pos in enumerate(positions):
            self.get_logger().info(f"--- Waypoint {i+1}/{len(positions)} ---")

            success = self.move_to_joint_positions(joint_pos, duration_sec=3.0)

            if not success:
                self.get_logger().error(f"Failed to reach waypoint {i+1}")
                return False

            # Pause at each waypoint
            import time
            time.sleep(1.0)

        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("✓ SQUARE MOVEMENT COMPLETED SUCCESSFULLY!")
        self.get_logger().info("="*60)
        return True


def main(args=None):
    if not IMPORTS_OK:
        print("ERROR: Required imports failed.")
        return

    rclpy.init(args=args)
    node = SafeSquareMovement()

    # Wait for joint states
    node.get_logger().info("Waiting for robot connection...")
    import time
    timeout = 10.0
    start_time = time.time()

    while node.current_joint_state is None and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.current_joint_state is None:
        node.get_logger().error("Timeout waiting for robot!")
        node.get_logger().error("Make sure fake UR5e is running: ./4231_scripts/setupFakeur5e.sh")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("✓ Robot connected\n")

    try:
        success = node.run_safe_square()

        if success:
            node.get_logger().info("\n🎉 Mission accomplished!")
        else:
            node.get_logger().error("\n❌ Mission failed!")

    except KeyboardInterrupt:
        node.get_logger().info("\nInterrupted by user")
    except Exception as e:
        node.get_logger().error(f"\nUnexpected error: {str(e)}")
        import traceback
        node.get_logger().error(traceback.format_exc())
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
