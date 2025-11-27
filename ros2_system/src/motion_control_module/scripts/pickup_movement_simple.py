#!/usr/bin/env python3
"""
Simple Pickup Movement using Joint Trajectory Controller
Bypasses MoveIt and sends trajectories directly to the controller.

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
from rclpy.action import ActionClient
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from sensor_msgs.msg import JointState
from builtin_interfaces.msg import Duration
import math
import time


class SimplePickupMovement(Node):
    def __init__(self):
        super().__init__('simple_pickup_movement')

        # Create action client for the scaled joint trajectory controller
        self.trajectory_client = ActionClient(
            self,
            FollowJointTrajectory,
            '/scaled_joint_trajectory_controller/follow_joint_trajectory'
        )

        # Subscribe to joint states
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )

        self.current_joint_state = None

        # Joint names in the order the controller expects
        # This is [J6, J1, J2, J3, J4, J5]
        self.joint_names = [
            'shoulder_pan_joint',   # Joint 6 (base)
            'shoulder_lift_joint',  # Joint 1
            'elbow_joint',          # Joint 2
            'wrist_1_joint',        # Joint 3
            'wrist_2_joint',        # Joint 4
            'wrist_3_joint'         # Joint 5
        ]

        self.get_logger().info("Simple Pickup Movement initialized")
        self.get_logger().info("Waiting for trajectory controller...")

    def joint_state_callback(self, msg):
        """Store current joint states"""
        self.current_joint_state = msg

    def get_current_joint_positions(self):
        """Get current joint positions in [J6, J1, J2, J3, J4, J5] order"""
        if not self.current_joint_state:
            return None

        positions = []
        for joint_name in self.joint_names:
            try:
                idx = self.current_joint_state.name.index(joint_name)
                positions.append(self.current_joint_state.position[idx])
            except ValueError:
                self.get_logger().error(f"Joint {joint_name} not found")
                return None

        return positions

    def display_current_state(self):
        """Display current joint positions"""
        positions = self.get_current_joint_positions()
        if not positions:
            return

        self.get_logger().info("\nCurrent joint positions [J6, J1, J2, J3, J4, J5]:")
        labels = ["J6 (base)", "J1", "J2", "J3", "J4", "J5"]

        for label, pos_rad in zip(labels, positions):
            pos_deg = math.degrees(pos_rad)
            self.get_logger().info(f"  {label:12s}: {pos_rad:8.4f} rad ({pos_deg:7.2f}°)")

    def move_to_joint_positions(self, target_positions, duration_sec=5.0):
        """Send trajectory to joint positions

        Args:
            target_positions: List of 6 joint positions in RADIANS [J6, J1, J2, J3, J4, J5]
            duration_sec: Time to reach the target
        """
        if not self.trajectory_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("Trajectory controller not available!")
            self.get_logger().error("Make sure the robot is running with the trajectory controller")
            return False

        # Ensure all positions are floats
        target_positions = [float(p) for p in target_positions]

        # Log what we're sending
        self.get_logger().info("="*80)
        self.get_logger().info("Sending trajectory to controller:")
        labels = ["J6 (shoulder_pan)", "J1 (shoulder_lift)", "J2 (elbow)",
                  "J3 (wrist_1)", "J4 (wrist_2)", "J5 (wrist_3)"]

        for label, pos_rad in zip(labels, target_positions):
            pos_deg = math.degrees(pos_rad)
            self.get_logger().info(f"  {label:20s}: {pos_rad:8.4f} rad ({pos_deg:7.2f}°)")
        self.get_logger().info("="*80)

        # Create trajectory message
        trajectory = JointTrajectory()
        trajectory.joint_names = self.joint_names

        # Create trajectory point
        point = JointTrajectoryPoint()
        point.positions = target_positions
        point.velocities = [0.0] * 6  # Stop at the end
        point.time_from_start = Duration(sec=int(duration_sec), nanosec=0)

        trajectory.points = [point]

        # Create goal
        goal = FollowJointTrajectory.Goal()
        goal.trajectory = trajectory

        # Send goal
        self.get_logger().info(f"Executing trajectory (duration: {duration_sec}s)...")
        send_goal_future = self.trajectory_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self, send_goal_future)

        goal_handle = send_goal_future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Trajectory goal rejected!")
            return False

        self.get_logger().info("Trajectory goal accepted, executing...")

        # Wait for result
        result_future = goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future, timeout_sec=duration_sec + 5.0)

        if result_future.result() is None:
            self.get_logger().error("No result received (timeout?)")
            return False

        result = result_future.result().result
        error_code = result.error_code

        if error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.get_logger().info("✓ Trajectory executed successfully!")
            return True
        else:
            self.get_logger().error(f"✗ Trajectory execution failed with error code: {error_code}")
            return False

    def get_default_sequence(self):
        """Get the default pickup sequence in radians"""
        # Define in degrees, convert to radians
        sequences_degrees = [
            ('Home point',         [  0.0, -75.0,  90.0, -105.0, -90.0,  0.0]),
            ('Pick up point',      [  0.0, -53.0,  88.0, -127.0, -86.0,  9.0]),
            ('Lift up point',      [-12.0, -60.0,  82.0, -113.0, -86.0,  9.0]),
            ('Put down point',     [-12.0, -53.0,  88.0, -126.0, -86.0,  9.0]),
            ('Home point (return)', [  0.0, -75.0,  90.0, -105.0, -90.0,  0.0])
        ]

        positions = []
        for name, degrees in sequences_degrees:
            radians = [math.radians(d) for d in degrees]
            positions.append({
                'name': name,
                'degrees': degrees,
                'radians': radians
            })

        return positions

    def run_pickup_sequence(self, positions):
        """Execute pickup movement sequence"""
        self.get_logger().info("\n" + "="*80)
        self.get_logger().info("Starting pickup movement sequence (Direct Trajectory Control)")
        self.get_logger().info("="*80)

        # Display current state
        self.display_current_state()

        for i, position in enumerate(positions):
            self.get_logger().info(f"\n{'='*80}")
            self.get_logger().info(f"Position {i+1}/{len(positions)}: {position['name']}")
            self.get_logger().info(f"{'='*80}")

            # Wait for user confirmation
            print(f"\nPress ENTER to move to '{position['name']}'...")
            input()

            # Execute movement
            success = self.move_to_joint_positions(position['radians'], duration_sec=5.0)

            if not success:
                self.get_logger().error(f"Failed to reach {position['name']}")
                return False

            # Wait for movement to complete
            time.sleep(1.0)

            # Show where we ended up
            self.display_current_state()

        self.get_logger().info("\n" + "="*80)
        self.get_logger().info("✓ Pickup sequence completed!")
        self.get_logger().info("="*80)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = SimplePickupMovement()

    # Wait for joint states
    node.get_logger().info("Waiting for joint states...")
    timeout = 10.0
    start_time = time.time()

    while node.current_joint_state is None and (time.time() - start_time) < timeout:
        rclpy.spin_once(node, timeout_sec=0.1)

    if node.current_joint_state is None:
        node.get_logger().error("Timeout waiting for joint states!")
        node.get_logger().error("Make sure the robot controller is running")
        node.destroy_node()
        rclpy.shutdown()
        return

    node.get_logger().info("Joint states received!")

    try:
        print("\n" + "="*80)
        print("SIMPLE PICKUP MOVEMENT - Direct Trajectory Controller")
        print("="*80)

        # Get and run default sequence
        positions = node.get_default_sequence()
        success = node.run_pickup_sequence(positions)

        if success:
            node.get_logger().info("\n✓ Mission accomplished!")
        else:
            node.get_logger().error("\n✗ Mission failed!")

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
