#!/usr/bin/env python3
"""
Pickup Movement Script for UR5e using MoveIt
Moves through pick-up, lift, and put-down positions with user confirmation.
Supports saving and loading custom sequences.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose, PoseStamped
import sys
import math
import json
import os
from pathlib import Path

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


class PickupMovement(Node):
    def __init__(self):
        super().__init__('pickup_movement')

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

        # Path for saving/loading sequences
        self.sequences_dir = Path.home() / "Documents/mtrn4231_jakos/saved_sequences"
        self.sequences_dir.mkdir(parents=True, exist_ok=True)

        self.get_logger().info("Pickup Movement Node initialized with MoveIt")
        self.get_logger().info(f"Planning group: {self.planning_group}")
        self.get_logger().info(f"Sequences directory: {self.sequences_dir}")

    def joint_state_callback(self, msg):
        """Store current joint states"""
        self.current_joint_state = msg

    def get_current_joint_positions(self):
        """Get current joint positions in order"""
        if not self.current_joint_state:
            return None

        joint_names = [
            'shoulder_pan_joint',
            'shoulder_lift_joint',
            'elbow_joint',
            'wrist_1_joint',
            'wrist_2_joint',
            'wrist_3_joint'
        ]

        positions = []
        for joint_name in joint_names:
            try:
                idx = self.current_joint_state.name.index(joint_name)
                positions.append(self.current_joint_state.position[idx])
            except ValueError:
                self.get_logger().error(f"Joint {joint_name} not found in joint states")
                return None

        return positions

    def record_waypoint(self, waypoint_name):
        """Record current robot position as a waypoint"""
        # Joint states are always in radians from ROS
        positions_rad = self.get_current_joint_positions()
        if positions_rad is None:
            return None

        # Convert to degrees for storage and sending to MoveIt (as floats)
        positions_deg = [float(math.degrees(r)) for r in positions_rad]

        waypoint = {
            'name': waypoint_name,
            'degrees': positions_deg,
            'radians': [float(r) for r in positions_rad]
        }

        self.get_logger().info(f"Recorded waypoint '{waypoint_name}':")
        self.get_logger().info(f"  Degrees: {[f'{d:.2f}' for d in positions_deg]}")
        self.get_logger().info(f"  Radians: {[f'{r:.6f}' for r in positions_rad]}")

        return waypoint

    def save_sequence(self, sequence_name, waypoints):
        """Save a sequence of waypoints to a JSON file"""
        filepath = self.sequences_dir / f"{sequence_name}.json"

        data = {
            'name': sequence_name,
            'waypoints': waypoints
        }

        with open(filepath, 'w') as f:
            json.dump(data, f, indent=2)

        self.get_logger().info(f"Saved sequence '{sequence_name}' to {filepath}")
        return filepath

    def load_sequence(self, sequence_name):
        """Load a sequence from a JSON file"""
        filepath = self.sequences_dir / f"{sequence_name}.json"

        if not filepath.exists():
            self.get_logger().error(f"Sequence file not found: {filepath}")
            return None

        with open(filepath, 'r') as f:
            data = json.load(f)

        self.get_logger().info(f"Loaded sequence '{sequence_name}' with {len(data['waypoints'])} waypoints")
        return data['waypoints']

    def list_saved_sequences(self):
        """List all saved sequence files"""
        sequences = []
        for filepath in self.sequences_dir.glob("*.json"):
            sequences.append(filepath.stem)
        return sorted(sequences)

    def get_default_sequence(self):
        """Get the default pickup sequence"""
        import math

        # Define in degrees (will be sent as-is to MoveIt)
        sequences_degrees = [
            ('Home point', [0.0, -75.0, 90.0, -105.0, -90.0, 0.0]),
            ('Pick up point', [0.0, -53.0, 88.0, -127.0, -86.0, 9.0]),
            ('Lift up point', [-12.0, -60.0, 82.0, -113.0, -86.0, 9.0]),
            ('Put down point', [-12.0, -53.0, 88.0, -126.0, -86.0, 9.0]),
            ('Home point (return)', [0.0, -75.0, 90.0, -105.0, -90.0, 0.0])
        ]

        positions = []
        for name, degrees in sequences_degrees:
            # Keep degrees as floats for MoveIt
            degrees_float = [float(d) for d in degrees]
            radians = [math.radians(d) for d in degrees]
            positions.append({
                'name': name,
                'degrees': degrees_float,
                'radians': radians
            })

        return positions

    def record_new_sequence(self):
        """Interactive recording of a new sequence"""
        print("\n" + "="*60)
        print("TEACH PENDANT MODE - Record New Sequence")
        print("="*60)
        print("\nInstructions:")
        print("1. Manually move the robot to desired positions")
        print("2. Press ENTER at each position to record it")
        print("3. Type 'done' when finished recording")
        print("="*60 + "\n")

        sequence_name = input("Enter name for this sequence: ").strip()
        if not sequence_name:
            print("Invalid sequence name. Aborting.")
            return None

        waypoints = []
        waypoint_num = 1

        while True:
            print(f"\n--- Waypoint {waypoint_num} ---")
            user_input = input(f"Move robot to position, then press ENTER to record (or type 'done' to finish): ").strip().lower()

            if user_input == 'done':
                if len(waypoints) == 0:
                    print("No waypoints recorded. Aborting.")
                    return None
                break

            waypoint_name = input(f"Enter name for waypoint {waypoint_num} (or press ENTER for default): ").strip()
            if not waypoint_name:
                waypoint_name = f"Waypoint {waypoint_num}"

            waypoint = self.record_waypoint(waypoint_name)
            if waypoint:
                waypoints.append(waypoint)
                print(f"✓ Recorded '{waypoint_name}'")
                waypoint_num += 1
            else:
                print("Failed to record waypoint. Try again.")

        # Save the sequence
        print(f"\nRecorded {len(waypoints)} waypoints total.")
        save_choice = input("Save this sequence? (y/n): ").strip().lower()

        if save_choice == 'y':
            self.save_sequence(sequence_name, waypoints)
            print(f"✓ Sequence '{sequence_name}' saved!")
            return waypoints
        else:
            print("Sequence not saved.")
            return waypoints

    def move_to_joint_positions(self, joint_positions, duration_sec=5.0):
        """Plan and execute movement to joint positions using MoveIt"""
        if not self.move_group_client.wait_for_server(timeout_sec=5.0):
            self.get_logger().error("MoveIt action server not available!")
            return False

        # Ensure all positions are floats
        joint_positions = [float(p) for p in joint_positions]

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
            joint_constraint.position = float(position)  # Ensure float
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

    def run_pickup_sequence(self, positions):
        """Execute pickup movement sequence with user confirmation"""
        self.get_logger().info("Starting pickup movement sequence...")

        # Print current joint state for debugging
        if self.current_joint_state:
            self.get_logger().info("Current joint positions:")
            for name, pos in zip(self.current_joint_state.name, self.current_joint_state.position):
                self.get_logger().info(f"  {name}: {pos:.6f}")

        for i, position in enumerate(positions):
            self.get_logger().info(f"\n{'='*60}")
            self.get_logger().info(f"Position {i+1}/{len(positions)}: {position['name']}")
            self.get_logger().info(f"Target angles (degrees): {position['degrees']}")
            self.get_logger().info(f"Target angles (radians): {[f'{r:.6f}' for r in position['radians']]}")
            self.get_logger().info(f"{'='*60}")

            # Wait for user confirmation
            print(f"\nPress ENTER to move to {position['name']}...")
            input()

            # Send degree values directly (MoveIt will interpret as radians)
            # WARNING: This means 90 degrees sends 90.0 which MoveIt treats as 90 radians!
            success = self.move_to_joint_positions(position['radians'], duration_sec=5.0)

            if not success:
                self.get_logger().error(f"Failed to reach {position['name']}")
                return False

            self.get_logger().info(f"✓ Reached {position['name']}")

        self.get_logger().info("\n" + "="*60)
        self.get_logger().info("Pickup sequence completed successfully!")
        self.get_logger().info("="*60)
        return True


def main(args=None):
    if not IMPORTS_OK:
        print("ERROR: Required imports failed. Make sure all dependencies are installed.")
        return

    rclpy.init(args=args)
    node = PickupMovement()

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

    node.get_logger().info("Joint states received!")

    try:
        # Main menu
        print("\n" + "="*60)
        print("PICKUP MOVEMENT - Main Menu")
        print("="*60)
        print("\n1. Run default sequence (5 waypoints)")
        print("2. Record new sequence (teach pendant mode)")
        print("3. Load saved sequence")
        print("4. List saved sequences")
        print("5. Exit")
        print("="*60)

        choice = input("\nEnter choice (1-5): ").strip()

        positions = None

        if choice == '1':
            # Use default sequence
            print("\nUsing default sequence...")
            positions = node.get_default_sequence()

        elif choice == '2':
            # Record new sequence
            positions = node.record_new_sequence()
            if positions is None:
                print("Recording cancelled.")
                node.destroy_node()
                rclpy.shutdown()
                return

        elif choice == '3':
            # Load saved sequence
            saved_sequences = node.list_saved_sequences()
            if not saved_sequences:
                print("\nNo saved sequences found.")
                print(f"Sequences are saved in: {node.sequences_dir}")
                node.destroy_node()
                rclpy.shutdown()
                return

            print("\nSaved sequences:")
            for i, seq_name in enumerate(saved_sequences, 1):
                print(f"  {i}. {seq_name}")

            seq_choice = input(f"\nEnter sequence number (1-{len(saved_sequences)}): ").strip()
            try:
                seq_idx = int(seq_choice) - 1
                if 0 <= seq_idx < len(saved_sequences):
                    positions = node.load_sequence(saved_sequences[seq_idx])
                else:
                    print("Invalid sequence number.")
                    node.destroy_node()
                    rclpy.shutdown()
                    return
            except ValueError:
                print("Invalid input.")
                node.destroy_node()
                rclpy.shutdown()
                return

        elif choice == '4':
            # List saved sequences
            saved_sequences = node.list_saved_sequences()
            print("\nSaved sequences:")
            if not saved_sequences:
                print("  (none)")
                print(f"\nSequences directory: {node.sequences_dir}")
            else:
                for seq_name in saved_sequences:
                    print(f"  - {seq_name}")
            node.destroy_node()
            rclpy.shutdown()
            return

        elif choice == '5':
            # Exit
            print("Exiting...")
            node.destroy_node()
            rclpy.shutdown()
            return

        else:
            print("Invalid choice.")
            node.destroy_node()
            rclpy.shutdown()
            return

        # Execute sequence if positions were selected
        if positions:
            success = node.run_pickup_sequence(positions)

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
