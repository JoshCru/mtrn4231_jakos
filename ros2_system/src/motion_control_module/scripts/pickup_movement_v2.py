#!/usr/bin/env python3
"""
Pickup Movement Script for UR5e using MoveIt Commander
Moves through pick-up, lift, and put-down positions with user confirmation.

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
import sys
import math
import time

try:
    import moveit_commander
    from geometry_msgs.msg import Pose
    IMPORTS_OK = True
except ImportError as e:
    print(f"Import error: {e}")
    IMPORTS_OK = False


class PickupMovementV2(Node):
    def __init__(self):
        super().__init__('pickup_movement_v2')

        # Initialize moveit_commander
        moveit_commander.roscpp_initialize(sys.argv)

        # Create robot and scene interfaces
        self.robot = moveit_commander.RobotCommander()
        self.scene = moveit_commander.PlanningSceneInterface()

        # Create move group for the ur_manipulator
        self.move_group = moveit_commander.MoveGroupCommander("ur_manipulator")

        # Set planning parameters
        self.move_group.set_planning_time(10.0)
        self.move_group.set_num_planning_attempts(10)
        self.move_group.set_max_velocity_scaling_factor(0.2)
        self.move_group.set_max_acceleration_scaling_factor(0.2)
        self.move_group.set_goal_joint_tolerance(0.01)  # 0.01 rad = ~0.57°

        self.get_logger().info("Pickup Movement V2 initialized with MoveIt Commander")
        self.get_logger().info(f"Planning group: {self.move_group.get_name()}")
        self.get_logger().info(f"Planning frame: {self.move_group.get_planning_frame()}")
        self.get_logger().info(f"End effector: {self.move_group.get_end_effector_link()}")

    def get_current_joint_values(self):
        """Get current joint values in [J6, J1, J2, J3, J4, J5] order"""
        joint_values = self.move_group.get_current_joint_values()
        return joint_values

    def display_current_state(self):
        """Display current robot state"""
        joint_values = self.get_current_joint_values()

        self.get_logger().info("\nCurrent joint positions [J6, J1, J2, J3, J4, J5]:")
        joint_names = ["J6 (base)", "J1", "J2", "J3", "J4", "J5"]

        for i, (name, val) in enumerate(zip(joint_names, joint_values)):
            deg = math.degrees(val)
            self.get_logger().info(f"  {name:12s}: {val:8.4f} rad ({deg:7.2f}°)")

    def move_to_joint_positions(self, joint_positions):
        """Move to joint positions using MoveIt Commander

        Args:
            joint_positions: List of 6 joint positions in RADIANS [J6, J1, J2, J3, J4, J5]
        """
        # Ensure all positions are floats
        joint_positions = [float(p) for p in joint_positions]

        # Log what we're sending
        self.get_logger().info("="*80)
        self.get_logger().info("Setting joint goal [J6, J1, J2, J3, J4, J5]:")

        joint_names = ["J6 (shoulder_pan)", "J1 (shoulder_lift)", "J2 (elbow)",
                       "J3 (wrist_1)", "J4 (wrist_2)", "J5 (wrist_3)"]

        for i, (name, pos_rad) in enumerate(zip(joint_names, joint_positions)):
            pos_deg = math.degrees(pos_rad)
            self.get_logger().info(f"  {name:20s}: {pos_rad:8.4f} rad ({pos_deg:7.2f}°)")

        # Set the joint value target
        self.move_group.set_joint_value_target(joint_positions)

        # Plan the motion
        self.get_logger().info("Planning trajectory...")
        success, plan, planning_time, error_code = self.move_group.plan()

        if not success:
            self.get_logger().error(f"Planning failed with error code: {error_code}")
            return False

        self.get_logger().info(f"Planning succeeded (took {planning_time:.2f}s)")

        # Execute the plan
        self.get_logger().info("Executing trajectory...")
        success = self.move_group.execute(plan, wait=True)

        # Stop the robot (just to be sure)
        self.move_group.stop()

        if success:
            self.get_logger().info("✓ Movement successful!")
            return True
        else:
            self.get_logger().error("✗ Execution failed!")
            return False

    def get_default_sequence(self):
        """Get the default pickup sequence

        Joint order: [Joint6, Joint1, Joint2, Joint3, Joint4, Joint5]
        """
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
        self.get_logger().info("Starting pickup movement sequence...")
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
            success = self.move_to_joint_positions(position['radians'])

            if not success:
                self.get_logger().error(f"Failed to reach {position['name']}")
                return False

            # Brief pause to settle
            time.sleep(0.5)

            # Show where we ended up
            self.display_current_state()

        self.get_logger().info("\n" + "="*80)
        self.get_logger().info("✓ Pickup sequence completed successfully!")
        self.get_logger().info("="*80)
        return True


def main(args=None):
    if not IMPORTS_OK:
        print("ERROR: Required imports failed. Install moveit_commander:")
        print("  sudo apt install ros-humble-moveit-commander")
        return

    rclpy.init(args=args)
    node = PickupMovementV2()

    # Give time for connections
    time.sleep(1.0)

    try:
        print("\n" + "="*80)
        print("PICKUP MOVEMENT - Using MoveIt Commander")
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
        moveit_commander.roscpp_shutdown()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
