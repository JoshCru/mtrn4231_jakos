#!/usr/bin/env python3
"""
Pick and Place Demo Script

Demonstrates a simple pick and place process with defined picking and placing areas.

Z Heights (tool0 frame):
  - Home Z: 371mm
  - Descend Point Z: 212mm
  - Pickup/Place Point Z: 182mm

Picking Area corners:
  - (-787.12, 50), (-420, 50), (-420.57, -252.76), (-787.12, -252)

Placing Area corners:
  - (-787.12, -252), (-787.12, -391.21), (-420.57, -252), (-420.57, -391.21)

Process:
  Pickup: Home -> Descend (picking area) -> Pickup Point -> Descend
  Place:  Descend -> Descend (placing area) -> Place Point -> Home

Press Enter before each movement for safety.
"""

import rclpy
from rclpy.node import Node
from sort_interfaces.srv import MoveToCartesian
import sys

# Z Heights (mm) - tool0 frame
Z_HOME = 371
Z_DESCEND = 212
Z_PICKUP = 182
Z_PLACE = 182

# Default orientation (facing down)
RX = 2.221
RY = 2.221
RZ = 0.0

# Example points in each area (center-ish of each area)
# Picking Area: X: -787.12 to -420, Y: 50 to -252
PICK_X = -600.0
PICK_Y = -100.0

# Placing Area: X: -787.12 to -420.57, Y: -252 to -391.21
PLACE_X = -600.0
PLACE_Y = -320.0


class PickAndPlaceDemo(Node):
    def __init__(self):
        super().__init__('pick_and_place_demo')

        # Create service client
        self.move_client = self.create_client(
            MoveToCartesian,
            '/motion_control/move_to_cartesian'
        )

        self.get_logger().info('Waiting for move_to_cartesian service...')
        if not self.move_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Service not available!')
            raise RuntimeError('Service not available')

        self.get_logger().info('Service available!')

    def move_to(self, x, y, z, rx=RX, ry=RY, rz=RZ, description=""):
        """Move to specified position with confirmation."""
        print(f"\n{'='*60}")
        print(f"Next movement: {description}")
        print(f"  Position: X={x:.1f}mm, Y={y:.1f}mm, Z={z:.1f}mm")
        print(f"  Rotation: rx={rx:.3f}, ry={ry:.3f}, rz={rz:.3f}")
        print(f"{'='*60}")

        input("Press ENTER to execute this movement...")

        request = MoveToCartesian.Request()
        request.x = float(x)
        request.y = float(y)
        request.z = float(z)
        request.rx = float(rx)
        request.ry = float(ry)
        request.rz = float(rz)

        self.get_logger().info(f'Executing: {description}')

        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        result = future.result()
        if result.success:
            self.get_logger().info(f'SUCCESS: {result.message}')
            return True
        else:
            self.get_logger().error(f'FAILED: {result.message}')
            return False

    def run_pick_and_place(self):
        """Execute the full pick and place sequence."""
        print("\n" + "="*70)
        print("PICK AND PLACE DEMO")
        print("="*70)
        print(f"\nPicking from: X={PICK_X}, Y={PICK_Y}")
        print(f"Placing at:   X={PLACE_X}, Y={PLACE_Y}")
        print(f"\nZ Heights: Home={Z_HOME}mm, Descend={Z_DESCEND}mm, Pick/Place={Z_PICKUP}mm")
        print("="*70)

        # ===== PICKUP SEQUENCE =====
        print("\n" + "#"*70)
        print("# PICKUP SEQUENCE")
        print("#"*70)

        # Step 1: Start at Home position
        if not self.move_to(PICK_X, PICK_Y, Z_HOME,
                           description="1. Move to HOME above picking area"):
            return False

        # Step 2: Descend to hover point
        if not self.move_to(PICK_X, PICK_Y, Z_DESCEND,
                           description="2. Descend to HOVER point in picking area"):
            return False

        # Step 3: Go down to pickup point
        if not self.move_to(PICK_X, PICK_Y, Z_PICKUP,
                           description="3. Lower to PICKUP point"):
            return False

        print("\n>>> GRIPPER WOULD CLOSE HERE <<<")
        input("Press ENTER to continue (simulating gripper close)...")

        # Step 4: Rise back to descend point
        if not self.move_to(PICK_X, PICK_Y, Z_DESCEND,
                           description="4. Rise to DESCEND point (with object)"):
            return False

        # ===== PLACE SEQUENCE =====
        print("\n" + "#"*70)
        print("# PLACE SEQUENCE")
        print("#"*70)

        # Step 5: Move to descend point above placing area
        if not self.move_to(PLACE_X, PLACE_Y, Z_DESCEND,
                           description="5. Move to DESCEND point above placing area"):
            return False

        # Step 6: Go down to place point
        if not self.move_to(PLACE_X, PLACE_Y, Z_PLACE,
                           description="6. Lower to PLACE point"):
            return False

        print("\n>>> GRIPPER WOULD OPEN HERE <<<")
        input("Press ENTER to continue (simulating gripper open)...")

        # Step 7: Rise and return to home
        if not self.move_to(PLACE_X, PLACE_Y, Z_HOME,
                           description="7. Return to HOME position"):
            return False

        print("\n" + "="*70)
        print("PICK AND PLACE SEQUENCE COMPLETED SUCCESSFULLY!")
        print("="*70)
        return True


def main(args=None):
    rclpy.init(args=args)

    print("\n" + "="*70)
    print("PICK AND PLACE DEMO")
    print("="*70)
    print("\nThis script will demonstrate a pick and place sequence.")
    print("You will need to press ENTER before each movement.")
    print("\nMake sure the robot is in a safe position before starting!")
    print("="*70)

    input("\nPress ENTER to begin...")

    try:
        node = PickAndPlaceDemo()
        success = node.run_pick_and_place()

        if success:
            print("\nDemo completed successfully!")
        else:
            print("\nDemo failed - check robot state")

    except KeyboardInterrupt:
        print("\nDemo interrupted by user")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
