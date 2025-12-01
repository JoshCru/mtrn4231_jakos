#!/usr/bin/env python3
"""
Position Check for Simulated Perception

This script moves the robot to each simulated weight's position at Z_PICKUP
to allow verification before starting the sorting system.
"""

import rclpy
from rclpy.node import Node
from sort_interfaces.msg import DetectedObjects, BoundingBox
from sort_interfaces.srv import MoveToCartesian, GripperControl
import yaml
import sys


class PositionCheckNode(Node):
    def __init__(self):
        super().__init__('position_check_node')

        # Load robot config
        self.load_config()

        # Create service clients
        self.move_client = self.create_client(MoveToCartesian, '/motion_control/move_to_cartesian')
        self.gripper_client = self.create_client(GripperControl, '/motion_control/gripper_control')

        # Wait for services
        self.get_logger().info('Waiting for motion control services...')
        self.move_client.wait_for_service(timeout_sec=10.0)
        self.gripper_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Services ready!')

        # Subscribe to detected objects
        self.detected_objects = []
        self.objects_received = False
        self.subscription = self.create_subscription(
            DetectedObjects,
            '/perception/detected_objects',
            self.objects_callback,
            10
        )

    def load_config(self):
        """Load configuration from robot_config.yaml"""
        config_path = '/home/joshc/mtrn4231_jakos/4231_scripts/robot_config.yaml'

        # Default values
        self.z_home = 371.0
        self.z_pickup = 180.0
        self.rx = 2.221
        self.ry = 2.221
        self.rz = 0.0

        try:
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)

                if 'z_heights' in config:
                    self.z_home = config['z_heights'].get('home', self.z_home)
                    self.z_pickup = config['z_heights'].get('pickup', self.z_pickup)

                if 'orientation' in config:
                    self.rx = config['orientation'].get('rx', self.rx)
                    self.ry = config['orientation'].get('ry', self.ry)
                    self.rz = config['orientation'].get('rz', self.rz)

            self.get_logger().info(f'Loaded config from {config_path}')
            self.get_logger().info(f'Z_HOME: {self.z_home}, Z_PICKUP: {self.z_pickup}')
        except Exception as e:
            self.get_logger().warn(f'Could not load config: {e}, using defaults')

    def objects_callback(self, msg):
        """Callback for detected objects"""
        if not self.objects_received:
            self.detected_objects = msg.objects  # Array of BoundingBox
            self.objects_received = True
            self.get_logger().info(f'Received {len(self.detected_objects)} detected objects')

    def move_to(self, x, y, z):
        """Move robot to specified position"""
        request = MoveToCartesian.Request()
        request.x = x
        request.y = y
        request.z = z
        request.rx = self.rx
        request.ry = self.ry
        request.rz = self.rz

        future = self.move_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)

        if future.result() is not None and future.result().success:
            return True
        else:
            self.get_logger().error('Move failed!')
            return False

    def open_gripper(self):
        """Open the gripper"""
        request = GripperControl.Request()
        request.command = 'W'  # Open command
        request.weight = 0

        future = self.gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None and future.result().success:
            return True
        else:
            self.get_logger().warn('Gripper open failed')
            return False

    def close_gripper(self):
        """Close the gripper"""
        request = GripperControl.Request()
        request.command = 'S'  # Close command
        request.weight = 0

        future = self.gripper_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=10.0)

        if future.result() is not None and future.result().success:
            return True
        else:
            self.get_logger().warn('Gripper close failed')
            return False

    def run_position_check(self):
        """Main position checking routine"""
        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('SIMULATED PERCEPTION POSITION CHECK')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')

        # Wait for objects
        self.get_logger().info('Waiting for simulated objects from /perception/detected_objects...')
        timeout_count = 0
        while not self.objects_received and timeout_count < 50:
            rclpy.spin_once(self, timeout_sec=0.1)
            timeout_count += 1

        if not self.objects_received or len(self.detected_objects) == 0:
            self.get_logger().error('No objects received! Is simulated_perception_node running?')
            self.get_logger().error('Topic: /perception/detected_objects')
            return False

        self.get_logger().info(f'Found {len(self.detected_objects)} simulated objects')
        self.get_logger().info('')

        # Open gripper
        self.get_logger().info('Opening gripper...')
        self.open_gripper()

        # Visit each position
        for i, obj in enumerate(self.detected_objects):
            # Calculate center position from bounding box
            center_x = (obj.x_min + obj.x_max) / 2.0
            center_y = (obj.y_min + obj.y_max) / 2.0

            # Extract weight from class_name (e.g., "100g" -> 100)
            weight_str = obj.class_name.replace('g', '')
            try:
                weight = int(weight_str)
            except:
                weight = "Unknown"

            self.get_logger().info('')
            self.get_logger().info(f'--- Object {i+1}/{len(self.detected_objects)} ---')
            self.get_logger().info(f'ID: {obj.id}')
            self.get_logger().info(f'Position: X={center_x:.1f}, Y={center_y:.1f}')
            self.get_logger().info(f'Perceived Weight: {weight}g')
            self.get_logger().info('')

            # Move to position at Z_PICKUP
            self.get_logger().info(f'Moving to position {i+1} at Z_PICKUP...')
            if not self.move_to(center_x, center_y, self.z_pickup):
                self.get_logger().error(f'Failed to move to position {i+1}')
                return False

            self.get_logger().info('')
            self.get_logger().info('*** POSITION CHECK ***')
            self.get_logger().info(f'Robot is at position {i+1} with gripper open at Z_PICKUP.')
            self.get_logger().info('Verify the position is correct for this weight.')

            # Wait for user confirmation
            try:
                input('Press Enter to continue to next position...')
            except:
                pass

        # Return to home
        self.get_logger().info('')
        self.get_logger().info('All positions checked! Returning to Z_HOME...')

        # Use the first object's X/Y position for safety
        if len(self.detected_objects) > 0:
            first_obj = self.detected_objects[0]
            center_x = (first_obj.x_min + first_obj.x_max) / 2.0
            center_y = (first_obj.y_min + first_obj.y_max) / 2.0
            self.move_to(center_x, center_y, self.z_home)

        # Close gripper
        self.get_logger().info('Closing gripper...')
        self.close_gripper()

        self.get_logger().info('')
        self.get_logger().info('=' * 70)
        self.get_logger().info('POSITION CHECK COMPLETE!')
        self.get_logger().info('=' * 70)
        self.get_logger().info('')

        return True


def main(args=None):
    rclpy.init(args=args)

    node = PositionCheckNode()

    try:
        success = node.run_position_check()
        if success:
            node.get_logger().info('Position check completed successfully')
            sys.exit(0)
        else:
            node.get_logger().error('Position check failed')
            sys.exit(1)
    except KeyboardInterrupt:
        node.get_logger().info('Position check interrupted by user')
        sys.exit(1)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
