#!/usr/bin/env python3
"""
Script to spawn objects into the Gazebo simulation
Can spawn objects at random or specified positions
"""

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import SpawnEntity, DeleteEntity
import random
import argparse
import sys
from pathlib import Path


class ObjectSpawner(Node):
    """Node to spawn objects in Gazebo"""

    def __init__(self):
        super().__init__('object_spawner')

        # Service clients
        self.spawn_client = self.create_client(SpawnEntity, '/spawn_entity')
        self.delete_client = self.create_client(DeleteEntity, '/delete_entity')

        # Wait for services
        self.get_logger().info('Waiting for Gazebo spawn service...')
        self.spawn_client.wait_for_service(timeout_sec=10.0)
        self.get_logger().info('Spawn service ready!')

        # Object counter for unique naming
        self.object_counter = 0

    def spawn_object(self, model_type, x=None, y=None, z=0.05):
        """
        Spawn an object in the simulation

        Args:
            model_type: 'bolt_50g', 'part_100g', or 'part_200g'
            x, y, z: Position coordinates (if None, random within workspace)
        """
        # Random position if not specified
        if x is None:
            x = random.uniform(-0.4, 0.4)
        if y is None:
            y = random.uniform(-0.4, 0.4)

        # Load model SDF
        model_path = Path(__file__).parent.parent / 'models' / model_type / 'model.sdf'

        if not model_path.exists():
            self.get_logger().error(f'Model file not found: {model_path}')
            return False

        with open(model_path, 'r') as f:
            model_xml = f.read()

        # Create spawn request
        request = SpawnEntity.Request()
        request.name = f'{model_type}_{self.object_counter}'
        request.xml = model_xml
        request.initial_pose.position.x = x
        request.initial_pose.position.y = y
        request.initial_pose.position.z = z
        request.initial_pose.orientation.w = 1.0

        # Call spawn service
        future = self.spawn_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info(
                f'Spawned {model_type} at ({x:.2f}, {y:.2f}, {z:.2f})'
            )
            self.object_counter += 1
            return True
        else:
            self.get_logger().error(f'Failed to spawn {model_type}')
            return False

    def delete_object(self, name):
        """Delete an object from the simulation"""
        request = DeleteEntity.Request()
        request.name = name

        future = self.delete_client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)

        if future.result() is not None and future.result().success:
            self.get_logger().info(f'Deleted {name}')
            return True
        else:
            self.get_logger().error(f'Failed to delete {name}')
            return False

    def spawn_random_set(self, count=5):
        """Spawn a random set of objects"""
        models = ['bolt_50g', 'part_100g', 'part_200g']

        for _ in range(count):
            model_type = random.choice(models)
            self.spawn_object(model_type)

    def spawn_test_set(self):
        """Spawn a predefined test set with one of each type"""
        self.spawn_object('bolt_50g', x=-0.2, y=0.0, z=0.05)
        self.spawn_object('part_100g', x=0.0, y=0.0, z=0.05)
        self.spawn_object('part_200g', x=0.2, y=0.0, z=0.05)


def main():
    parser = argparse.ArgumentParser(
        description='Spawn objects in Gazebo weight sorting simulation'
    )
    parser.add_argument(
        '--type',
        choices=['bolt_50g', 'part_100g', 'part_200g', 'random', 'test'],
        default='test',
        help='Type of object to spawn (or "test" for test set, "random" for random set)'
    )
    parser.add_argument(
        '--count',
        type=int,
        default=5,
        help='Number of random objects to spawn (only for --type random)'
    )
    parser.add_argument(
        '--x',
        type=float,
        default=None,
        help='X position (random if not specified)'
    )
    parser.add_argument(
        '--y',
        type=float,
        default=None,
        help='Y position (random if not specified)'
    )
    parser.add_argument(
        '--z',
        type=float,
        default=0.05,
        help='Z position (height above table)'
    )

    args = parser.parse_args()

    # Initialize ROS
    rclpy.init()
    spawner = ObjectSpawner()

    try:
        if args.type == 'test':
            spawner.get_logger().info('Spawning test set (one of each type)...')
            spawner.spawn_test_set()
        elif args.type == 'random':
            spawner.get_logger().info(f'Spawning {args.count} random objects...')
            spawner.spawn_random_set(args.count)
        else:
            spawner.get_logger().info(f'Spawning single {args.type}...')
            spawner.spawn_object(args.type, x=args.x, y=args.y, z=args.z)

        spawner.get_logger().info('Done!')

    except Exception as e:
        spawner.get_logger().error(f'Error: {e}')
    finally:
        spawner.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
