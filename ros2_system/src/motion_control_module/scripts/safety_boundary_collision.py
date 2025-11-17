#!/usr/bin/env python3
"""
Safety Boundary Collision Objects
Adds safety planes as collision objects to MoveIt planning scene.
This prevents the robot from planning paths that would violate the safety boundaries.
"""

import rclpy
from rclpy.node import Node
from moveit_msgs.msg import PlanningScene, CollisionObject
from shape_msgs.msg import Plane, SolidPrimitive
from geometry_msgs.msg import Pose, Point, Quaternion
from std_msgs.msg import Header
import math


class SafetyBoundaryCollision(Node):
    def __init__(self):
        super().__init__('safety_boundary_collision')

        # Publisher for planning scene
        self.planning_scene_pub = self.create_publisher(
            PlanningScene,
            '/planning_scene',
            10
        )

        # Safety planes configuration
        # Each plane is defined by its position and orientation
        self.safety_objects = []

        # Table plane (z = 0) - large box below z=0
        self.add_box_boundary(
            name="table_boundary",
            center=[0.0, 0.0, -0.25],  # Center at z=-0.25m
            size=[4.0, 4.0, 0.5]  # 4m x 4m x 0.5m thick
        )

        # Back plane (y = -0.3) - large box behind y=-0.3
        self.add_box_boundary(
            name="back_boundary",
            center=[0.0, -0.3 - 1.0, 0.3275],  # Center 1m behind the plane
            size=[4.0, 2.0, 0.655]  # 4m wide, 2m deep, full height
        )

        # Side plane (x = -0.3) - large box to the side of x=-0.3
        self.add_box_boundary(
            name="side_boundary",
            center=[-0.3 - 1.0, 0.0, 0.3275],  # Center 1m to the side
            size=[2.0, 4.0, 0.655]  # 2m wide, 4m long, full height
        )

        # Ceiling plane (z = 0.655) - large box above z=0.655
        self.add_box_boundary(
            name="ceiling_boundary",
            center=[0.0, 0.0, 0.655 + 0.25],  # Center at z=0.905m
            size=[4.0, 4.0, 0.5]  # 4m x 4m x 0.5m thick
        )

        # Wait for planning scene to be ready
        self.get_logger().info('Waiting for planning scene...')
        rclpy.spin_once(self, timeout_sec=2.0)

        # Publish collision objects
        self.publish_collision_objects()

        self.get_logger().info('Safety boundary collision objects added to planning scene')

    def add_box_boundary(self, name, center, size):
        """Add a box collision object to represent a boundary."""
        collision_object = CollisionObject()
        collision_object.header.frame_id = "base_link"
        collision_object.id = name
        collision_object.operation = CollisionObject.ADD

        # Create box primitive
        box = SolidPrimitive()
        box.type = SolidPrimitive.BOX
        box.dimensions = size  # [x_size, y_size, z_size]

        # Set pose
        pose = Pose()
        pose.position.x = center[0]
        pose.position.y = center[1]
        pose.position.z = center[2]
        pose.orientation.w = 1.0

        collision_object.primitives.append(box)
        collision_object.primitive_poses.append(pose)

        self.safety_objects.append(collision_object)

        self.get_logger().info(
            f'Added {name}: center=({center[0]:.2f}, {center[1]:.2f}, {center[2]:.2f}), '
            f'size=({size[0]:.2f}, {size[1]:.2f}, {size[2]:.2f})'
        )

    def publish_collision_objects(self):
        """Publish all collision objects to the planning scene."""
        planning_scene_msg = PlanningScene()
        planning_scene_msg.is_diff = True  # Only update what we specify

        for obj in self.safety_objects:
            obj.header.stamp = self.get_clock().now().to_msg()
            planning_scene_msg.world.collision_objects.append(obj)

        self.planning_scene_pub.publish(planning_scene_msg)
        self.get_logger().info(f'Published {len(self.safety_objects)} collision objects')

    def remove_collision_objects(self):
        """Remove all safety boundary collision objects."""
        planning_scene_msg = PlanningScene()
        planning_scene_msg.is_diff = True

        for obj in self.safety_objects:
            remove_obj = CollisionObject()
            remove_obj.header.frame_id = "base_link"
            remove_obj.header.stamp = self.get_clock().now().to_msg()
            remove_obj.id = obj.id
            remove_obj.operation = CollisionObject.REMOVE
            planning_scene_msg.world.collision_objects.append(remove_obj)

        self.planning_scene_pub.publish(planning_scene_msg)
        self.get_logger().info('Removed safety boundary collision objects')


def main(args=None):
    rclpy.init(args=args)

    node = SafetyBoundaryCollision()

    # Keep node alive to maintain collision objects
    # They persist in the planning scene, but we keep the node for potential updates
    try:
        node.get_logger().info('Safety boundaries active. Press Ctrl+C to remove and exit.')
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Removing safety boundaries...')
        node.remove_collision_objects()
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
