#!/usr/bin/env python3
"""
Simulated Perception Node

Publishes fake detected objects at predefined positions in the picking area
for testing the sorting system without a real camera.

The node publishes a set of "weights" at random positions within the picking area.
Objects can be removed from the list when they are picked up.

Also publishes RViz visualization markers for the weights.
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, ColorRGBA, Int32
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray
from sort_interfaces.msg import DetectedObjects, BoundingBox
import random
import math


class SimulatedPerceptionNode(Node):
    """Simulates perception by publishing fake detected objects."""

    # Picking area bounds from pick_and_place_demo.py
    # These are the coordinates sent to the cartesian controller
    # Controller negates x/y, so robot moves to positive x/y in RViz
    PICKING_AREA = {
        'x_min': -795.0, 'x_max': -415.0,
        'y_min': -252.0, 'y_max': 56.0
    }

    # Placing area bounds (adjacent to picking area)
    PLACING_AREA = {
        'x_min': -795.0, 'x_max': -415.0,
        'y_min': -391.0, 'y_max': -252.0
    }

    # Z height for objects (on the table surface)
    Z_SURFACE = 0.0  # meters (base_link frame)

    # Weight sizes based on mass (mm diameter)
    # 500g = 43mm, 200g = 32mm, 100g = 25mm
    WEIGHT_SIZES = {
        500: 43.0,
        200: 32.0,
        100: 25.0
    }

    # Default object size (mm)
    OBJECT_SIZE = 25.0

    # Minimum distance between objects (mm)
    MIN_SPACING = 70.0

    # Predefined weights to spawn: 2x100g, 1x200g, 1x500g
    PREDEFINED_WEIGHTS = [100, 100, 200, 500]

    def __init__(self):
        super().__init__('simulated_perception_node')

        # Parameters
        self.declare_parameter('num_objects', 5)
        self.declare_parameter('publish_rate', 5.0)
        self.declare_parameter('randomize_positions', True)

        num_objects = self.get_parameter('num_objects').value
        publish_rate = self.get_parameter('publish_rate').value
        randomize = self.get_parameter('randomize_positions').value

        # Publisher for detected objects
        self.objects_pub = self.create_publisher(
            DetectedObjects,
            '/perception/detected_objects',
            10
        )

        # Publisher for RViz visualization
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/perception/weight_markers',
            10
        )

        # Publisher for zone visualization
        self.zone_marker_pub = self.create_publisher(
            MarkerArray,
            '/perception/zone_markers',
            10
        )

        # Publisher for weight estimates (when object is picked)
        self.weight_pub = self.create_publisher(
            Int32,
            '/recognition/estimated_mass',
            10
        )

        # Store objects with their simulated weights
        self.objects: list[BoundingBox] = []
        self.object_weights: dict[int, float] = {}  # object_id -> weight in grams
        self.next_id = 1

        self.generate_objects(num_objects, randomize)

        # Subscribe to remove objects when picked
        self.remove_sub = self.create_subscription(
            BoundingBox,
            '/perception/remove_object',
            self.remove_object_callback,
            10
        )

        # Timer for publishing
        self.timer = self.create_timer(1.0 / publish_rate, self.publish_objects)

        # Publish zone markers once at startup (and periodically)
        self.zone_timer = self.create_timer(2.0, self.publish_zone_markers)

        self.get_logger().info(f'Simulated Perception Node started with {num_objects} objects')
        self.get_logger().info(f'Publishing to /perception/detected_objects at {publish_rate} Hz')
        self.get_logger().info(f'RViz markers on /perception/weight_markers and /perception/zone_markers')

    def generate_objects(self, num_objects: int, randomize: bool = True):
        """Generate simulated objects in the picking area with no overlap."""
        self.objects = []
        self.object_weights = {}

        # Use predefined weights (2x100g, 1x200g, 1x500g)
        weights_to_place = self.PREDEFINED_WEIGHTS[:num_objects]

        if randomize:
            # Shuffle the weights so they appear in random order
            random.shuffle(weights_to_place)

            # Generate random positions with collision avoidance
            max_attempts = 100
            placed_positions = []

            for i, weight_grams in enumerate(weights_to_place):
                for attempt in range(max_attempts):
                    x = random.uniform(
                        self.PICKING_AREA['x_min'] + self.OBJECT_SIZE + 20,
                        self.PICKING_AREA['x_max'] - self.OBJECT_SIZE - 20
                    )
                    y = random.uniform(
                        self.PICKING_AREA['y_min'] + self.OBJECT_SIZE + 20,
                        self.PICKING_AREA['y_max'] - self.OBJECT_SIZE - 20
                    )

                    # Check for overlap with existing objects
                    is_valid = True
                    for px, py in placed_positions:
                        dist = math.sqrt((x - px) ** 2 + (y - py) ** 2)
                        if dist < self.MIN_SPACING:
                            is_valid = False
                            break

                    if is_valid:
                        placed_positions.append((x, y))
                        self.add_object(x, y, weight_grams)
                        break
                else:
                    self.get_logger().warn(f'Could not place object {i+1} without overlap after {max_attempts} attempts')
        else:
            # Generate grid positions (guaranteed no overlap)
            num_cols = 3
            num_rows = (len(weights_to_place) + num_cols - 1) // num_cols

            x_spacing = (self.PICKING_AREA['x_max'] - self.PICKING_AREA['x_min'] - 100) / max(num_cols - 1, 1)
            y_spacing = (self.PICKING_AREA['y_max'] - self.PICKING_AREA['y_min'] - 100) / max(num_rows - 1, 1)

            for i, weight_grams in enumerate(weights_to_place):
                col = i % num_cols
                row = i // num_cols

                x = self.PICKING_AREA['x_min'] + 50 + col * x_spacing
                y = self.PICKING_AREA['y_min'] + 50 + row * y_spacing

                self.add_object(x, y, weight_grams)

        self.get_logger().info(f'Generated {len(self.objects)} simulated objects')

    def get_weight_size(self, weight_grams: float) -> float:
        """Get the diameter (mm) for a given weight."""
        # Find the closest predefined weight
        if weight_grams >= 400:
            return self.WEIGHT_SIZES[500]  # 43mm
        elif weight_grams >= 150:
            return self.WEIGHT_SIZES[200]  # 32mm
        else:
            return self.WEIGHT_SIZES[100]  # 25mm

    def add_object(self, center_x: float, center_y: float, weight_grams: float = None):
        """Add a simulated object at the given position with specified weight."""
        obj = BoundingBox()
        obj.id = self.next_id
        self.next_id += 1

        # Use weight-based size
        if weight_grams is None:
            weight_grams = random.choice([100, 200, 500])

        size = self.get_weight_size(weight_grams)
        half_size = size / 2.0

        obj.x_min = center_x - half_size
        obj.x_max = center_x + half_size
        obj.y_min = center_y - half_size
        obj.y_max = center_y + half_size
        obj.confidence = 0.95 + random.uniform(-0.05, 0.05)
        obj.class_name = f'{weight_grams}g'  # Include weight in class name for gripper control

        # Store the weight
        self.object_weights[obj.id] = float(weight_grams)

        self.objects.append(obj)
        self.get_logger().info(f'Added object {obj.id} at ({center_x:.1f}, {center_y:.1f}) - {weight_grams:.0f}g ({size:.0f}mm)')

    def get_object_weight(self, object_id: int) -> float:
        """Get the simulated weight for an object."""
        return self.object_weights.get(object_id, 100.0)

    def remove_object_callback(self, msg: BoundingBox):
        """Remove an object when it's picked up and publish its weight."""
        initial_count = len(self.objects)
        self.objects = [o for o in self.objects if o.id != msg.id]

        if len(self.objects) < initial_count:
            weight = self.object_weights.get(msg.id, 100.0)
            self.get_logger().info(f'Removed object {msg.id} ({weight:.0f}g), {len(self.objects)} remaining')

            # Publish the weight estimate for this object (matching Asad's format)
            weight_msg = Int32()
            weight_msg.data = int(weight)
            self.weight_pub.publish(weight_msg)
            self.get_logger().info(f'Published weight estimate: {weight:.0f}g for object {msg.id}')
        else:
            self.get_logger().warn(f'Object {msg.id} not found for removal')

    def publish_objects(self):
        """Publish the current list of detected objects."""
        msg = DetectedObjects()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.objects = self.objects

        self.objects_pub.publish(msg)

        # Also publish visualization markers
        self.publish_weight_markers()

    def publish_weight_markers(self):
        """Publish RViz markers for the simulated weights."""
        marker_array = MarkerArray()

        for i, obj in enumerate(self.objects):
            # Get the weight and corresponding size
            weight = self.object_weights.get(obj.id, 100.0)
            size_mm = self.get_weight_size(weight)

            # Cylinder marker for each weight
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'weights'
            marker.id = obj.id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            # Position (convert mm to meters, negate to show where robot actually goes)
            # Controller negates x/y, so we negate here to show the RViz position
            center_x = (obj.x_min + obj.x_max) / 2.0 / 1000.0
            center_y = (obj.y_min + obj.y_max) / 2.0 / 1000.0
            marker.pose.position.x = -center_x
            marker.pose.position.y = -center_y
            marker.pose.position.z = 0.015  # 15mm above surface

            marker.pose.orientation.w = 1.0

            # Size based on weight (cylinder: diameter x diameter x height)
            marker.scale.x = size_mm / 1000.0  # diameter
            marker.scale.y = size_mm / 1000.0  # diameter
            marker.scale.z = 0.03  # 30mm height

            # Color: gold/brass color for weights
            marker.color.r = 0.85
            marker.color.g = 0.65
            marker.color.b = 0.13
            marker.color.a = 1.0

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 500000000  # 0.5 seconds

            marker_array.markers.append(marker)

            # Add text label above weight showing the weight value
            text_marker = Marker()
            text_marker.header.frame_id = 'base_link'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'weight_labels'
            text_marker.id = obj.id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = -center_x
            text_marker.pose.position.y = -center_y
            text_marker.pose.position.z = 0.05  # 50mm above surface

            text_marker.scale.z = 0.02  # Text height

            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            # Show actual simulated weight
            weight = self.object_weights.get(obj.id, 0)
            text_marker.text = f"{weight:.0f}g"

            text_marker.lifetime.sec = 0
            text_marker.lifetime.nanosec = 500000000

            marker_array.markers.append(text_marker)

        self.marker_pub.publish(marker_array)

    def publish_zone_markers(self):
        """Publish RViz markers for picking and placing zones."""
        marker_array = MarkerArray()

        # Picking zone (green)
        pick_marker = Marker()
        pick_marker.header.frame_id = 'base_link'
        pick_marker.header.stamp = self.get_clock().now().to_msg()
        pick_marker.ns = 'zones'
        pick_marker.id = 1
        pick_marker.type = Marker.LINE_STRIP
        pick_marker.action = Marker.ADD

        # Convert mm to meters and negate to show RViz position
        px_min = -self.PICKING_AREA['x_min'] / 1000.0
        px_max = -self.PICKING_AREA['x_max'] / 1000.0
        py_min = -self.PICKING_AREA['y_min'] / 1000.0
        py_max = -self.PICKING_AREA['y_max'] / 1000.0

        # Rectangle corners (swap min/max since we negated)
        pick_marker.points = [
            Point(x=px_max, y=py_max, z=0.001),
            Point(x=px_min, y=py_max, z=0.001),
            Point(x=px_min, y=py_min, z=0.001),
            Point(x=px_max, y=py_min, z=0.001),
            Point(x=px_max, y=py_max, z=0.001),  # Close the loop
        ]

        pick_marker.scale.x = 0.005  # Line width
        pick_marker.color.r = 0.0
        pick_marker.color.g = 1.0
        pick_marker.color.b = 0.0
        pick_marker.color.a = 0.8

        marker_array.markers.append(pick_marker)

        # Picking zone label
        pick_label = Marker()
        pick_label.header.frame_id = 'base_link'
        pick_label.header.stamp = self.get_clock().now().to_msg()
        pick_label.ns = 'zone_labels'
        pick_label.id = 10
        pick_label.type = Marker.TEXT_VIEW_FACING
        pick_label.action = Marker.ADD

        pick_label.pose.position.x = (px_min + px_max) / 2
        pick_label.pose.position.y = py_max + 0.02
        pick_label.pose.position.z = 0.01

        pick_label.scale.z = 0.03
        pick_label.color.r = 0.0
        pick_label.color.g = 1.0
        pick_label.color.b = 0.0
        pick_label.color.a = 1.0
        pick_label.text = "PICKING ZONE"

        marker_array.markers.append(pick_label)

        # Placing zone (blue)
        place_marker = Marker()
        place_marker.header.frame_id = 'base_link'
        place_marker.header.stamp = self.get_clock().now().to_msg()
        place_marker.ns = 'zones'
        place_marker.id = 2
        place_marker.type = Marker.LINE_STRIP
        place_marker.action = Marker.ADD

        # Convert mm to meters and negate to show RViz position
        plx_min = -self.PLACING_AREA['x_min'] / 1000.0
        plx_max = -self.PLACING_AREA['x_max'] / 1000.0
        ply_min = -self.PLACING_AREA['y_min'] / 1000.0
        ply_max = -self.PLACING_AREA['y_max'] / 1000.0

        place_marker.points = [
            Point(x=plx_max, y=ply_max, z=0.001),
            Point(x=plx_min, y=ply_max, z=0.001),
            Point(x=plx_min, y=ply_min, z=0.001),
            Point(x=plx_max, y=ply_min, z=0.001),
            Point(x=plx_max, y=ply_max, z=0.001),
        ]

        place_marker.scale.x = 0.005
        place_marker.color.r = 0.0
        place_marker.color.g = 0.5
        place_marker.color.b = 1.0
        place_marker.color.a = 0.8

        marker_array.markers.append(place_marker)

        # Placing zone label
        place_label = Marker()
        place_label.header.frame_id = 'base_link'
        place_label.header.stamp = self.get_clock().now().to_msg()
        place_label.ns = 'zone_labels'
        place_label.id = 11
        place_label.type = Marker.TEXT_VIEW_FACING
        place_label.action = Marker.ADD

        place_label.pose.position.x = (plx_min + plx_max) / 2
        place_label.pose.position.y = ply_min - 0.02
        place_label.pose.position.z = 0.01

        place_label.scale.z = 0.03
        place_label.color.r = 0.0
        place_label.color.g = 0.5
        place_label.color.b = 1.0
        place_label.color.a = 1.0
        place_label.text = "PLACING ZONE"

        marker_array.markers.append(place_label)

        self.zone_marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    node = SimulatedPerceptionNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
