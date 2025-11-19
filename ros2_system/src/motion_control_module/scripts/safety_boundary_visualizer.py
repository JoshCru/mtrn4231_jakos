#!/usr/bin/env python3
"""
Safety Boundary Visualizer
Displays safety planes in RViz based on UR robot safety configuration.

Planes in Hessian normal form: [normal.x, normal.y, normal.z, distance_from_origin]
"""

import rclpy
from rclpy.node import Node
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Point
from std_msgs.msg import ColorRGBA
import math


class SafetyBoundaryVisualizer(Node):
    def __init__(self):
        super().__init__('safety_boundary_visualizer')

        # Publisher for visualization markers
        self.marker_pub = self.create_publisher(
            MarkerArray,
            '/safety_boundaries',
            10
        )

        # Safety planes configuration (Hessian normal form)
        # [normal_x, normal_y, normal_z, distance_from_origin]
        self.safety_planes = {
            'Table': {
                'normal': [0.0, 0.0, 1.0],
                'distance': 0.0,
                'color': [0.5, 0.5, 0.5, 0.3],  # Gray, semi-transparent
                'restricts_elbow': True
            },
            'Back_Plane': {
                'normal': [0.0, -1.0, 0.0],
                'distance': 0.3,
                'color': [1.0, 0.0, 0.0, 0.3],  # Red, semi-transparent
                'restricts_elbow': True
            },
            'Side_Plane': {
                'normal': [-1.0, 0.0, 0.0],
                'distance': 0.3,
                'color': [0.0, 0.0, 1.0, 0.3],  # Blue, semi-transparent
                'restricts_elbow': True
            },
            'Ceiling': {
                'normal': [0.0, 0.0, -1.0],  # Points downward
                # 'distance': -0.655,  # z = 0.655m
                'distance': -2.655,  # z = 0.655m
                'color': [1.0, 1.0, 0.0, 0.3],  # Yellow, semi-transparent
                'restricts_elbow': True
            }
        }

        # Plane visualization size (meters)
        self.plane_size = 2.0  # 2m x 2m planes

        # Timer to publish markers periodically
        self.timer = self.create_timer(1.0, self.publish_markers)

        self.get_logger().info('Safety Boundary Visualizer initialized')
        self.get_logger().info('Publishing safety planes:')
        for name, config in self.safety_planes.items():
            n = config['normal']
            d = config['distance']
            self.get_logger().info(f'  {name}: normal=[{n[0]}, {n[1]}, {n[2]}], dist={d}m')

    def create_plane_marker(self, name, plane_config, marker_id):
        """Create a plane marker for RViz visualization."""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "safety_boundaries"
        marker.id = marker_id
        marker.type = Marker.TRIANGLE_LIST
        marker.action = Marker.ADD

        normal = plane_config['normal']
        distance = plane_config['distance']
        color = plane_config['color']

        # Calculate plane center point
        # For plane equation: n·p = d, a point on the plane is p = d*n (if n is unit vector)
        norm_magnitude = math.sqrt(sum(n**2 for n in normal))
        if norm_magnitude > 0:
            unit_normal = [n / norm_magnitude for n in normal]
        else:
            unit_normal = [0.0, 0.0, 1.0]

        # Point on plane closest to origin
        center = [distance * unit_normal[i] for i in range(3)]

        # Create two perpendicular vectors in the plane
        # Find a vector not parallel to normal
        if abs(unit_normal[2]) < 0.9:
            up = [0.0, 0.0, 1.0]
        else:
            up = [1.0, 0.0, 0.0]

        # First perpendicular vector (cross product)
        v1 = [
            up[1] * unit_normal[2] - up[2] * unit_normal[1],
            up[2] * unit_normal[0] - up[0] * unit_normal[2],
            up[0] * unit_normal[1] - up[1] * unit_normal[0]
        ]
        v1_mag = math.sqrt(sum(x**2 for x in v1))
        v1 = [x / v1_mag for x in v1]

        # Second perpendicular vector
        v2 = [
            unit_normal[1] * v1[2] - unit_normal[2] * v1[1],
            unit_normal[2] * v1[0] - unit_normal[0] * v1[2],
            unit_normal[0] * v1[1] - unit_normal[1] * v1[0]
        ]

        # Create rectangle corners
        half_size = self.plane_size / 2.0
        corners = []
        for s1, s2 in [(-1, -1), (1, -1), (1, 1), (-1, 1)]:
            corner = Point()
            corner.x = center[0] + s1 * half_size * v1[0] + s2 * half_size * v2[0]
            corner.y = center[1] + s1 * half_size * v1[1] + s2 * half_size * v2[1]
            corner.z = center[2] + s1 * half_size * v1[2] + s2 * half_size * v2[2]
            corners.append(corner)

        # Create two triangles from the rectangle
        # Triangle 1: corners 0, 1, 2
        marker.points.append(corners[0])
        marker.points.append(corners[1])
        marker.points.append(corners[2])

        # Triangle 2: corners 0, 2, 3
        marker.points.append(corners[0])
        marker.points.append(corners[2])
        marker.points.append(corners[3])

        # Set color
        marker.color.r = color[0]
        marker.color.g = color[1]
        marker.color.b = color[2]
        marker.color.a = color[3]

        # Scale (not used for TRIANGLE_LIST, but set anyway)
        marker.scale.x = 1.0
        marker.scale.y = 1.0
        marker.scale.z = 1.0

        marker.pose.orientation.w = 1.0

        return marker

    def create_text_marker(self, name, plane_config, marker_id):
        """Create a text label for the plane."""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "safety_boundary_labels"
        marker.id = marker_id
        marker.type = Marker.TEXT_VIEW_FACING
        marker.action = Marker.ADD

        normal = plane_config['normal']
        distance = plane_config['distance']

        # Calculate position for text (slightly offset from plane)
        norm_magnitude = math.sqrt(sum(n**2 for n in normal))
        if norm_magnitude > 0:
            unit_normal = [n / norm_magnitude for n in normal]
        else:
            unit_normal = [0.0, 0.0, 1.0]

        # Position text on the plane
        marker.pose.position.x = distance * unit_normal[0]
        marker.pose.position.y = distance * unit_normal[1]
        marker.pose.position.z = distance * unit_normal[2]

        # Offset text slightly toward viewer
        offset = 0.05  # 5cm offset
        marker.pose.position.x -= offset * unit_normal[0]
        marker.pose.position.y -= offset * unit_normal[1]
        marker.pose.position.z -= offset * unit_normal[2]

        marker.pose.orientation.w = 1.0

        marker.text = name
        marker.scale.z = 0.1  # Text height

        # White text
        marker.color.r = 1.0
        marker.color.g = 1.0
        marker.color.b = 1.0
        marker.color.a = 1.0

        return marker

    def create_boundary_box_marker(self):
        """Create a wireframe box showing the safe workspace boundaries."""
        marker = Marker()
        marker.header.frame_id = "base_link"
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = "safety_boundary_box"
        marker.id = 100
        marker.type = Marker.LINE_LIST
        marker.action = Marker.ADD

        # Workspace boundaries based on planes:
        # Table: z >= 0
        # Ceiling: z <= 0.655
        # Back_Plane: y >= -0.3
        # Side_Plane: x >= -0.3
        # Front/Right boundaries: extend to +2m for visualization

        x_min, x_max = -0.3, 1.5
        y_min, y_max = -0.3, 1.5
        z_min, z_max = 0.0, 0.655

        # Define 8 corners of the box
        corners = [
            Point(x=x_min, y=y_min, z=z_min),  # 0
            Point(x=x_max, y=y_min, z=z_min),  # 1
            Point(x=x_max, y=y_max, z=z_min),  # 2
            Point(x=x_min, y=y_max, z=z_min),  # 3
            Point(x=x_min, y=y_min, z=z_max),  # 4
            Point(x=x_max, y=y_min, z=z_max),  # 5
            Point(x=x_max, y=y_max, z=z_max),  # 6
            Point(x=x_min, y=y_max, z=z_max),  # 7
        ]

        # Define edges (pairs of corner indices)
        edges = [
            (0, 1), (1, 2), (2, 3), (3, 0),  # Bottom face
            (4, 5), (5, 6), (6, 7), (7, 4),  # Top face
            (0, 4), (1, 5), (2, 6), (3, 7)   # Vertical edges
        ]

        for start_idx, end_idx in edges:
            marker.points.append(corners[start_idx])
            marker.points.append(corners[end_idx])

        marker.scale.x = 0.005  # Line width

        # Orange color for boundary box
        marker.color.r = 1.0
        marker.color.g = 0.5
        marker.color.b = 0.0
        marker.color.a = 0.8

        marker.pose.orientation.w = 1.0

        return marker

    def publish_markers(self):
        """Publish all safety boundary markers."""
        marker_array = MarkerArray()

        marker_id = 0

        # Add plane markers
        for name, config in self.safety_planes.items():
            # Plane surface
            plane_marker = self.create_plane_marker(name, config, marker_id)
            marker_array.markers.append(plane_marker)
            marker_id += 1

            # Text label
            text_marker = self.create_text_marker(name, config, marker_id)
            marker_array.markers.append(text_marker)
            marker_id += 1

        # Add boundary box
        box_marker = self.create_boundary_box_marker()
        marker_array.markers.append(box_marker)

        self.marker_pub.publish(marker_array)


def main(args=None):
    rclpy.init(args=args)

    node = SafetyBoundaryVisualizer()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
