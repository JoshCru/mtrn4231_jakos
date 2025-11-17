#!/usr/bin/env python3
"""
Check current TCP position from MoveIt/TF
Compares with UR teach pendant coordinates
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
import tf2_ros
from tf2_ros import TransformException
import math


def quaternion_to_rotation_vector(qx, qy, qz, qw):
    """
    Convert quaternion to UR rotation vector (axis-angle)
    """
    # Normalize quaternion
    norm = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if norm < 1e-10:
        return [0.0, 0.0, 0.0]

    qx, qy, qz, qw = qx/norm, qy/norm, qz/norm, qw/norm

    # Calculate angle
    angle = 2.0 * math.acos(max(-1.0, min(1.0, qw)))

    if angle < 1e-10:
        return [0.0, 0.0, 0.0]

    # Calculate axis
    sin_half = math.sin(angle / 2.0)
    if abs(sin_half) < 1e-10:
        return [0.0, 0.0, 0.0]

    ax = qx / sin_half
    ay = qy / sin_half
    az = qz / sin_half

    # Rotation vector = axis * angle
    rx = ax * angle
    ry = ay * angle
    rz = az * angle

    return [rx, ry, rz]


class TCPPositionChecker(Node):
    def __init__(self):
        super().__init__('tcp_position_checker')

        # TF buffer and listener
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # Joint state subscription
        self.current_joints = {}
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10
        )

        # Frames to check
        self.base_frame = 'base_link'
        self.tcp_frames = ['tool0', 'gripper_tip']  # Check both if available

    def joint_callback(self, msg):
        """Store current joint positions"""
        for i, name in enumerate(msg.name):
            if i < len(msg.position):
                self.current_joints[name] = msg.position[i]

    def get_tcp_pose(self, tcp_frame='tool0'):
        """Get current TCP pose from TF"""
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame,
                tcp_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=2.0)
            )

            # Extract position (convert to mm)
            x_mm = transform.transform.translation.x * 1000.0
            y_mm = transform.transform.translation.y * 1000.0
            z_mm = transform.transform.translation.z * 1000.0

            # Extract orientation as quaternion
            qx = transform.transform.rotation.x
            qy = transform.transform.rotation.y
            qz = transform.transform.rotation.z
            qw = transform.transform.rotation.w

            # Convert to rotation vector
            rx, ry, rz = quaternion_to_rotation_vector(qx, qy, qz, qw)

            return {
                'x': x_mm,
                'y': y_mm,
                'z': z_mm,
                'rx': rx,
                'ry': ry,
                'rz': rz,
                'qx': qx,
                'qy': qy,
                'qz': qz,
                'qw': qw
            }

        except TransformException as ex:
            self.get_logger().warn(f'Could not get transform for {tcp_frame}: {ex}')
            return None

    def print_info(self):
        """Print current TCP position and joint states"""
        print("\n" + "="*60)
        print("CURRENT ROBOT STATE")
        print("="*60)

        # Print joint positions
        if self.current_joints:
            print("\nJoint Positions:")
            joint_order = [
                'shoulder_pan_joint',
                'shoulder_lift_joint',
                'elbow_joint',
                'wrist_1_joint',
                'wrist_2_joint',
                'wrist_3_joint'
            ]
            for name in joint_order:
                if name in self.current_joints:
                    rad = self.current_joints[name]
                    deg = math.degrees(rad)
                    print(f"  {name:25s}: {rad:8.4f} rad ({deg:8.2f} deg)")
        else:
            print("\nNo joint states received yet")

        # Print TCP positions for each frame
        for tcp_frame in self.tcp_frames:
            pose = self.get_tcp_pose(tcp_frame)
            if pose:
                print(f"\n{tcp_frame} Position (in base_link frame):")
                print(f"  Position:")
                print(f"    X: {pose['x']:10.3f} mm")
                print(f"    Y: {pose['y']:10.3f} mm")
                print(f"    Z: {pose['z']:10.3f} mm")
                print(f"  Rotation Vector (UR format):")
                print(f"    RX: {pose['rx']:10.6f} rad")
                print(f"    RY: {pose['ry']:10.6f} rad")
                print(f"    RZ: {pose['rz']:10.6f} rad")
                print(f"  Quaternion:")
                print(f"    x: {pose['qx']:10.6f}")
                print(f"    y: {pose['qy']:10.6f}")
                print(f"    z: {pose['qz']:10.6f}")
                print(f"    w: {pose['qw']:10.6f}")

        print("\n" + "="*60)


def main():
    rclpy.init()
    node = TCPPositionChecker()

    print("Waiting for TF data...")
    rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.spin_once(node, timeout_sec=1.0)
    rclpy.spin_once(node, timeout_sec=1.0)

    node.print_info()

    print("\nPress Ctrl+C to exit, or the script will check again in 3 seconds...")

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=3.0)
            node.print_info()
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
