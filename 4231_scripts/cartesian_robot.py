#!/usr/bin/env python3
"""
Cartesian Robot Controller - Following UR Robot Driver Pattern

Provides a clean interface for Cartesian movements with safety boundary checking.
Based on the ur_robot_driver example.py structure.

Safety Boundaries (relative to base_link):
  - x >= -0.3m (-300mm)
  - y >= -0.3m (-300mm)
  - 0m <= z <= 0.655m (0mm to 655mm)

Usage:
    from cartesian_robot import CartesianRobot
    robot = CartesianRobot(node)
    robot.move_to_cartesian(x=100, y=400, z=300, rx=-1.571, ry=0.0, rz=0.0)
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from moveit_msgs.srv import GetCartesianPath
from control_msgs.action import FollowJointTrajectory
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
from builtin_interfaces.msg import Duration
import math

TIMEOUT_WAIT_SERVICE = 10
TIMEOUT_WAIT_SERVICE_INITIAL = 60
TIMEOUT_WAIT_ACTION = 10

ROBOT_JOINTS = [
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint",
]

# Safety boundaries (in meters, relative to base_link)
SAFETY_BOUNDS = {
    'x_min': -0.3,
    'x_max': 2.0,  # Reasonable upper limit
    'y_min': -0.3,
    'y_max': 2.0,
    'z_min': 0.0,
    'z_max': 0.655,
}


# Helper functions (following UR driver pattern)
def waitForService(node, srv_name, srv_type, timeout=TIMEOUT_WAIT_SERVICE):
    """Wait for a ROS2 service to become available."""
    client = node.create_client(srv_type, srv_name)
    if client.wait_for_service(timeout) is False:
        raise Exception(f"Could not reach service '{srv_name}' within timeout of {timeout}")

    node.get_logger().info(f"Successfully connected to service '{srv_name}'")
    return client


def waitForAction(node, action_name, action_type, timeout=TIMEOUT_WAIT_ACTION):
    """Wait for a ROS2 action server to become available."""
    client = ActionClient(node, action_type, action_name)
    if client.wait_for_server(timeout) is False:
        raise Exception(
            f"Could not reach action server '{action_name}' within timeout of {timeout}"
        )

    node.get_logger().info(f"Successfully connected to action '{action_name}'")
    return client


def rotation_vector_to_quaternion(rx, ry, rz):
    """Convert UR rotation vector (axis-angle) to quaternion."""
    angle = math.sqrt(rx*rx + ry*ry + rz*rz)
    if angle < 1e-10:
        return [0.0, 0.0, 0.0, 1.0]

    axis = [rx/angle, ry/angle, rz/angle]
    half_angle = angle / 2.0
    sin_half = math.sin(half_angle)
    cos_half = math.cos(half_angle)

    return [axis[0]*sin_half, axis[1]*sin_half, axis[2]*sin_half, cos_half]


class CartesianRobot:
    """
    Cartesian Robot Controller

    Provides high-level Cartesian movement interface with safety checking.
    Following the pattern from ur_robot_driver/example.py
    """

    def __init__(self, node, use_fake_hardware=True):
        """
        Initialize the Cartesian Robot controller.

        Args:
            node: ROS2 Node instance
            use_fake_hardware: True for fake hardware, False for real robot
        """
        self.node = node
        self.use_fake = use_fake_hardware
        self.current_joint_state = None

        # Select appropriate controller
        if use_fake_hardware:
            controller_name = '/joint_trajectory_controller/follow_joint_trajectory'
        else:
            controller_name = '/scaled_joint_trajectory_controller/follow_joint_trajectory'

        self.controller_name = controller_name

        self.service_interfaces = {
            "/compute_cartesian_path": GetCartesianPath,
        }

        self.init_robot()

    def init_robot(self):
        """Initialize robot connections (services, actions, topics)."""
        # Subscribe to joint states
        from rclpy.qos import QoSProfile, QoSDurabilityPolicy
        qos = QoSProfile(depth=10)
        qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL

        self.node.create_subscription(
            JointState,
            '/joint_states',
            self._joint_state_callback,
            qos
        )

        # Wait for joint states
        self.node.get_logger().info('Waiting for joint states...')
        timeout = 10.0
        start = self.node.get_clock().now()
        while self.current_joint_state is None:
            rclpy.spin_once(self.node, timeout_sec=0.1)
            elapsed = (self.node.get_clock().now() - start).nanoseconds / 1e9
            if elapsed > timeout:
                raise Exception("Joint states not received within timeout")

        self.node.get_logger().info('Joint states received!')

        # Initialize service clients
        self.service_clients = {
            srv_name: waitForService(self.node, srv_name, srv_type, TIMEOUT_WAIT_SERVICE_INITIAL)
            for (srv_name, srv_type) in self.service_interfaces.items()
        }

        # Initialize action client for trajectory execution
        self.trajectory_action_client = waitForAction(
            self.node,
            self.controller_name,
            FollowJointTrajectory,
            TIMEOUT_WAIT_ACTION
        )

        self.node.get_logger().info(f"Cartesian Robot initialized with controller: {self.controller_name}")

    def _joint_state_callback(self, msg):
        """Callback for joint state updates."""
        self.current_joint_state = msg

    def check_boundaries(self, x, y, z):
        """
        Check if position is within safety boundaries.

        Args:
            x, y, z: Position in millimeters

        Returns:
            tuple: (is_safe, error_message)
        """
        # Convert mm to meters for checking
        x_m, y_m, z_m = x/1000.0, y/1000.0, z/1000.0

        if x_m < SAFETY_BOUNDS['x_min']:
            return False, f"x={x}mm ({x_m:.3f}m) is less than minimum {SAFETY_BOUNDS['x_min']}m"
        if x_m > SAFETY_BOUNDS['x_max']:
            return False, f"x={x}mm ({x_m:.3f}m) exceeds maximum {SAFETY_BOUNDS['x_max']}m"
        if y_m < SAFETY_BOUNDS['y_min']:
            return False, f"y={y}mm ({y_m:.3f}m) is less than minimum {SAFETY_BOUNDS['y_min']}m"
        if y_m > SAFETY_BOUNDS['y_max']:
            return False, f"y={y}mm ({y_m:.3f}m) exceeds maximum {SAFETY_BOUNDS['y_max']}m"
        if z_m < SAFETY_BOUNDS['z_min']:
            return False, f"z={z}mm ({z_m:.3f}m) is less than minimum {SAFETY_BOUNDS['z_min']}m"
        if z_m > SAFETY_BOUNDS['z_max']:
            return False, f"z={z}mm ({z_m:.3f}m) exceeds maximum {SAFETY_BOUNDS['z_max']}m"

        return True, "Position within safety boundaries"

    def move_to_cartesian(self, x, y, z, rx, ry, rz, check_bounds=True):
        """
        Move to a Cartesian position.

        Args:
            x, y, z: Position in millimeters (relative to base_link)
            rx, ry, rz: Rotation vector in radians (UR axis-angle format)
            check_bounds: If True, verify position is within safety boundaries

        Returns:
            bool: True if movement successful, False otherwise

        Example:
            # Move gripper facing down to (100mm, 400mm, 300mm)
            robot.move_to_cartesian(100, 400, 300, -1.571, 0.0, 0.0)
        """
        # Safety boundary check
        if check_bounds:
            is_safe, msg = self.check_boundaries(x, y, z)
            if not is_safe:
                self.node.get_logger().error(f"SAFETY VIOLATION: {msg}")
                raise Exception(f"Position outside safety boundaries: {msg}")

        self.node.get_logger().info(f"Moving to: x={x}mm, y={y}mm, z={z}mm, rx={rx}rad, ry={ry}rad, rz={rz}rad")

        # Convert mm to meters
        x_m, y_m, z_m = x/1000.0, y/1000.0, z/1000.0
        quat = rotation_vector_to_quaternion(rx, ry, rz)

        self.node.get_logger().info(f"Target (meters): x={x_m:.4f}, y={y_m:.4f}, z={z_m:.4f}")
        self.node.get_logger().info(f"Quaternion: [{quat[0]:.4f}, {quat[1]:.4f}, {quat[2]:.4f}, {quat[3]:.4f}]")

        # Create Cartesian path request
        req = GetCartesianPath.Request()
        req.header.frame_id = 'base_link'
        req.header.stamp = self.node.get_clock().now().to_msg()
        req.start_state.joint_state = self.current_joint_state
        req.group_name = 'ur_manipulator'
        req.link_name = 'gripper_tip'

        pose = Pose()
        pose.position.x, pose.position.y, pose.position.z = x_m, y_m, z_m
        pose.orientation.x, pose.orientation.y, pose.orientation.z, pose.orientation.w = quat
        req.waypoints = [pose]
        req.max_step = 0.01  # 1cm interpolation step
        req.jump_threshold = 0.0  # No jump allowed
        req.avoid_collisions = True  # Enable collision checking (respects safety boundaries)

        # Plan path using MoveIt
        self.node.get_logger().info('Planning Cartesian path...')
        path_result = self.call_service("/compute_cartesian_path", req)

        if path_result is None or path_result.fraction < 0.95:
            fraction = path_result.fraction * 100 if path_result else 0
            error_msg = f"Only {fraction:.1f}% of path could be computed"
            if fraction < 1.0:
                error_msg += " - likely collision with safety boundaries!"
            self.node.get_logger().error(error_msg)
            raise Exception(error_msg)

        self.node.get_logger().info(f'Path computed: {path_result.fraction*100:.1f}%')
        self.node.get_logger().info(f'Trajectory has {len(path_result.solution.joint_trajectory.points)} points')

        # Execute trajectory using FollowJointTrajectory
        trajectory_goal = FollowJointTrajectory.Goal()
        trajectory_goal.trajectory = path_result.solution.joint_trajectory

        # Ensure joint names are set
        if not trajectory_goal.trajectory.joint_names:
            trajectory_goal.trajectory.joint_names = ROBOT_JOINTS

        self.node.get_logger().info(f'Executing trajectory with {len(trajectory_goal.trajectory.points)} points...')

        # Send goal and wait for result
        goal_response = self.call_action(self.trajectory_action_client, trajectory_goal)
        if not goal_response.accepted:
            raise Exception("Trajectory goal was not accepted by controller")

        self.node.get_logger().info('Trajectory accepted, executing...')

        # Wait for result
        result = self.get_result(self.trajectory_action_client, goal_response)

        if result.error_code == FollowJointTrajectory.Result.SUCCESSFUL:
            self.node.get_logger().info('Movement completed successfully!')
            return True
        else:
            self.node.get_logger().error(f'Movement failed: {result.error_string} (code {result.error_code})')
            return False

    def call_service(self, srv_name, request):
        """Call a service synchronously (following UR driver pattern)."""
        future = self.service_clients[srv_name].call_async(request)
        rclpy.spin_until_future_complete(self.node, future)
        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling service: {future.exception()}")

    def call_action(self, ac_client, goal):
        """Call an action synchronously (following UR driver pattern)."""
        future = ac_client.send_goal_async(goal)
        rclpy.spin_until_future_complete(self.node, future)

        if future.result() is not None:
            return future.result()
        else:
            raise Exception(f"Exception while calling action: {future.exception()}")

    def get_result(self, ac_client, goal_response):
        """Get action result synchronously (following UR driver pattern)."""
        future_res = ac_client._get_result_async(goal_response)
        rclpy.spin_until_future_complete(self.node, future_res)
        if future_res.result() is not None:
            return future_res.result().result
        else:
            raise Exception(f"Exception while getting result: {future_res.exception()}")


# Example usage
if __name__ == "__main__":
    rclpy.init()
    node = Node("cartesian_robot_example")

    # Initialize robot (use_fake_hardware=True for simulation, False for real robot)
    robot = CartesianRobot(node, use_fake_hardware=True)

    # Example movements (positions in mm, rotations in radians)
    # Gripper facing down: rx=-1.571 (approximately -π/2)

    try:
        # Move to position 1
        node.get_logger().info("=== Moving to position 1 ===")
        robot.move_to_cartesian(x=100, y=400, z=300, rx=-1.571, ry=0.0, rz=0.0)

        # Move to position 2
        node.get_logger().info("=== Moving to position 2 ===")
        robot.move_to_cartesian(x=200, y=300, z=400, rx=-1.571, ry=0.0, rz=0.0)

        # Move back to position 1
        node.get_logger().info("=== Moving back to position 1 ===")
        robot.move_to_cartesian(x=100, y=400, z=300, rx=-1.571, ry=0.0, rz=0.0)

        node.get_logger().info("=== All movements completed successfully! ===")

    except Exception as e:
        node.get_logger().error(f"Movement failed: {e}")

    finally:
        node.destroy_node()
        rclpy.shutdown()
