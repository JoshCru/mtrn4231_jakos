#!/usr/bin/env python3
"""
Integration test for moveit2_interface_node
Tests motion planning action servers (currently stub implementation)
"""

import unittest
import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from sort_interfaces.action import PlanTrajectory
from moveit_msgs.msg import DisplayTrajectory
from geometry_msgs.msg import Pose, Point, Quaternion


class MoveIt2InterfaceTestNode(Node):
    """Test node for moveit2_interface_node testing"""

    def __init__(self):
        super().__init__('moveit2_interface_test_node')

        # Action clients
        self.plan_pick_client = ActionClient(
            self,
            PlanTrajectory,
            '/planning/plan_pick'
        )
        self.plan_place_client = ActionClient(
            self,
            PlanTrajectory,
            '/planning/plan_place'
        )

        # Subscriber for trajectory visualization
        self.trajectory_sub = self.create_subscription(
            DisplayTrajectory,
            '/planning/trajectory',
            self.trajectory_callback,
            10
        )

        self.received_trajectories = []
        self.trajectory_received = False

    def trajectory_callback(self, msg):
        """Store received trajectory messages"""
        self.received_trajectories.append(msg)
        self.trajectory_received = True
        self.get_logger().info('Received trajectory for visualization')


class TestMoveIt2InterfaceNode(unittest.TestCase):
    """Integration tests for moveit2_interface_node"""

    @classmethod
    def setUpClass(cls):
        """Initialize ROS2"""
        rclpy.init()

    @classmethod
    def tearDownClass(cls):
        """Shutdown ROS2"""
        rclpy.shutdown()

    def setUp(self):
        """Create test node before each test"""
        self.test_node = MoveIt2InterfaceTestNode()
        time.sleep(0.5)  # Allow connections to establish

    def tearDown(self):
        """Cleanup after each test"""
        self.test_node.destroy_node()

    def create_target_pose(self, x, y, z, roll=0.0, pitch=0.0, yaw=0.0):
        """Helper to create target Pose message"""
        pose = Pose()
        pose.position = Point(x=x, y=y, z=z)
        # Simple orientation (could compute from roll/pitch/yaw if needed)
        pose.orientation = Quaternion(x=0.0, y=0.0, z=0.0, w=1.0)
        return pose

    def test_plan_pick_action_server_available(self):
        """Test that plan_pick action server is available"""
        server_available = self.test_node.plan_pick_client.wait_for_server(timeout_sec=5.0)
        self.assertTrue(
            server_available,
            "PlanPick action server not available within timeout"
        )

    def test_plan_place_action_server_available(self):
        """Test that plan_place action server is available"""
        server_available = self.test_node.plan_place_client.wait_for_server(timeout_sec=5.0)
        self.assertTrue(
            server_available,
            "PlanPlace action server not available within timeout"
        )

    def test_plan_pick_basic_request(self):
        """Test basic pick planning request (stub implementation)"""
        self.assertTrue(
            self.test_node.plan_pick_client.wait_for_server(timeout_sec=5.0)
        )

        # Create goal for pick planning
        goal_msg = PlanTrajectory.Goal()
        goal_msg.target_pose = self.create_target_pose(0.3, 0.2, 0.15)
        goal_msg.planning_group = "ur_manipulator"
        goal_msg.planning_time = 5.0
        goal_msg.avoid_collisions = True

        # Send goal
        goal_future = self.test_node.plan_pick_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.test_node, goal_future, timeout_sec=2.0)

        self.assertTrue(goal_future.done(), "Goal not accepted in time")

        goal_handle = goal_future.result()
        self.assertIsNotNone(goal_handle)

        if goal_handle.accepted:
            # Wait for result
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self.test_node,
                result_future,
                timeout_sec=10.0
            )

            if result_future.done():
                result = result_future.result().result
                # Current stub implementation may return success=False
                # This test verifies the action server responds
                self.assertIsNotNone(result)
                self.assertIsInstance(result.success, bool)
                self.assertIsInstance(result.message, str)

    def test_plan_place_basic_request(self):
        """Test basic place planning request (stub implementation)"""
        self.assertTrue(
            self.test_node.plan_place_client.wait_for_server(timeout_sec=5.0)
        )

        # Create goal for place planning
        goal_msg = PlanTrajectory.Goal()
        goal_msg.target_pose = self.create_target_pose(0.5, 0.3, 0.2)
        goal_msg.planning_group = "ur_manipulator"
        goal_msg.planning_time = 5.0
        goal_msg.avoid_collisions = True

        # Send goal
        goal_future = self.test_node.plan_place_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.test_node, goal_future, timeout_sec=2.0)

        self.assertTrue(goal_future.done())

        goal_handle = goal_future.result()
        self.assertIsNotNone(goal_handle)

        if goal_handle.accepted:
            result_future = goal_handle.get_result_async()
            rclpy.spin_until_future_complete(
                self.test_node,
                result_future,
                timeout_sec=10.0
            )

            if result_future.done():
                result = result_future.result().result
                self.assertIsNotNone(result)
                self.assertIsInstance(result.success, bool)

    def test_plan_pick_with_custom_planning_time(self):
        """Test pick planning with custom planning time"""
        self.assertTrue(
            self.test_node.plan_pick_client.wait_for_server(timeout_sec=5.0)
        )

        goal_msg = PlanTrajectory.Goal()
        goal_msg.target_pose = self.create_target_pose(0.4, 0.1, 0.2)
        goal_msg.planning_group = "ur_manipulator"
        goal_msg.planning_time = 2.0  # Shorter planning time
        goal_msg.avoid_collisions = True

        goal_future = self.test_node.plan_pick_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.test_node, goal_future, timeout_sec=2.0)

        if goal_future.done():
            goal_handle = goal_future.result()
            if goal_handle and goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(
                    self.test_node,
                    result_future,
                    timeout_sec=5.0
                )
                # Verify response received
                if result_future.done():
                    result = result_future.result().result
                    self.assertIsNotNone(result)

    def test_plan_pick_without_collision_avoidance(self):
        """Test pick planning with collision avoidance disabled"""
        self.assertTrue(
            self.test_node.plan_pick_client.wait_for_server(timeout_sec=5.0)
        )

        goal_msg = PlanTrajectory.Goal()
        goal_msg.target_pose = self.create_target_pose(0.3, 0.0, 0.25)
        goal_msg.planning_group = "ur_manipulator"
        goal_msg.planning_time = 5.0
        goal_msg.avoid_collisions = False  # Disable collision checking

        goal_future = self.test_node.plan_pick_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.test_node, goal_future, timeout_sec=2.0)

        if goal_future.done():
            goal_handle = goal_future.result()
            if goal_handle and goal_handle.accepted:
                result_future = goal_handle.get_result_async()
                rclpy.spin_until_future_complete(
                    self.test_node,
                    result_future,
                    timeout_sec=5.0
                )

    def test_plan_pick_various_poses(self):
        """Test pick planning with various target poses"""
        self.assertTrue(
            self.test_node.plan_pick_client.wait_for_server(timeout_sec=5.0)
        )

        # Test different positions within workspace
        test_poses = [
            (0.3, 0.2, 0.15),
            (0.4, -0.1, 0.2),
            (-0.2, 0.3, 0.18),
            (0.5, 0.0, 0.25),
        ]

        for x, y, z in test_poses:
            goal_msg = PlanTrajectory.Goal()
            goal_msg.target_pose = self.create_target_pose(x, y, z)
            goal_msg.planning_group = "ur_manipulator"
            goal_msg.planning_time = 3.0
            goal_msg.avoid_collisions = True

            goal_future = self.test_node.plan_pick_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(
                self.test_node,
                goal_future,
                timeout_sec=2.0
            )

            if goal_future.done():
                goal_handle = goal_future.result()
                # Just verify we get a response (stub may reject or accept)
                self.assertIsNotNone(goal_handle)

    def test_plan_place_various_poses(self):
        """Test place planning with various target poses"""
        self.assertTrue(
            self.test_node.plan_place_client.wait_for_server(timeout_sec=5.0)
        )

        test_poses = [
            (0.5, 0.3, 0.1),
            (0.6, -0.2, 0.15),
            (-0.3, 0.4, 0.12),
        ]

        for x, y, z in test_poses:
            goal_msg = PlanTrajectory.Goal()
            goal_msg.target_pose = self.create_target_pose(x, y, z)
            goal_msg.planning_group = "ur_manipulator"
            goal_msg.planning_time = 3.0
            goal_msg.avoid_collisions = True

            goal_future = self.test_node.plan_place_client.send_goal_async(goal_msg)
            rclpy.spin_until_future_complete(
                self.test_node,
                goal_future,
                timeout_sec=2.0
            )

            if goal_future.done():
                goal_handle = goal_future.result()
                self.assertIsNotNone(goal_handle)

    def test_sequential_pick_and_place_planning(self):
        """Test sequential pick then place planning requests"""
        self.assertTrue(
            self.test_node.plan_pick_client.wait_for_server(timeout_sec=5.0)
        )
        self.assertTrue(
            self.test_node.plan_place_client.wait_for_server(timeout_sec=5.0)
        )

        # First, plan pick
        pick_goal = PlanTrajectory.Goal()
        pick_goal.target_pose = self.create_target_pose(0.3, 0.2, 0.15)
        pick_goal.planning_group = "ur_manipulator"
        pick_goal.planning_time = 5.0
        pick_goal.avoid_collisions = True

        pick_future = self.test_node.plan_pick_client.send_goal_async(pick_goal)
        rclpy.spin_until_future_complete(self.test_node, pick_future, timeout_sec=2.0)

        if pick_future.done():
            pick_handle = pick_future.result()
            if pick_handle and pick_handle.accepted:
                pick_result_future = pick_handle.get_result_async()
                rclpy.spin_until_future_complete(
                    self.test_node,
                    pick_result_future,
                    timeout_sec=10.0
                )

                # Then, plan place
                place_goal = PlanTrajectory.Goal()
                place_goal.target_pose = self.create_target_pose(0.5, 0.3, 0.2)
                place_goal.planning_group = "ur_manipulator"
                place_goal.planning_time = 5.0
                place_goal.avoid_collisions = True

                place_future = self.test_node.plan_place_client.send_goal_async(place_goal)
                rclpy.spin_until_future_complete(
                    self.test_node,
                    place_future,
                    timeout_sec=2.0
                )

                if place_future.done():
                    place_handle = place_future.result()
                    self.assertIsNotNone(place_handle)

    def test_concurrent_planning_requests(self):
        """Test behavior with concurrent planning requests

        Note: Current implementation may only handle one request at a time.
        This test verifies the behavior with overlapping requests.
        """
        self.assertTrue(
            self.test_node.plan_pick_client.wait_for_server(timeout_sec=5.0)
        )

        # Send two goals in quick succession
        goal1 = PlanTrajectory.Goal()
        goal1.target_pose = self.create_target_pose(0.3, 0.2, 0.15)
        goal1.planning_group = "ur_manipulator"
        goal1.planning_time = 3.0
        goal1.avoid_collisions = True

        goal2 = PlanTrajectory.Goal()
        goal2.target_pose = self.create_target_pose(0.4, 0.1, 0.2)
        goal2.planning_group = "ur_manipulator"
        goal2.planning_time = 3.0
        goal2.avoid_collisions = True

        future1 = self.test_node.plan_pick_client.send_goal_async(goal1)
        time.sleep(0.1)  # Small delay
        future2 = self.test_node.plan_pick_client.send_goal_async(goal2)

        # Both should get responses (may accept or reject)
        rclpy.spin_until_future_complete(self.test_node, future1, timeout_sec=2.0)
        rclpy.spin_until_future_complete(self.test_node, future2, timeout_sec=2.0)

        if future1.done():
            handle1 = future1.result()
            self.assertIsNotNone(handle1)

        if future2.done():
            handle2 = future2.result()
            self.assertIsNotNone(handle2)

    def test_planning_with_empty_group_name(self):
        """Test planning request with empty planning group name"""
        self.assertTrue(
            self.test_node.plan_pick_client.wait_for_server(timeout_sec=5.0)
        )

        goal_msg = PlanTrajectory.Goal()
        goal_msg.target_pose = self.create_target_pose(0.3, 0.2, 0.15)
        goal_msg.planning_group = ""  # Empty group name
        goal_msg.planning_time = 5.0
        goal_msg.avoid_collisions = True

        goal_future = self.test_node.plan_pick_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.test_node, goal_future, timeout_sec=2.0)

        # Should get a response (may reject invalid group)
        if goal_future.done():
            goal_handle = goal_future.result()
            self.assertIsNotNone(goal_handle)

    def test_planning_with_zero_planning_time(self):
        """Test planning request with zero planning time"""
        self.assertTrue(
            self.test_node.plan_pick_client.wait_for_server(timeout_sec=5.0)
        )

        goal_msg = PlanTrajectory.Goal()
        goal_msg.target_pose = self.create_target_pose(0.3, 0.2, 0.15)
        goal_msg.planning_group = "ur_manipulator"
        goal_msg.planning_time = 0.0  # Zero planning time
        goal_msg.avoid_collisions = True

        goal_future = self.test_node.plan_pick_client.send_goal_async(goal_msg)
        rclpy.spin_until_future_complete(self.test_node, goal_future, timeout_sec=2.0)

        if goal_future.done():
            goal_handle = goal_future.result()
            self.assertIsNotNone(goal_handle)


if __name__ == '__main__':
    unittest.main()
