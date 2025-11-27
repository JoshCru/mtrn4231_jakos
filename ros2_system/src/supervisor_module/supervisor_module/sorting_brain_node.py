#!/usr/bin/env python3
"""
Sorting Brain Node - Central Orchestrator for Weight Sorting System

This node manages the full closed-loop sorting cycle:
1. Wait for detected objects from perception (Kevin)
2. Pick up a weight from the picking area
3. Wait for weight estimation from calibration (Asad)
4. Decide placement position based on sorted order
5. Rearrange existing weights if needed
6. Place the weight in the correct position

State Machine:
IDLE -> WAITING_FOR_DETECTION -> PICKING -> WEIGHING -> DECIDING_PLACEMENT ->
  -> (if needed) REARRANGING -> PLACING -> IDLE
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from enum import Enum
from dataclasses import dataclass
from typing import List, Optional, Tuple
import time

# Messages
from std_msgs.msg import String, Float32, Header
from geometry_msgs.msg import Pose
from visualization_msgs.msg import Marker, MarkerArray
from sort_interfaces.msg import DetectedObjects, WeightEstimate, BoundingBox
from sort_interfaces.srv import MoveToCartesian, GripperControl
import random

# Actions
from sort_interfaces.action import PickObject, PlaceObject


class SortingState(Enum):
    IDLE = "idle"
    WAITING_FOR_DETECTION = "waiting_for_detection"
    MOVING_TO_HOME = "moving_to_home"
    PICKING = "picking"
    WEIGHING = "weighing"
    DECIDING_PLACEMENT = "deciding_placement"
    REARRANGING = "rearranging"
    PLACING = "placing"
    ERROR = "error"


@dataclass
class PlacedWeight:
    """Represents a weight that has been placed in the placing area."""
    object_id: int
    weight_grams: float
    x_mm: float
    y_mm: float


class SortingBrainNode(Node):
    """Central orchestrator for the weight sorting system."""

    # Zone definitions from pick_and_place_demo.py
    # These are the coordinates sent to the cartesian controller
    # Controller negates x/y, so robot moves to positive x/y in RViz
    PICKING_AREA = {
        'x_min': -787.0, 'x_max': -420.0,
        'y_min': -252.0, 'y_max': 50.0
    }
    PLACING_AREA = {
        'x_min': -787.0, 'x_max': -420.0,
        'y_min': -391.0, 'y_max': -252.0
    }

    # Z heights (mm) - tool0 frame
    Z_HOME = 371.0
    Z_DESCEND = 212.0
    Z_PICKUP = 182.0
    Z_PLACE = 182.0

    # Default orientation (facing down)
    RX = 2.221
    RY = 2.221
    RZ = 0.0

    # Spacing between placed weights (mm) - increased for larger weights
    WEIGHT_SPACING = 60.0

    # Weight sizes based on mass (mm diameter)
    # 500g = 43mm, 200g = 32mm, 100g = 25mm
    WEIGHT_SIZES = {
        500: 43.0,
        200: 32.0,
        100: 25.0
    }

    def __init__(self):
        super().__init__('sorting_brain_node')

        # Callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()

        # State machine
        self.state = SortingState.IDLE
        self.previous_state = SortingState.IDLE

        # Memory of placed weights (sorted by position)
        self.placed_weights: List[PlacedWeight] = []

        # Current operation context
        self.current_object_id: Optional[int] = None
        self.current_weight: Optional[float] = None
        self.current_pick_position: Optional[Tuple[float, float]] = None
        self.detected_objects: List[BoundingBox] = []

        # Timeout tracking
        self.weight_wait_start: Optional[float] = None
        self.weight_timeout_sec = 10.0

        # Track last known robot position (x, y in mm)
        # Start at a safe home position (center of picking area)
        self.last_x: float = -600.0
        self.last_y: float = -100.0
        self.last_z: float = self.Z_HOME

        # Flag to prevent re-entry during long operations
        self.operation_in_progress: bool = False

        # Fake weight estimation tracking
        self._fake_weight_published: bool = False

        # Track when holding an object (for visualization)
        self.holding_object: bool = False
        self.held_weight_grams: float = 0.0

        self._init_services()
        self._init_subscribers()
        self._init_publishers()
        self._init_timers()

        self.get_logger().info('Sorting Brain Node initialized')
        self.get_logger().info(f'Picking area: X[{self.PICKING_AREA["x_min"]}, {self.PICKING_AREA["x_max"]}] '
                               f'Y[{self.PICKING_AREA["y_min"]}, {self.PICKING_AREA["y_max"]}]')
        self.get_logger().info(f'Placing area: X[{self.PLACING_AREA["x_min"]}, {self.PLACING_AREA["x_max"]}] '
                               f'Y[{self.PLACING_AREA["y_min"]}, {self.PLACING_AREA["y_max"]}]')

    def _init_services(self):
        """Initialize service clients."""
        self.move_client = self.create_client(
            MoveToCartesian,
            '/motion_control/move_to_cartesian',
            callback_group=self.callback_group
        )
        self.gripper_client = self.create_client(
            GripperControl,
            '/motion_control/gripper_control',
            callback_group=self.callback_group
        )

        self.get_logger().info('Waiting for motion control services...')
        self.move_client.wait_for_service(timeout_sec=30.0)
        self.get_logger().info('Move service ready')

        # Check if gripper service is available (don't block if not)
        self.gripper_available = self.gripper_client.wait_for_service(timeout_sec=2.0)
        if self.gripper_available:
            self.get_logger().info('Gripper service ready')
        else:
            self.get_logger().warn('Gripper service NOT available - gripper operations will be skipped')

    def _init_subscribers(self):
        """Initialize topic subscribers."""
        # Dashboard commands
        self.command_sub = self.create_subscription(
            String,
            '/sorting/command',
            self.command_callback,
            10,
            callback_group=self.callback_group
        )

        # Perception from Kevin
        self.detected_objects_sub = self.create_subscription(
            DetectedObjects,
            '/perception/detected_objects',
            self.detected_objects_callback,
            10,
            callback_group=self.callback_group
        )

        # Weight estimation from Asad
        self.weight_estimate_sub = self.create_subscription(
            WeightEstimate,
            '/recognition/estimated_weights',
            self.weight_estimate_callback,
            10,
            callback_group=self.callback_group
        )

        # Fallback: gripper force feedback
        self.force_feedback_sub = self.create_subscription(
            Float32,
            '/motion_control/force_feedback',
            self.force_feedback_callback,
            10,
            callback_group=self.callback_group
        )

    def _init_publishers(self):
        """Initialize topic publishers."""
        self.status_pub = self.create_publisher(String, '/sorting/status', 10)
        self.state_pub = self.create_publisher(String, '/sorting/state', 10)

        # Fake weight estimation publisher (for simulation without Asad's calibration node)
        self.fake_weight_pub = self.create_publisher(
            WeightEstimate,
            '/recognition/estimated_weights',
            10
        )

        # Publisher to notify perception that an object was picked up
        self.remove_object_pub = self.create_publisher(
            BoundingBox,
            '/perception/remove_object',
            10
        )

        # Publisher for placed weights visualization
        self.placed_weights_marker_pub = self.create_publisher(
            MarkerArray,
            '/sorting/placed_weight_markers',
            10
        )

        # Publisher for held weight visualization (attached to gripper)
        self.held_weight_marker_pub = self.create_publisher(
            Marker,
            '/sorting/held_weight_marker',
            10
        )

    def _init_timers(self):
        """Initialize timers."""
        # Main state machine timer (10 Hz)
        self.state_timer = self.create_timer(
            0.1,
            self.state_machine_tick,
            callback_group=self.callback_group
        )

        # Status publish timer (1 Hz)
        self.status_timer = self.create_timer(
            1.0,
            self.publish_status,
            callback_group=self.callback_group
        )

        # Held weight marker timer (20 Hz for smooth following)
        self.held_marker_timer = self.create_timer(
            0.05,
            self.publish_held_weight_marker,
            callback_group=self.callback_group
        )

    # ==================== Callbacks ====================

    def command_callback(self, msg: String):
        """Handle commands from the dashboard."""
        command = msg.data.lower()
        self.get_logger().info(f"Received command: {command}")

        if command == "start":
            if self.state == SortingState.IDLE:
                self.state = SortingState.WAITING_FOR_DETECTION
                self.publish_status("System started - waiting for objects")
            else:
                self.publish_status(f"Cannot start - already in state: {self.state.value}")

        elif command == "stop":
            if self.state != SortingState.IDLE:
                self.state = SortingState.IDLE
                self.operation_in_progress = False
                self.publish_status("System stopped")
            else:
                self.publish_status("System already stopped")

        elif command == "reset":
            self.state = SortingState.IDLE
            self.operation_in_progress = False
            self.placed_weights.clear()
            self.current_object_id = None
            self.current_weight = None
            self.current_pick_position = None
            self.detected_objects.clear()
            self.holding_object = False
            self.publish_status("System reset - all data cleared")
            self.get_logger().info("System reset complete")

        elif command == "emergency_stop":
            self.state = SortingState.ERROR
            self.operation_in_progress = False
            self.publish_status("EMERGENCY STOP ACTIVATED!")
            self.get_logger().error("EMERGENCY STOP ACTIVATED")

        else:
            self.publish_status(f"Unknown command: {command}")

    def detected_objects_callback(self, msg: DetectedObjects):
        """Handle detected objects from perception."""
        # Filter objects in picking area
        valid_objects = []
        for obj in msg.objects:
            center_x = (obj.x_min + obj.x_max) / 2.0
            center_y = (obj.y_min + obj.y_max) / 2.0

            if (self.PICKING_AREA['x_min'] <= center_x <= self.PICKING_AREA['x_max'] and
                self.PICKING_AREA['y_min'] <= center_y <= self.PICKING_AREA['y_max']):
                valid_objects.append(obj)

        self.detected_objects = valid_objects

        if valid_objects and self.state == SortingState.WAITING_FOR_DETECTION:
            self.get_logger().info(f'Detected {len(valid_objects)} objects in picking area')

    def weight_estimate_callback(self, msg: WeightEstimate):
        """Handle weight estimate from perception/calibration node."""
        # Accept weight if it matches current object (regardless of state, since timing can vary)
        if msg.object_id == self.current_object_id or self.current_object_id is None:
            self.current_weight = msg.estimated_weight
            self.get_logger().info(f'Received weight estimate: {msg.estimated_weight:.1f}g '
                                   f'for object {msg.object_id} (confidence: {msg.confidence:.2f})')

    def force_feedback_callback(self, msg: Float32):
        """Fallback: use gripper force as weight estimate."""
        # Only use if in WEIGHING state and no weight received yet
        if self.state == SortingState.WEIGHING and self.current_weight is None:
            # Wait a bit before using fallback
            if self.weight_wait_start and time.time() - self.weight_wait_start > 3.0:
                self.current_weight = msg.data
                self.get_logger().info(f'Using gripper force as weight estimate: {msg.data:.1f}g')

    # ==================== State Machine ====================

    def state_machine_tick(self):
        """Main state machine loop."""
        # Prevent re-entry during long operations
        if self.operation_in_progress:
            return

        # Debug: log that timer is firing
        self.get_logger().debug(f'State machine tick: state={self.state.value}')

        try:
            if self.state == SortingState.IDLE:
                self._handle_idle()
            elif self.state == SortingState.WAITING_FOR_DETECTION:
                self._handle_waiting_for_detection()
            elif self.state == SortingState.MOVING_TO_HOME:
                self._handle_moving_to_home()
            elif self.state == SortingState.PICKING:
                self._handle_picking()
            elif self.state == SortingState.WEIGHING:
                self._handle_weighing()
            elif self.state == SortingState.DECIDING_PLACEMENT:
                self._handle_deciding_placement()
            elif self.state == SortingState.REARRANGING:
                self._handle_rearranging()
            elif self.state == SortingState.PLACING:
                self._handle_placing()
            elif self.state == SortingState.ERROR:
                self._handle_error()
        except Exception as e:
            self.get_logger().error(f'State machine error: {e}')
            self.transition_state(SortingState.ERROR)

    def transition_state(self, new_state: SortingState):
        """Transition to a new state."""
        if self.state != new_state:
            self.get_logger().info(f'State transition: {self.state.value} -> {new_state.value}')
            self.previous_state = self.state
            self.state = new_state

            # Publish state change
            msg = String()
            msg.data = new_state.value
            self.state_pub.publish(msg)

    def _handle_idle(self):
        """IDLE state: wait for start command or auto-start if objects detected."""
        # Auto-transition to waiting for detection
        self.transition_state(SortingState.WAITING_FOR_DETECTION)

    def _handle_waiting_for_detection(self):
        """Wait for objects to be detected in picking area."""
        if self.detected_objects:
            # Select the first available object
            obj = self.detected_objects[0]
            self.current_object_id = obj.id
            self.current_pick_position = (
                (obj.x_min + obj.x_max) / 2.0,
                (obj.y_min + obj.y_max) / 2.0
            )
            self.get_logger().info(f'Selected object {obj.id} at ({self.current_pick_position[0]:.1f}, '
                                   f'{self.current_pick_position[1]:.1f})')
            self.transition_state(SortingState.PICKING)

    def _handle_moving_to_home(self):
        """Move robot to home position."""
        # This is called when we need to return home between operations
        self.transition_state(SortingState.WAITING_FOR_DETECTION)

    def _handle_picking(self):
        """Execute pick sequence using staged movements."""
        if self.current_pick_position is None:
            self.get_logger().error('No pick position set')
            self.transition_state(SortingState.ERROR)
            return

        # Set flag to prevent re-entry
        self.operation_in_progress = True

        x, y = self.current_pick_position

        try:
            # 1. Use staged movement to get to descend height above object
            # This will: lift to Z_HOME -> move to x,y at Z_HOME -> descend to Z_DESCEND
            self.get_logger().info(f'=== PICKING from ({x:.1f}, {y:.1f}) ===')
            self.get_logger().info(f'Step: Moving to approach position (staged)...')
            if not self.move_staged(x, y, self.Z_DESCEND):
                self.get_logger().error('Failed to move to hover position')
                self.transition_state(SortingState.ERROR)
                return

            # 2. Open gripper
            self.get_logger().info('Step: Opening gripper...')
            if not self.gripper_control('open'):
                self.get_logger().error('Failed to open gripper')
                self.transition_state(SortingState.ERROR)
                return

            # 3. Move down to pickup height (simple vertical move)
            self.get_logger().info('Step: Descending to pickup height...')
            if not self.move_to(x, y, self.Z_PICKUP):
                self.get_logger().error('Failed to move to pickup position')
                self.transition_state(SortingState.ERROR)
                return

            # 4. Close gripper
            self.get_logger().info('Step: Closing gripper...')
            if not self.gripper_control('close'):
                self.get_logger().error('Failed to close gripper')
                self.transition_state(SortingState.ERROR)
                return

            # Mark that we're now holding an object (for visualization)
            self.holding_object = True

            # Reset weight tracking BEFORE publishing remove message
            # (because the weight callback will be triggered immediately)
            self.weight_wait_start = time.time()
            self.current_weight = None
            self._fake_weight_published = False

            # Notify perception that this object was picked up (for simulation)
            # This triggers the perception node to publish the weight estimate
            if self.current_object_id is not None:
                remove_msg = BoundingBox()
                remove_msg.id = self.current_object_id
                self.remove_object_pub.publish(remove_msg)
                self.get_logger().info(f'Notified perception: removed object {self.current_object_id}')

            # 5. Lift to descend height (simple vertical move)
            self.get_logger().info('Step: Lifting object...')
            if not self.move_to(x, y, self.Z_DESCEND):
                self.get_logger().error('Failed to lift object')
                self.transition_state(SortingState.ERROR)
                return

            self.get_logger().info('=== PICK COMPLETE, waiting for weight... ===')
            self.transition_state(SortingState.WEIGHING)
        finally:
            self.operation_in_progress = False

    def _handle_weighing(self):
        """Wait for weight estimation from perception node."""
        # Check if we already have a weight
        if self.current_weight is not None:
            self.get_logger().info(f'Weight determined: {self.current_weight:.1f}g')
            self.transition_state(SortingState.DECIDING_PLACEMENT)
            return

        # Check elapsed time - weight should come from perception when object was picked
        if self.weight_wait_start:
            elapsed = time.time() - self.weight_wait_start

            # Log waiting status periodically
            if elapsed > 1.0 and not self._fake_weight_published:
                self.get_logger().info(f'Waiting for weight from perception... ({elapsed:.1f}s)')
                self._fake_weight_published = True  # Just to avoid spamming

            # Timeout fallback
            if elapsed > self.weight_timeout_sec:
                self.get_logger().warn('Weight estimation timeout, using default (100g)')
                self.current_weight = 100.0
                self.transition_state(SortingState.DECIDING_PLACEMENT)

    def _publish_fake_weight(self, weight_grams: float):
        """Publish a fake weight estimate (for simulation)."""
        msg = WeightEstimate()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.object_id = self.current_object_id or 0
        msg.estimated_weight = weight_grams
        msg.confidence = 0.95
        msg.volume = weight_grams / 7.8  # Approximate for steel density

        self.fake_weight_pub.publish(msg)
        self.get_logger().info(f'Published FAKE weight estimate: {weight_grams:.1f}g (simulation)')

    def _handle_deciding_placement(self):
        """Decide where to place the weight based on sorted order."""
        if self.current_weight is None:
            self.transition_state(SortingState.ERROR)
            return

        # Find insertion position (sorted by weight, ascending)
        # Use < instead of <= so equal weights don't trigger rearrangement
        insert_index = 0
        for i, placed in enumerate(self.placed_weights):
            if self.current_weight < placed.weight_grams:
                break
            insert_index = i + 1

        self.target_insert_index = insert_index
        total_weights_after = len(self.placed_weights) + 1

        self.get_logger().info(f'Weight {self.current_weight:.1f}g will be at position {insert_index} of {total_weights_after}')

        # Check if we need to rearrange (insert not at the rightmost position)
        if insert_index < len(self.placed_weights):
            self.get_logger().info(f'Need to rearrange: shift weights {insert_index} to {len(self.placed_weights)-1} to the right')
            self.transition_state(SortingState.REARRANGING)
        else:
            # No rearrangement needed, just place at the end
            place_x, place_y = self.calculate_placement_position_simple(insert_index)
            self.current_place_position = (place_x, place_y)
            self.get_logger().info(f'Placing at rightmost position: ({place_x:.1f}, {place_y:.1f})')
            self.transition_state(SortingState.PLACING)

    def calculate_placement_position_simple(self, index: int) -> Tuple[float, float]:
        """Calculate X,Y position for a weight at the given index (left-to-right, no centering)."""
        # Start from left side of placing area with some margin
        start_x = self.PLACING_AREA['x_min'] + 50.0
        center_y = (self.PLACING_AREA['y_min'] + self.PLACING_AREA['y_max']) / 2.0

        # Simple left-to-right positioning
        x = start_x + (index * self.WEIGHT_SPACING)
        y = center_y

        return x, y

    def calculate_placement_position(self, index: int) -> Tuple[float, float]:
        """Calculate the X,Y position for placing a weight at the given index."""
        return self.calculate_placement_position_simple(index)

    def _handle_rearranging(self):
        """Rearrange existing weights to make space for insertion."""
        # This is called when we need to insert a lighter weight
        # We need to:
        # 1. Put the current weight back at its ORIGINAL pick position (in picking area)
        # 2. Move all weights in placing area from insert_index onwards one position to the right
        # 3. Pick up the weight from original pick position
        # 4. Place it at the now-empty insert_index position

        self.operation_in_progress = True
        insert_index = self.target_insert_index
        total_after = len(self.placed_weights) + 1

        # Use the original pick position as the staging area (in the picking zone, not placing zone)
        if self.current_pick_position is None:
            self.get_logger().error('No pick position available for staging')
            self.transition_state(SortingState.ERROR)
            return

        staging_x, staging_y = self.current_pick_position

        try:
            # Step 1: Put current weight back at its original pick position (in picking area)
            # Optimize: go directly to pickup height without lifting to home
            self.get_logger().info(f'=== REARRANGING: Returning weight to original position ({staging_x:.1f}, {staging_y:.1f}) ===')

            # Move directly to staging position at pickup height (no need to lift to home)
            if not self.move_to(staging_x, staging_y, self.Z_PICKUP):
                self.get_logger().error('Failed to move to staging position')
                self.transition_state(SortingState.ERROR)
                return
            if not self.gripper_control('open'):
                self.get_logger().error('Failed to open gripper at staging')
                self.transition_state(SortingState.ERROR)
                return

            self.holding_object = False
            self.staged_weight = self.current_weight
            self.staged_object_id = self.current_object_id

            # Lift slightly to clear the weight
            if not self.move_to(staging_x, staging_y, self.Z_DESCEND):
                self.get_logger().error('Failed to retreat from staging')

            # Step 2: Move weights from rightmost down to insert_index, each one position right
            # We need to recalculate all positions based on the new total
            self.get_logger().info(f'Moving {len(self.placed_weights) - insert_index} weights to the right')

            for i in range(len(self.placed_weights) - 1, insert_index - 1, -1):
                pw = self.placed_weights[i]
                old_x, old_y = pw.x_mm, pw.y_mm
                new_x, new_y = self.calculate_placement_position_simple(i + 1)

                self.get_logger().info(f'Moving weight {pw.object_id} ({pw.weight_grams:.0f}g) from pos {i} to pos {i+1}')

                # Pick up the weight
                if not self.move_staged(old_x, old_y, self.Z_DESCEND):
                    continue
                if not self.move_to(old_x, old_y, self.Z_PICKUP):
                    continue
                if not self.gripper_control('close'):
                    continue
                self.holding_object = True
                if not self.move_to(old_x, old_y, self.Z_DESCEND):
                    continue

                # Place at new position
                if not self.move_staged(new_x, new_y, self.Z_DESCEND):
                    continue
                if not self.move_to(new_x, new_y, self.Z_PLACE):
                    continue
                if not self.gripper_control('open'):
                    continue
                self.holding_object = False
                if not self.move_to(new_x, new_y, self.Z_DESCEND):
                    pass

                # Update stored position
                pw.x_mm = new_x
                pw.y_mm = new_y

            # Step 3: Pick up the weight from its original pick position
            # Optimize: descend directly to pickup height (we're already at staging_x, staging_y, Z_DESCEND)
            self.get_logger().info(f'Picking up weight from original position ({staging_x:.1f}, {staging_y:.1f})')
            if not self.move_to(staging_x, staging_y, self.Z_PICKUP):
                self.get_logger().error('Failed to descend to pickup at staging')
                self.transition_state(SortingState.ERROR)
                return
            if not self.gripper_control('close'):
                self.get_logger().error('Failed to pick up staged weight')
                self.transition_state(SortingState.ERROR)
                return
            self.holding_object = True
            self.current_weight = self.staged_weight
            self.current_object_id = self.staged_object_id

            # Lift to clear
            if not self.move_to(staging_x, staging_y, self.Z_DESCEND):
                pass

            # Step 4: Calculate final position for the new weight
            place_x, place_y = self.calculate_placement_position_simple(insert_index)
            self.current_place_position = (place_x, place_y)

            self.get_logger().info(f'=== REARRANGING COMPLETE, placing at ({place_x:.1f}, {place_y:.1f}) ===')
            self.transition_state(SortingState.PLACING)

        finally:
            self.operation_in_progress = False

    def _handle_placing(self):
        """Execute place sequence using staged movements."""
        if not hasattr(self, 'current_place_position') or self.current_place_position is None:
            self.get_logger().error('No place position set')
            self.transition_state(SortingState.ERROR)
            return

        # Set flag to prevent re-entry
        self.operation_in_progress = True

        x, y = self.current_place_position

        try:
            # 1. Use staged movement to get to descend height above placement
            # This will: lift to Z_HOME -> move to x,y at Z_HOME -> descend to Z_DESCEND
            self.get_logger().info(f'=== PLACING at ({x:.1f}, {y:.1f}) ===')
            self.get_logger().info(f'Step: Moving to approach position (staged)...')
            if not self.move_staged(x, y, self.Z_DESCEND):
                self.get_logger().error('Failed to move to hover position')
                self.transition_state(SortingState.ERROR)
                return

            # 2. Move down to place height (simple vertical move)
            self.get_logger().info('Step: Descending to place height...')
            if not self.move_to(x, y, self.Z_PLACE):
                self.get_logger().error('Failed to move to place position')
                self.transition_state(SortingState.ERROR)
                return

            # 3. Open gripper to release
            self.get_logger().info('Step: Opening gripper to release...')
            if not self.gripper_control('open'):
                self.get_logger().error('Failed to open gripper')
                self.transition_state(SortingState.ERROR)
                return

            # Mark that we're no longer holding an object (for visualization)
            self.holding_object = False

            # 4. Lift to descend height (simple vertical move)
            self.get_logger().info('Step: Retreating...')
            if not self.move_to(x, y, self.Z_DESCEND):
                self.get_logger().error('Failed to retreat')
                # Don't fail - object was placed

            # Record the placed weight at the correct sorted position
            new_weight = PlacedWeight(
                object_id=self.current_object_id or 0,
                weight_grams=self.current_weight or 0.0,
                x_mm=x,
                y_mm=y
            )

            # Insert at the target index (maintains sorted order)
            insert_idx = getattr(self, 'target_insert_index', len(self.placed_weights))
            self.placed_weights.insert(insert_idx, new_weight)

            self.get_logger().info(f'=== PLACE COMPLETE! Weight {new_weight.weight_grams:.0f}g at index {insert_idx}. Total: {len(self.placed_weights)} ===')

            # Print sorted order for verification
            weights_str = ', '.join([f'{pw.weight_grams:.0f}g' for pw in self.placed_weights])
            self.get_logger().info(f'Sorted order: [{weights_str}]')

            # Store object_id before resetting
            placed_object_id = self.current_object_id

            # Reset for next cycle
            self.current_object_id = None
            self.current_weight = None
            self.current_pick_position = None
            self.current_place_position = None
            self.target_insert_index = None

            # Remove the picked object from detected list
            if self.detected_objects and placed_object_id is not None:
                self.detected_objects = [o for o in self.detected_objects
                                         if o.id != placed_object_id]

            self.transition_state(SortingState.WAITING_FOR_DETECTION)
        finally:
            self.operation_in_progress = False

    def _handle_error(self):
        """Handle error state."""
        self.get_logger().error('In ERROR state, attempting recovery...')

        # Try to open gripper and move to home
        try:
            self.gripper_control('open')
            self.move_to(-600.0, -100.0, self.Z_HOME)
        except Exception as e:
            self.get_logger().error(f'Recovery failed: {e}')

        # Reset state
        self.current_object_id = None
        self.current_weight = None
        self.current_pick_position = None
        self.holding_object = False  # Clear held object visualization

        time.sleep(2.0)  # Wait before retrying
        self.transition_state(SortingState.IDLE)

    # ==================== Helper Methods ====================

    def move_to(self, x: float, y: float, z: float) -> bool:
        """Move robot to specified position using cartesian service."""
        request = MoveToCartesian.Request()
        request.x = x
        request.y = y
        request.z = z
        request.rx = self.RX
        request.ry = self.RY
        request.rz = self.RZ

        self.get_logger().info(f'Moving to: ({x:.1f}, {y:.1f}, {z:.1f})')

        try:
            future = self.move_client.call_async(request)

            # Wait for future without blocking the executor
            # (spin_until_future_complete interferes with MultiThreadedExecutor)
            timeout = 120.0
            start = time.time()
            while not future.done():
                if time.time() - start > timeout:
                    self.get_logger().error('Move service call timed out')
                    return False
                time.sleep(0.05)  # Small sleep to avoid busy waiting

            result = future.result()
            if result is not None:
                if result.success:
                    self.get_logger().info(f'Move completed: {result.message}')
                    # Update last known position
                    self.last_x = x
                    self.last_y = y
                    self.last_z = z
                    return True
                else:
                    self.get_logger().error(f'Move failed: {result.message}')
                    return False
            else:
                self.get_logger().error('Move service returned None')
                return False
        except Exception as e:
            self.get_logger().error(f'Move service call failed: {e}')
            return False

    def move_staged(self, target_x: float, target_y: float, target_z: float) -> bool:
        """
        Move robot to target position using staged movements to avoid collisions.

        Movement sequence:
        1. If not at Z_HOME, first lift to Z_HOME at current x,y
        2. Move horizontally to target x,y at Z_HOME
        3. Descend to target z

        This avoids trying to plan complex paths through obstacles.
        """
        self.get_logger().info(f'Staged move to: ({target_x:.1f}, {target_y:.1f}, {target_z:.1f})')
        self.get_logger().info(f'Current position: ({self.last_x:.1f}, {self.last_y:.1f}, {self.last_z:.1f})')

        # Step 1: If we're lower than Z_HOME, first lift up at current x,y
        if self.last_z < self.Z_HOME - 5.0:  # 5mm tolerance
            self.get_logger().info(f'Step 1: Lifting to Z_HOME at current x,y...')
            if not self.move_to(self.last_x, self.last_y, self.Z_HOME):
                self.get_logger().error('Failed to lift to Z_HOME')
                return False
        else:
            self.get_logger().info('Step 1: Already at safe height, skipping lift')

        # Step 2: Move horizontally to target x,y at Z_HOME
        if abs(self.last_x - target_x) > 5.0 or abs(self.last_y - target_y) > 5.0:
            self.get_logger().info(f'Step 2: Moving horizontally to ({target_x:.1f}, {target_y:.1f}) at Z_HOME...')
            if not self.move_to(target_x, target_y, self.Z_HOME):
                self.get_logger().error('Failed to move horizontally at Z_HOME')
                return False
        else:
            self.get_logger().info('Step 2: Already at target x,y, skipping horizontal move')

        # Step 3: Descend to target z
        if abs(self.last_z - target_z) > 5.0:
            self.get_logger().info(f'Step 3: Descending to z={target_z:.1f}...')
            if not self.move_to(target_x, target_y, target_z):
                self.get_logger().error('Failed to descend to target z')
                return False
        else:
            self.get_logger().info('Step 3: Already at target z, skipping descent')

        self.get_logger().info('Staged move complete!')
        return True

    def gripper_control(self, command: str) -> bool:
        """Control gripper using gripper service."""
        # Skip service call if gripper service not available
        if not self.gripper_available:
            self.get_logger().info(f'Gripper {command}: SKIPPED (service not available)')
            return True  # Return success to continue operation

        request = GripperControl.Request()
        request.command = command

        try:
            future = self.gripper_client.call_async(request)

            # Wait for future without blocking the executor
            timeout = 10.0
            start = time.time()
            while not future.done():
                if time.time() - start > timeout:
                    self.get_logger().error('Gripper service call timed out')
                    return False
                time.sleep(0.05)

            result = future.result()
            if result is not None:
                if result.success:
                    self.get_logger().info(f'Gripper {command}: {result.message}')
                    return True
                else:
                    self.get_logger().error(f'Gripper {command} failed: {result.message}')
                    return False
            else:
                self.get_logger().error('Gripper service returned None')
                return False
        except Exception as e:
            self.get_logger().error(f'Gripper service call failed: {e}')
            return False

    def publish_status(self):
        """Publish current status."""
        msg = String()
        status = {
            'state': self.state.value,
            'detected_objects': len(self.detected_objects),
            'placed_weights': len(self.placed_weights),
            'current_object_id': self.current_object_id,
            'current_weight': self.current_weight
        }
        msg.data = str(status)
        self.status_pub.publish(msg)

        # Also publish placed weight markers
        self.publish_placed_weight_markers()

    def get_weight_size(self, weight_grams: float) -> float:
        """Get the diameter (mm) for a given weight."""
        if weight_grams >= 400:
            return self.WEIGHT_SIZES[500]  # 43mm
        elif weight_grams >= 150:
            return self.WEIGHT_SIZES[200]  # 32mm
        else:
            return self.WEIGHT_SIZES[100]  # 25mm

    def publish_placed_weight_markers(self):
        """Publish RViz markers for placed weights in the placing zone."""
        marker_array = MarkerArray()

        # Use stored physical positions (weights are physically moved during rearrangement)
        for i, pw in enumerate(self.placed_weights):
            # Use the stored physical position (convert mm to meters and negate for RViz)
            display_x = -pw.x_mm / 1000.0
            display_y = -pw.y_mm / 1000.0

            # Get weight-based size
            size_mm = self.get_weight_size(pw.weight_grams)

            # Cylinder marker for each placed weight
            marker = Marker()
            marker.header.frame_id = 'base_link'
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.ns = 'placed_weights'
            marker.id = pw.object_id
            marker.type = Marker.CYLINDER
            marker.action = Marker.ADD

            marker.pose.position.x = display_x
            marker.pose.position.y = display_y
            marker.pose.position.z = 0.015  # 15mm above surface

            marker.pose.orientation.w = 1.0

            # Size based on weight
            marker.scale.x = size_mm / 1000.0  # diameter
            marker.scale.y = size_mm / 1000.0
            marker.scale.z = 0.03  # 30mm height

            # Color: green for placed weights
            marker.color.r = 0.2
            marker.color.g = 0.8
            marker.color.b = 0.2
            marker.color.a = 1.0

            marker.lifetime.sec = 2  # Refresh every 2 seconds

            marker_array.markers.append(marker)

            # Text label with weight
            text_marker = Marker()
            text_marker.header.frame_id = 'base_link'
            text_marker.header.stamp = self.get_clock().now().to_msg()
            text_marker.ns = 'placed_weight_labels'
            text_marker.id = pw.object_id + 1000
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD

            text_marker.pose.position.x = display_x
            text_marker.pose.position.y = display_y
            text_marker.pose.position.z = 0.06  # 60mm above surface

            text_marker.scale.z = 0.02  # Text height

            text_marker.color.r = 1.0
            text_marker.color.g = 1.0
            text_marker.color.b = 1.0
            text_marker.color.a = 1.0

            text_marker.text = f"{pw.weight_grams:.0f}g"
            text_marker.lifetime.sec = 2

            marker_array.markers.append(text_marker)

        self.placed_weights_marker_pub.publish(marker_array)

    def publish_held_weight_marker(self):
        """Publish a marker for the weight currently held by the gripper."""
        marker = Marker()
        marker.header.frame_id = 'tool0'  # Attached to gripper frame
        marker.header.stamp = self.get_clock().now().to_msg()
        marker.ns = 'held_weight'
        marker.id = 0
        marker.type = Marker.CYLINDER

        if self.holding_object:
            marker.action = Marker.ADD

            # Position relative to tool0 (slightly below gripper tip)
            marker.pose.position.x = 0.0
            marker.pose.position.y = 0.0
            marker.pose.position.z = 0.17  # Below the gripper tip

            marker.pose.orientation.w = 1.0

            # Size based on current weight
            size_mm = self.get_weight_size(self.current_weight or 100.0)
            marker.scale.x = size_mm / 1000.0  # diameter
            marker.scale.y = size_mm / 1000.0
            marker.scale.z = 0.03  # 30mm height

            # Color: orange when held
            marker.color.r = 1.0
            marker.color.g = 0.5
            marker.color.b = 0.0
            marker.color.a = 1.0

            marker.lifetime.sec = 0
            marker.lifetime.nanosec = 100000000  # 0.1 seconds
        else:
            # Delete the marker when not holding
            marker.action = Marker.DELETE

        self.held_weight_marker_pub.publish(marker)


def main(args=None):
    rclpy.init(args=args)

    node = SortingBrainNode()

    # Use MultiThreadedExecutor for concurrent callbacks
    executor = MultiThreadedExecutor()
    executor.add_node(node)

    try:
        node.get_logger().info('Sorting Brain Node running...')
        executor.spin()
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down...')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
