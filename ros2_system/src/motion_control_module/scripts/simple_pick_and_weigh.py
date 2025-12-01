#!/usr/bin/env python3
"""
Simple Pick and Weigh Script

Performs a simple pick-and-weigh operation from the robot's current X-Y position:
1. Moves to Z_DESCEND at current X-Y
2. Calls weight calibration
3. Waits for calibration to complete
4. Opens gripper
5. Descends to Z_PICKUP
6. Closes gripper
7. Lifts to Z_DESCEND
8. Waits 10 seconds for weight to stabilize
9. Reads and displays the weight from /estimated_mass

Usage:
    ros2 run motion_control_module simple_pick_and_weigh.py

    # With custom grip weight (for gripper angle):
    ros2 run motion_control_module simple_pick_and_weigh.py --ros-args -p grip_weight:=200

Prerequisites:
    - UR5e robot connected and controllers running
    - Weight detection module running
    - Gripper controller running and activated
    - Cartesian controller running
"""

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import Bool, Int32
from sensor_msgs.msg import JointState
from sort_interfaces.srv import MoveToCartesian, GripperControl, CalibrateBaseline
import time


class SimplePickAndWeigh(Node):
    # Z heights (mm) - tool0 frame
    Z_HOME = 371.0
    Z_DESCEND = 210.0
    Z_PICKUP = 180.0

    # Default orientation (facing down)
    RX = 2.221
    RY = 2.221
    RZ = 0.0

    def __init__(self):
        super().__init__('simple_pick_and_weigh')

        # Parameters
        self.declare_parameter('grip_weight', 100)  # Default grip weight
        self.grip_weight = self.get_parameter('grip_weight').value

        # Callback group for concurrent operations
        self.callback_group = ReentrantCallbackGroup()

        # State tracking
        self.calibration_in_progress = False
        self.current_weight = None
        self.current_x = None
        self.current_y = None
        self.current_z = None

        # Initialize clients
        self._init_clients()

        # Initialize subscribers
        self._init_subscribers()

        self.get_logger().info('Simple Pick and Weigh Node initialized')
        self.get_logger().info(f'Grip weight: {self.grip_weight}g')

        # Start the pick and weigh sequence after a short delay (using a one-shot timer)
        # This ensures the executor is spinning before we start making service calls
        self.get_logger().info('Waiting 2 seconds for all systems to be ready...')
        self.sequence_started = False
        self.start_timer = self.create_timer(
            2.0,
            self._timer_callback,
            callback_group=self.callback_group
        )

    def _timer_callback(self):
        """One-shot timer callback to start the sequence."""
        if not self.sequence_started:
            self.sequence_started = True
            self.start_timer.cancel()  # Make it one-shot
            self.execute_pick_and_weigh()

    def _init_clients(self):
        """Initialize service clients."""
        # Motion control
        self.move_client = self.create_client(
            MoveToCartesian,
            '/motion_control/move_to_cartesian',
            callback_group=self.callback_group
        )

        # Gripper control
        self.gripper_client = self.create_client(
            GripperControl,
            '/motion_control/gripper_control',
            callback_group=self.callback_group
        )

        # Weight calibration
        self.calibrate_weight_client = self.create_client(
            CalibrateBaseline,
            '/weight_detection/calibrate_baseline',
            callback_group=self.callback_group
        )

        self.get_logger().info('Waiting for services...')

        # Wait for services
        if not self.move_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Move service not available!')
            raise RuntimeError('Move service not available')

        if not self.gripper_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Gripper service not available!')
            raise RuntimeError('Gripper service not available')

        if not self.calibrate_weight_client.wait_for_service(timeout_sec=10.0):
            self.get_logger().error('Weight calibration service not available!')
            raise RuntimeError('Weight calibration service not available')

        self.get_logger().info('All services ready!')

    def _init_subscribers(self):
        """Initialize topic subscribers."""
        # Weight calibration status
        self.calibration_status_sub = self.create_subscription(
            Bool,
            '/weight_detection/calibration_status',
            self.calibration_status_callback,
            10,
            callback_group=self.callback_group
        )

        # Weight measurement
        self.weight_sub = self.create_subscription(
            Int32,
            '/estimated_mass',
            self.weight_callback,
            10,
            callback_group=self.callback_group
        )

        # Joint states for current position
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10,
            callback_group=self.callback_group
        )

    def calibration_status_callback(self, msg: Bool):
        """Callback for weight calibration status."""
        self.calibration_in_progress = msg.data
        if msg.data:
            self.get_logger().info('🔧 Weight calibration in progress...')
        else:
            self.get_logger().info('✅ Weight calibration complete!')

    def weight_callback(self, msg: Int32):
        """Callback for weight measurements."""
        self.current_weight = msg.data

    def joint_states_callback(self, msg: JointState):
        """Callback for joint states - we'll use this to get TCP position if needed."""
        # We don't strictly need this since we're staying at current X-Y,
        # but it's here if you want to extend functionality
        pass

    def move_to(self, x: float, y: float, z: float, rx: float = None, ry: float = None, rz: float = None) -> bool:
        """Move to Cartesian position."""
        if rx is None:
            rx = self.RX
        if ry is None:
            ry = self.RY
        if rz is None:
            rz = self.RZ

        req = MoveToCartesian.Request()
        req.x = x
        req.y = y
        req.z = z
        req.rx = rx
        req.ry = ry
        req.rz = rz

        self.get_logger().info(f'Moving to: X={x:.1f}, Y={y:.1f}, Z={z:.1f}')

        future = self.move_client.call_async(req)

        # Wait for future without blocking the executor
        # (same pattern as sorting_brain_node.py)
        timeout = 120.0
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                self.get_logger().error('Move service call timed out')
                return False
            time.sleep(0.05)  # Small sleep to avoid busy waiting

        result = future.result()
        if result is not None and result.success:
            self.current_x = x
            self.current_y = y
            self.current_z = z
            return True
        else:
            self.get_logger().error(f'Move failed!')
            return False

    def gripper_control(self, command: str, weight: int = 0) -> bool:
        """Control gripper."""
        req = GripperControl.Request()
        req.command = command
        req.weight = weight

        cmd_name = {'W': 'OPEN', 'S': 'CLOSE', 'e': f'SET_ANGLE({weight}g)'}
        self.get_logger().info(f'Gripper: {cmd_name.get(command, command)}')

        future = self.gripper_client.call_async(req)

        # Wait for future without blocking the executor
        # Add extra timeout for wait_time_sec
        timeout = 15.0
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                self.get_logger().warn(f'Gripper command {command} timed out')
                return False
            time.sleep(0.05)

        result = future.result()
        if result is not None and result.success:
            return True
        else:
            self.get_logger().warn(f'Gripper command {command} failed')
            return False

    def call_calibration(self) -> bool:
        """Call weight calibration service."""
        self.get_logger().info('📞 Calling weight calibration service...')

        req = CalibrateBaseline.Request()
        future = self.calibrate_weight_client.call_async(req)

        # Wait for future without blocking the executor
        timeout = 10.0
        start = time.time()
        while not future.done():
            if time.time() - start > timeout:
                self.get_logger().error('Calibration service call timed out')
                return False
            time.sleep(0.05)

        result = future.result()
        if result is not None and result.success:
            self.get_logger().info(f'Calibration service response: {result.message}')
            return True
        else:
            self.get_logger().error('Calibration service call failed!')
            return False

    def wait_for_calibration(self, timeout_sec: float = 10.0) -> bool:
        """Wait for calibration to complete."""
        self.get_logger().info('⏳ Waiting for calibration to complete...')

        start_time = time.time()
        while self.calibration_in_progress:
            if time.time() - start_time > timeout_sec:
                self.get_logger().error('Calibration timeout!')
                return False
            time.sleep(0.1)
            rclpy.spin_once(self, timeout_sec=0.01)

        return True

    def execute_pick_and_weigh(self):
        """Execute the full pick and weigh sequence."""
        self.get_logger().info('')
        self.get_logger().info('=' * 60)
        self.get_logger().info('STARTING SIMPLE PICK AND WEIGH')
        self.get_logger().info('=' * 60)
        self.get_logger().info('')

        # Note: We assume robot is already at a position above the object
        # We'll use current X-Y and just control Z

        # For this simple script, we'll use a default X-Y position
        # You can modify this to read current position from /tf or joint_states
        default_x = -600.0  # Center of workspace
        default_y = -100.0

        self.get_logger().info(f'Using position: X={default_x}, Y={default_y}')
        self.get_logger().info('')

        try:
            # Step 1: Move to Z_DESCEND (calibration height)
            self.get_logger().info('[1/10] Moving to Z_DESCEND (calibration height)...')
            if not self.move_to(default_x, default_y, self.Z_DESCEND):
                self.get_logger().error('Failed to move to Z_DESCEND')
                return
            self.get_logger().info('')

            # Step 2: Call calibration service
            self.get_logger().info('[2/10] Calling calibration service...')
            if not self.call_calibration():
                self.get_logger().error('Failed to call calibration')
                return
            self.get_logger().info('')

            # Step 3: Wait for calibration to complete
            self.get_logger().info('[3/10] Waiting for calibration (~5.5 seconds)...')
            if not self.wait_for_calibration(timeout_sec=10.0):
                self.get_logger().error('Calibration failed or timed out')
                return
            self.get_logger().info('')

            # Step 4: Set gripper angle for perceived weight
            self.get_logger().info(f'[4/10] Setting gripper angle for {self.grip_weight}g...')
            if not self.gripper_control('e', weight=self.grip_weight):
                self.get_logger().warn('Failed to set gripper angle (continuing anyway)')
            time.sleep(1.0)
            self.get_logger().info('')

            # Step 5: Open gripper
            self.get_logger().info('[5/10] Opening gripper...')
            if not self.gripper_control('W'):
                self.get_logger().warn('Failed to open gripper (continuing anyway)')
            time.sleep(3.0)
            self.get_logger().info('')

            # Step 6: Descend to Z_PICKUP
            self.get_logger().info('[6/10] Descending to Z_PICKUP...')
            if not self.move_to(default_x, default_y, self.Z_PICKUP):
                self.get_logger().error('Failed to descend to Z_PICKUP')
                return
            self.get_logger().info('')

            # Step 7: Close gripper
            self.get_logger().info('[7/10] Closing gripper...')
            if not self.gripper_control('S'):
                self.get_logger().warn('Failed to close gripper (continuing anyway)')
            time.sleep(3.0)
            self.get_logger().info('')

            # Step 8: Lift to Z_DESCEND (MUST be same height as calibration!)
            self.get_logger().info('[8/10] Lifting to Z_DESCEND (weighing height)...')
            if not self.move_to(default_x, default_y, self.Z_DESCEND):
                self.get_logger().error('Failed to lift to Z_DESCEND')
                return
            self.get_logger().info('')

            # Step 9: Wait for weight to stabilize
            self.get_logger().info('[9/10] Waiting 10 seconds for weight to stabilize...')
            for i in range(10, 0, -1):
                self.get_logger().info(f'    ⏱️  {i} seconds remaining...')
                time.sleep(1.0)
                rclpy.spin_once(self, timeout_sec=0.01)
            self.get_logger().info('')

            # Step 10: Read weight
            self.get_logger().info('[10/10] Reading weight from /estimated_mass...')

            # Spin a few times to make sure we get the latest weight
            for _ in range(10):
                rclpy.spin_once(self, timeout_sec=0.1)

            if self.current_weight is not None:
                self.get_logger().info('')
                self.get_logger().info('=' * 60)
                self.get_logger().info(f'⚖️  MEASURED WEIGHT: {self.current_weight} grams')
                self.get_logger().info('=' * 60)
                self.get_logger().info('')
            else:
                self.get_logger().warn('No weight measurement received!')
                self.get_logger().warn('Check that weight_detection_module is running')

            # Success!
            self.get_logger().info('✅ Pick and weigh sequence complete!')
            self.get_logger().info('')
            self.get_logger().info('Press Ctrl+C to exit')

        except Exception as e:
            self.get_logger().error(f'Error during pick and weigh: {e}')
            import traceback
            traceback.print_exc()


def main(args=None):
    rclpy.init(args=args)

    try:
        node = SimplePickAndWeigh()

        # Use MultiThreadedExecutor for concurrent callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(node)

        # Keep node alive to show final weight
        node.get_logger().info('Simple Pick and Weigh Node running...')
        executor.spin()

    except KeyboardInterrupt:
        print('\nShutdown requested by user')
    except Exception as e:
        print(f'Error: {e}')
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
