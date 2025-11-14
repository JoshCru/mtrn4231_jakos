#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float32
import numpy as np
from collections import deque
import matplotlib.pyplot as plt

class KalmanFilter:
    def __init__(self, process_variance=0.0015, measurement_variance=0.04):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = 0.0
        self.error_covariance = 1.0
        self.initialized = False
    
    def update(self, measurement):
        if not self.initialized:
            self.estimate = measurement
            self.initialized = True
            return self.estimate
        
        self.error_covariance += self.process_variance
        kalman_gain = self.error_covariance / (self.error_covariance + self.measurement_variance)
        self.estimate += kalman_gain * (measurement - self.estimate)
        self.error_covariance *= (1 - kalman_gain)
        
        return self.estimate

class UR5eKinematics:
    def __init__(self):
        self.d1 = 0.1625
        self.a2 = -0.425
        self.a3 = -0.3922
        self.d4 = 0.1333
        self.d5 = 0.0997
        self.d6 = 0.0996
        self.g = 9.81
    
    def compute_dh_transform(self, theta, d, a, alpha):
        ct = np.cos(theta)
        st = np.sin(theta)
        ca = np.cos(alpha)
        sa = np.sin(alpha)
        
        return np.array([
            [ct, -st*ca, st*sa, a*ct],
            [st, ct*ca, -ct*sa, a*st],
            [0, sa, ca, d],
            [0, 0, 0, 1]
        ])
    
    def forward_kinematics(self, joint_angles):
        q = joint_angles
        
        T01 = self.compute_dh_transform(q[0], self.d1, 0, np.pi/2)
        T12 = self.compute_dh_transform(q[1], 0, self.a2, 0)
        T23 = self.compute_dh_transform(q[2], 0, self.a3, 0)
        T34 = self.compute_dh_transform(q[3], self.d4, 0, np.pi/2)
        T45 = self.compute_dh_transform(q[4], self.d5, 0, -np.pi/2)
        T56 = self.compute_dh_transform(q[5], self.d6, 0, 0)
        
        T02 = T01 @ T12
        T03 = T02 @ T23
        T04 = T03 @ T34
        T05 = T04 @ T45
        T06 = T05 @ T56
        
        joint_positions = {
            'shoulder': T01[:3, 3],
            'elbow': T02[:3, 3],
            'wrist1': T03[:3, 3],
            'end_effector': T06[:3, 3]
        }
        
        return joint_positions, T01
    
    def compute_moment_arms(self, joint_angles):
        positions, T01 = self.forward_kinematics(joint_angles)
        ee_pos = positions['end_effector']
        
        joint2_pos = positions['shoulder']
        joint3_pos = positions['elbow']
        joint4_pos = positions['wrist1']
        
        joint2_axis = np.array([0, 0, 1])
        joint3_axis = T01[:3, :3] @ joint2_axis
        joint4_axis = joint3_axis
        
        vec_to_ee_j2 = ee_pos - joint2_pos
        vec_to_ee_j3 = ee_pos - joint3_pos
        vec_to_ee_j4 = ee_pos - joint4_pos
        
        moment_j2 = np.linalg.norm(np.cross(joint2_axis, vec_to_ee_j2))
        moment_j3 = np.linalg.norm(np.cross(joint3_axis, vec_to_ee_j3))
        moment_j4 = np.linalg.norm(np.cross(joint4_axis, vec_to_ee_j4))
        
        return np.array([moment_j2, moment_j3, moment_j4])
    
    def estimate_mass(self, torque_deltas, moment_arms):
        valid_indices = np.where(np.abs(moment_arms) > 0.01)[0]
        
        if len(valid_indices) == 0:
            return 0.0
        
        mass_estimates = []
        for idx in valid_indices:
            mass = torque_deltas[idx] / (self.g * moment_arms[idx])
            mass_estimates.append(mass)
        
        return np.median(mass_estimates)

class WeightDetector(Node):
    def __init__(self):
        super().__init__('weight_detector')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.mass_publisher = self.create_publisher(Float32, '/estimated_mass', 10)
        
        self.num_joints = 6
        self.history_length = 100
        self.torque_history = [deque(maxlen=self.history_length) for _ in range(self.num_joints)]
        self.filtered_torque_history = [deque(maxlen=self.history_length) for _ in range(self.num_joints)]
        
        self.kalman_filters = [KalmanFilter(process_variance=0.0015, measurement_variance=0.04) 
                               for _ in range(self.num_joints)]
        
        self.kinematics = UR5eKinematics()
        
        self.calibration_factor = 5.0
        self.active_joints = [0, 1]
        
        self.baseline_torques = None
        self.baseline_samples = []
        self.baseline_sample_size = 10
        self.calibrating_baseline = True
        
        self.current_joint_angles = None
        self.estimated_mass_grams = 0.0
        self.mass_history = deque(maxlen=self.history_length)
        
        self.declare_parameter('calibration_factor', self.calibration_factor)
        self.declare_parameter('active_joints', self.active_joints)
        
        self.param_timer = self.create_timer(0.5, self.update_parameters)
        
        plt.ion()
        self.fig = plt.figure(figsize=(14, 10))
        gs = self.fig.add_gridspec(3, 3, hspace=0.3)
        
        self.axes = []
        for i in range(6):
            row = i // 3
            col = i % 3
            ax = self.fig.add_subplot(gs[row, col])
            self.axes.append(ax)
        
        self.mass_ax = self.fig.add_subplot(gs[2, :])
        
        self.raw_lines = []
        self.filtered_lines = []
        
        for i in range(self.num_joints):
            raw_line, = self.axes[i].plot([], [], 'b-', alpha=0.3, label='Raw')
            filtered_line, = self.axes[i].plot([], [], 'r-', linewidth=2, label='Filtered')
            self.raw_lines.append(raw_line)
            self.filtered_lines.append(filtered_line)
            self.axes[i].set_xlabel('Sample')
            self.axes[i].set_ylabel('Torque (Nm)')
            self.axes[i].set_title(f'Joint {i+1}')
            self.axes[i].grid(True)
            self.axes[i].legend()
        
        self.mass_line, = self.mass_ax.plot([], [], 'g-', linewidth=2)
        self.mass_ax.set_xlabel('Sample')
        self.mass_ax.set_ylabel('Mass (g)')
        self.mass_ax.set_title('Estimated Mass')
        self.mass_ax.grid(True)
        
        self.get_logger().info('Weight detection module initialized - calibrating baseline...')
        self.get_logger().info('Using proper 3D moment arm calculations')
        self.get_logger().info('Note: UR5e joints reordered from [6,1,2,3,4,5] to [1,2,3,4,5,6] for display')
    
    def update_parameters(self):
        self.calibration_factor = self.get_parameter('calibration_factor').value
        self.active_joints = self.get_parameter('active_joints').value
    
    def joint_state_callback(self, msg):
        joint_torques = msg.effort[:self.num_joints] if len(msg.effort) >= self.num_joints else msg.effort
        joint_positions = msg.position[:self.num_joints] if len(msg.position) >= self.num_joints else msg.position
        
        if len(joint_torques) < self.num_joints or len(joint_positions) < self.num_joints:
            return
        
        # UR5e returns joints in order: [6, 1, 2, 3, 4, 5] (indices [5, 0, 1, 2, 3, 4])
        # Reorder to standard [1, 2, 3, 4, 5, 6] for display and physics
        reorder_indices = [5, 0, 1, 2, 3, 4]
        joint_torques_reordered = [joint_torques[i] for i in reorder_indices]
        joint_positions_reordered = [joint_positions[i] for i in reorder_indices]
        
        self.current_joint_angles = np.array(joint_positions_reordered)
        
        filtered_torques = []
        for i in range(self.num_joints):
            filtered_torque = self.kalman_filters[i].update(joint_torques_reordered[i])
            self.torque_history[i].append(joint_torques_reordered[i])
            self.filtered_torque_history[i].append(filtered_torque)
            filtered_torques.append(filtered_torque)
        
        # Use joints 2, 3, 4 (indices 1, 2, 3 after reordering)
        if self.calibrating_baseline:
            self.baseline_samples.append(filtered_torques[1:4])
            if len(self.baseline_samples) >= self.baseline_sample_size:
                self.baseline_torques = np.mean(self.baseline_samples, axis=0)
                self.calibrating_baseline = False
                self.get_logger().info("Baseline calibration complete")
                self.get_logger().info(f"Baseline torques for Joints 2,3,4: {self.baseline_torques}")
        else:
            self.estimate_mass_physics(filtered_torques[1:4])
        
        self.update_plots()
    
    def estimate_mass_physics(self, current_torques):
        if self.baseline_torques is None or self.current_joint_angles is None:
            return
        
        torque_deltas = np.array(current_torques) - self.baseline_torques
        
        moment_arms = self.kinematics.compute_moment_arms(self.current_joint_angles)
        
        selected_torque_deltas = torque_deltas[self.active_joints]
        selected_moment_arms = moment_arms[self.active_joints]
        
        raw_mass = self.kinematics.estimate_mass(selected_torque_deltas, selected_moment_arms)
        
        self.estimated_mass_grams = raw_mass * self.calibration_factor * 1000
        
        self.estimated_mass_grams = round(self.estimated_mass_grams / 5) * 5
        
        self.mass_history.append(self.estimated_mass_grams)
        
        mass_msg = Float32()
        mass_msg.data = self.estimated_mass_grams / 1000.0
        self.mass_publisher.publish(mass_msg)
    
    def update_plots(self):
        for i in range(self.num_joints):
            if len(self.torque_history[i]) > 0:
                self.raw_lines[i].set_data(range(len(self.torque_history[i])), 
                                          list(self.torque_history[i]))
                self.filtered_lines[i].set_data(range(len(self.filtered_torque_history[i])), 
                                               list(self.filtered_torque_history[i]))
                
                self.axes[i].relim()
                self.axes[i].autoscale_view()
        
        if len(self.mass_history) > 0:
            self.mass_line.set_data(range(len(self.mass_history)), 
                                   list(self.mass_history))
            self.mass_ax.relim()
            self.mass_ax.autoscale_view()
            
            self.mass_ax.set_title(f'Estimated Mass (Calib={self.calibration_factor:.2f}): {self.estimated_mass_grams:.2f}g')
        
        self.fig.canvas.draw()
        self.fig.canvas.flush_events()

def main(args=None):
    rclpy.init(args=args)
    weight_detector = WeightDetector()
    
    try:
        rclpy.spin(weight_detector)
    except KeyboardInterrupt:
        pass
    finally:
        weight_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()