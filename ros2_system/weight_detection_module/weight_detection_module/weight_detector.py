#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from collections import deque
import numpy as np
from std_msgs.msg import Float32


class KalmanFilter:
    def __init__(self, process_variance=1e-5, measurement_variance=0.1):
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
        self.d1 = 0.08946
        self.a2 = -0.42500
        self.a3 = -0.39225
        self.d4 = 0.10915
        self.d5 = 0.09465
        self.d6 = 0.0823
        
        self.g = 9.81
    
    def forward_kinematics(self, joint_angles):
        q1, q2, q3, q4, q5, q6 = joint_angles
        
        x = (self.a2 * np.cos(q2) + 
             self.a3 * np.cos(q2 + q3) + 
             self.d4 * np.sin(q2 + q3 + q4))
        
        y = (self.a2 * np.sin(q2) + 
             self.a3 * np.sin(q2 + q3) + 
             self.d4 * np.cos(q2 + q3 + q4))
        
        return x, y
    
    def compute_moment_arms(self, joint_angles):
        q1, q2, q3, q4, q5, q6 = joint_angles
        
        ee_x, ee_y = self.forward_kinematics(joint_angles)
        
        r2 = ee_x
        
        elbow_x = self.a2 * np.cos(q2)
        elbow_y = self.a2 * np.sin(q2)
        r3 = ee_x - elbow_x
        
        wrist_x = self.a2 * np.cos(q2) + self.a3 * np.cos(q2 + q3)
        wrist_y = self.a2 * np.sin(q2) + self.a3 * np.sin(q2 + q3)
        r4 = ee_x - wrist_x
        
        return np.array([r2, r3, r4])
    
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
        
        self.calibration_factor = 5.75
        self.active_joints_for_estimation = [1,2]
        
        self.baseline_torques = None
        self.baseline_samples = []
        self.baseline_sample_size = 50
        self.calibrating_baseline = True
        
        self.current_joint_angles = None
        self.estimated_mass_grams = 0.0
        self.mass_history = deque(maxlen=self.history_length)
        
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

    def joint_state_callback(self, msg):
        joint_torques = msg.effort
        joint_positions = msg.position
        
        if len(joint_torques) != self.num_joints or len(joint_positions) != self.num_joints:
            return
        
        reordered_torques = [
            joint_torques[5],
            joint_torques[0],
            joint_torques[1],
            joint_torques[2],
            joint_torques[3],
            joint_torques[4]
        ]
        
        reordered_positions = [
            joint_positions[5],
            joint_positions[0],
            joint_positions[1],
            joint_positions[2],
            joint_positions[3],
            joint_positions[4]
        ]
        
        self.current_joint_angles = reordered_positions
        
        filtered_torques = []
        for i, torque in enumerate(reordered_torques):
            self.torque_history[i].append(torque)
            filtered_torque = self.kalman_filters[i].update(torque)
            self.filtered_torque_history[i].append(filtered_torque)
            filtered_torques.append(filtered_torque)
        
        if self.calibrating_baseline:
            self.baseline_samples.append(filtered_torques)
            if len(self.baseline_samples) >= self.baseline_sample_size:
                self.baseline_torques = np.mean(self.baseline_samples, axis=0)
                self.calibrating_baseline = False
                self.get_logger().info('Baseline calibration complete')
        else:
            self.estimate_mass_physics(filtered_torques)
        
        self.update_plot()
    
    def estimate_mass_physics(self, current_torques):
        if self.baseline_torques is None or self.current_joint_angles is None:
            return
        
        torque_deltas = np.array(current_torques) - self.baseline_torques
        
        moment_arms = self.kinematics.compute_moment_arms(self.current_joint_angles)
        
        selected_torque_deltas = torque_deltas[self.active_joints_for_estimation]
        selected_moment_arms = moment_arms[[j-1 for j in self.active_joints_for_estimation]]
        
        mass_kg = self.kinematics.estimate_mass(selected_torque_deltas, selected_moment_arms)
        
        mass_kg = max(0.0, mass_kg) * self.calibration_factor
        self.estimated_mass_grams = mass_kg * 1000.0
        self.mass_history.append(self.estimated_mass_grams)
        
        mass_msg = Float32()
        mass_msg.data = float(self.estimated_mass_grams)
        self.mass_publisher.publish(mass_msg)

    def update_plot(self):
        for i in range(self.num_joints):
            if len(self.torque_history[i]) > 0:
                x_data = np.arange(len(self.torque_history[i]))
                raw_data = np.array(self.torque_history[i])
                filtered_data = np.array(self.filtered_torque_history[i])
                
                self.raw_lines[i].set_data(x_data, raw_data)
                self.filtered_lines[i].set_data(x_data, filtered_data)
                self.axes[i].relim()
                self.axes[i].autoscale_view()
        
        if len(self.mass_history) > 0:
            x_mass = np.arange(len(self.mass_history))
            mass_data = np.array(self.mass_history)
            self.mass_line.set_data(x_mass, mass_data)
            self.mass_ax.relim()
            self.mass_ax.autoscale_view()
            self.mass_ax.set_title(f'Estimated Mass: {self.estimated_mass_grams:.1f} g')
        
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
        plt.close('all')


if __name__ == '__main__':
    main()