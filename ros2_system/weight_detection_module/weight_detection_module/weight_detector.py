#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
import matplotlib.pyplot as plt
from collections import deque
import numpy as np


class KalmanFilter:
    def __init__(self, process_variance=1e-2, measurement_variance=0.1):
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


class WeightDetector(Node):
    def __init__(self):
        super().__init__('weight_detector')
        
        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10
        )
        
        self.num_joints = 6
        self.history_length = 100
        self.torque_history = [deque(maxlen=self.history_length) for _ in range(self.num_joints)]
        self.filtered_torque_history = [deque(maxlen=self.history_length) for _ in range(self.num_joints)]
        
        self.kalman_filters = [KalmanFilter(process_variance=0.001, measurement_variance=0.01) 
                               for _ in range(self.num_joints)]
        
        plt.ion()
        self.fig, self.axes = plt.subplots(2, 3, figsize=(12, 8))
        self.axes = self.axes.flatten()
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
        
        self.fig.tight_layout()
        
        self.get_logger().info('Weight detection module initialized')

    def joint_state_callback(self, msg):
        joint_torques = msg.effort
        
        if len(joint_torques) != self.num_joints:
            return
        
        reordered_torques = [
            joint_torques[5],
            joint_torques[0],
            joint_torques[1],
            joint_torques[2],
            joint_torques[3],
            joint_torques[4]
        ]
        
        for i, torque in enumerate(reordered_torques):
            self.torque_history[i].append(torque)
            filtered_torque = self.kalman_filters[i].update(torque)
            self.filtered_torque_history[i].append(filtered_torque)
        
        self.update_plot()

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