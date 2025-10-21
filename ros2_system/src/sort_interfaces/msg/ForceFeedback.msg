# Gripper force and weight sensor feedback
std_msgs/Header header
float32 gripper_force     # in Newtons
float32 measured_weight   # in grams
float32 gripper_position  # 0.0=open, 1.0=closed
bool object_detected
float32 raw_sensor_value
