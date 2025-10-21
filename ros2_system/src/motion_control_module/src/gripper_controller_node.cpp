/**
 * @file gripper_controller_node.cpp
 * @brief Arduino servo gripper controller with weight sensing (lifecycle node)
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/string.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include "sort_interfaces/msg/force_feedback.hpp"
#include "sort_interfaces/srv/calibrate_gripper.hpp"
#include <vector>
#include <deque>

// TODO: Include serial library for Arduino communication
// #include <serial/serial.h>

using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class GripperControllerNode : public LifecycleNode
{
public:
    GripperControllerNode() : LifecycleNode("gripper_controller_node")
    {
        // Declare parameters
        this->declare_parameter("serial_port", "/dev/ttyACM0");
        this->declare_parameter("baud_rate", 115200);
        this->declare_parameter("publish_rate", 20.0);  // Hz
        this->declare_parameter("force_sensor_pin", "A0");
        this->declare_parameter("servo_pin", 9);
        this->declare_parameter("min_servo_angle", 0);
        this->declare_parameter("max_servo_angle", 180);
        this->declare_parameter("filter_window_size", 10);
        this->declare_parameter("weight_calibration_factor", 1.0);
        this->declare_parameter("zero_offset", 0.0);
        this->declare_parameter("object_detection_threshold", 50.0);  // grams
        this->declare_parameter("enable_joystick_control", false);

        RCLCPP_INFO(this->get_logger(), "Gripper Controller Node constructed");
    }

    // Lifecycle callbacks
    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "Configuring...");

        // Get parameters
        serial_port_ = this->get_parameter("serial_port").as_string();
        baud_rate_ = this->get_parameter("baud_rate").as_int();
        double publish_rate = this->get_parameter("publish_rate").as_double();
        filter_window_size_ = this->get_parameter("filter_window_size").as_int();
        calibration_factor_ = this->get_parameter("weight_calibration_factor").as_double();
        zero_offset_ = this->get_parameter("zero_offset").as_double();
        object_threshold_ = this->get_parameter("object_detection_threshold").as_double();
        enable_joystick_ = this->get_parameter("enable_joystick_control").as_bool();

        // Subscribers
        gripper_command_sub_ = this->create_subscription<std_msgs::msg::Float32>(
            "/motion_control/gripper_command", 10,
            std::bind(&GripperControllerNode::gripper_command_callback, this,
                     std::placeholders::_1));

        if (enable_joystick_) {
            joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
                "/joy", 10,
                std::bind(&GripperControllerNode::joy_callback, this,
                         std::placeholders::_1));
        }

        // Publishers
        gripper_state_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/motion_control/gripper_state", 10);
        force_feedback_pub_ = this->create_publisher<sort_interfaces::msg::ForceFeedback>(
            "/motion_control/force_feedback", 10);

        // Service
        calibration_service_ = this->create_service<sort_interfaces::srv::CalibrateGripper>(
            "/motion_control/calibrate_gripper",
            std::bind(&GripperControllerNode::handle_calibration, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Timer for reading sensor and publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            std::bind(&GripperControllerNode::timer_callback, this));

        RCLCPP_INFO(this->get_logger(), "Serial port: %s @ %d baud", serial_port_.c_str(), baud_rate_);
        RCLCPP_WARN(this->get_logger(),
                   "TODO: Initialize serial connection to Arduino");

        // TODO: Open serial port
        // try {
        //     serial_ = std::make_unique<serial::Serial>(serial_port_, baud_rate_,
        //                                                serial::Timeout::simpleTimeout(1000));
        //     if (serial_->isOpen()) {
        //         RCLCPP_INFO(this->get_logger(), "Serial port opened successfully");
        //     }
        // } catch (serial::IOException& e) {
        //     RCLCPP_ERROR(this->get_logger(), "Failed to open serial port: %s", e.what());
        //     return LifecycleCallbackReturn::FAILURE;
        // }

        current_position_ = 0.0f;
        is_configured_ = true;

        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "Activating...");

        // TODO: Send initialization commands to Arduino
        // Example: Reset servo to neutral position
        send_gripper_command(0.0f);

        is_active_ = true;

        RCLCPP_INFO(this->get_logger(), "Gripper controller activated");
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "Deactivating...");

        is_active_ = false;

        // TODO: Send safe state command to Arduino (e.g., open gripper)
        send_gripper_command(0.0f);

        RCLCPP_INFO(this->get_logger(), "Gripper controller deactivated");
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "Cleaning up...");

        // TODO: Close serial port
        // if (serial_ && serial_->isOpen()) {
        //     serial_->close();
        // }

        timer_.reset();
        is_configured_ = false;

        RCLCPP_INFO(this->get_logger(), "Gripper controller cleaned up");
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "Shutting down...");

        // Safe shutdown
        send_gripper_command(0.0f);  // Open gripper

        return LifecycleCallbackReturn::SUCCESS;
    }

private:
    void gripper_command_callback(const std_msgs::msg::Float32::SharedPtr msg)
    {
        if (!is_active_) {
            RCLCPP_WARN(this->get_logger(), "Gripper command ignored: Node not active");
            return;
        }

        float position = std::clamp(msg->data, 0.0f, 1.0f);
        RCLCPP_DEBUG(this->get_logger(), "Gripper command: %.2f", position);

        send_gripper_command(position);
    }

    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
    {
        if (!is_active_ || !enable_joystick_) {
            return;
        }

        // Example: Use joystick buttons/axes for manual gripper control
        // Button 0: Open, Button 1: Close
        if (msg->buttons.size() >= 2) {
            if (msg->buttons[0]) {
                send_gripper_command(0.0f);  // Open
            } else if (msg->buttons[1]) {
                send_gripper_command(1.0f);  // Close
            }
        }
    }

    void timer_callback()
    {
        if (!is_active_) {
            return;
        }

        // TODO: Read force sensor data from Arduino
        float raw_force = read_force_sensor();

        // Apply calibration and filtering
        float calibrated_weight = apply_calibration(raw_force);
        float filtered_weight = apply_filter(calibrated_weight);

        // Publish force feedback
        auto feedback_msg = sort_interfaces::msg::ForceFeedback();
        feedback_msg.header.stamp = this->now();
        feedback_msg.gripper_force = filtered_weight / 9.81f;  // Convert grams to Newtons
        feedback_msg.measured_weight = filtered_weight;
        feedback_msg.gripper_position = current_position_;
        feedback_msg.object_detected = (filtered_weight > object_threshold_);
        feedback_msg.raw_sensor_value = raw_force;

        force_feedback_pub_->publish(feedback_msg);

        // Publish gripper state
        auto state_msg = std_msgs::msg::Float32();
        state_msg.data = current_position_;
        gripper_state_pub_->publish(state_msg);
    }

    void send_gripper_command(float position)
    {
        current_position_ = position;

        // TODO: Send command to Arduino via serial
        // Protocol example: "G<angle>\n" where angle = position * 180
        int angle = static_cast<int>(position * 180.0f);

        std::string command = "G" + std::to_string(angle) + "\n";

        RCLCPP_DEBUG(this->get_logger(), "Sending to Arduino: %s", command.c_str());

        // if (serial_ && serial_->isOpen()) {
        //     serial_->write(command);
        // } else {
        //     RCLCPP_WARN(this->get_logger(), "Serial port not open");
        // }
    }

    float read_force_sensor()
    {
        // TODO: Read from Arduino via serial
        // Protocol example: Arduino sends "W<value>\n" where value is ADC reading

        // if (serial_ && serial_->isOpen() && serial_->available()) {
        //     std::string response = serial_->readline();
        //     if (response[0] == 'W') {
        //         return std::stof(response.substr(1));
        //     }
        // }

        // Placeholder: return simulated weight based on gripper position
        if (current_position_ > 0.5f) {
            return 150.0f + (rand() % 20 - 10);  // Simulated: 140-160 grams
        }
        return 0.0f + (rand() % 5);  // Noise when open
    }

    float apply_calibration(float raw_value)
    {
        // Apply calibration: weight = (raw - offset) * factor
        return (raw_value - zero_offset_) * calibration_factor_;
    }

    float apply_filter(float value)
    {
        // Moving average filter
        weight_buffer_.push_back(value);

        if (weight_buffer_.size() > static_cast<size_t>(filter_window_size_)) {
            weight_buffer_.pop_front();
        }

        float sum = 0.0f;
        for (float w : weight_buffer_) {
            sum += w;
        }

        return sum / weight_buffer_.size();
    }

    void handle_calibration(
        const std::shared_ptr<sort_interfaces::srv::CalibrateGripper::Request> request,
        std::shared_ptr<sort_interfaces::srv::CalibrateGripper::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Calibration requested");

        if (request->tare_weight_sensor) {
            // Tare: set current reading as zero offset
            float current_reading = read_force_sensor();
            zero_offset_ = current_reading;
            RCLCPP_INFO(this->get_logger(), "Weight sensor tared (offset: %.2f)", zero_offset_);
        }

        if (request->calibrate_position) {
            // TODO: Calibrate gripper position limits
            send_gripper_command(0.0f);
            rclcpp::sleep_for(std::chrono::seconds(1));
            response->min_position = 0.0f;

            send_gripper_command(1.0f);
            rclcpp::sleep_for(std::chrono::seconds(1));
            response->max_position = 1.0f;
        }

        response->zero_force_offset = zero_offset_;
        response->success = true;
        response->message = "Calibration completed";

        RCLCPP_INFO(this->get_logger(), "Calibration complete");
    }

    // Member variables
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_command_sub_;
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;

    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::Float32>::SharedPtr gripper_state_pub_;
    rclcpp_lifecycle::LifecyclePublisher<sort_interfaces::msg::ForceFeedback>::SharedPtr force_feedback_pub_;

    rclcpp::Service<sort_interfaces::srv::CalibrateGripper>::SharedPtr calibration_service_;

    rclcpp::TimerBase::SharedPtr timer_;

    // TODO: Serial port
    // std::unique_ptr<serial::Serial> serial_;

    std::string serial_port_;
    int baud_rate_;
    int filter_window_size_;
    double calibration_factor_;
    double zero_offset_;
    double object_threshold_;
    bool enable_joystick_;
    bool is_configured_;
    bool is_active_;

    float current_position_;
    std::deque<float> weight_buffer_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperControllerNode>();

    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
