/**
 * @file system_controller_node.cpp
 * @brief System supervisor node for controlling the sort-by-weight robot system
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sort_interfaces/msg/target_area.hpp"
#include "sort_interfaces/srv/system_command.hpp"
#include <vector>
#include <string>

class SystemControllerNode : public rclcpp::Node
{
public:
    SystemControllerNode() : Node("system_controller_node")
    {
        // Declare parameters
        this->declare_parameter("num_target_areas", 3);
        this->declare_parameter("auto_start", false);
        this->declare_parameter("publish_rate", 1.0);

        // Get parameters
        num_target_areas_ = this->get_parameter("num_target_areas").as_int();
        auto_start_ = this->get_parameter("auto_start").as_bool();
        double publish_rate = this->get_parameter("publish_rate").as_double();

        // Publishers
        command_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/system/commands", 10);
        target_areas_pub_ = this->create_publisher<sort_interfaces::msg::TargetArea>(
            "/system/target_areas", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/system/status", 10);

        // Services
        start_service_ = this->create_service<sort_interfaces::srv::SystemCommand>(
            "/system/start",
            std::bind(&SystemControllerNode::handle_start, this,
                     std::placeholders::_1, std::placeholders::_2));

        stop_service_ = this->create_service<sort_interfaces::srv::SystemCommand>(
            "/system/stop",
            std::bind(&SystemControllerNode::handle_stop, this,
                     std::placeholders::_1, std::placeholders::_2));

        emergency_stop_service_ = this->create_service<sort_interfaces::srv::SystemCommand>(
            "/system/emergency_stop",
            std::bind(&SystemControllerNode::handle_emergency_stop, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Timer for periodic publishing
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate)),
            std::bind(&SystemControllerNode::timer_callback, this));

        // Initialize target areas
        initialize_target_areas();

        RCLCPP_INFO(this->get_logger(), "System Controller Node initialized");

        if (auto_start_) {
            system_state_ = "running";
            RCLCPP_INFO(this->get_logger(), "Auto-start enabled, system starting...");
        } else {
            system_state_ = "idle";
        }
    }

private:
    void initialize_target_areas()
    {
        // TODO: Load from parameter file or configuration
        target_areas_.clear();

        for (int i = 0; i < num_target_areas_; ++i) {
            sort_interfaces::msg::TargetArea area;
            area.id = i;
            area.pose.position.x = 0.5 + i * 0.3;  // Example positions
            area.pose.position.y = 0.3;
            area.pose.position.z = 0.1;
            area.pose.orientation.w = 1.0;

            // Example weight ranges
            area.weight_min = i * 100.0f;
            area.weight_max = (i + 1) * 100.0f;
            area.label = "Area_" + std::to_string(i);

            target_areas_.push_back(area);
            RCLCPP_INFO(this->get_logger(),
                       "Target Area %d: %.1f-%.1f grams at [%.2f, %.2f, %.2f]",
                       area.id, area.weight_min, area.weight_max,
                       area.pose.position.x, area.pose.position.y, area.pose.position.z);
        }
    }

    void timer_callback()
    {
        // Publish system status
        auto status_msg = std_msgs::msg::String();
        status_msg.data = system_state_;
        status_pub_->publish(status_msg);

        // Publish target areas periodically
        for (const auto& area : target_areas_) {
            target_areas_pub_->publish(area);
        }
    }

    void handle_start(
        const std::shared_ptr<sort_interfaces::srv::SystemCommand::Request> request,
        std::shared_ptr<sort_interfaces::srv::SystemCommand::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Start service called: %s", request->command.c_str());

        if (system_state_ == "emergency_stopped") {
            response->success = false;
            response->message = "Cannot start: System is in emergency stop state. Reset required.";
            RCLCPP_WARN(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        system_state_ = "running";

        auto cmd_msg = std_msgs::msg::String();
        cmd_msg.data = "start";
        command_pub_->publish(cmd_msg);

        response->success = true;
        response->message = "System started successfully";
        RCLCPP_INFO(this->get_logger(), "System started");
    }

    void handle_stop(
        const std::shared_ptr<sort_interfaces::srv::SystemCommand::Request> request,
        std::shared_ptr<sort_interfaces::srv::SystemCommand::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Stop service called: %s", request->command.c_str());

        system_state_ = "stopped";

        auto cmd_msg = std_msgs::msg::String();
        cmd_msg.data = "stop";
        command_pub_->publish(cmd_msg);

        response->success = true;
        response->message = "System stopped successfully";
        RCLCPP_INFO(this->get_logger(), "System stopped");
    }

    void handle_emergency_stop(
        const std::shared_ptr<sort_interfaces::srv::SystemCommand::Request> request,
        std::shared_ptr<sort_interfaces::srv::SystemCommand::Response> response)
    {
        RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP activated: %s", request->command.c_str());

        system_state_ = "emergency_stopped";

        auto cmd_msg = std_msgs::msg::String();
        cmd_msg.data = "emergency_stop";
        command_pub_->publish(cmd_msg);

        response->success = true;
        response->message = "EMERGENCY STOP activated";
        RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP - All operations halted");
    }

    // Member variables
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr command_pub_;
    rclcpp::Publisher<sort_interfaces::msg::TargetArea>::SharedPtr target_areas_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;

    rclcpp::Service<sort_interfaces::srv::SystemCommand>::SharedPtr start_service_;
    rclcpp::Service<sort_interfaces::srv::SystemCommand>::SharedPtr stop_service_;
    rclcpp::Service<sort_interfaces::srv::SystemCommand>::SharedPtr emergency_stop_service_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::vector<sort_interfaces::msg::TargetArea> target_areas_;
    std::string system_state_;
    int num_target_areas_;
    bool auto_start_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<SystemControllerNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
