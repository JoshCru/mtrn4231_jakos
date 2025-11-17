/**
 * @file gripper_button_interface.cpp
 * @brief Simple keyboard/button interface for manually controlling the gripper
 *
 * Press 'w' to open gripper
 * Press 's' to close gripper
 * Press 'q' to quit
 */

#include <rclcpp/rclcpp.hpp>
#include "sort_interfaces/srv/gripper_control.hpp"
#include "sort_interfaces/msg/force_feedback.hpp"
#include <iostream>
#include <termios.h>
#include <unistd.h>
#include <fcntl.h>

class GripperButtonInterface : public rclcpp::Node
{
public:
    GripperButtonInterface() : Node("gripper_button_interface")
    {
        // Create service client
        gripper_client_ = this->create_client<sort_interfaces::srv::GripperControl>(
            "/motion_control/gripper_control");

        // Subscribe to force feedback
        force_sub_ = this->create_subscription<sort_interfaces::msg::ForceFeedback>(
            "/motion_control/force_feedback", 10,
            std::bind(&GripperButtonInterface::force_callback, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Gripper Button Interface started");
        RCLCPP_INFO(this->get_logger(), "Waiting for gripper controller service...");

        // Wait for service to be available
        while (!gripper_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Still waiting for gripper controller...");
        }

        RCLCPP_INFO(this->get_logger(), "Gripper controller service found!");
        print_instructions();

        // Set terminal to raw mode for non-blocking keyboard input
        set_terminal_raw_mode();
    }

    ~GripperButtonInterface()
    {
        // Restore terminal settings
        restore_terminal_mode();
    }

    void run()
    {
        char key;
        while (rclcpp::ok()) {
            // Check for keyboard input (non-blocking)
            if (kbhit()) {
                key = getchar();

                if (key == 'w' || key == 'W') {
                    send_gripper_command("open");
                } else if (key == 's' || key == 'S') {
                    send_gripper_command("close");
                } else if (key == 'q' || key == 'Q') {
                    RCLCPP_INFO(this->get_logger(), "Quitting...");
                    break;
                } else if (key == 'h' || key == 'H') {
                    print_instructions();
                }
            }

            // Spin ROS callbacks
            rclcpp::spin_some(this->get_node_base_interface());
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
        }
    }

private:
    void send_gripper_command(const std::string& command)
    {
        auto request = std::make_shared<sort_interfaces::srv::GripperControl::Request>();
        request->command = command;

        RCLCPP_INFO(this->get_logger(), "Sending command: %s", command.c_str());

        auto result_future = gripper_client_->async_send_request(request);

        // Wait for result (with timeout)
        auto status = result_future.wait_for(std::chrono::seconds(5));
        if (status == std::future_status::ready) {
            auto result = result_future.get();
            if (result->success) {
                RCLCPP_INFO(this->get_logger(), "%s (position: %.2f)",
                           result->message.c_str(), result->final_position);
            } else {
                RCLCPP_WARN(this->get_logger(), "Command failed: %s", result->message.c_str());
            }
        } else {
            RCLCPP_WARN(this->get_logger(), "Service call timed out");
        }
    }

    void force_callback(const sort_interfaces::msg::ForceFeedback::SharedPtr msg)
    {
        // Optionally display force feedback
        if (msg->object_detected) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                               "Object detected: %.1fg (%.2fN)",
                               msg->measured_weight, msg->gripper_force);
        }
    }

    void print_instructions()
    {
        std::cout << "\n";
        std::cout << "======================================\n";
        std::cout << "    GRIPPER MANUAL CONTROL\n";
        std::cout << "======================================\n";
        std::cout << "  [W] - Open gripper\n";
        std::cout << "  [S] - Close gripper\n";
        std::cout << "  [H] - Show this help\n";
        std::cout << "  [Q] - Quit\n";
        std::cout << "======================================\n";
        std::cout << "\n";
    }

    // Terminal manipulation for non-blocking input
    void set_terminal_raw_mode()
    {
        tcgetattr(STDIN_FILENO, &orig_termios_);
        struct termios raw = orig_termios_;
        raw.c_lflag &= ~(ICANON | ECHO);
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &raw);

        // Set non-blocking
        int flags = fcntl(STDIN_FILENO, F_GETFL, 0);
        fcntl(STDIN_FILENO, F_SETFL, flags | O_NONBLOCK);
    }

    void restore_terminal_mode()
    {
        tcsetattr(STDIN_FILENO, TCSAFLUSH, &orig_termios_);
    }

    int kbhit()
    {
        struct timeval tv = { 0L, 0L };
        fd_set fds;
        FD_ZERO(&fds);
        FD_SET(STDIN_FILENO, &fds);
        return select(STDIN_FILENO + 1, &fds, NULL, NULL, &tv) > 0;
    }

    rclcpp::Client<sort_interfaces::srv::GripperControl>::SharedPtr gripper_client_;
    rclcpp::Subscription<sort_interfaces::msg::ForceFeedback>::SharedPtr force_sub_;
    struct termios orig_termios_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<GripperButtonInterface>();
    node->run();
    rclcpp::shutdown();
    return 0;
}
