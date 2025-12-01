/**
 * @file simple_pick_and_weigh_node.cpp
 * @brief Simple Pick and Weigh Node - C++ Implementation
 *
 * Performs a simple pick-and-weigh operation:
 * 1. Moves to Z_DESCEND at specified X-Y
 * 2. Calls weight calibration
 * 3. Waits for calibration to complete
 * 4. Opens gripper
 * 5. Descends to Z_PICKUP
 * 6. Closes gripper
 * 7. Lifts to Z_DESCEND
 * 8. Waits for weight to stabilize
 * 9. Reads and displays the weight
 */

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/int32.hpp>
#include <sort_interfaces/srv/move_to_cartesian.hpp>
#include <sort_interfaces/srv/gripper_control.hpp>
#include <sort_interfaces/srv/calibrate_baseline.hpp>
#include <chrono>
#include <thread>
#include <mutex>
#include <memory>

namespace motion_control_module
{

class SimplePickAndWeighNode : public rclcpp::Node
{
public:
    // Z heights (mm) - tool0 frame
    static constexpr double Z_HOME = 371.0;
    static constexpr double Z_DESCEND = 212.0;
    static constexpr double Z_PICKUP = 182.0;

    // Default orientation (facing down)
    static constexpr double RX = 2.221;
    static constexpr double RY = 2.221;
    static constexpr double RZ = 0.0;

    SimplePickAndWeighNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("simple_pick_and_weigh", options)
    {
        // Declare parameters
        this->declare_parameter("grip_weight", 100);
        grip_weight_ = this->get_parameter("grip_weight").as_int();

        // Use a reentrant callback group
        callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        init_clients();
        init_subscribers();

        RCLCPP_INFO(this->get_logger(), "Simple Pick and Weigh Node initialized");
        RCLCPP_INFO(this->get_logger(), "Grip weight: %dg", grip_weight_);
        RCLCPP_INFO(this->get_logger(), "Waiting 2 seconds for all systems to be ready...");

        // Start sequence after 2 seconds using a one-shot timer
        start_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&SimplePickAndWeighNode::execute_pick_and_weigh, this),
            callback_group_);
    }

private:
    void init_clients()
    {
        // Motion control
        move_client_ = this->create_client<sort_interfaces::srv::MoveToCartesian>(
            "/motion_control/move_to_cartesian",
            rmw_qos_profile_services_default,
            callback_group_);

        // Gripper control
        gripper_client_ = this->create_client<sort_interfaces::srv::GripperControl>(
            "/motion_control/gripper_control",
            rmw_qos_profile_services_default,
            callback_group_);

        // Weight calibration
        calibrate_weight_client_ = this->create_client<sort_interfaces::srv::CalibrateBaseline>(
            "/weight_detection/calibrate_baseline",
            rmw_qos_profile_services_default,
            callback_group_);

        RCLCPP_INFO(this->get_logger(), "Waiting for services...");

        // Wait for services
        if (!move_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Move service not available!");
            throw std::runtime_error("Move service not available");
        }

        if (!gripper_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Gripper service not available!");
            throw std::runtime_error("Gripper service not available");
        }

        if (!calibrate_weight_client_->wait_for_service(std::chrono::seconds(10))) {
            RCLCPP_ERROR(this->get_logger(), "Weight calibration service not available!");
            throw std::runtime_error("Weight calibration service not available");
        }

        RCLCPP_INFO(this->get_logger(), "All services ready!");
    }

    void init_subscribers()
    {
        // Weight calibration status
        calibration_status_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "/weight_detection/calibration_status",
            10,
            std::bind(&SimplePickAndWeighNode::calibration_status_callback, this, std::placeholders::_1),
            rclcpp::SubscriptionOptions());

        // Weight measurement
        weight_sub_ = this->create_subscription<std_msgs::msg::Int32>(
            "/estimated_mass",
            10,
            std::bind(&SimplePickAndWeighNode::weight_callback, this, std::placeholders::_1),
            rclcpp::SubscriptionOptions());
    }

    void calibration_status_callback(const std_msgs::msg::Bool::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(calibration_mutex_);
        calibration_in_progress_ = msg->data;
        if (msg->data) {
            RCLCPP_INFO(this->get_logger(), "Weight calibration in progress...");
        } else {
            RCLCPP_INFO(this->get_logger(), "Weight calibration complete!");
        }
    }

    void weight_callback(const std_msgs::msg::Int32::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(weight_mutex_);
        current_weight_ = msg->data;
    }

    bool move_to(double x, double y, double z)
    {
        auto request = std::make_shared<sort_interfaces::srv::MoveToCartesian::Request>();
        request->x = x;
        request->y = y;
        request->z = z;
        request->rx = RX;
        request->ry = RY;
        request->rz = RZ;

        RCLCPP_INFO(this->get_logger(), "Moving to: X=%.1f, Y=%.1f, Z=%.1f", x, y, z);

        auto future = move_client_->async_send_request(request);

        // Wait for response (MultiThreadedExecutor handles callback processing)
        auto timeout = std::chrono::seconds(30);
        if (future.wait_for(timeout) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Move service call timed out");
            return false;
        }

        auto result = future.get();
        if (result && result->success) {
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Move failed!");
            return false;
        }
    }

    bool gripper_control(const std::string& command, int weight = 0)
    {
        auto request = std::make_shared<sort_interfaces::srv::GripperControl::Request>();
        request->command = command;
        request->weight = weight;

        std::string cmd_name = command;
        if (command == "W") cmd_name = "OPEN";
        else if (command == "S") cmd_name = "CLOSE";
        else if (command == "e") cmd_name = "SET_ANGLE(" + std::to_string(weight) + "g)";

        RCLCPP_INFO(this->get_logger(), "Gripper: %s", cmd_name.c_str());

        auto future = gripper_client_->async_send_request(request);

        auto timeout = std::chrono::seconds(10);
        if (future.wait_for(timeout) != std::future_status::ready) {
            RCLCPP_WARN(this->get_logger(), "Gripper command %s timed out", command.c_str());
            return false;
        }

        auto result = future.get();
        if (result && result->success) {
            return true;
        } else {
            RCLCPP_WARN(this->get_logger(), "Gripper command %s failed", command.c_str());
            return false;
        }
    }

    bool call_calibration()
    {
        RCLCPP_INFO(this->get_logger(), "Calling weight calibration service...");

        auto request = std::make_shared<sort_interfaces::srv::CalibrateBaseline::Request>();
        auto future = calibrate_weight_client_->async_send_request(request);

        auto timeout = std::chrono::seconds(10);
        if (future.wait_for(timeout) != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Calibration service call timed out");
            return false;
        }

        auto result = future.get();
        if (result && result->success) {
            RCLCPP_INFO(this->get_logger(), "Calibration service response: %s", result->message.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Calibration service call failed!");
            return false;
        }
    }

    bool wait_for_calibration(double timeout_sec = 10.0)
    {
        RCLCPP_INFO(this->get_logger(), "Waiting for calibration to complete...");

        auto start = std::chrono::steady_clock::now();
        while (rclcpp::ok()) {
            {
                std::lock_guard<std::mutex> lock(calibration_mutex_);
                if (!calibration_in_progress_) {
                    return true;
                }
            }

            auto elapsed = std::chrono::duration<double>(
                std::chrono::steady_clock::now() - start).count();
            if (elapsed > timeout_sec) {
                RCLCPP_ERROR(this->get_logger(), "Calibration timeout!");
                return false;
            }

            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }

        return false;
    }

    void execute_pick_and_weigh()
    {
        // Cancel the timer to make it one-shot
        start_timer_->cancel();

        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "STARTING SIMPLE PICK AND WEIGH");
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "");

        double default_x = -600.0;
        double default_y = -100.0;

        RCLCPP_INFO(this->get_logger(), "Using position: X=%.1f, Y=%.1f", default_x, default_y);
        RCLCPP_INFO(this->get_logger(), "");

        try {
            // Step 1: Move to Z_DESCEND
            RCLCPP_INFO(this->get_logger(), "[1/10] Moving to Z_DESCEND (calibration height)...");
            if (!move_to(default_x, default_y, Z_DESCEND)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to move to Z_DESCEND");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "");

            // Step 2: Call calibration
            RCLCPP_INFO(this->get_logger(), "[2/10] Calling calibration service...");
            if (!call_calibration()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to call calibration");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "");

            // Step 3: Wait for calibration
            RCLCPP_INFO(this->get_logger(), "[3/10] Waiting for calibration (~5.5 seconds)...");
            if (!wait_for_calibration(10.0)) {
                RCLCPP_ERROR(this->get_logger(), "Calibration failed or timed out");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "");

            // Step 4: Set gripper angle
            RCLCPP_INFO(this->get_logger(), "[4/10] Setting gripper angle for %dg...", grip_weight_);
            if (!gripper_control("e", grip_weight_)) {
                RCLCPP_WARN(this->get_logger(), "Failed to set gripper angle (continuing anyway)");
            }
            std::this_thread::sleep_for(std::chrono::seconds(1));
            RCLCPP_INFO(this->get_logger(), "");

            // Step 5: Open gripper
            RCLCPP_INFO(this->get_logger(), "[5/10] Opening gripper...");
            if (!gripper_control("W")) {
                RCLCPP_WARN(this->get_logger(), "Failed to open gripper (continuing anyway)");
            }
            std::this_thread::sleep_for(std::chrono::seconds(3));
            RCLCPP_INFO(this->get_logger(), "");

            // Step 6: Descend to Z_PICKUP
            RCLCPP_INFO(this->get_logger(), "[6/10] Descending to Z_PICKUP...");
            if (!move_to(default_x, default_y, Z_PICKUP)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to descend to Z_PICKUP");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "");

            // Step 7: Close gripper
            RCLCPP_INFO(this->get_logger(), "[7/10] Closing gripper...");
            if (!gripper_control("S")) {
                RCLCPP_WARN(this->get_logger(), "Failed to close gripper (continuing anyway)");
            }
            std::this_thread::sleep_for(std::chrono::seconds(3));
            RCLCPP_INFO(this->get_logger(), "");

            // Step 8: Lift to Z_DESCEND
            RCLCPP_INFO(this->get_logger(), "[8/10] Lifting to Z_DESCEND (weighing height)...");
            if (!move_to(default_x, default_y, Z_DESCEND)) {
                RCLCPP_ERROR(this->get_logger(), "Failed to lift to Z_DESCEND");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "");

            // Step 9: Wait for weight to stabilize
            RCLCPP_INFO(this->get_logger(), "[9/10] Waiting 10 seconds for weight to stabilize...");
            for (int i = 10; i > 0; i--) {
                RCLCPP_INFO(this->get_logger(), "    %d seconds remaining...", i);
                std::this_thread::sleep_for(std::chrono::seconds(1));
            }
            RCLCPP_INFO(this->get_logger(), "");

            // Step 10: Read weight
            RCLCPP_INFO(this->get_logger(), "[10/10] Reading weight from /estimated_mass...");
            std::this_thread::sleep_for(std::chrono::milliseconds(500));

            int weight;
            {
                std::lock_guard<std::mutex> lock(weight_mutex_);
                weight = current_weight_;
            }

            if (weight >= 0) {
                RCLCPP_INFO(this->get_logger(), "");
                RCLCPP_INFO(this->get_logger(), "============================================================");
                RCLCPP_INFO(this->get_logger(), "MEASURED WEIGHT: %d grams", weight);
                RCLCPP_INFO(this->get_logger(), "============================================================");
                RCLCPP_INFO(this->get_logger(), "");
            } else {
                RCLCPP_WARN(this->get_logger(), "No weight measurement received!");
                RCLCPP_WARN(this->get_logger(), "Check that weight_detection_module is running");
            }

            RCLCPP_INFO(this->get_logger(), "Pick and weigh sequence complete!");
            RCLCPP_INFO(this->get_logger(), "");
            RCLCPP_INFO(this->get_logger(), "Press Ctrl+C to exit");

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Error during pick and weigh: %s", e.what());
        }
    }

    // Member variables
    int grip_weight_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;
    rclcpp::TimerBase::SharedPtr start_timer_;

    rclcpp::Client<sort_interfaces::srv::MoveToCartesian>::SharedPtr move_client_;
    rclcpp::Client<sort_interfaces::srv::GripperControl>::SharedPtr gripper_client_;
    rclcpp::Client<sort_interfaces::srv::CalibrateBaseline>::SharedPtr calibrate_weight_client_;

    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr calibration_status_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr weight_sub_;

    bool calibration_in_progress_ = false;
    int current_weight_ = -1;
    std::mutex calibration_mutex_;
    std::mutex weight_mutex_;
};

}  // namespace motion_control_module

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<motion_control_module::SimplePickAndWeighNode>();

    // Use MultiThreadedExecutor to allow callbacks to run during service handling
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Simple Pick and Weigh Node running...");

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
