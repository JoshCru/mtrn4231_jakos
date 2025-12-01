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
#include <fstream>
#include <iostream>
#include <map>

namespace motion_control_module
{

class SimplePickAndWeighNode : public rclcpp::Node
{
public:
    // Z heights (mm) - tool0 frame - loaded from config
    double Z_HOME = 371.0;
    double Z_DESCEND = 210.0;
    double Z_PICKUP = 180.0;
    double Z_PLACE = 180.0;

    // Default orientation (facing down) - loaded from config
    double RX = 2.221;
    double RY = 2.221;
    double RZ = 0.0;

    // Default positions - loaded from config
    double default_x = -600.0;
    double default_y = -100.0;

    SimplePickAndWeighNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("simple_pick_and_weigh", options)
    {
        // Declare parameters
        this->declare_parameter("grip_weight", 100);
        this->declare_parameter("initial_positioning", false);
        this->declare_parameter("config_path", "/home/joshc/mtrn4231_jakos/4231_scripts/robot_config.yaml");

        grip_weight_ = this->get_parameter("grip_weight").as_int();
        initial_positioning_ = this->get_parameter("initial_positioning").as_bool();
        std::string config_path = this->get_parameter("config_path").as_string();

        // Load config file
        load_robot_config(config_path);

        // Use a reentrant callback group
        callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        init_clients();
        init_subscribers();

        RCLCPP_INFO(this->get_logger(), "");
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "Simple Pick and Weigh Node Initialized");
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "Grip weight: %dg", grip_weight_);
        RCLCPP_INFO(this->get_logger(), "Weighting time: %d seconds", weighting_time_);
        RCLCPP_INFO(this->get_logger(), "Initial positioning: %s", initial_positioning_ ? "enabled" : "disabled");
        RCLCPP_INFO(this->get_logger(), "Config loaded from: %s", config_path.c_str());
        RCLCPP_INFO(this->get_logger(), "============================================================");
        RCLCPP_INFO(this->get_logger(), "Waiting 2 seconds for all systems to be ready...");
        RCLCPP_INFO(this->get_logger(), "");

        // Start sequence after 2 seconds using a one-shot timer
        start_timer_ = this->create_wall_timer(
            std::chrono::seconds(2),
            std::bind(&SimplePickAndWeighNode::execute_pick_and_weigh, this),
            callback_group_);
    }

private:
    void load_robot_config(const std::string& config_path)
    {
        RCLCPP_INFO(this->get_logger(), "Loading robot configuration from: %s", config_path.c_str());

        std::ifstream config_file(config_path);
        if (!config_file.is_open()) {
            RCLCPP_WARN(this->get_logger(), "Could not open config file, using defaults");
            weighting_time_ = 10;
            return;
        }

        std::string line;
        std::string current_section = "";
        int default_weighting_time = 10;

        // Maps to store weight-specific configurations
        std::map<int, int> weight_times;  // weight -> weighting_time

        while (std::getline(config_file, line)) {
            // Remove comments
            size_t comment_pos = line.find('#');
            if (comment_pos != std::string::npos) {
                line = line.substr(0, comment_pos);
            }

            // Trim whitespace
            line.erase(0, line.find_first_not_of(" \t"));
            line.erase(line.find_last_not_of(" \t") + 1);

            // Skip empty lines
            if (line.empty()) continue;

            // Check if this is a section header (ends with :)
            if (line.back() == ':') {
                current_section = line.substr(0, line.length() - 1);
                continue;
            }

            // Parse key: value pairs
            size_t colon_pos = line.find(':');
            if (colon_pos == std::string::npos) continue;

            std::string key = line.substr(0, colon_pos);
            std::string value = line.substr(colon_pos + 1);

            // Trim key and value
            key.erase(0, key.find_first_not_of(" \t"));
            key.erase(key.find_last_not_of(" \t") + 1);
            value.erase(0, value.find_first_not_of(" \t"));
            value.erase(value.find_last_not_of(" \t") + 1);

            try {
                // Parse based on current section and key
                if (current_section == "z_heights") {
                    if (key == "home") Z_HOME = std::stod(value);
                    else if (key == "descend") Z_DESCEND = std::stod(value);
                    else if (key == "pickup") Z_PICKUP = std::stod(value);
                    else if (key == "place") Z_PLACE = std::stod(value);
                }
                else if (current_section == "orientation") {
                    if (key == "rx") RX = std::stod(value);
                    else if (key == "ry") RY = std::stod(value);
                    else if (key == "rz") RZ = std::stod(value);
                }
                else if (current_section == "positions") {
                    if (key == "default_x") default_x = std::stod(value);
                    else if (key == "default_y") default_y = std::stod(value);
                }
                else if (current_section == "weighting_defaults") {
                    if (key == "default_time") default_weighting_time = std::stoi(value);
                }
                else if (current_section.find("weights") == 0) {
                    // We're in the weights section
                    // Check if key is a weight value (100, 200, 500)
                    if (key == "100" || key == "200" || key == "500") {
                        current_section = "weights/" + key;
                    }
                }
                else if (current_section.find("weights/") == 0) {
                    // We're in a specific weight subsection
                    std::string weight_str = current_section.substr(8); // Remove "weights/"
                    int weight_val = std::stoi(weight_str);

                    if (key == "weighting_time") {
                        weight_times[weight_val] = std::stoi(value);
                    }
                }
            } catch (const std::exception& e) {
                RCLCPP_WARN(this->get_logger(), "Error parsing config line '%s': %s",
                           line.c_str(), e.what());
            }
        }

        config_file.close();

        // Determine weighting time based on grip_weight
        if (weight_times.find(grip_weight_) != weight_times.end()) {
            weighting_time_ = weight_times[grip_weight_];
            RCLCPP_INFO(this->get_logger(), "Using configured weighting time of %ds for %dg",
                       weighting_time_, grip_weight_);
        } else {
            weighting_time_ = default_weighting_time;
            RCLCPP_INFO(this->get_logger(), "Using default weighting time of %ds for %dg",
                       weighting_time_, grip_weight_);
        }

        // Log loaded configuration
        RCLCPP_INFO(this->get_logger(), "Loaded config - Z_HOME: %.1f, Z_DESCEND: %.1f, Z_PICKUP: %.1f",
                   Z_HOME, Z_DESCEND, Z_PICKUP);
        RCLCPP_INFO(this->get_logger(), "Loaded config - Position: (%.1f, %.1f)", default_x, default_y);
        RCLCPP_INFO(this->get_logger(), "Loaded config - Orientation: (%.3f, %.3f, %.3f)", RX, RY, RZ);
    }

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
        RCLCPP_INFO(this->get_logger(), "Using position: X=%.1f, Y=%.1f", default_x, default_y);
        RCLCPP_INFO(this->get_logger(), "");

        try {
            // Initial positioning step (if enabled)
            if (initial_positioning_) {
                RCLCPP_INFO(this->get_logger(), "[INITIAL] Opening gripper for positioning check...");
                if (!gripper_control("W")) {
                    RCLCPP_WARN(this->get_logger(), "Failed to open gripper (continuing anyway)");
                }
                std::this_thread::sleep_for(std::chrono::seconds(2));
                RCLCPP_INFO(this->get_logger(), "");

                RCLCPP_INFO(this->get_logger(), "[INITIAL] Descending to Z_PICKUP for positioning check...");
                if (!move_to(default_x, default_y, Z_PICKUP)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to descend to Z_PICKUP");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "");

                RCLCPP_INFO(this->get_logger(), "*** POSITION CHECK ***");
                RCLCPP_INFO(this->get_logger(), "Robot is now at Z_PICKUP with gripper open.");
                RCLCPP_INFO(this->get_logger(), "Check the position and placement of the weight.");
                std::cout << "Press Enter to continue with the full procedure..." << std::flush;
                std::cin.get();
                RCLCPP_INFO(this->get_logger(), "");

                RCLCPP_INFO(this->get_logger(), "[INITIAL] Lifting back to Z_HOME (safe height)...");
                if (!move_to(default_x, default_y, Z_HOME)) {
                    RCLCPP_ERROR(this->get_logger(), "Failed to lift to Z_HOME");
                    return;
                }
                RCLCPP_INFO(this->get_logger(), "");
            }
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
            RCLCPP_INFO(this->get_logger(), "[9/10] Waiting %d seconds for weight to stabilize...", weighting_time_);
            for (int i = weighting_time_; i > 0; i--) {
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
    int weighting_time_ = 10;
    bool initial_positioning_ = false;
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
