/**
 * @file brain_node.hpp
 * @brief Central Brain Node for monitoring and orchestrating the sort-by-weight robot system
 */

#ifndef SUPERVISOR_MODULE__BRAIN_NODE_HPP_
#define SUPERVISOR_MODULE__BRAIN_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>

#include "sort_interfaces/msg/detected_objects.hpp"
#include "sort_interfaces/msg/weight_estimate.hpp"
#include "sort_interfaces/msg/sort_decision.hpp"
#include "sort_interfaces/msg/force_feedback.hpp"
#include "sort_interfaces/msg/target_area.hpp"
#include "sort_interfaces/srv/system_command.hpp"
#include "sort_interfaces/srv/gripper_control.hpp"
#include "sort_interfaces/srv/move_to_cartesian.hpp"
#include <std_srvs/srv/set_bool.hpp>

#include <string>
#include <vector>
#include <map>
#include <deque>
#include <mutex>
#include <chrono>
#include <sstream>
#include <iomanip>

namespace supervisor_module
{

/**
 * @brief System state enumeration
 */
enum class SystemState
{
    IDLE,
    INITIALIZING,
    RUNNING,
    PAUSED,
    ERROR,
    EMERGENCY_STOP,
    SHUTDOWN
};

/**
 * @brief Convert SystemState to string
 */
inline std::string state_to_string(SystemState state)
{
    switch (state)
    {
        case SystemState::IDLE: return "IDLE";
        case SystemState::INITIALIZING: return "INITIALIZING";
        case SystemState::RUNNING: return "RUNNING";
        case SystemState::PAUSED: return "PAUSED";
        case SystemState::ERROR: return "ERROR";
        case SystemState::EMERGENCY_STOP: return "EMERGENCY_STOP";
        case SystemState::SHUTDOWN: return "SHUTDOWN";
        default: return "UNKNOWN";
    }
}

/**
 * @brief Node health status structure
 */
struct NodeStatus
{
    std::string name;
    bool is_alive = false;
    std::chrono::system_clock::time_point last_seen;
    uint64_t message_count = 0;
    std::chrono::system_clock::time_point last_message;
};

/**
 * @brief Topic monitor structure
 */
struct TopicMonitor
{
    std::string topic_name;
    std::string msg_type;
    uint64_t message_count = 0;
    std::chrono::system_clock::time_point last_message_time;
    std::string last_message_data;
    std::deque<std::string> history;
    double frequency_hz = 0.0;
};

/**
 * @brief Event log entry
 */
struct EventEntry
{
    std::chrono::system_clock::time_point timestamp;
    std::string type;
    std::string message;
    std::string data;
};

/**
 * @brief Main Brain Node class
 */
class BrainNode : public rclcpp::Node
{
public:
    explicit BrainNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions());
    ~BrainNode() = default;

    // Public accessors for UI
    SystemState get_system_state() const { return system_state_; }
    std::string get_all_node_statuses() const;
    std::string get_all_topic_data() const;
    std::vector<EventEntry> get_recent_events(size_t count = 50) const;
    std::string get_system_metrics() const;

private:
    // Initialization
    void init_parameters();
    void init_publishers();
    void init_subscribers();
    void init_services();
    void init_timers();

    // Node discovery and health
    void discover_nodes();
    void check_node_health();
    void check_critical_nodes();

    // Topic callbacks
    void system_status_callback(const std_msgs::msg::String::SharedPtr msg);
    void system_commands_callback(const std_msgs::msg::String::SharedPtr msg);
    void detected_objects_callback(const sort_interfaces::msg::DetectedObjects::SharedPtr msg);
    void weight_estimate_callback(const sort_interfaces::msg::WeightEstimate::SharedPtr msg);
    void sort_decision_callback(const sort_interfaces::msg::SortDecision::SharedPtr msg);
    void robot_status_callback(const std_msgs::msg::String::SharedPtr msg);
    void joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg);
    void gripper_state_callback(const std_msgs::msg::Float32::SharedPtr msg);
    void force_feedback_callback(const sort_interfaces::msg::ForceFeedback::SharedPtr msg);
    void target_area_callback(const sort_interfaces::msg::TargetArea::SharedPtr msg);

    // State management
    void transition_state(SystemState new_state, const std::string& reason = "");
    void log_state_change(SystemState state, const std::string& reason);
    void log_event(const std::string& message, const std::string& type = "INFO");

    // Publishing
    void publish_brain_status();
    void publish_system_metrics();
    void publish_node_health();
    void publish_topic_stats();

    // Service handlers
    void handle_brain_command(
        const std::shared_ptr<sort_interfaces::srv::SystemCommand::Request> request,
        std::shared_ptr<sort_interfaces::srv::SystemCommand::Response> response);
    void handle_get_status(
        const std::shared_ptr<sort_interfaces::srv::SystemCommand::Request> request,
        std::shared_ptr<sort_interfaces::srv::SystemCommand::Response> response);

    // Helper methods
    void log_topic_data(const std::string& topic_name, const std::string& data);
    void update_node_activity(const std::string& node_name);
    std::string get_timestamp_string() const;
    double get_uptime_seconds() const;
    std::string escape_json_string(const std::string& input) const;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr brain_status_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr brain_command_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr node_health_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr topic_stats_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr system_metrics_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr event_log_pub_;

    // Subscribers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_status_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_commands_sub_;
    rclcpp::Subscription<sort_interfaces::msg::DetectedObjects>::SharedPtr detected_objects_sub_;
    rclcpp::Subscription<sort_interfaces::msg::WeightEstimate>::SharedPtr weight_estimate_sub_;
    rclcpp::Subscription<sort_interfaces::msg::SortDecision>::SharedPtr sort_decision_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr robot_status_sub_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_sub_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr gripper_state_sub_;
    rclcpp::Subscription<sort_interfaces::msg::ForceFeedback>::SharedPtr force_feedback_sub_;
    rclcpp::Subscription<sort_interfaces::msg::TargetArea>::SharedPtr target_area_sub_;

    // Service clients
    rclcpp::Client<sort_interfaces::srv::SystemCommand>::SharedPtr start_client_;
    rclcpp::Client<sort_interfaces::srv::SystemCommand>::SharedPtr stop_client_;
    rclcpp::Client<sort_interfaces::srv::SystemCommand>::SharedPtr emergency_stop_client_;
    rclcpp::Client<sort_interfaces::srv::GripperControl>::SharedPtr gripper_control_client_;
    rclcpp::Client<sort_interfaces::srv::MoveToCartesian>::SharedPtr move_cartesian_client_;
    rclcpp::Client<std_srvs::srv::SetBool>::SharedPtr toggle_frame_client_;

    // Cartesian movement helpers
    bool move_to_cartesian(double x, double y, double z, double rx, double ry, double rz, std::string& result_msg);
    bool set_end_effector_frame(bool use_gripper_tip, std::string& result_msg);
    bool move_to_home(std::string& result_msg);

    // Home position (in input coordinates: negated x/y)
    static constexpr double HOME_X = -588.0;
    static constexpr double HOME_Y = -133.0;
    static constexpr double HOME_Z = 222.0;
    static constexpr double HOME_RX = -2.221;
    static constexpr double HOME_RY = 2.221;
    static constexpr double HOME_RZ = 0.0;

    // Service servers
    rclcpp::Service<sort_interfaces::srv::SystemCommand>::SharedPtr brain_command_service_;
    rclcpp::Service<sort_interfaces::srv::SystemCommand>::SharedPtr get_status_service_;

    // Timers
    rclcpp::TimerBase::SharedPtr discovery_timer_;
    rclcpp::TimerBase::SharedPtr health_timer_;
    rclcpp::TimerBase::SharedPtr status_timer_;
    rclcpp::TimerBase::SharedPtr metrics_timer_;

    // State
    SystemState system_state_;
    std::deque<EventEntry> event_history_;
    std::chrono::system_clock::time_point start_time_;

    // Node monitoring
    std::map<std::string, NodeStatus> known_nodes_;
    std::vector<std::string> expected_nodes_;

    // Topic monitoring
    std::map<std::string, TopicMonitor> topic_monitors_;
    std::deque<std::string> topic_data_log_;

    // Performance metrics
    uint64_t total_operations_ = 0;
    uint64_t successful_operations_ = 0;
    uint64_t failed_operations_ = 0;

    // Thread safety
    mutable std::mutex data_mutex_;

    // Parameters
    double discovery_rate_;
    double health_check_rate_;
    double status_publish_rate_;
    double metrics_publish_rate_;
    size_t max_history_size_;
    size_t max_event_history_;
};

}  // namespace supervisor_module

#endif  // SUPERVISOR_MODULE__BRAIN_NODE_HPP_
