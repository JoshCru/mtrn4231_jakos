/**
 * @file brain_node.cpp
 * @brief Implementation of the Brain Node - Central Orchestrator
 */

#include "supervisor_module/brain_node.hpp"
#include <iomanip>
#include <sstream>
#include <algorithm>
#include <set>

namespace supervisor_module
{

BrainNode::BrainNode(const rclcpp::NodeOptions & options)
: Node("brain_node", options),
  system_state_(SystemState::IDLE),
  start_time_(std::chrono::system_clock::now())
{
    init_parameters();
    init_publishers();
    init_subscribers();
    init_services();
    init_timers();

    // Initialize expected nodes
    expected_nodes_ = {
        "system_controller_node",
        "gripper_controller_node",
        "recognition_node",
        "perception_node",
        "planning_node",
        "sort_node"
    };

    log_event("Brain Node initialized - Central Orchestrator Active", "STARTUP");
    RCLCPP_INFO(this->get_logger(), "Brain Node initialized successfully");
}

void BrainNode::init_parameters()
{
    this->declare_parameter("discovery_rate", 0.5);  // Hz
    this->declare_parameter("health_check_rate", 1.0);  // Hz
    this->declare_parameter("status_publish_rate", 2.0);  // Hz
    this->declare_parameter("metrics_publish_rate", 0.2);  // Hz
    this->declare_parameter("max_history_size", 100);
    this->declare_parameter("max_event_history", 1000);

    discovery_rate_ = this->get_parameter("discovery_rate").as_double();
    health_check_rate_ = this->get_parameter("health_check_rate").as_double();
    status_publish_rate_ = this->get_parameter("status_publish_rate").as_double();
    metrics_publish_rate_ = this->get_parameter("metrics_publish_rate").as_double();
    max_history_size_ = static_cast<size_t>(this->get_parameter("max_history_size").as_int());
    max_event_history_ = static_cast<size_t>(this->get_parameter("max_event_history").as_int());
}

void BrainNode::init_publishers()
{
    brain_status_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/brain/status", 10);
    brain_command_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/brain/commands", 10);
    node_health_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/brain/node_health", 10);
    topic_stats_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/brain/topic_stats", 10);
    system_metrics_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/brain/system_metrics", 10);
    event_log_pub_ = this->create_publisher<std_msgs::msg::String>(
        "/brain/event_log", 10);

    RCLCPP_DEBUG(this->get_logger(), "Publishers initialized");
}

void BrainNode::init_subscribers()
{
    // System level
    system_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/system/status", 10,
        std::bind(&BrainNode::system_status_callback, this, std::placeholders::_1));
    system_commands_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/system/commands", 10,
        std::bind(&BrainNode::system_commands_callback, this, std::placeholders::_1));

    // Perception module
    detected_objects_sub_ = this->create_subscription<sort_interfaces::msg::DetectedObjects>(
        "/perception/detected_objects", 10,
        std::bind(&BrainNode::detected_objects_callback, this, std::placeholders::_1));

    // Recognition module
    weight_estimate_sub_ = this->create_subscription<sort_interfaces::msg::WeightEstimate>(
        "/recognition/estimated_weights", 10,
        std::bind(&BrainNode::weight_estimate_callback, this, std::placeholders::_1));

    // Planning module
    sort_decision_sub_ = this->create_subscription<sort_interfaces::msg::SortDecision>(
        "/planning/sort_decisions", 10,
        std::bind(&BrainNode::sort_decision_callback, this, std::placeholders::_1));

    // Control module
    robot_status_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/control/robot_status", 10,
        std::bind(&BrainNode::robot_status_callback, this, std::placeholders::_1));
    joint_states_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
        "/joint_states", 10,
        std::bind(&BrainNode::joint_states_callback, this, std::placeholders::_1));

    // Gripper/Motion control
    gripper_state_sub_ = this->create_subscription<std_msgs::msg::Float32>(
        "/motion_control/gripper_state", 10,
        std::bind(&BrainNode::gripper_state_callback, this, std::placeholders::_1));
    force_feedback_sub_ = this->create_subscription<sort_interfaces::msg::ForceFeedback>(
        "/motion_control/force_feedback", 10,
        std::bind(&BrainNode::force_feedback_callback, this, std::placeholders::_1));

    // Target areas
    target_area_sub_ = this->create_subscription<sort_interfaces::msg::TargetArea>(
        "/system/target_areas", 10,
        std::bind(&BrainNode::target_area_callback, this, std::placeholders::_1));

    RCLCPP_DEBUG(this->get_logger(), "Subscribers initialized");
}

void BrainNode::init_services()
{
    // Service clients
    start_client_ = this->create_client<sort_interfaces::srv::SystemCommand>("/system/start");
    stop_client_ = this->create_client<sort_interfaces::srv::SystemCommand>("/system/stop");
    emergency_stop_client_ = this->create_client<sort_interfaces::srv::SystemCommand>("/system/emergency_stop");
    gripper_control_client_ = this->create_client<sort_interfaces::srv::GripperControl>("/motion_control/gripper_control");

    // Service servers
    brain_command_service_ = this->create_service<sort_interfaces::srv::SystemCommand>(
        "/brain/execute_command",
        std::bind(&BrainNode::handle_brain_command, this, std::placeholders::_1, std::placeholders::_2));

    get_status_service_ = this->create_service<sort_interfaces::srv::SystemCommand>(
        "/brain/get_status",
        std::bind(&BrainNode::handle_get_status, this, std::placeholders::_1, std::placeholders::_2));

    RCLCPP_DEBUG(this->get_logger(), "Services initialized");
}

void BrainNode::init_timers()
{
    discovery_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / discovery_rate_)),
        std::bind(&BrainNode::discover_nodes, this));

    health_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / health_check_rate_)),
        std::bind(&BrainNode::check_node_health, this));

    status_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / status_publish_rate_)),
        std::bind(&BrainNode::publish_brain_status, this));

    metrics_timer_ = this->create_wall_timer(
        std::chrono::milliseconds(static_cast<int>(1000.0 / metrics_publish_rate_)),
        std::bind(&BrainNode::publish_system_metrics, this));

    RCLCPP_DEBUG(this->get_logger(), "Timers initialized");
}

// === Helper for JSON string escaping ===
std::string BrainNode::escape_json_string(const std::string& input) const
{
    std::string output;
    for (char c : input)
    {
        switch (c)
        {
            case '"': output += "\\\""; break;
            case '\\': output += "\\\\"; break;
            case '\n': output += "\\n"; break;
            case '\r': output += "\\r"; break;
            case '\t': output += "\\t"; break;
            default: output += c;
        }
    }
    return output;
}

// === Node Discovery and Health ===

void BrainNode::discover_nodes()
{
    try
    {
        auto graph_names = this->get_node_names();

        std::lock_guard<std::mutex> lock(data_mutex_);
        std::set<std::string> current_nodes;

        for (const auto& name : graph_names)
        {
            current_nodes.insert(name);

            if (known_nodes_.find(name) == known_nodes_.end())
            {
                NodeStatus status;
                status.name = name;
                status.is_alive = true;
                status.last_seen = std::chrono::system_clock::now();
                known_nodes_[name] = status;
                log_event("New node discovered: " + name, "DISCOVERY");
            }
            else
            {
                known_nodes_[name].last_seen = std::chrono::system_clock::now();
                known_nodes_[name].is_alive = true;
            }
        }

        // Mark nodes not seen as dead
        for (auto& [name, status] : known_nodes_)
        {
            if (current_nodes.find(name) == current_nodes.end())
            {
                if (status.is_alive)
                {
                    log_event("Node disappeared: " + name, "WARNING");
                }
                status.is_alive = false;
            }
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error discovering nodes: %s", e.what());
    }
}

void BrainNode::check_node_health()
{
    publish_node_health();
    check_critical_nodes();
}

void BrainNode::check_critical_nodes()
{
    std::vector<std::string> missing_nodes;

    std::lock_guard<std::mutex> lock(data_mutex_);
    for (const auto& expected : expected_nodes_)
    {
        bool found = false;
        for (const auto& [name, status] : known_nodes_)
        {
            if (name.find(expected) != std::string::npos && status.is_alive)
            {
                found = true;
                break;
            }
        }
        if (!found)
        {
            missing_nodes.push_back(expected);
        }
    }

    if (!missing_nodes.empty() && system_state_ == SystemState::RUNNING)
    {
        std::string missing_str;
        for (const auto& n : missing_nodes)
        {
            missing_str += n + ", ";
        }
        log_event("Critical nodes missing: " + missing_str, "WARNING");
    }
}

// === Topic Callbacks ===

void BrainNode::system_status_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::ostringstream ss;
    ss << "{\"status\": \"" << escape_json_string(msg->data) << "\"}";
    log_topic_data("/system/status", ss.str());
    update_node_activity("system_controller_node");

    if (msg->data == "running" && system_state_ != SystemState::RUNNING)
    {
        transition_state(SystemState::RUNNING, "System controller reports running");
    }
    else if (msg->data == "emergency_stopped")
    {
        transition_state(SystemState::EMERGENCY_STOP, "Emergency stop activated");
    }
    else if (msg->data == "stopped")
    {
        transition_state(SystemState::IDLE, "System stopped");
    }
}

void BrainNode::system_commands_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::ostringstream ss;
    ss << "{\"command\": \"" << escape_json_string(msg->data) << "\"}";
    log_topic_data("/system/commands", ss.str());
    log_event("System command: " + msg->data, "COMMAND");
}

void BrainNode::detected_objects_callback(const sort_interfaces::msg::DetectedObjects::SharedPtr msg)
{
    size_t object_count = msg->objects.size();
    std::ostringstream ss;
    ss << "{\"count\": " << object_count
       << ", \"timestamp\": \"" << get_timestamp_string() << "\"}";
    log_topic_data("/perception/detected_objects", ss.str());
    update_node_activity("perception_node");

    if (object_count > 0)
    {
        log_event("Detected " + std::to_string(object_count) + " objects", "PERCEPTION");
    }
}

void BrainNode::weight_estimate_callback(const sort_interfaces::msg::WeightEstimate::SharedPtr msg)
{
    std::ostringstream ss;
    ss << "{\"estimated_weight\": " << msg->estimated_weight
       << ", \"confidence\": " << msg->confidence
       << ", \"timestamp\": \"" << get_timestamp_string() << "\"}";
    log_topic_data("/recognition/estimated_weights", ss.str());
    update_node_activity("recognition_node");
}

void BrainNode::sort_decision_callback(const sort_interfaces::msg::SortDecision::SharedPtr msg)
{
    std::ostringstream ss;
    ss << "{\"object_id\": " << msg->object_id
       << ", \"target_area_id\": " << static_cast<int>(msg->target_area_id)
       << ", \"estimated_weight\": " << msg->estimated_weight
       << ", \"timestamp\": \"" << get_timestamp_string() << "\"}";
    log_topic_data("/planning/sort_decisions", ss.str());
    update_node_activity("planning_node");

    std::ostringstream event_ss;
    event_ss << "Sort decision: Object " << msg->object_id << " -> Area " << static_cast<int>(msg->target_area_id);
    log_event(event_ss.str(), "PLANNING");
}

void BrainNode::robot_status_callback(const std_msgs::msg::String::SharedPtr msg)
{
    std::ostringstream ss;
    ss << "{\"status\": \"" << escape_json_string(msg->data) << "\"}";
    log_topic_data("/control/robot_status", ss.str());
    update_node_activity("robot_driver_node");
}

void BrainNode::joint_states_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
{
    std::ostringstream ss;
    ss << "{\"joint_count\": " << msg->name.size() << ", \"positions\": [";
    for (size_t i = 0; i < msg->position.size(); ++i)
    {
        if (i > 0) ss << ", ";
        ss << std::fixed << std::setprecision(4) << msg->position[i];
    }
    ss << "]}";
    log_topic_data("/joint_states", ss.str());
}

void BrainNode::gripper_state_callback(const std_msgs::msg::Float32::SharedPtr msg)
{
    std::ostringstream ss;
    ss << "{\"position\": " << std::fixed << std::setprecision(4) << msg->data << "}";
    log_topic_data("/motion_control/gripper_state", ss.str());
    update_node_activity("gripper_controller_node");
}

void BrainNode::force_feedback_callback(const sort_interfaces::msg::ForceFeedback::SharedPtr msg)
{
    std::ostringstream ss;
    ss << "{\"gripper_force\": " << msg->gripper_force
       << ", \"measured_weight\": " << msg->measured_weight
       << ", \"object_detected\": " << (msg->object_detected ? "true" : "false") << "}";
    log_topic_data("/motion_control/force_feedback", ss.str());
}

void BrainNode::target_area_callback(const sort_interfaces::msg::TargetArea::SharedPtr msg)
{
    std::ostringstream ss;
    ss << "{\"id\": " << msg->id
       << ", \"label\": \"" << escape_json_string(msg->label) << "\""
       << ", \"weight_min\": " << msg->weight_min
       << ", \"weight_max\": " << msg->weight_max << "}";
    log_topic_data("/system/target_areas", ss.str());
}

// === State Management ===

void BrainNode::transition_state(SystemState new_state, const std::string& reason)
{
    if (system_state_ != new_state)
    {
        SystemState old_state = system_state_;
        system_state_ = new_state;
        log_state_change(new_state, reason);

        RCLCPP_INFO(this->get_logger(), "State transition: %s -> %s (%s)",
            state_to_string(old_state).c_str(),
            state_to_string(new_state).c_str(),
            reason.c_str());

        // Publish state change event
        auto event_msg = std_msgs::msg::String();
        std::ostringstream ss;
        ss << "{\"type\": \"state_change\""
           << ", \"old_state\": \"" << state_to_string(old_state) << "\""
           << ", \"new_state\": \"" << state_to_string(new_state) << "\""
           << ", \"reason\": \"" << escape_json_string(reason) << "\""
           << ", \"timestamp\": \"" << get_timestamp_string() << "\"}";
        event_msg.data = ss.str();
        event_log_pub_->publish(event_msg);
    }
}

void BrainNode::log_state_change(SystemState state, const std::string& reason)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    EventEntry entry;
    entry.timestamp = std::chrono::system_clock::now();
    entry.type = "STATE_CHANGE";
    entry.message = reason;

    std::ostringstream ss;
    ss << "{\"state\": \"" << state_to_string(state) << "\""
       << ", \"reason\": \"" << escape_json_string(reason) << "\"}";
    entry.data = ss.str();

    event_history_.push_back(entry);
    if (event_history_.size() > max_event_history_)
    {
        event_history_.pop_front();
    }
}

void BrainNode::log_event(const std::string& message, const std::string& type)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    EventEntry entry;
    entry.timestamp = std::chrono::system_clock::now();
    entry.type = type;
    entry.message = message;

    event_history_.push_back(entry);
    if (event_history_.size() > max_event_history_)
    {
        event_history_.pop_front();
    }

    // Publish event
    auto event_msg = std_msgs::msg::String();
    std::ostringstream ss;
    ss << "{\"type\": \"" << escape_json_string(type) << "\""
       << ", \"message\": \"" << escape_json_string(message) << "\""
       << ", \"timestamp\": \"" << get_timestamp_string() << "\"}";
    event_msg.data = ss.str();
    event_log_pub_->publish(event_msg);

    RCLCPP_INFO(this->get_logger(), "[%s] %s", type.c_str(), message.c_str());
}

// === Helper Methods ===

void BrainNode::log_topic_data(const std::string& topic_name, const std::string& data)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    if (topic_monitors_.find(topic_name) == topic_monitors_.end())
    {
        TopicMonitor monitor;
        monitor.topic_name = topic_name;
        monitor.msg_type = "json";
        topic_monitors_[topic_name] = monitor;
    }

    auto& monitor = topic_monitors_[topic_name];
    monitor.message_count++;
    monitor.last_message_time = std::chrono::system_clock::now();
    monitor.last_message_data = data;

    monitor.history.push_back(data);
    if (monitor.history.size() > max_history_size_)
    {
        monitor.history.pop_front();
    }

    // Global log
    std::ostringstream ss;
    ss << "{\"topic\": \"" << escape_json_string(topic_name) << "\""
       << ", \"timestamp\": \"" << get_timestamp_string() << "\""
       << ", \"data\": " << data << "}";
    topic_data_log_.push_back(ss.str());
    if (topic_data_log_.size() > 10000)
    {
        topic_data_log_.pop_front();
    }
}

void BrainNode::update_node_activity(const std::string& node_name)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    for (auto& [name, status] : known_nodes_)
    {
        if (name.find(node_name) != std::string::npos)
        {
            status.message_count++;
            status.last_message = std::chrono::system_clock::now();
            break;
        }
    }
}

std::string BrainNode::get_timestamp_string() const
{
    auto now = std::chrono::system_clock::now();
    auto time_t = std::chrono::system_clock::to_time_t(now);
    auto ms = std::chrono::duration_cast<std::chrono::milliseconds>(
        now.time_since_epoch()) % 1000;

    std::ostringstream ss;
    ss << std::put_time(std::localtime(&time_t), "%Y-%m-%dT%H:%M:%S")
       << '.' << std::setfill('0') << std::setw(3) << ms.count();
    return ss.str();
}

double BrainNode::get_uptime_seconds() const
{
    auto now = std::chrono::system_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::seconds>(now - start_time_);
    return static_cast<double>(duration.count());
}

// === Publishing Methods ===

void BrainNode::publish_brain_status()
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    int alive_count = 0;
    for (const auto& [name, status] : known_nodes_)
    {
        if (status.is_alive) alive_count++;
    }

    std::ostringstream ss;
    ss << "{\"system_state\": \"" << state_to_string(system_state_) << "\""
       << ", \"uptime_seconds\": " << std::fixed << std::setprecision(1) << get_uptime_seconds()
       << ", \"total_nodes\": " << known_nodes_.size()
       << ", \"alive_nodes\": " << alive_count
       << ", \"topics_monitored\": " << topic_monitors_.size()
       << ", \"total_messages_logged\": " << topic_data_log_.size()
       << ", \"total_operations\": " << total_operations_
       << ", \"successful_operations\": " << successful_operations_
       << ", \"failed_operations\": " << failed_operations_
       << ", \"timestamp\": \"" << get_timestamp_string() << "\"}";

    auto msg = std_msgs::msg::String();
    msg.data = ss.str();
    brain_status_pub_->publish(msg);
}

void BrainNode::publish_system_metrics()
{
    publish_topic_stats();
}

void BrainNode::publish_node_health()
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    int alive_count = 0;
    int dead_count = 0;

    std::ostringstream nodes_ss;
    nodes_ss << "{";
    bool first = true;
    for (const auto& [name, status] : known_nodes_)
    {
        if (status.is_alive)
        {
            alive_count++;
        }
        else
        {
            dead_count++;
        }

        auto time_t = std::chrono::system_clock::to_time_t(status.last_seen);
        std::ostringstream time_ss;
        time_ss << std::put_time(std::localtime(&time_t), "%H:%M:%S");

        if (!first) nodes_ss << ", ";
        first = false;
        nodes_ss << "\"" << escape_json_string(name) << "\": {"
                 << "\"alive\": " << (status.is_alive ? "true" : "false")
                 << ", \"last_seen\": \"" << time_ss.str() << "\""
                 << ", \"message_count\": " << status.message_count << "}";
    }
    nodes_ss << "}";

    std::ostringstream ss;
    ss << "{\"timestamp\": \"" << get_timestamp_string() << "\""
       << ", \"total_nodes\": " << known_nodes_.size()
       << ", \"alive_nodes\": " << alive_count
       << ", \"dead_nodes\": " << dead_count
       << ", \"nodes\": " << nodes_ss.str() << "}";

    auto msg = std_msgs::msg::String();
    msg.data = ss.str();
    node_health_pub_->publish(msg);
}

void BrainNode::publish_topic_stats()
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    std::ostringstream topics_ss;
    topics_ss << "{";
    bool first = true;
    for (const auto& [topic, monitor] : topic_monitors_)
    {
        auto time_t = std::chrono::system_clock::to_time_t(monitor.last_message_time);
        std::ostringstream time_ss;
        time_ss << std::put_time(std::localtime(&time_t), "%H:%M:%S");

        if (!first) topics_ss << ", ";
        first = false;
        topics_ss << "\"" << escape_json_string(topic) << "\": {"
                  << "\"message_count\": " << monitor.message_count
                  << ", \"last_message_time\": \"" << time_ss.str() << "\""
                  << ", \"last_data\": " << monitor.last_message_data << "}";
    }
    topics_ss << "}";

    std::ostringstream ss;
    ss << "{\"timestamp\": \"" << get_timestamp_string() << "\""
       << ", \"topics\": " << topics_ss.str() << "}";

    auto msg = std_msgs::msg::String();
    msg.data = ss.str();
    topic_stats_pub_->publish(msg);
}

// === Service Handlers ===

void BrainNode::handle_brain_command(
    const std::shared_ptr<sort_interfaces::srv::SystemCommand::Request> request,
    std::shared_ptr<sort_interfaces::srv::SystemCommand::Response> response)
{
    std::string command = request->command;
    std::transform(command.begin(), command.end(), command.begin(), ::tolower);

    RCLCPP_INFO(this->get_logger(), "Brain command received: %s", command.c_str());
    log_event("Command received: " + command, "COMMAND");

    try
    {
        if (command == "start")
        {
            transition_state(SystemState::INITIALIZING, "Start command received");
            if (start_client_->wait_for_service(std::chrono::seconds(2)))
            {
                auto req = std::make_shared<sort_interfaces::srv::SystemCommand::Request>();
                req->command = "start";
                start_client_->async_send_request(req);
                response->success = true;
                response->message = "Start command forwarded to system controller";
            }
            else
            {
                response->success = false;
                response->message = "System controller not available";
            }
        }
        else if (command == "stop")
        {
            transition_state(SystemState::SHUTDOWN, "Stop command received");
            if (stop_client_->wait_for_service(std::chrono::seconds(2)))
            {
                auto req = std::make_shared<sort_interfaces::srv::SystemCommand::Request>();
                req->command = "stop";
                stop_client_->async_send_request(req);
                response->success = true;
                response->message = "Stop command forwarded to system controller";
            }
            else
            {
                response->success = false;
                response->message = "System controller not available";
            }
        }
        else if (command == "emergency_stop")
        {
            transition_state(SystemState::EMERGENCY_STOP, "Emergency stop command");
            if (emergency_stop_client_->wait_for_service(std::chrono::seconds(1)))
            {
                auto req = std::make_shared<sort_interfaces::srv::SystemCommand::Request>();
                req->command = "emergency_stop";
                emergency_stop_client_->async_send_request(req);
                response->success = true;
                response->message = "EMERGENCY STOP activated";
            }
            else
            {
                response->success = false;
                response->message = "Emergency stop service not available";
            }
        }
        else if (command == "reset")
        {
            transition_state(SystemState::IDLE, "System reset");
            response->success = true;
            response->message = "System reset to IDLE state";
        }
        else if (command == "get_nodes")
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            std::ostringstream ss;
            ss << "[";
            bool first = true;
            for (const auto& [name, _] : known_nodes_)
            {
                if (!first) ss << ", ";
                first = false;
                ss << "\"" << escape_json_string(name) << "\"";
            }
            ss << "]";
            response->success = true;
            response->message = ss.str();
        }
        else if (command == "get_topics")
        {
            std::lock_guard<std::mutex> lock(data_mutex_);
            std::ostringstream ss;
            ss << "[";
            bool first = true;
            for (const auto& [topic, _] : topic_monitors_)
            {
                if (!first) ss << ", ";
                first = false;
                ss << "\"" << escape_json_string(topic) << "\"";
            }
            ss << "]";
            response->success = true;
            response->message = ss.str();
        }
        else
        {
            response->success = false;
            response->message = "Unknown command: " + command;
        }
    }
    catch (const std::exception& e)
    {
        RCLCPP_ERROR(this->get_logger(), "Error handling command: %s", e.what());
        response->success = false;
        response->message = e.what();
    }
}

void BrainNode::handle_get_status(
    const std::shared_ptr<sort_interfaces::srv::SystemCommand::Request> /*request*/,
    std::shared_ptr<sort_interfaces::srv::SystemCommand::Response> response)
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    // Build nodes JSON
    std::ostringstream nodes_ss;
    nodes_ss << "{";
    bool first = true;
    for (const auto& [name, status] : known_nodes_)
    {
        if (!first) nodes_ss << ", ";
        first = false;
        nodes_ss << "\"" << escape_json_string(name) << "\": {"
                 << "\"alive\": " << (status.is_alive ? "true" : "false")
                 << ", \"msg_count\": " << status.message_count << "}";
    }
    nodes_ss << "}";

    // Build topics JSON
    std::ostringstream topics_ss;
    topics_ss << "[";
    first = true;
    for (const auto& [topic, _] : topic_monitors_)
    {
        if (!first) topics_ss << ", ";
        first = false;
        topics_ss << "\"" << escape_json_string(topic) << "\"";
    }
    topics_ss << "]";

    // Build recent events JSON
    std::ostringstream events_ss;
    events_ss << "[";
    size_t start_idx = event_history_.size() > 10 ? event_history_.size() - 10 : 0;
    first = true;
    for (size_t i = start_idx; i < event_history_.size(); ++i)
    {
        const auto& event = event_history_[i];
        auto time_t = std::chrono::system_clock::to_time_t(event.timestamp);
        std::ostringstream time_ss;
        time_ss << std::put_time(std::localtime(&time_t), "%H:%M:%S");

        if (!first) events_ss << ", ";
        first = false;
        events_ss << "{\"time\": \"" << time_ss.str() << "\""
                  << ", \"type\": \"" << escape_json_string(event.type) << "\""
                  << ", \"message\": \"" << escape_json_string(event.message) << "\"}";
    }
    events_ss << "]";

    std::ostringstream ss;
    ss << "{\n  \"system_state\": \"" << state_to_string(system_state_) << "\",\n"
       << "  \"uptime_seconds\": " << std::fixed << std::setprecision(1) << get_uptime_seconds() << ",\n"
       << "  \"nodes\": " << nodes_ss.str() << ",\n"
       << "  \"topics\": " << topics_ss.str() << ",\n"
       << "  \"recent_events\": " << events_ss.str() << "\n}";

    response->success = true;
    response->message = ss.str();
}

// === Public Accessors ===

std::string BrainNode::get_all_node_statuses() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::ostringstream ss;
    ss << "{";
    bool first = true;

    for (const auto& [name, status] : known_nodes_)
    {
        auto time_t = std::chrono::system_clock::to_time_t(status.last_seen);
        std::ostringstream time_ss;
        time_ss << std::put_time(std::localtime(&time_t), "%H:%M:%S");

        if (!first) ss << ", ";
        first = false;
        ss << "\"" << escape_json_string(name) << "\": {"
           << "\"alive\": " << (status.is_alive ? "true" : "false")
           << ", \"last_seen\": \"" << time_ss.str() << "\""
           << ", \"message_count\": " << status.message_count << "}";
    }
    ss << "}";
    return ss.str();
}

std::string BrainNode::get_all_topic_data() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::ostringstream ss;
    ss << "{";
    bool first = true;

    for (const auto& [topic, monitor] : topic_monitors_)
    {
        auto time_t = std::chrono::system_clock::to_time_t(monitor.last_message_time);
        std::ostringstream time_ss;
        time_ss << std::put_time(std::localtime(&time_t), "%H:%M:%S");

        if (!first) ss << ", ";
        first = false;
        ss << "\"" << escape_json_string(topic) << "\": {"
           << "\"message_count\": " << monitor.message_count
           << ", \"last_message_time\": \"" << time_ss.str() << "\""
           << ", \"last_message_data\": " << monitor.last_message_data << "}";
    }
    ss << "}";
    return ss.str();
}

std::vector<EventEntry> BrainNode::get_recent_events(size_t count) const
{
    std::lock_guard<std::mutex> lock(data_mutex_);
    std::vector<EventEntry> result;

    size_t start_idx = event_history_.size() > count ? event_history_.size() - count : 0;
    for (size_t i = start_idx; i < event_history_.size(); ++i)
    {
        result.push_back(event_history_[i]);
    }
    return result;
}

std::string BrainNode::get_system_metrics() const
{
    std::lock_guard<std::mutex> lock(data_mutex_);

    int alive_count = 0;
    for (const auto& [_, status] : known_nodes_)
    {
        if (status.is_alive) alive_count++;
    }

    std::ostringstream ss;
    ss << "{\"system_state\": \"" << state_to_string(system_state_) << "\""
       << ", \"uptime_seconds\": " << std::fixed << std::setprecision(1) << get_uptime_seconds()
       << ", \"total_nodes\": " << known_nodes_.size()
       << ", \"alive_nodes\": " << alive_count
       << ", \"topics_monitored\": " << topic_monitors_.size()
       << ", \"event_count\": " << event_history_.size()
       << ", \"timestamp\": \"" << get_timestamp_string() << "\"}";
    return ss.str();
}

}  // namespace supervisor_module

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<supervisor_module::BrainNode>();

    RCLCPP_INFO(node->get_logger(), "Brain Node started - Central System Orchestrator");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
