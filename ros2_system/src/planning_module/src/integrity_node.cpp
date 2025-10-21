/**
 * @file integrity_node.cpp
 * @brief Environment safety checks and workspace monitoring
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <std_msgs/msg/string.hpp>
#include "sort_interfaces/msg/environment_status.hpp"
#include "sort_interfaces/srv/validate_workspace.hpp"
#include <geometry_msgs/msg/point.hpp>
#include <vector>

class IntegrityNode : public rclcpp::Node
{
public:
    IntegrityNode() : Node("integrity_node")
    {
        // Declare parameters
        this->declare_parameter("check_rate", 10.0);  // Hz
        this->declare_parameter("workspace_min_x", -0.6);
        this->declare_parameter("workspace_max_x", 0.6);
        this->declare_parameter("workspace_min_y", -0.6);
        this->declare_parameter("workspace_max_y", 0.6);
        this->declare_parameter("workspace_min_z", 0.0);
        this->declare_parameter("workspace_max_z", 0.6);
        this->declare_parameter("obstacle_detection_threshold", 0.05);  // meters
        this->declare_parameter("enable_collision_checking", true);

        // Get parameters
        double check_rate = this->get_parameter("check_rate").as_double();
        workspace_limits_.min_x = this->get_parameter("workspace_min_x").as_double();
        workspace_limits_.max_x = this->get_parameter("workspace_max_x").as_double();
        workspace_limits_.min_y = this->get_parameter("workspace_min_y").as_double();
        workspace_limits_.max_y = this->get_parameter("workspace_max_y").as_double();
        workspace_limits_.min_z = this->get_parameter("workspace_min_z").as_double();
        workspace_limits_.max_z = this->get_parameter("workspace_max_z").as_double();
        obstacle_threshold_ = this->get_parameter("obstacle_detection_threshold").as_double();
        enable_collision_checking_ = this->get_parameter("enable_collision_checking").as_bool();

        // Subscribers
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/pointcloud", 10,
            std::bind(&IntegrityNode::pointcloud_callback, this, std::placeholders::_1));

        system_status_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/system/status", 10,
            std::bind(&IntegrityNode::system_status_callback, this, std::placeholders::_1));

        // Publisher
        environment_status_pub_ = this->create_publisher<sort_interfaces::msg::EnvironmentStatus>(
            "/planning/environment_status", 10);

        // Service
        validate_workspace_service_ = this->create_service<sort_interfaces::srv::ValidateWorkspace>(
            "/planning/validate_workspace",
            std::bind(&IntegrityNode::handle_validate_workspace, this,
                     std::placeholders::_1, std::placeholders::_2));

        // Timer for periodic checks
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / check_rate)),
            std::bind(&IntegrityNode::check_environment, this));

        RCLCPP_INFO(this->get_logger(), "Integrity Node initialized");
        RCLCPP_INFO(this->get_logger(), "Workspace: [%.2f,%.2f] x [%.2f,%.2f] x [%.2f,%.2f]",
                   workspace_limits_.min_x, workspace_limits_.max_x,
                   workspace_limits_.min_y, workspace_limits_.max_y,
                   workspace_limits_.min_z, workspace_limits_.max_z);
        RCLCPP_INFO(this->get_logger(), "Check rate: %.1f Hz", check_rate);
    }

private:
    struct WorkspaceLimits {
        double min_x, max_x;
        double min_y, max_y;
        double min_z, max_z;
    };

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_pointcloud_ = msg;
    }

    void system_status_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        system_status_ = msg->data;
    }

    void check_environment()
    {
        auto status_msg = sort_interfaces::msg::EnvironmentStatus();
        status_msg.header.stamp = this->now();
        status_msg.header.frame_id = "world";

        // Default to safe
        status_msg.is_safe = true;
        status_msg.workspace_clear = true;
        status_msg.robot_in_bounds = true;

        // TODO: Implement actual safety checks using point cloud data
        // 1. Check for unexpected obstacles in workspace
        if (enable_collision_checking_ && latest_pointcloud_) {
            auto obstacles = detect_obstacles();
            if (!obstacles.empty()) {
                status_msg.workspace_clear = false;
                status_msg.is_safe = false;
                status_msg.detected_obstacles.push_back("Unknown object in workspace");
                status_msg.obstacle_positions = obstacles;
                status_msg.warning_message = "Obstacles detected in workspace";

                RCLCPP_WARN(this->get_logger(),
                           "Warning: %zu obstacles detected in workspace",
                           obstacles.size());
            }
        }

        // 2. Check robot position bounds
        // TODO: Get actual robot position from /joint_states or /tf
        bool robot_in_bounds = check_robot_bounds();
        if (!robot_in_bounds) {
            status_msg.robot_in_bounds = false;
            status_msg.is_safe = false;
            status_msg.detected_obstacles.push_back("Robot out of bounds");
            status_msg.warning_message = "Robot is outside safe workspace";

            RCLCPP_ERROR(this->get_logger(), "ERROR: Robot is outside safe workspace!");
        }

        // 3. Check emergency stop status
        if (system_status_ == "emergency_stopped") {
            status_msg.is_safe = false;
            status_msg.warning_message = "System in emergency stop state";
        }

        // Publish status
        environment_status_pub_->publish(status_msg);

        // Log warnings periodically
        static int warning_count = 0;
        if (!status_msg.is_safe && warning_count++ % 50 == 0) {
            RCLCPP_WARN(this->get_logger(), "Environment unsafe: %s",
                       status_msg.warning_message.c_str());
        }
    }

    std::vector<geometry_msgs::msg::Point> detect_obstacles()
    {
        std::vector<geometry_msgs::msg::Point> obstacles;

        // TODO: Implement actual obstacle detection from point cloud
        // Approach:
        // 1. Filter point cloud to workspace region
        // 2. Remove known objects (from object detection)
        // 3. Cluster remaining points
        // 4. If clusters exist -> obstacles detected

        // Placeholder: return empty (no obstacles)
        return obstacles;
    }

    bool check_robot_bounds()
    {
        // TODO: Get actual robot position from TF or joint states
        // For now, assume robot is in bounds
        return true;
    }

    void handle_validate_workspace(
        const std::shared_ptr<sort_interfaces::srv::ValidateWorkspace::Request> request,
        std::shared_ptr<sort_interfaces::srv::ValidateWorkspace::Response> response)
    {
        RCLCPP_INFO(this->get_logger(),
                   "Validating workspace for %zu waypoints", request->planned_waypoints.size());

        response->is_safe = true;
        response->warnings.clear();
        response->collision_points.clear();

        // Check each waypoint
        for (size_t i = 0; i < request->planned_waypoints.size(); ++i) {
            const auto& pose = request->planned_waypoints[i];

            // Check if waypoint is within workspace bounds
            if (!is_in_workspace(pose.position.x, pose.position.y, pose.position.z)) {
                response->is_safe = false;
                response->warnings.push_back(
                    "Waypoint " + std::to_string(i) + " is outside workspace bounds");

                geometry_msgs::msg::Point collision_point;
                collision_point = pose.position;
                response->collision_points.push_back(collision_point);

                RCLCPP_WARN(this->get_logger(),
                           "Waypoint %zu at [%.2f, %.2f, %.2f] is out of bounds",
                           i, pose.position.x, pose.position.y, pose.position.z);
            }

            // TODO: Check for collisions with known obstacles
            // TODO: Check for singularities or joint limits
        }

        if (response->is_safe) {
            RCLCPP_INFO(this->get_logger(), "Workspace validation passed");
        } else {
            RCLCPP_WARN(this->get_logger(),
                       "Workspace validation failed: %zu warnings",
                       response->warnings.size());
        }
    }

    bool is_in_workspace(double x, double y, double z) const
    {
        return (x >= workspace_limits_.min_x && x <= workspace_limits_.max_x &&
                y >= workspace_limits_.min_y && y <= workspace_limits_.max_y &&
                z >= workspace_limits_.min_z && z <= workspace_limits_.max_z);
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr system_status_sub_;
    rclcpp::Publisher<sort_interfaces::msg::EnvironmentStatus>::SharedPtr environment_status_pub_;
    rclcpp::Service<sort_interfaces::srv::ValidateWorkspace>::SharedPtr validate_workspace_service_;
    rclcpp::TimerBase::SharedPtr timer_;

    sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
    std::string system_status_;

    WorkspaceLimits workspace_limits_;
    double obstacle_threshold_;
    bool enable_collision_checking_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<IntegrityNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
