/**
 * @file robot_driver_node.cpp
 * @brief UR5e robot driver interface with lifecycle management
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/lifecycle_node.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/string.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <vector>

using LifecycleNode = rclcpp_lifecycle::LifecycleNode;
using LifecycleCallbackReturn = rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn;

class RobotDriverNode : public LifecycleNode
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

    RobotDriverNode() : LifecycleNode("robot_driver_node")
    {
        // Declare parameters
        this->declare_parameter("robot_ip", "192.168.1.100");
        this->declare_parameter("joint_state_rate", 50.0);  // Hz
        this->declare_parameter("num_joints", 6);
        this->declare_parameter("emergency_stop_enabled", true);

        RCLCPP_INFO(this->get_logger(), "Robot Driver Node constructed");
    }

    // Lifecycle callbacks
    LifecycleCallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "Configuring...");

        // Get parameters
        robot_ip_ = this->get_parameter("robot_ip").as_string();
        double joint_state_rate = this->get_parameter("joint_state_rate").as_double();
        num_joints_ = this->get_parameter("num_joints").as_int();

        // Publishers
        joint_states_pub_ = this->create_publisher<sensor_msgs::msg::JointState>(
            "/control/joint_states", 10);
        robot_status_pub_ = this->create_publisher<std_msgs::msg::String>(
            "/control/robot_status", 10);

        // Subscriber
        trajectory_sub_ = this->create_subscription<trajectory_msgs::msg::JointTrajectory>(
            "/planning/trajectory", 10,
            std::bind(&RobotDriverNode::trajectory_callback, this, std::placeholders::_1));

        emergency_stop_sub_ = this->create_subscription<std_msgs::msg::String>(
            "/system/commands", 10,
            std::bind(&RobotDriverNode::system_command_callback, this, std::placeholders::_1));

        // Action server
        execute_trajectory_server_ = rclcpp_action::create_server<FollowJointTrajectory>(
            this->get_node_base_interface(),
            this->get_node_clock_interface(),
            this->get_node_logging_interface(),
            this->get_node_waitables_interface(),
            "/control/execute_trajectory",
            std::bind(&RobotDriverNode::handle_goal, this,
                     std::placeholders::_1, std::placeholders::_2),
            std::bind(&RobotDriverNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&RobotDriverNode::handle_accepted, this, std::placeholders::_1));

        // Timer for publishing joint states
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(static_cast<int>(1000.0 / joint_state_rate)),
            std::bind(&RobotDriverNode::publish_joint_states, this));

        RCLCPP_INFO(this->get_logger(), "Robot IP: %s", robot_ip_.c_str());
        RCLCPP_WARN(this->get_logger(), "TODO: Connect to actual UR5e robot via UR RTDE or driver");

        robot_state_ = "configured";
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "Activating...");

        // TODO: Establish connection to robot
        // TODO: Enable robot motors
        // TODO: Load initial joint positions

        robot_state_ = "active";
        emergency_stopped_ = false;

        RCLCPP_INFO(this->get_logger(), "Robot driver activated and ready");
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "Deactivating...");

        robot_state_ = "inactive";

        // TODO: Stop any ongoing motion
        // TODO: Maintain connection but disable commands

        RCLCPP_INFO(this->get_logger(), "Robot driver deactivated");
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "Cleaning up...");

        // TODO: Disconnect from robot
        timer_.reset();
        robot_state_ = "unconfigured";

        RCLCPP_INFO(this->get_logger(), "Robot driver cleaned up");
        return LifecycleCallbackReturn::SUCCESS;
    }

    LifecycleCallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override
    {
        (void)previous_state;
        RCLCPP_INFO(this->get_logger(), "Shutting down...");

        // TODO: Safe shutdown of robot connection

        return LifecycleCallbackReturn::SUCCESS;
    }

private:
    void trajectory_callback(const trajectory_msgs::msg::JointTrajectory::SharedPtr msg)
    {
        if (emergency_stopped_) {
            RCLCPP_ERROR(this->get_logger(), "Trajectory rejected: Emergency stop active");
            return;
        }

        RCLCPP_INFO(this->get_logger(),
                   "Received trajectory with %zu points", msg->points.size());

        // TODO: Send trajectory to robot controller
    }

    void system_command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data == "emergency_stop") {
            emergency_stopped_ = true;
            robot_state_ = "emergency_stopped";
            // TODO: Send emergency stop command to robot
            RCLCPP_ERROR(this->get_logger(), "EMERGENCY STOP - Robot halted");
        } else if (msg->data == "stop") {
            // TODO: Stop current motion gracefully
            RCLCPP_INFO(this->get_logger(), "Stop command received");
        }
    }

    void publish_joint_states()
    {
        if (robot_state_ != "active") {
            return;
        }

        // TODO: Get actual joint states from robot
        // For now, publish dummy joint states

        auto joint_state_msg = sensor_msgs::msg::JointState();
        joint_state_msg.header.stamp = this->now();

        joint_state_msg.name = {"shoulder_pan_joint", "shoulder_lift_joint",
                               "elbow_joint", "wrist_1_joint",
                               "wrist_2_joint", "wrist_3_joint"};

        // Dummy values (home position)
        joint_state_msg.position = {0.0, -1.57, 1.57, -1.57, -1.57, 0.0};
        joint_state_msg.velocity.resize(num_joints_, 0.0);
        joint_state_msg.effort.resize(num_joints_, 0.0);

        joint_states_pub_->publish(joint_state_msg);

        // Publish status periodically
        static int count = 0;
        if (count++ % 50 == 0) {
            auto status_msg = std_msgs::msg::String();
            status_msg.data = robot_state_;
            robot_status_pub_->publish(status_msg);
        }
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const FollowJointTrajectory::Goal> goal)
    {
        (void)uuid;
        if (emergency_stopped_) {
            RCLCPP_ERROR(this->get_logger(), "Goal rejected: Emergency stop active");
            return rclcpp_action::GoalResponse::REJECT;
        }

        RCLCPP_INFO(this->get_logger(),
                   "Trajectory execution request with %zu points",
                   goal->trajectory.points.size());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Trajectory execution cancel requested");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle)
    {
        std::thread{std::bind(&RobotDriverNode::execute_trajectory, this, goal_handle)}.detach();
    }

    void execute_trajectory(
        const std::shared_ptr<rclcpp_action::ServerGoalHandle<FollowJointTrajectory>> goal_handle)
    {
        RCLCPP_INFO(this->get_logger(), "Executing trajectory...");

        const auto goal = goal_handle->get_goal();
        auto result = std::make_shared<FollowJointTrajectory::Result>();

        // TODO: Send trajectory to robot and monitor execution
        // Placeholder: simulate execution
        rclcpp::sleep_for(std::chrono::seconds(2));

        result->error_code = FollowJointTrajectory::Result::SUCCESSFUL;
        result->error_string = "Trajectory executed successfully";

        goal_handle->succeed(result);
        RCLCPP_INFO(this->get_logger(), "Trajectory execution complete");
    }

    // Member variables
    rclcpp_lifecycle::LifecyclePublisher<sensor_msgs::msg::JointState>::SharedPtr joint_states_pub_;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>::SharedPtr robot_status_pub_;

    rclcpp::Subscription<trajectory_msgs::msg::JointTrajectory>::SharedPtr trajectory_sub_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr emergency_stop_sub_;

    rclcpp_action::Server<FollowJointTrajectory>::SharedPtr execute_trajectory_server_;

    rclcpp::TimerBase::SharedPtr timer_;

    std::string robot_ip_;
    std::string robot_state_;
    int num_joints_;
    bool emergency_stopped_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<RobotDriverNode>();

    // Lifecycle management
    rclcpp::executors::SingleThreadedExecutor executor;
    executor.add_node(node->get_node_base_interface());
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
