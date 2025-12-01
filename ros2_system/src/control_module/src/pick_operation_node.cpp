/**
 * @file pick_operation_node.cpp
 * @brief Pick operation coordinator with state machine
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include "sort_interfaces/action/pick_object.hpp"
#include "sort_interfaces/msg/force_feedback.hpp"
#include "sort_interfaces/msg/sort_decision.hpp"
#include "sort_interfaces/srv/move_to_cartesian.hpp"
#include "sort_interfaces/srv/gripper_control.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <string>
#include <mutex>

class PickOperationNode : public rclcpp::Node
{
public:
    using PickObject = sort_interfaces::action::PickObject;
    using GoalHandlePickObject = rclcpp_action::ServerGoalHandle<PickObject>;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using MoveToCartesian = sort_interfaces::srv::MoveToCartesian;
    using GripperControl = sort_interfaces::srv::GripperControl;

    // Z heights in mm (tool0 frame) - from pick_and_place_demo.py
    static constexpr double Z_HOME = 371.0;
    static constexpr double Z_DESCEND = 210.0;
    static constexpr double Z_PICKUP = 180.0;

    // Default orientation (facing down)
    static constexpr double RX = 2.221;
    static constexpr double RY = 2.221;
    static constexpr double RZ = 0.0;

    enum class PickState {
        IDLE,
        APPROACHING,
        PRE_GRASP,
        GRASPING,
        LIFTING,
        MEASURING_WEIGHT,
        COMPLETE,
        ERROR
    };

    PickOperationNode() : Node("pick_operation_node"), current_state_(PickState::IDLE)
    {
        // Create callback group for concurrent operations
        callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);
        // Declare parameters
        this->declare_parameter("approach_distance", 0.1);   // meters
        this->declare_parameter("lift_height", 0.15);        // meters
        this->declare_parameter("grasp_close_value", 0.8);   // 0.0=open, 1.0=closed
        this->declare_parameter("weight_measurement_time", 2.0);  // seconds
        this->declare_parameter("max_pick_attempts", 3);

        // Get parameters
        approach_distance_ = this->get_parameter("approach_distance").as_double();
        lift_height_ = this->get_parameter("lift_height").as_double();
        grasp_close_value_ = this->get_parameter("grasp_close_value").as_double();
        weight_measurement_time_ = this->get_parameter("weight_measurement_time").as_double();
        max_pick_attempts_ = this->get_parameter("max_pick_attempts").as_int();

        // Subscribers
        force_feedback_sub_ = this->create_subscription<sort_interfaces::msg::ForceFeedback>(
            "/motion_control/force_feedback", 10,
            std::bind(&PickOperationNode::force_feedback_callback, this,
                     std::placeholders::_1));

        sort_decision_sub_ = this->create_subscription<sort_interfaces::msg::SortDecision>(
            "/planning/sort_decisions", 10,
            std::bind(&PickOperationNode::sort_decision_callback, this,
                     std::placeholders::_1));

        // Publishers
        gripper_command_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/motion_control/gripper_command", 10);
        actual_weight_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/control/actual_weight", 10);

        // Action server
        pick_action_server_ = rclcpp_action::create_server<PickObject>(
            this,
            "/control/pick_object",
            std::bind(&PickOperationNode::handle_goal, this,
                     std::placeholders::_1, std::placeholders::_2),
            std::bind(&PickOperationNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&PickOperationNode::handle_accepted, this, std::placeholders::_1));

        // Action client for trajectory execution
        trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/control/execute_trajectory");

        // Service clients for cartesian movement and gripper control
        move_client_ = this->create_client<MoveToCartesian>(
            "/motion_control/move_to_cartesian",
            rmw_qos_profile_services_default,
            callback_group_);

        gripper_client_ = this->create_client<GripperControl>(
            "/motion_control/gripper_control",
            rmw_qos_profile_services_default,
            callback_group_);

        // Wait for services
        RCLCPP_INFO(this->get_logger(), "Waiting for motion control services...");
        move_client_->wait_for_service(std::chrono::seconds(30));
        gripper_client_->wait_for_service(std::chrono::seconds(30));
        RCLCPP_INFO(this->get_logger(), "Motion control services ready");

        RCLCPP_INFO(this->get_logger(), "Pick Operation Node initialized");
        RCLCPP_INFO(this->get_logger(), "Z heights: descend=%.1f mm, pickup=%.1f mm",
                   Z_DESCEND, Z_PICKUP);
    }

private:
    void force_feedback_callback(const sort_interfaces::msg::ForceFeedback::SharedPtr msg)
    {
        latest_force_feedback_ = msg;
    }

    void sort_decision_callback(const sort_interfaces::msg::SortDecision::SharedPtr msg)
    {
        // Auto-trigger pick operation when sort decision is received
        // In a real system, this might be managed by a higher-level coordinator
        RCLCPP_INFO(this->get_logger(),
                   "Sort decision received for object %u", msg->object_id);
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const PickObject::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(),
                   "Pick request for object %u at [%.2f, %.2f, %.2f]",
                   goal->object_id,
                   goal->target_pose.position.x,
                   goal->target_pose.position.y,
                   goal->target_pose.position.z);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePickObject> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Pick operation cancel requested");
        current_state_ = PickState::IDLE;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePickObject> goal_handle)
    {
        std::thread{std::bind(&PickOperationNode::execute_pick, this, goal_handle)}.detach();
    }

    void execute_pick(const std::shared_ptr<GoalHandlePickObject> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<PickObject::Feedback>();
        auto result = std::make_shared<PickObject::Result>();

        RCLCPP_INFO(this->get_logger(), "Starting pick operation...");

        // State machine for pick operation
        current_state_ = PickState::APPROACHING;

        // 1. Approach object
        feedback->current_state = "approaching";
        feedback->progress = 0.2;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "State: APPROACHING");

        if (!move_to_approach_position(goal->target_pose, goal->approach_distance)) {
            result->success = false;
            result->message = "Failed to approach object";
            goal_handle->abort(result);
            current_state_ = PickState::ERROR;
            return;
        }

        // 2. Open gripper
        current_state_ = PickState::PRE_GRASP;
        open_gripper();
        rclcpp::sleep_for(std::chrono::milliseconds(500));

        // 3. Move to grasp position
        feedback->current_state = "grasping";
        feedback->progress = 0.4;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "State: GRASPING");

        if (!move_to_grasp_position(goal->target_pose)) {
            result->success = false;
            result->message = "Failed to reach grasp position";
            goal_handle->abort(result);
            current_state_ = PickState::ERROR;
            return;
        }

        // 4. Close gripper
        current_state_ = PickState::GRASPING;
        close_gripper();
        rclcpp::sleep_for(std::chrono::milliseconds(1000));

        // Check if object was grasped
        if (!latest_force_feedback_ || !latest_force_feedback_->object_detected) {
            RCLCPP_WARN(this->get_logger(), "No object detected in gripper");
            result->success = false;
            result->message = "Failed to grasp object";
            goal_handle->abort(result);
            current_state_ = PickState::ERROR;
            return;
        }

        // 5. Lift object
        feedback->current_state = "lifting";
        feedback->progress = 0.6;
        goal_handle->publish_feedback(feedback);
        current_state_ = PickState::LIFTING;
        RCLCPP_INFO(this->get_logger(), "State: LIFTING");

        if (!lift_object(goal->target_pose)) {
            result->success = false;
            result->message = "Failed to lift object";
            goal_handle->abort(result);
            current_state_ = PickState::ERROR;
            return;
        }

        // 6. Measure weight
        feedback->current_state = "measuring";
        feedback->progress = 0.8;
        goal_handle->publish_feedback(feedback);
        current_state_ = PickState::MEASURING_WEIGHT;
        RCLCPP_INFO(this->get_logger(), "State: MEASURING WEIGHT");

        float measured_weight = measure_weight();

        // Publish actual weight
        auto weight_msg = std_msgs::msg::Float32();
        weight_msg.data = measured_weight;
        actual_weight_pub_->publish(weight_msg);

        // Complete
        current_state_ = PickState::COMPLETE;
        result->success = true;
        result->actual_weight = measured_weight;
        result->message = "Pick operation completed successfully";

        RCLCPP_INFO(this->get_logger(),
                   "✓ Pick complete - Object weight: %.1f g", measured_weight);

        goal_handle->succeed(result);
        current_state_ = PickState::IDLE;
    }

    bool move_to_cartesian(double x_mm, double y_mm, double z_mm)
    {
        // Call the cartesian movement service
        auto request = std::make_shared<MoveToCartesian::Request>();
        request->x = x_mm;
        request->y = y_mm;
        request->z = z_mm;
        request->rx = RX;
        request->ry = RY;
        request->rz = RZ;

        RCLCPP_INFO(this->get_logger(), "Moving to (%.1f, %.1f, %.1f) mm...",
                   x_mm, y_mm, z_mm);

        auto future = move_client_->async_send_request(request);

        // Wait for result with timeout
        auto status = future.wait_for(std::chrono::seconds(120));
        if (status != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Move service timed out");
            return false;
        }

        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Move completed: %s", response->message.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Move failed: %s", response->message.c_str());
            return false;
        }
    }

    bool move_to_approach_position(const geometry_msgs::msg::Pose& target, float approach_dist)
    {
        // Convert target pose from meters to mm
        // The target_pose comes in meters from the action goal
        double x_mm = target.position.x * 1000.0;
        double y_mm = target.position.y * 1000.0;

        RCLCPP_INFO(this->get_logger(), "Moving to approach position above target...");

        // Move to Z_DESCEND height above the target
        return move_to_cartesian(x_mm, y_mm, Z_DESCEND);
    }

    bool move_to_grasp_position(const geometry_msgs::msg::Pose& target)
    {
        double x_mm = target.position.x * 1000.0;
        double y_mm = target.position.y * 1000.0;

        RCLCPP_INFO(this->get_logger(), "Moving down to grasp position...");

        // Move to Z_PICKUP height
        return move_to_cartesian(x_mm, y_mm, Z_PICKUP);
    }

    bool lift_object(const geometry_msgs::msg::Pose& current_pose)
    {
        double x_mm = current_pose.position.x * 1000.0;
        double y_mm = current_pose.position.y * 1000.0;

        RCLCPP_INFO(this->get_logger(), "Lifting object to descend height...");

        // Lift back to Z_DESCEND
        return move_to_cartesian(x_mm, y_mm, Z_DESCEND);
    }

    bool gripper_control(const std::string& command)
    {
        auto request = std::make_shared<GripperControl::Request>();
        request->command = command;

        RCLCPP_INFO(this->get_logger(), "Gripper command: %s", command.c_str());

        auto future = gripper_client_->async_send_request(request);

        auto status = future.wait_for(std::chrono::seconds(10));
        if (status != std::future_status::ready) {
            RCLCPP_ERROR(this->get_logger(), "Gripper service timed out");
            return false;
        }

        auto response = future.get();
        if (response->success) {
            RCLCPP_INFO(this->get_logger(), "Gripper %s: %s", command.c_str(), response->message.c_str());
            return true;
        } else {
            RCLCPP_ERROR(this->get_logger(), "Gripper %s failed: %s", command.c_str(), response->message.c_str());
            return false;
        }
    }

    void open_gripper()
    {
        // Use service for more reliable control
        if (!gripper_control("open")) {
            // Fallback to topic
            auto cmd = std_msgs::msg::Float32();
            cmd.data = 0.0f;
            gripper_command_pub_->publish(cmd);
        }
    }

    void close_gripper()
    {
        // Use service for more reliable control
        if (!gripper_control("close")) {
            // Fallback to topic
            auto cmd = std_msgs::msg::Float32();
            cmd.data = grasp_close_value_;
            gripper_command_pub_->publish(cmd);
        }
    }

    float measure_weight()
    {
        RCLCPP_INFO(this->get_logger(), "Measuring weight for %.1f seconds...",
                   weight_measurement_time_);

        // Collect weight measurements
        std::vector<float> measurements;
        rclcpp::Rate rate(20);  // 20 Hz sampling

        auto start_time = this->now();
        while ((this->now() - start_time).seconds() < weight_measurement_time_) {
            if (latest_force_feedback_) {
                measurements.push_back(latest_force_feedback_->measured_weight);
            }
            rate.sleep();
        }

        // Calculate average (filtered weight)
        if (measurements.empty()) {
            RCLCPP_WARN(this->get_logger(), "No weight measurements received");
            return 0.0f;
        }

        float sum = 0.0f;
        for (float w : measurements) {
            sum += w;
        }
        return sum / measurements.size();
    }

    // Member variables
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp::Subscription<sort_interfaces::msg::ForceFeedback>::SharedPtr force_feedback_sub_;
    rclcpp::Subscription<sort_interfaces::msg::SortDecision>::SharedPtr sort_decision_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr actual_weight_pub_;

    rclcpp_action::Server<PickObject>::SharedPtr pick_action_server_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;

    // Service clients for motion control
    rclcpp::Client<MoveToCartesian>::SharedPtr move_client_;
    rclcpp::Client<GripperControl>::SharedPtr gripper_client_;

    sort_interfaces::msg::ForceFeedback::SharedPtr latest_force_feedback_;

    PickState current_state_;
    double approach_distance_;
    double lift_height_;
    double grasp_close_value_;
    double weight_measurement_time_;
    int max_pick_attempts_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PickOperationNode>();

    // Use MultiThreadedExecutor to allow concurrent service calls
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
