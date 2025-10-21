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
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <string>

class PickOperationNode : public rclcpp::Node
{
public:
    using PickObject = sort_interfaces::action::PickObject;
    using GoalHandlePickObject = rclcpp_action::ServerGoalHandle<PickObject>;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

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

        RCLCPP_INFO(this->get_logger(), "Pick Operation Node initialized");
        RCLCPP_INFO(this->get_logger(), "Approach distance: %.3f m, Lift height: %.3f m",
                   approach_distance_, lift_height_);
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

    bool move_to_approach_position(const geometry_msgs::msg::Pose& target, float approach_dist)
    {
        // TODO: Plan and execute trajectory to approach position
        // Approach position = target position + offset in Z direction
        RCLCPP_INFO(this->get_logger(), "Moving to approach position (%.3f m offset)...",
                   approach_dist);
        rclcpp::sleep_for(std::chrono::seconds(1));  // Simulate motion
        return true;
    }

    bool move_to_grasp_position(const geometry_msgs::msg::Pose& target)
    {
        // TODO: Plan and execute trajectory to exact grasp position
        RCLCPP_INFO(this->get_logger(), "Moving to grasp position...");
        rclcpp::sleep_for(std::chrono::milliseconds(500));  // Simulate motion
        return true;
    }

    bool lift_object(const geometry_msgs::msg::Pose& current_pose)
    {
        // TODO: Plan and execute vertical lift motion
        RCLCPP_INFO(this->get_logger(), "Lifting object by %.3f m...", lift_height_);
        rclcpp::sleep_for(std::chrono::milliseconds(800));  // Simulate motion
        return true;
    }

    void open_gripper()
    {
        auto cmd = std_msgs::msg::Float32();
        cmd.data = 0.0f;  // 0.0 = fully open
        gripper_command_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Opening gripper...");
    }

    void close_gripper()
    {
        auto cmd = std_msgs::msg::Float32();
        cmd.data = grasp_close_value_;
        gripper_command_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Closing gripper...");
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
    rclcpp::Subscription<sort_interfaces::msg::ForceFeedback>::SharedPtr force_feedback_sub_;
    rclcpp::Subscription<sort_interfaces::msg::SortDecision>::SharedPtr sort_decision_sub_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_command_pub_;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr actual_weight_pub_;

    rclcpp_action::Server<PickObject>::SharedPtr pick_action_server_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;

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
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
