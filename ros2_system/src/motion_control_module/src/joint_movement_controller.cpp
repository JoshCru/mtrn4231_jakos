/**
 * @file joint_movement_controller.cpp
 * @brief Joint movement controller using MoveIt for UR5e
 *
 * Joint Numbering Convention:
 *   Joint 6 = shoulder_pan_joint (base rotation)
 *   Joint 1 = shoulder_lift_joint
 *   Joint 2 = elbow_joint
 *   Joint 3 = wrist_1_joint
 *   Joint 4 = wrist_2_joint
 *   Joint 5 = wrist_3_joint
 *
 * Joint order in arrays: [Joint6, Joint1, Joint2, Joint3, Joint4, Joint5]
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/action/move_group.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/joint_constraint.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include "sort_interfaces/srv/move_to_joint_position.hpp"
#include <vector>
#include <string>
#include <cmath>

class JointMovementController : public rclcpp::Node
{
public:
    using MoveGroup = moveit_msgs::action::MoveGroup;
    using GoalHandleMoveGroup = rclcpp_action::ClientGoalHandle<MoveGroup>;

    JointMovementController() : Node("joint_movement_controller")
    {
        // Joint names in [J6, J1, J2, J3, J4, J5] order
        joint_names_ = {
            "shoulder_pan_joint",   // Joint 6 (base)
            "shoulder_lift_joint",  // Joint 1
            "elbow_joint",          // Joint 2
            "wrist_1_joint",        // Joint 3
            "wrist_2_joint",        // Joint 4
            "wrist_3_joint"         // Joint 5
        };

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Joint Movement Controller starting...");
        RCLCPP_INFO(this->get_logger(), "========================================");

        // Create MoveIt action client
        RCLCPP_INFO(this->get_logger(), "Creating action client for /move_action...");
        move_group_client_ = rclcpp_action::create_client<MoveGroup>(
            this, "/move_action");

        // Subscribe to joint states
        RCLCPP_INFO(this->get_logger(), "Subscribing to /joint_states...");
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", 10,
            std::bind(&JointMovementController::joint_state_callback, this, std::placeholders::_1));

        // Create service for moving to joint positions
        RCLCPP_INFO(this->get_logger(), "Creating service /move_to_joint_position...");
        move_service_ = this->create_service<sort_interfaces::srv::MoveToJointPosition>(
            "/move_to_joint_position",
            std::bind(&JointMovementController::handle_move_request, this,
                     std::placeholders::_1, std::placeholders::_2));

        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Waiting for MoveIt action server at /move_action...");
        RCLCPP_INFO(this->get_logger(), "(Make sure MoveIt is running!)");
        RCLCPP_INFO(this->get_logger(), "========================================");

        // Wait for action server with better feedback
        int wait_count = 0;
        while (!move_group_client_->wait_for_action_server(std::chrono::seconds(1)) && wait_count < 30) {
            wait_count++;
            if (wait_count % 5 == 0) {
                RCLCPP_WARN(this->get_logger(), "Still waiting for MoveIt... (%d seconds)", wait_count);
            }
        }

        if (wait_count >= 30) {
            RCLCPP_ERROR(this->get_logger(), "========================================");
            RCLCPP_ERROR(this->get_logger(), "TIMEOUT: MoveIt action server not available!");
            RCLCPP_ERROR(this->get_logger(), "Check that MoveIt is running:");
            RCLCPP_ERROR(this->get_logger(), "  ros2 action list | grep move_action");
            RCLCPP_ERROR(this->get_logger(), "========================================");
        } else {
            RCLCPP_INFO(this->get_logger(), "========================================");
            RCLCPP_INFO(this->get_logger(), "✓ Connected to MoveIt action server!");
            RCLCPP_INFO(this->get_logger(), "✓ Service ready at /move_to_joint_position");
            RCLCPP_INFO(this->get_logger(), "========================================");
        }
    }

private:
    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        current_joint_state_ = msg;
    }

    std::vector<double> get_current_joint_positions()
    {
        std::vector<double> positions;

        if (!current_joint_state_) {
            RCLCPP_WARN(this->get_logger(), "No joint states available yet");
            return positions;
        }

        for (const auto& joint_name : joint_names_) {
            auto it = std::find(current_joint_state_->name.begin(),
                              current_joint_state_->name.end(),
                              joint_name);

            if (it != current_joint_state_->name.end()) {
                size_t index = std::distance(current_joint_state_->name.begin(), it);
                positions.push_back(current_joint_state_->position[index]);
            } else {
                RCLCPP_ERROR(this->get_logger(), "Joint %s not found in joint states", joint_name.c_str());
                return std::vector<double>();
            }
        }

        return positions;
    }

    void handle_move_request(
        const std::shared_ptr<sort_interfaces::srv::MoveToJointPosition::Request> request,
        std::shared_ptr<sort_interfaces::srv::MoveToJointPosition::Response> response)
    {
        RCLCPP_INFO(this->get_logger(), "Received move request: %s", request->position_name.c_str());

        // Check if we have 6 joint positions
        if (request->joint_positions.size() != 6) {
            response->success = false;
            response->message = "Expected 6 joint positions, got " +
                              std::to_string(request->joint_positions.size());
            RCLCPP_ERROR(this->get_logger(), "%s", response->message.c_str());
            return;
        }

        // Log the target positions
        RCLCPP_INFO(this->get_logger(), "========================================");
        RCLCPP_INFO(this->get_logger(), "Moving to position: %s", request->position_name.c_str());
        RCLCPP_INFO(this->get_logger(), "Target joint angles [J6, J1, J2, J3, J4, J5]:");

        std::vector<std::string> joint_labels = {
            "J6 (shoulder_pan)", "J1 (shoulder_lift)", "J2 (elbow)",
            "J3 (wrist_1)", "J4 (wrist_2)", "J5 (wrist_3)"
        };

        for (size_t i = 0; i < request->joint_positions.size(); ++i) {
            double rad = request->joint_positions[i];
            double deg = rad * 180.0 / M_PI;
            RCLCPP_INFO(this->get_logger(), "  %s: %.4f rad (%.2f°)",
                       joint_labels[i].c_str(), rad, deg);
        }
        RCLCPP_INFO(this->get_logger(), "========================================");

        // Move to the position
        bool success = move_to_joint_positions(request->joint_positions);

        response->success = success;
        response->message = success ? "Movement successful" : "Movement failed";

        if (success) {
            RCLCPP_INFO(this->get_logger(), "✓ Successfully reached %s", request->position_name.c_str());
        } else {
            RCLCPP_ERROR(this->get_logger(), "✗ Failed to reach %s", request->position_name.c_str());
        }
    }

    bool move_to_joint_positions(const std::vector<double>& joint_positions)
    {
        if (!move_group_client_->wait_for_action_server(std::chrono::seconds(1))) {
            RCLCPP_ERROR(this->get_logger(), "MoveIt action server not available!");
            return false;
        }

        // Create MoveGroup goal
        auto goal_msg = MoveGroup::Goal();
        goal_msg.request.group_name = "ur_manipulator";
        goal_msg.request.num_planning_attempts = 10;
        goal_msg.request.allowed_planning_time = 10.0;
        goal_msg.request.max_velocity_scaling_factor = 0.2;
        goal_msg.request.max_acceleration_scaling_factor = 0.2;

        // Create joint constraints with tight tolerances
        moveit_msgs::msg::Constraints constraints;

        for (size_t i = 0; i < joint_names_.size(); ++i) {
            moveit_msgs::msg::JointConstraint joint_constraint;
            joint_constraint.joint_name = joint_names_[i];
            joint_constraint.position = joint_positions[i];
            joint_constraint.tolerance_above = 0.01;  // ~0.57°
            joint_constraint.tolerance_below = 0.01;
            joint_constraint.weight = 1.0;
            constraints.joint_constraints.push_back(joint_constraint);
        }

        goal_msg.request.goal_constraints.push_back(constraints);
        goal_msg.planning_options.plan_only = false;  // Plan and execute

        // Send goal
        RCLCPP_INFO(this->get_logger(), "Sending goal to MoveIt...");

        // Use a promise to track completion
        auto result_promise = std::make_shared<std::promise<bool>>();
        auto result_future = result_promise->get_future();

        // Send goal with result callback
        auto send_goal_options = rclcpp_action::Client<MoveGroup>::SendGoalOptions();

        send_goal_options.result_callback =
            [this, result_promise](const GoalHandleMoveGroup::WrappedResult & result) {
                int error_code = result.result->error_code.val;

                RCLCPP_INFO(this->get_logger(), "Received result with error code: %d", error_code);

                if (result.code == rclcpp_action::ResultCode::SUCCEEDED) {
                    if (error_code == moveit_msgs::msg::MoveItErrorCodes::SUCCESS) {
                        RCLCPP_INFO(this->get_logger(), "Movement completed successfully!");
                        result_promise->set_value(true);
                    } else {
                        RCLCPP_ERROR(this->get_logger(), "Movement failed with MoveIt error code: %d", error_code);
                        result_promise->set_value(false);
                    }
                } else if (result.code == rclcpp_action::ResultCode::ABORTED) {
                    RCLCPP_ERROR(this->get_logger(), "Movement was aborted");
                    result_promise->set_value(false);
                } else if (result.code == rclcpp_action::ResultCode::CANCELED) {
                    RCLCPP_ERROR(this->get_logger(), "Movement was canceled");
                    result_promise->set_value(false);
                } else {
                    RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                    result_promise->set_value(false);
                }
            };

        send_goal_options.goal_response_callback =
            [this](GoalHandleMoveGroup::SharedPtr goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by MoveIt");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by MoveIt, executing...");
                }
            };

        send_goal_options.feedback_callback =
            [this](GoalHandleMoveGroup::SharedPtr,
                   const std::shared_ptr<const MoveGroup::Feedback> feedback) {
                RCLCPP_INFO(this->get_logger(), "Received feedback from MoveIt: %s", feedback->state.c_str());
            };

        RCLCPP_INFO(this->get_logger(), "Sending goal to MoveIt (async, not waiting for completion)...");
        auto goal_handle_future = move_group_client_->async_send_goal(goal_msg, send_goal_options);

        // Just verify the goal was sent successfully
        RCLCPP_INFO(this->get_logger(), "Goal sent! MoveIt will execute in background.");
        RCLCPP_INFO(this->get_logger(), "Note: Not waiting for movement to complete (async mode)");

        // Return immediately - MoveIt will execute the movement
        return true;
    }

    // Member variables
    std::vector<std::string> joint_names_;
    rclcpp_action::Client<MoveGroup>::SharedPtr move_group_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Service<sort_interfaces::srv::MoveToJointPosition>::SharedPtr move_service_;
    sensor_msgs::msg::JointState::SharedPtr current_joint_state_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<JointMovementController>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
