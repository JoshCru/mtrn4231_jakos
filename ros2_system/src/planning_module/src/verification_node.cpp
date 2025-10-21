/**
 * @file verification_node.cpp
 * @brief Weight verification - compare estimated vs actual weight
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/bool.hpp>
#include "sort_interfaces/msg/weight_estimate.hpp"
#include "sort_interfaces/msg/force_feedback.hpp"
#include "sort_interfaces/action/verify_weight.hpp"
#include <map>
#include <cmath>

class VerificationNode : public rclcpp::Node
{
public:
    using VerifyWeight = sort_interfaces::action::VerifyWeight;
    using GoalHandleVerifyWeight = rclcpp_action::ServerGoalHandle<VerifyWeight>;

    VerificationNode() : Node("verification_node")
    {
        // Declare parameters
        this->declare_parameter("default_tolerance", 10.0);  // percentage
        this->declare_parameter("measurement_duration", 2.0);  // seconds
        this->declare_parameter("filter_window_size", 10);

        // Get parameters
        default_tolerance_ = this->get_parameter("default_tolerance").as_double();
        measurement_duration_ = this->get_parameter("measurement_duration").as_double();
        filter_window_size_ = this->get_parameter("filter_window_size").as_int();

        // Subscribers
        estimated_weights_sub_ = this->create_subscription<sort_interfaces::msg::WeightEstimate>(
            "/recognition/estimated_weights", 10,
            std::bind(&VerificationNode::estimated_weight_callback, this,
                     std::placeholders::_1));

        actual_weight_sub_ = this->create_subscription<sort_interfaces::msg::ForceFeedback>(
            "/motion_control/force_feedback", 10,
            std::bind(&VerificationNode::force_feedback_callback, this,
                     std::placeholders::_1));

        // Publisher
        verification_result_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/planning/verification_result", 10);

        // Action server
        verify_weight_action_server_ = rclcpp_action::create_server<VerifyWeight>(
            this,
            "/planning/verify_weight",
            std::bind(&VerificationNode::handle_goal, this,
                     std::placeholders::_1, std::placeholders::_2),
            std::bind(&VerificationNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&VerificationNode::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "Verification Node initialized");
        RCLCPP_INFO(this->get_logger(), "Default tolerance: %.1f%%", default_tolerance_);
    }

private:
    void estimated_weight_callback(const sort_interfaces::msg::WeightEstimate::SharedPtr msg)
    {
        estimated_weights_[msg->object_id] = msg->estimated_weight;
        RCLCPP_DEBUG(this->get_logger(),
                    "Stored estimated weight for object %u: %.1f g",
                    msg->object_id, msg->estimated_weight);
    }

    void force_feedback_callback(const sort_interfaces::msg::ForceFeedback::SharedPtr msg)
    {
        latest_actual_weight_ = msg->measured_weight;

        // Add to filter window
        weight_measurements_.push_back(msg->measured_weight);
        if (weight_measurements_.size() > static_cast<size_t>(filter_window_size_)) {
            weight_measurements_.erase(weight_measurements_.begin());
        }

        // Calculate filtered weight (moving average)
        filtered_weight_ = calculate_filtered_weight();
    }

    float calculate_filtered_weight()
    {
        if (weight_measurements_.empty()) {
            return 0.0f;
        }

        float sum = 0.0f;
        for (float w : weight_measurements_) {
            sum += w;
        }
        return sum / weight_measurements_.size();
    }

    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const VerifyWeight::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(),
                   "Received verification request for object %u (estimated: %.1f g)",
                   goal->object_id, goal->estimated_weight);
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleVerifyWeight> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received request to cancel verification");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandleVerifyWeight> goal_handle)
    {
        // Execute in a separate thread
        std::thread{std::bind(&VerificationNode::execute_verification, this, goal_handle)}.detach();
    }

    void execute_verification(const std::shared_ptr<GoalHandleVerifyWeight> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<VerifyWeight::Feedback>();
        auto result = std::make_shared<VerifyWeight::Result>();

        RCLCPP_INFO(this->get_logger(), "Starting weight verification...");

        // Clear previous measurements
        weight_measurements_.clear();

        // Collect measurements for the specified duration
        feedback->current_state = "measuring";
        double elapsed = 0.0;
        double update_rate = 10.0;  // Hz
        rclcpp::Rate rate(update_rate);

        while (elapsed < measurement_duration_ && rclcpp::ok()) {
            if (goal_handle->is_canceling()) {
                result->verified = false;
                result->message = "Verification cancelled";
                goal_handle->canceled(result);
                return;
            }

            feedback->measurement_progress = elapsed / measurement_duration_;
            goal_handle->publish_feedback(feedback);

            elapsed += 1.0 / update_rate;
            rate.sleep();
        }

        // Calculate final measured weight
        result->actual_weight = filtered_weight_;

        // Verify against estimated weight
        float tolerance = goal->tolerance > 0.0f ? goal->tolerance : default_tolerance_ / 100.0f;
        float error = std::abs(result->actual_weight - goal->estimated_weight);
        result->error_percentage = (error / goal->estimated_weight) * 100.0f;

        result->verified = (error <= goal->estimated_weight * tolerance);

        if (result->verified) {
            result->message = "Weight verified successfully";
            RCLCPP_INFO(this->get_logger(),
                       "✓ Weight verified: %.1f g (error: %.1f%%)",
                       result->actual_weight, result->error_percentage);
        } else {
            result->message = "Weight verification failed";
            RCLCPP_WARN(this->get_logger(),
                       "✗ Weight mismatch: Expected %.1f g, measured %.1f g (error: %.1f%%)",
                       goal->estimated_weight, result->actual_weight, result->error_percentage);
        }

        // Publish result
        auto verification_msg = std_msgs::msg::Bool();
        verification_msg.data = result->verified;
        verification_result_pub_->publish(verification_msg);

        goal_handle->succeed(result);
    }

    // Member variables
    rclcpp::Subscription<sort_interfaces::msg::WeightEstimate>::SharedPtr estimated_weights_sub_;
    rclcpp::Subscription<sort_interfaces::msg::ForceFeedback>::SharedPtr actual_weight_sub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr verification_result_pub_;
    rclcpp_action::Server<VerifyWeight>::SharedPtr verify_weight_action_server_;

    std::map<uint32_t, float> estimated_weights_;
    std::vector<float> weight_measurements_;

    float latest_actual_weight_;
    float filtered_weight_;

    double default_tolerance_;
    double measurement_duration_;
    int filter_window_size_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<VerificationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
