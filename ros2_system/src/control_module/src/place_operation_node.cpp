/**
 * @file place_operation_node.cpp
 * @brief Place operation coordinator with state machine
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include "sort_interfaces/action/place_object.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <string>

class PlaceOperationNode : public rclcpp::Node
{
public:
    using PlaceObject = sort_interfaces::action::PlaceObject;
    using GoalHandlePlaceObject = rclcpp_action::ServerGoalHandle<PlaceObject>;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;

    enum class PlaceState {
        IDLE,
        APPROACHING,
        LOWERING,
        PLACING,
        RELEASING,
        RETREATING,
        COMPLETE,
        ERROR
    };

    PlaceOperationNode() : Node("place_operation_node"), current_state_(PlaceState::IDLE)
    {
        // Declare parameters
        this->declare_parameter("approach_height", 0.15);    // meters above target
        this->declare_parameter("retreat_distance", 0.2);    // meters
        this->declare_parameter("place_offset", 0.01);       // meters above surface
        this->declare_parameter("gripper_open_value", 0.0);  // 0.0=open, 1.0=closed
        this->declare_parameter("release_delay", 0.5);       // seconds

        // Get parameters
        approach_height_ = this->get_parameter("approach_height").as_double();
        retreat_distance_ = this->get_parameter("retreat_distance").as_double();
        place_offset_ = this->get_parameter("place_offset").as_double();
        gripper_open_value_ = this->get_parameter("gripper_open_value").as_double();
        release_delay_ = this->get_parameter("release_delay").as_double();

        // Publisher
        gripper_command_pub_ = this->create_publisher<std_msgs::msg::Float32>(
            "/motion_control/gripper_command", 10);

        // Action server
        place_action_server_ = rclcpp_action::create_server<PlaceObject>(
            this,
            "/control/place_object",
            std::bind(&PlaceOperationNode::handle_goal, this,
                     std::placeholders::_1, std::placeholders::_2),
            std::bind(&PlaceOperationNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&PlaceOperationNode::handle_accepted, this, std::placeholders::_1));

        // Action client for trajectory execution
        trajectory_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, "/control/execute_trajectory");

        RCLCPP_INFO(this->get_logger(), "Place Operation Node initialized");
        RCLCPP_INFO(this->get_logger(),
                   "Approach height: %.3f m, Retreat distance: %.3f m",
                   approach_height_, retreat_distance_);
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const PlaceObject::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(),
                   "Place request for object %u at [%.2f, %.2f, %.2f]",
                   goal->object_id,
                   goal->target_pose.position.x,
                   goal->target_pose.position.y,
                   goal->target_pose.position.z);

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePlaceObject> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Place operation cancel requested");
        current_state_ = PlaceState::IDLE;
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePlaceObject> goal_handle)
    {
        std::thread{std::bind(&PlaceOperationNode::execute_place, this, goal_handle)}.detach();
    }

    void execute_place(const std::shared_ptr<GoalHandlePlaceObject> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<PlaceObject::Feedback>();
        auto result = std::make_shared<PlaceObject::Result>();

        RCLCPP_INFO(this->get_logger(), "Starting place operation...");

        // State machine for place operation
        current_state_ = PlaceState::APPROACHING;

        // 1. Approach target area from above
        feedback->current_state = "approaching";
        feedback->progress = 0.2;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "State: APPROACHING");

        if (!move_to_approach_position(goal->target_pose)) {
            result->success = false;
            result->message = "Failed to approach placement area";
            goal_handle->abort(result);
            current_state_ = PlaceState::ERROR;
            return;
        }

        // 2. Lower to placement height
        feedback->current_state = "placing";
        feedback->progress = 0.4;
        goal_handle->publish_feedback(feedback);
        current_state_ = PlaceState::LOWERING;
        RCLCPP_INFO(this->get_logger(), "State: LOWERING");

        if (!lower_to_place_position(goal->target_pose)) {
            result->success = false;
            result->message = "Failed to lower to placement position";
            goal_handle->abort(result);
            current_state_ = PlaceState::ERROR;
            return;
        }

        // 3. Place object (touch down gently)
        current_state_ = PlaceState::PLACING;
        feedback->progress = 0.6;
        goal_handle->publish_feedback(feedback);
        RCLCPP_INFO(this->get_logger(), "State: PLACING");

        // Small delay to ensure stable contact
        rclcpp::sleep_for(std::chrono::milliseconds(300));

        // 4. Release gripper
        feedback->current_state = "releasing";
        feedback->progress = 0.7;
        goal_handle->publish_feedback(feedback);
        current_state_ = PlaceState::RELEASING;
        RCLCPP_INFO(this->get_logger(), "State: RELEASING");

        release_gripper();
        rclcpp::sleep_for(std::chrono::duration_cast<std::chrono::nanoseconds>(
            std::chrono::duration<double>(release_delay_)));

        // 5. Retreat from object
        feedback->current_state = "retreating";
        feedback->progress = 0.9;
        goal_handle->publish_feedback(feedback);
        current_state_ = PlaceState::RETREATING;
        RCLCPP_INFO(this->get_logger(), "State: RETREATING");

        if (!retreat_from_object(goal->target_pose, goal->retreat_distance)) {
            RCLCPP_WARN(this->get_logger(), "Retreat motion incomplete, but object placed");
            // Don't fail - object was successfully placed
        }

        // Complete
        current_state_ = PlaceState::COMPLETE;
        result->success = true;
        result->message = "Place operation completed successfully";

        RCLCPP_INFO(this->get_logger(), "✓ Place operation complete");

        goal_handle->succeed(result);
        current_state_ = PlaceState::IDLE;
    }

    bool move_to_approach_position(const geometry_msgs::msg::Pose& target)
    {
        // TODO: Plan and execute trajectory to approach position
        // Approach position = target position + vertical offset
        RCLCPP_INFO(this->get_logger(),
                   "Moving to approach position (%.3f m above target)...",
                   approach_height_);

        rclcpp::sleep_for(std::chrono::seconds(1));  // Simulate motion
        return true;
    }

    bool lower_to_place_position(const geometry_msgs::msg::Pose& target)
    {
        // TODO: Plan and execute slow lowering motion
        // Final position should be slightly above surface to account for object size
        RCLCPP_INFO(this->get_logger(),
                   "Lowering to placement position (%.3f m offset)...",
                   place_offset_);

        rclcpp::sleep_for(std::chrono::milliseconds(800));  // Simulate slow motion
        return true;
    }

    bool retreat_from_object(const geometry_msgs::msg::Pose& current_pose, float retreat_dist)
    {
        // TODO: Plan and execute retreat motion
        // Move up and/or back to safe distance
        RCLCPP_INFO(this->get_logger(), "Retreating %.3f m...", retreat_dist);

        rclcpp::sleep_for(std::chrono::milliseconds(600));  // Simulate motion
        return true;
    }

    void release_gripper()
    {
        auto cmd = std_msgs::msg::Float32();
        cmd.data = gripper_open_value_;  // 0.0 = fully open
        gripper_command_pub_->publish(cmd);
        RCLCPP_INFO(this->get_logger(), "Releasing gripper...");
    }

    // Member variables
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_command_pub_;

    rclcpp_action::Server<PlaceObject>::SharedPtr place_action_server_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;

    PlaceState current_state_;
    double approach_height_;
    double retreat_distance_;
    double place_offset_;
    double gripper_open_value_;
    double release_delay_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PlaceOperationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
