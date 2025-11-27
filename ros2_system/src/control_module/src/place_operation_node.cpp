/**
 * @file place_operation_node.cpp
 * @brief Place operation coordinator with state machine
 *
 * Uses the cartesian controller service for actual robot movements.
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/float32.hpp>
#include "sort_interfaces/action/place_object.hpp"
#include "sort_interfaces/srv/move_to_cartesian.hpp"
#include "sort_interfaces/srv/gripper_control.hpp"
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <string>
#include <mutex>

class PlaceOperationNode : public rclcpp::Node
{
public:
    using PlaceObject = sort_interfaces::action::PlaceObject;
    using GoalHandlePlaceObject = rclcpp_action::ServerGoalHandle<PlaceObject>;
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using MoveToCartesian = sort_interfaces::srv::MoveToCartesian;
    using GripperControl = sort_interfaces::srv::GripperControl;

    // Z heights in mm (tool0 frame) - from pick_and_place_demo.py
    static constexpr double Z_HOME = 371.0;
    static constexpr double Z_DESCEND = 212.0;
    static constexpr double Z_PLACE = 182.0;

    // Default orientation (facing down)
    static constexpr double RX = 2.221;
    static constexpr double RY = 2.221;
    static constexpr double RZ = 0.0;

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
        // Create callback group for concurrent operations
        callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

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

        RCLCPP_INFO(this->get_logger(), "Place Operation Node initialized");
        RCLCPP_INFO(this->get_logger(), "Z heights: descend=%.1f mm, place=%.1f mm",
                   Z_DESCEND, Z_PLACE);
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

        RCLCPP_INFO(this->get_logger(), "Place operation complete");

        goal_handle->succeed(result);
        current_state_ = PlaceState::IDLE;
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

    bool move_to_approach_position(const geometry_msgs::msg::Pose& target)
    {
        // Convert target pose from meters to mm
        double x_mm = target.position.x * 1000.0;
        double y_mm = target.position.y * 1000.0;

        RCLCPP_INFO(this->get_logger(), "Moving to approach position above placement...");

        // Move to Z_DESCEND height above the target
        return move_to_cartesian(x_mm, y_mm, Z_DESCEND);
    }

    bool lower_to_place_position(const geometry_msgs::msg::Pose& target)
    {
        double x_mm = target.position.x * 1000.0;
        double y_mm = target.position.y * 1000.0;

        RCLCPP_INFO(this->get_logger(), "Lowering to placement position...");

        // Move to Z_PLACE height
        return move_to_cartesian(x_mm, y_mm, Z_PLACE);
    }

    bool retreat_from_object(const geometry_msgs::msg::Pose& current_pose, float retreat_dist)
    {
        double x_mm = current_pose.position.x * 1000.0;
        double y_mm = current_pose.position.y * 1000.0;

        RCLCPP_INFO(this->get_logger(), "Retreating to descend height...");

        // Retreat to Z_DESCEND
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

    void release_gripper()
    {
        // Use service for more reliable control
        if (!gripper_control("open")) {
            // Fallback to topic
            auto cmd = std_msgs::msg::Float32();
            cmd.data = gripper_open_value_;
            gripper_command_pub_->publish(cmd);
        }
    }

    // Member variables
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr gripper_command_pub_;

    rclcpp_action::Server<PlaceObject>::SharedPtr place_action_server_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_client_;

    // Service clients for motion control
    rclcpp::Client<MoveToCartesian>::SharedPtr move_client_;
    rclcpp::Client<GripperControl>::SharedPtr gripper_client_;

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

    // Use MultiThreadedExecutor to allow concurrent service calls
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);
    executor.spin();

    rclcpp::shutdown();
    return 0;
}
