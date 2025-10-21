/**
 * @file moveit2_interface_node.cpp
 * @brief MoveIt2 motion planning interface for UR5e
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/display_trajectory.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include "sort_interfaces/action/plan_trajectory.hpp"
#include <memory>

class MoveIt2InterfaceNode : public rclcpp::Node
{
public:
    using PlanTrajectory = sort_interfaces::action::PlanTrajectory;
    using GoalHandlePlanTrajectory = rclcpp_action::ServerGoalHandle<PlanTrajectory>;

    MoveIt2InterfaceNode() : Node("moveit2_interface_node")
    {
        // Declare parameters
        this->declare_parameter("planning_group", "ur_manipulator");
        this->declare_parameter("end_effector_link", "tool0");
        this->declare_parameter("reference_frame", "base_link");
        this->declare_parameter("default_planning_time", 5.0);
        this->declare_parameter("default_velocity_scaling", 0.5);
        this->declare_parameter("default_acceleration_scaling", 0.5);

        // Get parameters
        planning_group_ = this->get_parameter("planning_group").as_string();
        end_effector_link_ = this->get_parameter("end_effector_link").as_string();
        reference_frame_ = this->get_parameter("reference_frame").as_string();
        default_planning_time_ = this->get_parameter("default_planning_time").as_double();
        velocity_scaling_ = this->get_parameter("default_velocity_scaling").as_double();
        acceleration_scaling_ = this->get_parameter("default_acceleration_scaling").as_double();

        // Publisher for trajectory visualization
        trajectory_pub_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>(
            "/planning/trajectory", 10);

        // Action servers
        plan_pick_action_server_ = rclcpp_action::create_server<PlanTrajectory>(
            this,
            "/planning/plan_pick",
            std::bind(&MoveIt2InterfaceNode::handle_goal, this,
                     std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveIt2InterfaceNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&MoveIt2InterfaceNode::handle_accepted, this, std::placeholders::_1));

        plan_place_action_server_ = rclcpp_action::create_server<PlanTrajectory>(
            this,
            "/planning/plan_place",
            std::bind(&MoveIt2InterfaceNode::handle_goal, this,
                     std::placeholders::_1, std::placeholders::_2),
            std::bind(&MoveIt2InterfaceNode::handle_cancel, this, std::placeholders::_1),
            std::bind(&MoveIt2InterfaceNode::handle_accepted, this, std::placeholders::_1));

        RCLCPP_INFO(this->get_logger(), "MoveIt2 Interface Node initialized");
        RCLCPP_INFO(this->get_logger(), "Planning group: %s", planning_group_.c_str());
        RCLCPP_INFO(this->get_logger(), "End effector: %s", end_effector_link_.c_str());

        // TODO: Initialize MoveIt2 MoveGroupInterface
        // NOTE: This requires a running move_group node
        RCLCPP_WARN(this->get_logger(),
                   "TODO: Initialize MoveIt2 MoveGroupInterface - requires move_group node");

        try {
            // Attempt to create move group interface
            // move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(
            //     shared_from_this(), planning_group_);
            // planning_scene_interface_ =
            //     std::make_shared<moveit::planning_interface::PlanningSceneInterface>();

            RCLCPP_INFO(this->get_logger(), "MoveIt2 interfaces created successfully");
        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(),
                        "Failed to initialize MoveIt2: %s", e.what());
        }
    }

private:
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID& uuid,
        std::shared_ptr<const PlanTrajectory::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(this->get_logger(),
                   "Received planning request for group: %s", goal->planning_group.c_str());
        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandlePlanTrajectory> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received request to cancel planning");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    void handle_accepted(const std::shared_ptr<GoalHandlePlanTrajectory> goal_handle)
    {
        // Execute in a separate thread
        std::thread{std::bind(&MoveIt2InterfaceNode::execute_planning, this, goal_handle)}.detach();
    }

    void execute_planning(const std::shared_ptr<GoalHandlePlanTrajectory> goal_handle)
    {
        const auto goal = goal_handle->get_goal();
        auto feedback = std::make_shared<PlanTrajectory::Feedback>();
        auto result = std::make_shared<PlanTrajectory::Result>();

        RCLCPP_INFO(this->get_logger(), "Planning trajectory to target pose...");

        // TODO: Implement actual MoveIt2 planning
        // Steps:
        // 1. Set planning time
        // 2. Set target pose
        // 3. Call plan()
        // 4. Check if planning succeeded
        // 5. Return trajectory

        feedback->current_state = "planning";
        feedback->planning_progress = 0.5;
        goal_handle->publish_feedback(feedback);

        // Placeholder implementation
        result->success = false;
        result->message = "TODO: Implement MoveIt2 planning - requires move_group node and URDF";

        RCLCPP_WARN(this->get_logger(), "%s", result->message.c_str());

        /* Example MoveIt2 planning code (uncomment when MoveIt2 is configured):

        if (!move_group_) {
            result->success = false;
            result->message = "MoveGroup interface not initialized";
            goal_handle->abort(result);
            return;
        }

        // Set planning parameters
        move_group_->setPlanningTime(goal->planning_time > 0.0 ?
                                    goal->planning_time : default_planning_time_);
        move_group_->setMaxVelocityScalingFactor(velocity_scaling_);
        move_group_->setMaxAccelerationScalingFactor(acceleration_scaling_);

        // Set target pose
        move_group_->setPoseTarget(goal->target_pose);

        // Plan
        moveit::planning_interface::MoveGroupInterface::Plan plan;
        feedback->current_state = "computing";
        feedback->planning_progress = 0.7;
        goal_handle->publish_feedback(feedback);

        bool success = (move_group_->plan(plan) ==
                       moveit::core::MoveItErrorCode::SUCCESS);

        if (success) {
            result->success = true;
            result->trajectory = plan.trajectory_;
            result->message = "Planning succeeded";

            // Publish for visualization
            moveit_msgs::msg::DisplayTrajectory display_trajectory;
            display_trajectory.trajectory.push_back(plan.trajectory_);
            trajectory_pub_->publish(display_trajectory);

            RCLCPP_INFO(this->get_logger(), "✓ Planning succeeded");
            goal_handle->succeed(result);
        } else {
            result->success = false;
            result->message = "Planning failed";
            RCLCPP_ERROR(this->get_logger(), "✗ Planning failed");
            goal_handle->abort(result);
        }
        */

        // For now, just abort since MoveIt2 is not configured
        goal_handle->abort(result);
    }

    // Member variables
    rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr trajectory_pub_;
    rclcpp_action::Server<PlanTrajectory>::SharedPtr plan_pick_action_server_;
    rclcpp_action::Server<PlanTrajectory>::SharedPtr plan_place_action_server_;

    // TODO: Uncomment when MoveIt2 is properly configured
    // std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    // std::shared_ptr<moveit::planning_interface::PlanningSceneInterface> planning_scene_interface_;

    std::string planning_group_;
    std::string end_effector_link_;
    std::string reference_frame_;
    double default_planning_time_;
    double velocity_scaling_;
    double acceleration_scaling_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<MoveIt2InterfaceNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
