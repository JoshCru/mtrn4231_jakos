/**
 * @file cartesian_controller_node.cpp
 * @brief C++ Cartesian Controller Node for UR5e robot
 *
 * Provides a service interface for Cartesian movements that can be called
 * from the supervisor module. Uses MoveIt for path planning and direct
 * trajectory execution via FollowJointTrajectory action.
 *
 * Service: /motion_control/move_to_cartesian (sort_interfaces/srv/MoveToCartesian)
 *
 * Safety boundaries are enforced (relative to base_link):
 *   - x >= -600mm, x <= 2000mm
 *   - y >= -300mm, y <= 2000mm
 *   - z >= 0mm, z <= 655mm
 */

#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <moveit_msgs/srv/get_cartesian_path.hpp>
#include <control_msgs/action/follow_joint_trajectory.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <sort_interfaces/srv/move_to_cartesian.hpp>
#include <trajectory_msgs/msg/joint_trajectory.hpp>
#include <std_srvs/srv/set_bool.hpp>
#include <cmath>
#include <mutex>
#include <memory>
#include <atomic>
#include <condition_variable>

namespace motion_control_module
{

// Robot joint names
const std::vector<std::string> ROBOT_JOINTS = {
    "shoulder_pan_joint",
    "shoulder_lift_joint",
    "elbow_joint",
    "wrist_1_joint",
    "wrist_2_joint",
    "wrist_3_joint"
};

// Safety boundaries (in meters, base_link frame)
struct SafetyBounds {
    static constexpr double X_MIN = -0.6;
    static constexpr double X_MAX = 2.0;
    static constexpr double Y_MIN = -0.7;  // Extended to allow y=-588mm
    static constexpr double Y_MAX = 2.0;
    static constexpr double Z_MIN_TOOL0 = 0.180;       // 180mm minimum for tool0 frame
    static constexpr double Z_MIN_GRIPPER_TIP = 0.010; // 10mm minimum for gripper_tip frame
    static constexpr double Z_MAX = 0.655;
};

class CartesianControllerNode : public rclcpp::Node
{
public:
    using FollowJointTrajectory = control_msgs::action::FollowJointTrajectory;
    using GoalHandleFJT = rclcpp_action::ClientGoalHandle<FollowJointTrajectory>;

    CartesianControllerNode(const rclcpp::NodeOptions & options = rclcpp::NodeOptions())
    : Node("cartesian_controller_node", options)
    {
        // Use a reentrant callback group for the service to allow nested spinning
        callback_group_ = this->create_callback_group(
            rclcpp::CallbackGroupType::Reentrant);

        // Declare parameters
        this->declare_parameter("use_fake_hardware", true);
        use_fake_hardware_ = this->get_parameter("use_fake_hardware").as_bool();

        // Both simulation and real hardware use scaled_joint_trajectory_controller
        // Simulation: explicitly set in launch
        // Real hardware: default controller
        controller_name_ = "/scaled_joint_trajectory_controller/follow_joint_trajectory";

        RCLCPP_INFO(this->get_logger(), "Using controller: %s", controller_name_.c_str());

        init_clients();
        init_subscribers();
        init_services();

        RCLCPP_INFO(this->get_logger(), "Cartesian Controller Node initialized");
        RCLCPP_INFO(this->get_logger(), "Service available: /motion_control/move_to_cartesian");
    }

private:
    void init_clients()
    {
        // MoveIt Cartesian path planning service
        cartesian_path_client_ = this->create_client<moveit_msgs::srv::GetCartesianPath>(
            "/compute_cartesian_path",
            rmw_qos_profile_services_default,
            callback_group_);

        // Trajectory execution action client
        trajectory_action_client_ = rclcpp_action::create_client<FollowJointTrajectory>(
            this, controller_name_, callback_group_);

        // Wait for services
        RCLCPP_INFO(this->get_logger(), "Waiting for /compute_cartesian_path service...");
        while (!cartesian_path_client_->wait_for_service(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for service");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Still waiting for cartesian path service...");
        }
        RCLCPP_INFO(this->get_logger(), "Connected to /compute_cartesian_path");

        // Wait for action server
        RCLCPP_INFO(this->get_logger(), "Waiting for trajectory action server...");
        while (!trajectory_action_client_->wait_for_action_server(std::chrono::seconds(1))) {
            if (!rclcpp::ok()) {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for action server");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Still waiting for trajectory action server...");
        }
        RCLCPP_INFO(this->get_logger(), "Connected to trajectory action server");
    }

    void init_subscribers()
    {
        // Subscribe to joint states with transient local QoS
        rclcpp::QoS qos(10);
        qos.transient_local();

        rclcpp::SubscriptionOptions sub_options;
        sub_options.callback_group = callback_group_;

        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states", qos,
            std::bind(&CartesianControllerNode::joint_state_callback, this, std::placeholders::_1),
            sub_options);

        // Wait for joint states
        RCLCPP_INFO(this->get_logger(), "Waiting for joint states...");
        auto start = this->now();
        while (rclcpp::ok() && !current_joint_state_) {
            rclcpp::spin_some(this->get_node_base_interface());
            if ((this->now() - start).seconds() > 10.0) {
                RCLCPP_WARN(this->get_logger(), "Joint states not received within timeout");
                break;
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
        if (current_joint_state_) {
            RCLCPP_INFO(this->get_logger(), "Joint states received");
        }
    }

    void init_services()
    {
        // Main Cartesian movement service with reentrant callback group
        move_to_cartesian_srv_ = this->create_service<sort_interfaces::srv::MoveToCartesian>(
            "/motion_control/move_to_cartesian",
            std::bind(&CartesianControllerNode::handle_move_to_cartesian, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_);

        // Toggle service: true = gripper_tip, false = tool0
        toggle_frame_srv_ = this->create_service<std_srvs::srv::SetBool>(
            "/motion_control/use_gripper_tip",
            std::bind(&CartesianControllerNode::handle_toggle_frame, this,
                      std::placeholders::_1, std::placeholders::_2),
            rmw_qos_profile_services_default,
            callback_group_);

        RCLCPP_INFO(this->get_logger(), "Frame toggle service: /motion_control/use_gripper_tip");
        RCLCPP_INFO(this->get_logger(), "  true = gripper_tip, false = tool0");
    }

    void handle_toggle_frame(
        const std::shared_ptr<std_srvs::srv::SetBool::Request> request,
        std::shared_ptr<std_srvs::srv::SetBool::Response> response)
    {
        use_gripper_tip_ = request->data;
        std::string frame = use_gripper_tip_ ? "gripper_tip" : "tool0";
        RCLCPP_INFO(this->get_logger(), "Switched to frame: %s", frame.c_str());
        response->success = true;
        response->message = "Now using frame: " + frame;
    }

    void joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        std::lock_guard<std::mutex> lock(joint_state_mutex_);
        current_joint_state_ = msg;
    }

    // Convert rotation vector (axis-angle) to quaternion
    std::array<double, 4> rotation_vector_to_quaternion(double rx, double ry, double rz)
    {
        double angle = std::sqrt(rx*rx + ry*ry + rz*rz);
        if (angle < 1e-10) {
            return {0.0, 0.0, 0.0, 1.0};
        }

        double axis_x = rx / angle;
        double axis_y = ry / angle;
        double axis_z = rz / angle;
        double half_angle = angle / 2.0;
        double sin_half = std::sin(half_angle);
        double cos_half = std::cos(half_angle);

        return {axis_x * sin_half, axis_y * sin_half, axis_z * sin_half, cos_half};
    }

    // Check if position is within safety boundaries (after negation to robot frame)
    // use_gripper_tip: true = gripper_tip frame (Z_MIN=10mm), false = tool0 frame (Z_MIN=180mm)
    std::pair<bool, std::string> check_boundaries(double x_mm, double y_mm, double z_mm, bool use_gripper_tip)
    {
        // Apply same negation as coordinate transform
        double x = -x_mm / 1000.0;
        double y = -y_mm / 1000.0;
        double z = z_mm / 1000.0;

        // Select Z_MIN based on frame
        double z_min = use_gripper_tip ? SafetyBounds::Z_MIN_GRIPPER_TIP : SafetyBounds::Z_MIN_TOOL0;
        std::string frame_name = use_gripper_tip ? "gripper_tip" : "tool0";

        if (x < SafetyBounds::X_MIN) {
            return {false, "robot x=" + std::to_string(x * 1000) + "mm below minimum " +
                           std::to_string(SafetyBounds::X_MIN * 1000) + "mm"};
        }
        if (x > SafetyBounds::X_MAX) {
            return {false, "robot x=" + std::to_string(x * 1000) + "mm exceeds maximum " +
                           std::to_string(SafetyBounds::X_MAX * 1000) + "mm"};
        }
        if (y < SafetyBounds::Y_MIN) {
            return {false, "robot y=" + std::to_string(y * 1000) + "mm below minimum " +
                           std::to_string(SafetyBounds::Y_MIN * 1000) + "mm"};
        }
        if (y > SafetyBounds::Y_MAX) {
            return {false, "robot y=" + std::to_string(y * 1000) + "mm exceeds maximum " +
                           std::to_string(SafetyBounds::Y_MAX * 1000) + "mm"};
        }
        if (z < z_min) {
            return {false, frame_name + " z=" + std::to_string(z_mm) + "mm below minimum " +
                           std::to_string(z_min * 1000) + "mm"};
        }
        if (z > SafetyBounds::Z_MAX) {
            return {false, "robot z=" + std::to_string(z * 1000) + "mm exceeds maximum " +
                           std::to_string(SafetyBounds::Z_MAX * 1000) + "mm"};
        }

        return {true, "Position within safety boundaries"};
    }

    void handle_move_to_cartesian(
        const std::shared_ptr<sort_interfaces::srv::MoveToCartesian::Request> request,
        std::shared_ptr<sort_interfaces::srv::MoveToCartesian::Response> response)
    {
        RCLCPP_INFO(this->get_logger(),
            "Received move request: x=%.1f, y=%.1f, z=%.1f, rx=%.3f, ry=%.3f, rz=%.3f",
            request->x, request->y, request->z, request->rx, request->ry, request->rz);

        // Check safety boundaries (Z_MIN depends on frame: 180mm for tool0, 10mm for gripper_tip)
        auto [is_safe, safety_msg] = check_boundaries(request->x, request->y, request->z, use_gripper_tip_);
        if (!is_safe) {
            RCLCPP_ERROR(this->get_logger(), "SAFETY VIOLATION: %s", safety_msg.c_str());
            response->success = false;
            response->message = "Safety boundary violation: " + safety_msg;
            return;
        }

        // Check joint state availability
        sensor_msgs::msg::JointState::SharedPtr joint_state;
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            if (!current_joint_state_) {
                response->success = false;
                response->message = "No joint state available";
                return;
            }
            joint_state = current_joint_state_;
        }

        // Convert mm to meters (negate x/y for coordinate frame alignment)
        double x_m = -request->x / 1000.0;
        double y_m = -request->y / 1000.0;
        double z_m = request->z / 1000.0;
        // Negate ry to match coordinate frame transformation (x and y are also negated)
        auto quat = rotation_vector_to_quaternion(request->rx, -request->ry, request->rz);

        RCLCPP_INFO(this->get_logger(), "Target (meters): x=%.4f, y=%.4f, z=%.4f", x_m, y_m, z_m);

        // Create Cartesian path request
        auto path_request = std::make_shared<moveit_msgs::srv::GetCartesianPath::Request>();
        path_request->header.frame_id = "base_link";
        path_request->header.stamp = this->now();
        path_request->start_state.joint_state = *joint_state;
        path_request->group_name = "ur_manipulator";
        path_request->link_name = use_gripper_tip_ ? "gripper_tip" : "tool0";
        RCLCPP_INFO(this->get_logger(), "Using end-effector frame: %s", path_request->link_name.c_str());

        geometry_msgs::msg::Pose target_pose;
        target_pose.position.x = x_m;
        target_pose.position.y = y_m;
        target_pose.position.z = z_m;
        target_pose.orientation.x = quat[0];
        target_pose.orientation.y = quat[1];
        target_pose.orientation.z = quat[2];
        target_pose.orientation.w = quat[3];
        path_request->waypoints.push_back(target_pose);

        path_request->max_step = 0.01;  // 1cm interpolation
        path_request->jump_threshold = 0.0;
        path_request->avoid_collisions = true;

        // Plan path using MoveIt
        RCLCPP_INFO(this->get_logger(), "Planning Cartesian path...");
        auto future = cartesian_path_client_->async_send_request(path_request);

        // Wait for planning result
        if (future.wait_for(std::chrono::seconds(30)) != std::future_status::ready) {
            response->success = false;
            response->message = "Path planning timed out";
            return;
        }

        auto path_response = future.get();
        if (path_response->fraction < 0.95) {
            response->success = false;
            response->message = "Only " + std::to_string(path_response->fraction * 100) +
                               "% of path could be computed - likely collision";
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Path computed: %.1f%%, %zu points",
            path_response->fraction * 100,
            path_response->solution.joint_trajectory.points.size());

        // Execute trajectory
        auto trajectory_goal = FollowJointTrajectory::Goal();
        trajectory_goal.trajectory = path_response->solution.joint_trajectory;

        if (trajectory_goal.trajectory.joint_names.empty()) {
            trajectory_goal.trajectory.joint_names = ROBOT_JOINTS;
        }

        RCLCPP_INFO(this->get_logger(), "Executing trajectory with %zu points...",
            trajectory_goal.trajectory.points.size());

        // Store result in a promise to be filled by the callback
        auto result_promise = std::make_shared<std::promise<rclcpp_action::ClientGoalHandle<FollowJointTrajectory>::WrappedResult>>();
        auto result_future = result_promise->get_future();

        // Send goal with result callback
        auto send_goal_options = rclcpp_action::Client<FollowJointTrajectory>::SendGoalOptions();

        send_goal_options.goal_response_callback =
            [this](const GoalHandleFJT::SharedPtr & goal_handle) {
                if (!goal_handle) {
                    RCLCPP_ERROR(this->get_logger(), "Goal was rejected by server");
                } else {
                    RCLCPP_INFO(this->get_logger(), "Goal accepted by server, executing...");
                }
            };

        send_goal_options.result_callback =
            [this, result_promise](const GoalHandleFJT::WrappedResult & result) {
                RCLCPP_INFO(this->get_logger(), "Goal sent, waiting for result...");
                result_promise->set_value(result);
            };

        auto goal_future = trajectory_action_client_->async_send_goal(
            trajectory_goal, send_goal_options);

        // Wait for goal to be accepted
        if (goal_future.wait_for(std::chrono::seconds(10)) != std::future_status::ready) {
            response->success = false;
            response->message = "Failed to send trajectory goal (timeout)";
            RCLCPP_ERROR(this->get_logger(), "Goal send timeout");
            return;
        }

        auto goal_handle = goal_future.get();
        if (!goal_handle) {
            response->success = false;
            response->message = "Trajectory goal was rejected by server";
            RCLCPP_ERROR(this->get_logger(), "Goal rejected");
            return;
        }

        // Wait for result with timeout (callback will fill the promise)
        auto timeout = std::chrono::seconds(120);
        auto status = result_future.wait_for(timeout);

        if (status != std::future_status::ready) {
            response->success = false;
            response->message = "Trajectory execution timed out after 120 seconds";
            RCLCPP_ERROR(this->get_logger(), "Execution timeout");
            return;
        }

        // Get the result
        auto wrapped_result = result_future.get();

        bool execution_success = false;
        std::string execution_message;

        switch (wrapped_result.code) {
            case rclcpp_action::ResultCode::SUCCEEDED:
                if (wrapped_result.result->error_code == 0) {
                    execution_message = "Movement completed successfully";
                    execution_success = true;
                    RCLCPP_INFO(this->get_logger(), "Trajectory execution SUCCEEDED");
                } else {
                    execution_message = "Movement failed: " + wrapped_result.result->error_string;
                    execution_success = false;
                    RCLCPP_ERROR(this->get_logger(), "Trajectory failed: %s",
                        wrapped_result.result->error_string.c_str());
                }
                break;
            case rclcpp_action::ResultCode::ABORTED:
                execution_message = "Trajectory aborted";
                execution_success = false;
                RCLCPP_ERROR(this->get_logger(), "Trajectory ABORTED");
                break;
            case rclcpp_action::ResultCode::CANCELED:
                execution_message = "Trajectory canceled";
                execution_success = false;
                RCLCPP_WARN(this->get_logger(), "Trajectory CANCELED");
                break;
            default:
                execution_message = "Unknown result";
                execution_success = false;
                RCLCPP_ERROR(this->get_logger(), "Unknown result code");
                break;
        }

        response->success = execution_success;
        response->message = execution_message;

        // Return final joint positions
        {
            std::lock_guard<std::mutex> lock(joint_state_mutex_);
            if (current_joint_state_) {
                response->joint_positions = current_joint_state_->position;
            }
        }

        if (execution_success) {
            RCLCPP_INFO(this->get_logger(), "Movement completed successfully");
        } else {
            RCLCPP_ERROR(this->get_logger(), "Movement failed: %s", execution_message.c_str());
        }
    }

    // Member variables
    bool use_fake_hardware_;
    bool use_gripper_tip_ = false;  // true = gripper_tip, false = tool0 (default to tool0)
    std::string controller_name_;
    rclcpp::CallbackGroup::SharedPtr callback_group_;

    rclcpp::Client<moveit_msgs::srv::GetCartesianPath>::SharedPtr cartesian_path_client_;
    rclcpp_action::Client<FollowJointTrajectory>::SharedPtr trajectory_action_client_;
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Service<sort_interfaces::srv::MoveToCartesian>::SharedPtr move_to_cartesian_srv_;
    rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr toggle_frame_srv_;

    sensor_msgs::msg::JointState::SharedPtr current_joint_state_;
    std::mutex joint_state_mutex_;
};

}  // namespace motion_control_module

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<motion_control_module::CartesianControllerNode>();

    // Use MultiThreadedExecutor to allow callbacks to run during service handling
    rclcpp::executors::MultiThreadedExecutor executor;
    executor.add_node(node);

    RCLCPP_INFO(node->get_logger(), "Cartesian Controller Node started (MultiThreadedExecutor)");
    RCLCPP_INFO(node->get_logger(), "Call service: /motion_control/move_to_cartesian");

    executor.spin();
    rclcpp::shutdown();
    return 0;
}
