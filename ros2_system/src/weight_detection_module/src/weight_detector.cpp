// weight_detector.cpp - C++ implementation of weight detection node
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <sort_interfaces/srv/calibrate_baseline.hpp>
#include <Eigen/Dense>
#include <deque>
#include <vector>
#include <cmath>
#include <algorithm>
#include <numeric>
#include <chrono>
#include <map>

class KalmanFilter
{
public:
    KalmanFilter(double process_variance = 0.001, double measurement_variance = 0.15)
        : process_variance_(process_variance),
          measurement_variance_(measurement_variance),
          estimate_(0.0),
          error_covariance_(1.0),
          initialized_(false) {}

    double update(double measurement)
    {
        if (!initialized_)
        {
            estimate_ = measurement;
            initialized_ = true;
            return estimate_;
        }

        error_covariance_ += process_variance_;
        double kalman_gain = error_covariance_ / (error_covariance_ + measurement_variance_);
        estimate_ += kalman_gain * (measurement - estimate_);
        error_covariance_ *= (1 - kalman_gain);

        return estimate_;
    }

    void reset()
    {
        estimate_ = 0.0;
        error_covariance_ = 1.0;
        initialized_ = false;
    }

private:
    double process_variance_;
    double measurement_variance_;
    double estimate_;
    double error_covariance_;
    bool initialized_;
};

class UR5eKinematics
{
public:
    UR5eKinematics() : g_(9.81)
    {
        // UR5e DH parameters
        d1_ = 0.1625;
        a2_ = -0.425;
        a3_ = -0.3922;
        d4_ = 0.1333;
        d5_ = 0.0997;
        d6_ = 0.0996;
    }

    Eigen::Matrix4d computeDHTransform(double theta, double d, double a, double alpha)
    {
        double ct = std::cos(theta);
        double st = std::sin(theta);
        double ca = std::cos(alpha);
        double sa = std::sin(alpha);

        Eigen::Matrix4d T;
        T << ct, -st * ca, st * sa, a * ct,
            st, ct * ca, -ct * sa, a * st,
            0, sa, ca, d,
            0, 0, 0, 1;
        return T;
    }

    std::pair<std::map<std::string, Eigen::Vector3d>, Eigen::Matrix4d>
    forwardKinematics(const std::vector<double> &q)
    {
        Eigen::Matrix4d T01 = computeDHTransform(q[0], d1_, 0, M_PI / 2);
        Eigen::Matrix4d T12 = computeDHTransform(q[1], 0, a2_, 0);
        Eigen::Matrix4d T23 = computeDHTransform(q[2], 0, a3_, 0);
        Eigen::Matrix4d T34 = computeDHTransform(q[3], d4_, 0, M_PI / 2);
        Eigen::Matrix4d T45 = computeDHTransform(q[4], d5_, 0, -M_PI / 2);
        Eigen::Matrix4d T56 = computeDHTransform(q[5], d6_, 0, 0);

        Eigen::Matrix4d T02 = T01 * T12;
        Eigen::Matrix4d T03 = T02 * T23;
        Eigen::Matrix4d T04 = T03 * T34;
        Eigen::Matrix4d T05 = T04 * T45;
        Eigen::Matrix4d T06 = T05 * T56;

        std::map<std::string, Eigen::Vector3d> positions;
        positions["shoulder"] = T01.block<3, 1>(0, 3);
        positions["elbow"] = T02.block<3, 1>(0, 3);
        positions["wrist1"] = T03.block<3, 1>(0, 3);
        positions["end_effector"] = T06.block<3, 1>(0, 3);

        return {positions, T01};
    }

    Eigen::Vector3d computeMomentArms(const std::vector<double> &joint_angles)
    {
        auto [positions, T01] = forwardKinematics(joint_angles);
        Eigen::Vector3d ee_pos = positions["end_effector"];

        Eigen::Vector3d joint2_pos = positions["shoulder"];
        Eigen::Vector3d joint3_pos = positions["elbow"];
        Eigen::Vector3d joint4_pos = positions["wrist1"];

        Eigen::Vector3d joint2_axis(0, 0, 1);
        Eigen::Vector3d joint3_axis = T01.block<3, 3>(0, 0) * joint2_axis;
        Eigen::Vector3d joint4_axis = joint3_axis;

        Eigen::Vector3d vec_to_ee_j2 = ee_pos - joint2_pos;
        Eigen::Vector3d vec_to_ee_j3 = ee_pos - joint3_pos;
        Eigen::Vector3d vec_to_ee_j4 = ee_pos - joint4_pos;

        double moment_j2 = joint2_axis.cross(vec_to_ee_j2).norm();
        double moment_j3 = joint3_axis.cross(vec_to_ee_j3).norm();
        double moment_j4 = joint4_axis.cross(vec_to_ee_j4).norm();

        return Eigen::Vector3d(moment_j2, moment_j3, moment_j4);
    }

    double estimateMass(const Eigen::VectorXd &torque_deltas,
                        const Eigen::VectorXd &moment_arms)
    {
        std::vector<double> mass_estimates;

        for (int i = 0; i < torque_deltas.size(); ++i)
        {
            if (std::abs(moment_arms(i)) > 0.01)
            {
                double mass = torque_deltas(i) / (g_ * moment_arms(i));
                mass_estimates.push_back(mass);
            }
        }

        if (mass_estimates.empty())
            return 0.0;

        // Return median
        std::nth_element(mass_estimates.begin(),
                         mass_estimates.begin() + mass_estimates.size() / 2,
                         mass_estimates.end());
        return mass_estimates[mass_estimates.size() / 2];
    }

    double getGravity() const { return g_; }

private:
    double g_;
    double d1_, a2_, a3_, d4_, d5_, d6_;
};

class WeightDetector : public rclcpp::Node
{
public:
    WeightDetector() : Node("weight_detector_cpp"),
                       calibrating_baseline_(true),
                       calibration_factor_(5.0),
                       estimated_mass_grams_(0.0),
                       messages_received_(0),
                       callback_counter_(0),
                       process_decimation_(5)
    {

        // Parameters
        this->declare_parameter("active_joints", std::vector<int64_t>{1, 2}); // Joints 1 -> 6 (0-indexed), {1,2} = joints 2 and 3
        this->declare_parameter("process_decimation", 10);                    // Process every Nth message (500Hz/10 = 50Hz)
        this->declare_parameter("baseline_sample_size", 220);                 // At decimation=10, ~5.5 seconds at 500Hz

        active_joints_ = this->get_parameter("active_joints").as_integer_array();
        baseline_sample_size_ = this->get_parameter("baseline_sample_size").as_int();
        process_decimation_ = this->get_parameter("process_decimation").as_int();

        // Snapping mode toggle: when true, uses exponential calibration and
        // snaps to weight set (0, 50, 100, 200, 500)
        // When false, uses polynomial calibration and rounds to nearest 5g
        this->declare_parameter("useSnapping", true);
        use_snapping_ = this->get_parameter("useSnapping").as_bool();

        // Exponential calibration parameters
        exp_amplitude_light_ = 15;
        exp_amplitude_heavy_ = 8.45;
        decay_light_ = 5.75;
        decay_heavy_ = 2.95;

        // Thresholds
        mass_threshold_ = 0.3; // kg, used to switch between calibration gains (in both exponential & polynomial)
        mass_threshold_ /= calibration_factor_;
        min_threshold_ = 0.0175; // kg, values below this are ignored
        min_threshold_ /= calibration_factor_;

        // Polynomial calibration parameters
        this->declare_parameter("poly_coeff_a", 0.0);     // cubic term (unused)
        this->declare_parameter("poly_coeff_b", -0.1463); // quadratic term
        this->declare_parameter("poly_coeff_c", 19.55);   // linear term
        this->declare_parameter("poly_coeff_d", -152.7);  // constant term

        poly_coeff_a_ = this->get_parameter("poly_coeff_a").as_double();
        poly_coeff_b_ = this->get_parameter("poly_coeff_b").as_double();
        poly_coeff_c_ = this->get_parameter("poly_coeff_c").as_double();
        poly_coeff_d_ = this->get_parameter("poly_coeff_d").as_double();

        // Weight set for snapping
        weight_set_ = {50, 100, 200, 500};

        // Kalman filters (6 joints)
        for (int i = 0; i < 6; ++i)
        {
            double process_variance = 0.0005;
            double measurement_variance = 15;
            kalman_filters_.emplace_back(process_variance, measurement_variance);
        }

        // QoS for sensor data
        auto qos = rclcpp::SensorDataQoS();
        qos.keep_last(1);

        // Subscription to UR5e Joint State Data
        joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
            "/joint_states",
            qos,
            std::bind(&WeightDetector::jointStateCallback, this, std::placeholders::_1));

        // Publisher of estimated mass
        mass_pub_ = this->create_publisher<std_msgs::msg::Int32>("/estimated_mass", 10);
        calibration_status_pub_ = this->create_publisher<std_msgs::msg::Bool>(
            "/weight_detection/calibration_status", 10);

        // Calibration service call
        calibration_service_ = this->create_service<sort_interfaces::srv::CalibrateBaseline>(
            "/weight_detection/calibrate_baseline",
            std::bind(&WeightDetector::calibrateBaselineCallback, this,
                      std::placeholders::_1, std::placeholders::_2));

        // Diagnostic timer
        diagnostic_timer_ = this->create_wall_timer(
            std::chrono::seconds(5),
            std::bind(&WeightDetector::reportDiagnostics, this));

        // Parameter update timer
        param_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(500),
            std::bind(&WeightDetector::updateParameters, this));

        RCLCPP_INFO(this->get_logger(),
                    "Weight detector C++ node initialized - calibrating baseline...");
        RCLCPP_INFO(this->get_logger(),
                    "Active joints for mass estimation: [%ld, %ld]",
                    active_joints_.size() > 0 ? active_joints_[0] + 1 : 0,
                    active_joints_.size() > 1 ? active_joints_[1] + 1 : 0);
        RCLCPP_INFO(this->get_logger(),
                    "Snapping mode: %s", use_snapping_ ? "ENABLED" : "DISABLED");
    }

private:
    void updateParameters()
    {
        active_joints_ = this->get_parameter("active_joints").as_integer_array();
    }

    void calibrateBaselineCallback(
        const std::shared_ptr<sort_interfaces::srv::CalibrateBaseline::Request> /*request*/,
        std::shared_ptr<sort_interfaces::srv::CalibrateBaseline::Response> response)
    {

        RCLCPP_INFO(this->get_logger(),
                    "Baseline recalibration requested - resetting baseline and Kalman filters...");

        baseline_torques_.clear();
        baseline_samples_.clear();
        calibrating_baseline_ = true;
        messages_received_ = 0;

        // Reset Kalman filters
        for (auto &kf : kalman_filters_)
        {
            kf.reset();
        }

        // Publish calibration status
        auto status_msg = std_msgs::msg::Bool();
        status_msg.data = true;
        calibration_status_pub_->publish(status_msg);

        response->success = true;
        response->message = "Baseline recalibration started.";
        response->wait_time_ms = 26 * baseline_sample_size_;
    }

    void jointStateCallback(const sensor_msgs::msg::JointState::SharedPtr msg)
    {
        // Decimate input frequency: only process every Nth message
        callback_counter_++;
        if (callback_counter_ % process_decimation_ != 0)
        {
            return;
        }

        auto current_time = std::chrono::high_resolution_clock::now();

        // Track timing
        if (last_callback_time_.time_since_epoch().count() > 0)
        {
            auto dt = std::chrono::duration<double>(current_time - last_callback_time_).count();
            if (dt > 0)
            {
                callback_dt_history_.push_back(dt);
                if (callback_dt_history_.size() > 100)
                {
                    callback_dt_history_.pop_front();
                }
            }
        }
        last_callback_time_ = current_time;
        messages_received_++;

        // Check message validity
        if (msg->effort.size() < 6 || msg->position.size() < 6)
        {
            return;
        }

        // Reorder joints (matching Python: [5, 0, 1, 2, 3, 4])
        std::vector<int> reorder = {5, 0, 1, 2, 3, 4};
        std::vector<double> torques(6), positions(6);
        for (int i = 0; i < 6; ++i)
        {
            torques[i] = msg->effort[reorder[i]];
            positions[i] = msg->position[reorder[i]];
        }

        current_joint_angles_ = positions;

        // Filter torques
        std::vector<double> filtered_torques;
        for (int i = 0; i < 6; ++i)
        {
            filtered_torques.push_back(kalman_filters_[i].update(torques[i]));
        }

        if (calibrating_baseline_)
        {
            handleCalibration(filtered_torques);
        }
        else
        {
            estimateMassPhysics(filtered_torques);
        }
    }

    void handleCalibration(const std::vector<double> &filtered_torques)
    {
        std::vector<double> active_torques;
        for (auto idx : active_joints_)
        {
            active_torques.push_back(filtered_torques[idx]);
        }

        baseline_samples_.push_back(active_torques);

        if (static_cast<int>(baseline_samples_.size()) >= baseline_sample_size_)
        {
            // Calculate median baseline for each active joint
            baseline_torques_ = calculateMedian(baseline_samples_);
            calibrating_baseline_ = false;

            auto status_msg = std_msgs::msg::Bool();
            status_msg.data = false;
            calibration_status_pub_->publish(status_msg);

            RCLCPP_INFO(this->get_logger(), "Baseline calibration complete");

            std::string baseline_str = "Baseline torques: [";
            for (size_t i = 0; i < baseline_torques_.size(); ++i)
            {
                baseline_str += std::to_string(baseline_torques_[i]);
                if (i < baseline_torques_.size() - 1)
                    baseline_str += ", ";
            }
            baseline_str += "]";
            RCLCPP_INFO(this->get_logger(), "%s", baseline_str.c_str());

            // Report callback rate
            if (!callback_dt_history_.empty())
            {
                double avg_dt = std::accumulate(callback_dt_history_.begin(),
                                                callback_dt_history_.end(), 0.0) /
                                callback_dt_history_.size();
                RCLCPP_INFO(this->get_logger(),
                            "Actual callback rate: %.1f Hz", 1.0 / avg_dt);
            }
        }
    }

    void estimateMassPhysics(const std::vector<double> &filtered_torques)
    {
        if (baseline_torques_.empty() || current_joint_angles_.empty())
        {
            return;
        }

        Eigen::VectorXd torque_deltas(active_joints_.size());
        for (size_t i = 0; i < active_joints_.size(); ++i)
        {
            torque_deltas(i) = filtered_torques[active_joints_[i]] - baseline_torques_[i];
        }

        Eigen::Vector3d all_moment_arms = kinematics_.computeMomentArms(current_joint_angles_);

        Eigen::VectorXd active_moment_arms(active_joints_.size());
        for (size_t i = 0; i < active_joints_.size(); ++i)
        {
            active_moment_arms(i) = all_moment_arms(active_joints_[i] - 1);
        }

        double raw_mass = std::abs(kinematics_.estimateMass(torque_deltas, active_moment_arms));
        double raw_grams = raw_mass * 1000.0;

        int output_mass = 0;

        RCLCPP_DEBUG_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                              "Raw mass: %.4f grams", raw_grams);

        if (use_snapping_)
        {
            // Exponential calibration mode + snap to weight set (0, 50, 100, 200, 500)
            if (raw_mass < min_threshold_)
            {
                calibration_factor_ = 0.0;
            }
            else if (raw_mass < mass_threshold_)
            {
                calibration_factor_ = exp_amplitude_light_ * std::exp(-decay_light_ * raw_mass);
            }
            else
            {
                calibration_factor_ = exp_amplitude_heavy_ * std::exp(-decay_heavy_ * raw_mass);
            }

            estimated_mass_grams_ = raw_mass * calibration_factor_ * 1000.0;
            estimated_mass_grams_ = std::round(estimated_mass_grams_ / 5.0) * 5.0;

            output_mass = snapToWeightSet(estimated_mass_grams_);
        }
        else
        {
            // Polynomial calibration mode + round to nearest 5g (no snapping)
            double calibrated_grams = 0.0;
            double min_threshold_grams_ = min_threshold_ * 1000.0;

            if (raw_grams >= min_threshold_grams_)
            {
                if (raw_grams > 80.0)
                {
                    // Linear extrapolation for high values (during transients)
                    // Use gradient from 500g region
                    calibrated_grams = 500.0 + (raw_grams - 68.4) * 4.0;
                }
                else
                {
                    calibrated_grams = poly_coeff_b_ * raw_grams * raw_grams + poly_coeff_c_ * raw_grams + poly_coeff_d_;
                }
            }

            estimated_mass_grams_ = std::max(0.0, calibrated_grams);
            estimated_mass_grams_ = std::round(estimated_mass_grams_ / 5.0) * 5.0;

            output_mass = static_cast<int>(estimated_mass_grams_);
        }

        auto mass_msg = std_msgs::msg::Int32();
        mass_msg.data = output_mass;
        mass_pub_->publish(mass_msg);
    }

    int snapToWeightSet(double estimated_mass)
    {
        // Range-based snapping
        if (estimated_mass <= 17.5)
        {
            return 0;
        }
        else if (estimated_mass <= 45)
        {
            return 50;
        }
        else if (estimated_mass <= 185)
        {
            return 100;
        }
        else if (estimated_mass <= 375)
        {
            return 200;
        }
        else
        {
            return 500;
        }
    }

    std::vector<double> calculateMedian(const std::vector<std::vector<double>> &samples)
    {
        if (samples.empty())
            return {};

        size_t num_values = samples[0].size();
        std::vector<double> medians(num_values);

        for (size_t j = 0; j < num_values; ++j)
        {
            std::vector<double> column;
            for (const auto &sample : samples)
            {
                if (j < sample.size())
                {
                    column.push_back(sample[j]);
                }
            }

            if (!column.empty())
            {
                std::nth_element(column.begin(),
                                 column.begin() + column.size() / 2,
                                 column.end());
                medians[j] = column[column.size() / 2];
            }
        }

        return medians;
    }

    void reportDiagnostics()
    {
        if (!callback_dt_history_.empty() && messages_received_ > 0)
        {
            double avg_dt = std::accumulate(callback_dt_history_.begin(),
                                            callback_dt_history_.end(), 0.0) /
                            callback_dt_history_.size();
            RCLCPP_DEBUG(this->get_logger(),
                         "Diagnostics: %zu messages, callback rate: %.1f Hz",
                         messages_received_, 1.0 / avg_dt);
        }
    }

    // ROS interfaces
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_state_sub_;
    rclcpp::Publisher<std_msgs::msg::Int32>::SharedPtr mass_pub_;
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr calibration_status_pub_;
    rclcpp::Service<sort_interfaces::srv::CalibrateBaseline>::SharedPtr calibration_service_;
    rclcpp::TimerBase::SharedPtr diagnostic_timer_;
    rclcpp::TimerBase::SharedPtr param_timer_;

    // Kalman filters and kinematics
    std::vector<KalmanFilter> kalman_filters_;
    UR5eKinematics kinematics_;

    // Timing
    std::deque<double> callback_dt_history_;
    std::chrono::high_resolution_clock::time_point last_callback_time_;

    // State
    bool calibrating_baseline_;
    std::vector<std::vector<double>> baseline_samples_;
    std::vector<double> baseline_torques_;
    std::vector<double> current_joint_angles_;
    std::vector<int64_t> active_joints_;

    // Calibration parameters
    bool use_snapping_; // When false, uses polynomial calibration (equivalent to deprecated use_polynomial_calibration=true)
    double calibration_factor_;

    // Exponential calibration
    double exp_amplitude_light_;
    double exp_amplitude_heavy_;
    double decay_light_;
    double decay_heavy_;
    double mass_threshold_;
    double min_threshold_;

    // Polynomial calibration
    double poly_coeff_a_;
    double poly_coeff_b_;
    double poly_coeff_c_;
    double poly_coeff_d_;

    int baseline_sample_size_;

    double estimated_mass_grams_;
    size_t messages_received_;

    std::vector<int> weight_set_;

    // Input decimation
    size_t callback_counter_;
    int process_decimation_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<WeightDetector>());
    rclcpp::shutdown();
    return 0;
}
