/**
 * @file weight_estimation_node.cpp
 * @brief Weight estimation from volume and visual analysis
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include "sort_interfaces/msg/weight_estimate.hpp"
#include <vector>
#include <cmath>

class WeightEstimationNode : public rclcpp::Node
{
public:
    WeightEstimationNode() : Node("weight_estimation_node"), object_id_counter_(0)
    {
        // Declare parameters
        this->declare_parameter("default_density", 1.0);  // g/cm³
        this->declare_parameter("min_confidence", 0.3);
        this->declare_parameter("max_confidence", 0.9);
        this->declare_parameter("use_ml_model", false);

        // Material density presets (g/cm³)
        this->declare_parameter("density_plastic", 0.95);
        this->declare_parameter("density_wood", 0.6);
        this->declare_parameter("density_metal", 7.8);

        // Get parameters
        default_density_ = this->get_parameter("default_density").as_double();
        min_confidence_ = this->get_parameter("min_confidence").as_double();
        max_confidence_ = this->get_parameter("max_confidence").as_double();
        use_ml_model_ = this->get_parameter("use_ml_model").as_bool();

        density_map_["plastic"] = this->get_parameter("density_plastic").as_double();
        density_map_["wood"] = this->get_parameter("density_wood").as_double();
        density_map_["metal"] = this->get_parameter("density_metal").as_double();

        // Subscribers
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/pointcloud", 10,
            std::bind(&WeightEstimationNode::pointcloud_callback, this,
                     std::placeholders::_1));

        object_positions_sub_ = this->create_subscription<geometry_msgs::msg::PoseArray>(
            "/perception/object_positions", 10,
            std::bind(&WeightEstimationNode::object_positions_callback, this,
                     std::placeholders::_1));

        // Publisher
        weight_estimates_pub_ = this->create_publisher<sort_interfaces::msg::WeightEstimate>(
            "/recognition/estimated_weights", 10);

        RCLCPP_INFO(this->get_logger(), "Weight Estimation Node initialized");
        RCLCPP_INFO(this->get_logger(), "Default density: %.2f g/cm³", default_density_);

        if (use_ml_model_) {
            RCLCPP_WARN(this->get_logger(),
                       "TODO: Load ML model for weight estimation");
        }
    }

private:
    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_pointcloud_ = msg;
    }

    void object_positions_callback(const geometry_msgs::msg::PoseArray::SharedPtr msg)
    {
        if (!latest_pointcloud_) {
            RCLCPP_WARN(this->get_logger(), "No point cloud data available yet");
            return;
        }

        // Process each detected object
        for (const auto& pose : msg->poses) {
            auto weight_estimate = estimate_weight(pose, msg->header);

            if (weight_estimate) {
                weight_estimates_pub_->publish(*weight_estimate);

                RCLCPP_INFO(this->get_logger(),
                           "Object %u: Estimated weight = %.1f g (confidence: %.2f)",
                           weight_estimate->object_id,
                           weight_estimate->estimated_weight,
                           weight_estimate->confidence);
            }
        }
    }

    std::shared_ptr<sort_interfaces::msg::WeightEstimate>
    estimate_weight(const geometry_msgs::msg::Pose& pose, const std_msgs::msg::Header& header)
    {
        auto estimate = std::make_shared<sort_interfaces::msg::WeightEstimate>();
        estimate->header = header;
        estimate->object_id = object_id_counter_++;
        estimate->pose = pose;

        // TODO: Implement actual volume calculation from point cloud segmentation
        // For now, use placeholder volume estimation
        float volume = estimate_volume_from_pointcloud(pose);
        estimate->volume = volume;

        // TODO: Implement material classification (color analysis, ML model, etc.)
        std::string material = classify_material(pose);
        double density = get_density_for_material(material);

        // Calculate weight: weight = volume * density
        estimate->estimated_weight = volume * density;

        // Calculate confidence based on point cloud quality and material classification
        estimate->confidence = calculate_confidence(volume, material);

        return estimate;
    }

    float estimate_volume_from_pointcloud(const geometry_msgs::msg::Pose& pose)
    {
        // TODO: Extract actual object point cloud segment and calculate volume
        // Methods:
        // 1. Convex hull volume
        // 2. Voxel grid counting
        // 3. Bounding box approximation
        // 4. Alpha shapes

        // Placeholder: assume objects are roughly cubic with 5cm sides
        float side_length = 5.0f;  // cm
        float volume = std::pow(side_length, 3);  // cubic cm

        // Add some random variation for testing
        float variation = (std::rand() % 20 - 10) / 10.0f;  // ±10%
        volume *= (1.0f + variation);

        return volume;
    }

    std::string classify_material(const geometry_msgs::msg::Pose& pose)
    {
        // TODO: Implement material classification
        // Approaches:
        // 1. Color analysis (from RGB data)
        // 2. Surface properties (from point cloud normals)
        // 3. Deep learning model
        // 4. Thermal imaging (if available)

        if (use_ml_model_) {
            // TODO: Run ML inference
            RCLCPP_DEBUG(this->get_logger(), "TODO: ML-based material classification");
        }

        // Placeholder: randomly assign material for testing
        std::vector<std::string> materials = {"plastic", "wood", "metal"};
        int idx = std::rand() % materials.size();
        return materials[idx];
    }

    double get_density_for_material(const std::string& material)
    {
        auto it = density_map_.find(material);
        if (it != density_map_.end()) {
            return it->second;
        }
        return default_density_;
    }

    float calculate_confidence(float volume, const std::string& material)
    {
        // Confidence factors:
        // - Point cloud quality (completeness, noise level)
        // - Material classification certainty
        // - Object size (larger objects = better estimates)

        float confidence = 0.5f;  // Base confidence

        // Adjust based on volume (larger objects = higher confidence)
        if (volume > 100.0f) {
            confidence += 0.2f;
        } else if (volume < 10.0f) {
            confidence -= 0.2f;
        }

        // TODO: Adjust based on actual point cloud quality metrics

        // Clamp to valid range
        confidence = std::max(static_cast<float>(min_confidence_),
                            std::min(static_cast<float>(max_confidence_), confidence));

        return confidence;
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseArray>::SharedPtr object_positions_sub_;
    rclcpp::Publisher<sort_interfaces::msg::WeightEstimate>::SharedPtr weight_estimates_pub_;

    sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;

    double default_density_;
    double min_confidence_;
    double max_confidence_;
    bool use_ml_model_;

    std::map<std::string, double> density_map_;
    uint32_t object_id_counter_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<WeightEstimationNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
