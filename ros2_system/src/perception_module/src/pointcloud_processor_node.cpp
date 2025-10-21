/**
 * @file pointcloud_processor_node.cpp
 * @brief 3D segmentation and position extraction from point cloud
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <geometry_msgs/msg/pose_array.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include "sort_interfaces/msg/detected_objects.hpp"
#include <vector>
#include <memory>

class PointCloudProcessorNode : public rclcpp::Node
{
public:
    PointCloudProcessorNode() : Node("pointcloud_processor_node")
    {
        // Declare parameters
        this->declare_parameter("min_cluster_size", 100);
        this->declare_parameter("max_cluster_size", 10000);
        this->declare_parameter("cluster_tolerance", 0.02);
        this->declare_parameter("workspace_min_x", -0.5);
        this->declare_parameter("workspace_max_x", 0.5);
        this->declare_parameter("workspace_min_y", -0.5);
        this->declare_parameter("workspace_max_y", 0.5);
        this->declare_parameter("workspace_min_z", 0.0);
        this->declare_parameter("workspace_max_z", 0.5);

        // Get parameters
        min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
        max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
        cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();

        workspace_limits_.min_x = this->get_parameter("workspace_min_x").as_double();
        workspace_limits_.max_x = this->get_parameter("workspace_max_x").as_double();
        workspace_limits_.min_y = this->get_parameter("workspace_min_y").as_double();
        workspace_limits_.max_y = this->get_parameter("workspace_max_y").as_double();
        workspace_limits_.min_z = this->get_parameter("workspace_min_z").as_double();
        workspace_limits_.max_z = this->get_parameter("workspace_max_z").as_double();

        // Subscribers
        pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
            "/camera/pointcloud", 10,
            std::bind(&PointCloudProcessorNode::pointcloud_callback, this,
                     std::placeholders::_1));

        detected_objects_sub_ = this->create_subscription<sort_interfaces::msg::DetectedObjects>(
            "/perception/detected_objects", 10,
            std::bind(&PointCloudProcessorNode::detected_objects_callback, this,
                     std::placeholders::_1));

        // Publishers
        object_positions_pub_ = this->create_publisher<geometry_msgs::msg::PoseArray>(
            "/perception/object_positions", 10);

        markers_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
            "/perception/object_markers", 10);

        RCLCPP_INFO(this->get_logger(), "PointCloud Processor Node initialized");
        RCLCPP_INFO(this->get_logger(), "Workspace: [%.2f,%.2f] x [%.2f,%.2f] x [%.2f,%.2f]",
                   workspace_limits_.min_x, workspace_limits_.max_x,
                   workspace_limits_.min_y, workspace_limits_.max_y,
                   workspace_limits_.min_z, workspace_limits_.max_z);
    }

private:
    struct WorkspaceLimits {
        double min_x, max_x;
        double min_y, max_y;
        double min_z, max_z;
    };

    void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
    {
        latest_pointcloud_ = msg;

        // If we have detected objects, process them
        if (latest_detected_objects_) {
            process_objects();
        }
    }

    void detected_objects_callback(const sort_interfaces::msg::DetectedObjects::SharedPtr msg)
    {
        latest_detected_objects_ = msg;

        // If we have a point cloud, process objects
        if (latest_pointcloud_) {
            process_objects();
        }
    }

    void process_objects()
    {
        if (!latest_pointcloud_ || !latest_detected_objects_) {
            return;
        }

        // TODO: Implement actual 3D segmentation and position extraction
        // This requires PCL (Point Cloud Library) for:
        // - Passthrough filtering (crop to workspace)
        // - Statistical outlier removal
        // - Euclidean clustering
        // - Centroid calculation
        // - Integration with 2D bounding boxes

        geometry_msgs::msg::PoseArray pose_array;
        pose_array.header = latest_pointcloud_->header;

        visualization_msgs::msg::MarkerArray marker_array;

        // Placeholder: Create dummy positions for each detected object
        for (size_t i = 0; i < latest_detected_objects_->objects.size(); ++i) {
            const auto& obj = latest_detected_objects_->objects[i];

            // TODO: Extract actual 3D position from point cloud using bounding box
            geometry_msgs::msg::Pose pose;
            pose.position.x = 0.4;  // Dummy position
            pose.position.y = 0.0;
            pose.position.z = 0.1;
            pose.orientation.w = 1.0;

            pose_array.poses.push_back(pose);

            // Create visualization marker
            visualization_msgs::msg::Marker marker;
            marker.header = pose_array.header;
            marker.ns = "detected_objects";
            marker.id = obj.id;
            marker.type = visualization_msgs::msg::Marker::CUBE;
            marker.action = visualization_msgs::msg::Marker::ADD;
            marker.pose = pose;
            marker.scale.x = 0.05;
            marker.scale.y = 0.05;
            marker.scale.z = 0.05;
            marker.color.r = 0.0f;
            marker.color.g = 1.0f;
            marker.color.b = 0.0f;
            marker.color.a = 0.8f;
            marker.lifetime = rclcpp::Duration::from_seconds(1.0);

            marker_array.markers.push_back(marker);
        }

        // Publish results
        if (!pose_array.poses.empty()) {
            object_positions_pub_->publish(pose_array);
            markers_pub_->publish(marker_array);

            RCLCPP_INFO(this->get_logger(), "Published %zu object positions",
                       pose_array.poses.size());
        }
    }

    bool is_in_workspace(double x, double y, double z) const
    {
        return (x >= workspace_limits_.min_x && x <= workspace_limits_.max_x &&
                y >= workspace_limits_.min_y && y <= workspace_limits_.max_y &&
                z >= workspace_limits_.min_z && z <= workspace_limits_.max_z);
    }

    // Member variables
    rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
    rclcpp::Subscription<sort_interfaces::msg::DetectedObjects>::SharedPtr detected_objects_sub_;

    rclcpp::Publisher<geometry_msgs::msg::PoseArray>::SharedPtr object_positions_pub_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr markers_pub_;

    sensor_msgs::msg::PointCloud2::SharedPtr latest_pointcloud_;
    sort_interfaces::msg::DetectedObjects::SharedPtr latest_detected_objects_;

    int min_cluster_size_;
    int max_cluster_size_;
    double cluster_tolerance_;
    WorkspaceLimits workspace_limits_;
};

int main(int argc, char** argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<PointCloudProcessorNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
