/**
 * @file recognition_node.cpp
 * @brief Recognition node for weight estimation from point cloud
 *
 * Uses Point Cloud Library (PCL) to:
 * 1. Segment point cloud into individual objects
 * 2. Estimate volume using convex hull
 * 3. Calculate weight from volume (stainless steel density)
 * 4. Extract object positions
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>
#include <sort_interfaces/msg/weight_estimate.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/surface/convex_hull.h>
#include <pcl/common/centroid.h>

#include <vector>
#include <string>

class RecognitionNode : public rclcpp::Node
{
public:
  RecognitionNode() : Node("recognition_node")
  {
    // Declare parameters
    this->declare_parameter("material_density", 8000.0);  // kg/m³ (stainless steel)
    this->declare_parameter("min_cluster_size", 100);
    this->declare_parameter("max_cluster_size", 10000);
    this->declare_parameter("cluster_tolerance", 0.02);   // 2cm
    this->declare_parameter("voxel_size", 0.005);         // 5mm
    this->declare_parameter("workspace_min_x", -0.6);
    this->declare_parameter("workspace_max_x", 0.6);
    this->declare_parameter("workspace_min_y", -0.6);
    this->declare_parameter("workspace_max_y", 0.6);
    this->declare_parameter("workspace_min_z", 0.0);
    this->declare_parameter("workspace_max_z", 0.6);
    this->declare_parameter("confidence_threshold", 0.5);

    // Get parameters
    material_density_ = this->get_parameter("material_density").as_double();
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();
    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    voxel_size_ = this->get_parameter("voxel_size").as_double();
    workspace_min_x_ = this->get_parameter("workspace_min_x").as_double();
    workspace_max_x_ = this->get_parameter("workspace_max_x").as_double();
    workspace_min_y_ = this->get_parameter("workspace_min_y").as_double();
    workspace_max_y_ = this->get_parameter("workspace_max_y").as_double();
    workspace_min_z_ = this->get_parameter("workspace_min_z").as_double();
    workspace_max_z_ = this->get_parameter("workspace_max_z").as_double();
    confidence_threshold_ = this->get_parameter("confidence_threshold").as_double();

    // Subscriber to point cloud
    pointcloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      "/camera/pointcloud",
      10,
      std::bind(&RecognitionNode::pointcloud_callback, this, std::placeholders::_1)
    );

    // Publisher for weight estimates
    weight_pub_ = this->create_publisher<sort_interfaces::msg::WeightEstimate>(
      "/recognition/estimated_weights",
      10
    );

    // Initialize object ID counter
    next_object_id_ = 0;

    RCLCPP_INFO(this->get_logger(), "Recognition node initialized");
    RCLCPP_INFO(this->get_logger(), "Material density: %.2f kg/m³", material_density_);
    RCLCPP_INFO(this->get_logger(), "Cluster size: [%d, %d]", min_cluster_size_, max_cluster_size_);
    RCLCPP_INFO(this->get_logger(), "Cluster tolerance: %.3f m", cluster_tolerance_);
  }

private:
  void pointcloud_callback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    RCLCPP_DEBUG(this->get_logger(), "Received point cloud with %d points",
                 msg->width * msg->height);

    // Convert ROS message to PCL point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromROSMsg(*msg, *cloud);

    if (cloud->empty()) {
      RCLCPP_WARN(this->get_logger(), "Received empty point cloud");
      return;
    }

    // 1. Filter workspace (PassThrough filter)
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered(new pcl::PointCloud<pcl::PointXYZRGB>);
    filter_workspace(cloud, cloud_filtered);

    if (cloud_filtered->empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No points in workspace after filtering");
      return;
    }

    // 2. Downsample using voxel grid
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_downsampled(new pcl::PointCloud<pcl::PointXYZRGB>);
    downsample_cloud(cloud_filtered, cloud_downsampled);

    if (cloud_downsampled->empty()) {
      RCLCPP_WARN(this->get_logger(), "Empty cloud after downsampling");
      return;
    }

    // 3. Cluster extraction (Euclidean clustering)
    std::vector<pcl::PointIndices> cluster_indices;
    extract_clusters(cloud_downsampled, cluster_indices);

    RCLCPP_INFO(this->get_logger(), "Found %zu object clusters", cluster_indices.size());

    // 4. Process each cluster
    for (const auto& indices : cluster_indices) {
      process_cluster(cloud_downsampled, indices, msg->header);
    }
  }

  void filter_workspace(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
  {
    pcl::PassThrough<pcl::PointXYZRGB> pass;

    // Filter X
    pass.setInputCloud(cloud_in);
    pass.setFilterFieldName("x");
    pass.setFilterLimits(workspace_min_x_, workspace_max_x_);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_x(new pcl::PointCloud<pcl::PointXYZRGB>);
    pass.filter(*cloud_x);

    // Filter Y
    pass.setInputCloud(cloud_x);
    pass.setFilterFieldName("y");
    pass.setFilterLimits(workspace_min_y_, workspace_max_y_);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_xy(new pcl::PointCloud<pcl::PointXYZRGB>);
    pass.filter(*cloud_xy);

    // Filter Z
    pass.setInputCloud(cloud_xy);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(workspace_min_z_, workspace_max_z_);
    pass.filter(*cloud_out);
  }

  void downsample_cloud(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_in,
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud_out)
  {
    pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
    voxel_grid.setInputCloud(cloud_in);
    voxel_grid.setLeafSize(voxel_size_, voxel_size_, voxel_size_);
    voxel_grid.filter(*cloud_out);
  }

  void extract_clusters(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    std::vector<pcl::PointIndices>& cluster_indices)
  {
    // Convert to PointXYZ for clustering (RGB not needed)
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cloud, *cloud_xyz);

    // KD-tree for neighbor search
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud(cloud_xyz);

    // Euclidean clustering
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud_xyz);
    ec.extract(cluster_indices);
  }

  void process_cluster(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const pcl::PointIndices& indices,
    const std_msgs::msg::Header& header)
  {
    // Extract cluster points
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
    for (const auto& idx : indices.indices) {
      cluster->points.push_back(cloud->points[idx]);
    }
    cluster->width = cluster->points.size();
    cluster->height = 1;
    cluster->is_dense = true;

    // Calculate centroid (position)
    Eigen::Vector4f centroid;
    pcl::compute3DCentroid(*cluster, centroid);

    // Calculate volume using convex hull
    double volume = calculate_volume(cluster);

    if (volume <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "Invalid volume calculated, skipping cluster");
      return;
    }

    // Calculate weight (kg)
    // density in kg/m³, volume in m³
    double weight_kg = volume * material_density_;
    double weight_grams = weight_kg * 1000.0;  // Convert to grams

    // Calculate confidence (based on cluster size)
    double confidence = calculate_confidence(cluster->points.size());

    if (confidence < confidence_threshold_) {
      RCLCPP_DEBUG(this->get_logger(),
                   "Low confidence (%.2f) for cluster, skipping", confidence);
      return;
    }

    // Create and publish WeightEstimate message
    auto weight_msg = sort_interfaces::msg::WeightEstimate();
    weight_msg.header = header;
    weight_msg.header.stamp = this->now();
    weight_msg.object_id = next_object_id_++;
    weight_msg.estimated_weight = weight_grams;
    weight_msg.confidence = confidence;
    weight_msg.volume = volume;

    // Set position (centroid)
    weight_msg.pose.position.x = centroid[0];
    weight_msg.pose.position.y = centroid[1];
    weight_msg.pose.position.z = centroid[2];

    // Default orientation (upright)
    weight_msg.pose.orientation.x = 0.0;
    weight_msg.pose.orientation.y = 0.0;
    weight_msg.pose.orientation.z = 0.0;
    weight_msg.pose.orientation.w = 1.0;

    // Publish
    weight_pub_->publish(weight_msg);

    RCLCPP_INFO(this->get_logger(),
                "Object %d: weight=%.2fg, volume=%.6fm³, pos=(%.3f, %.3f, %.3f), confidence=%.2f",
                weight_msg.object_id,
                weight_grams,
                volume,
                centroid[0], centroid[1], centroid[2],
                confidence);
  }

  double calculate_volume(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cluster)
  {
    // Use convex hull for volume estimation
    pcl::PointCloud<pcl::PointXYZ>::Ptr cluster_xyz(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::copyPointCloud(*cluster, *cluster_xyz);

    pcl::ConvexHull<pcl::PointXYZ> convex_hull;
    convex_hull.setInputCloud(cluster_xyz);
    convex_hull.setComputeAreaVolume(true);

    pcl::PointCloud<pcl::PointXYZ> hull_points;
    convex_hull.reconstruct(hull_points);

    double volume = convex_hull.getTotalVolume();

    return volume;  // Returns volume in m³
  }

  double calculate_confidence(size_t num_points)
  {
    // Simple confidence model based on number of points
    // More points = higher confidence
    // This is a heuristic and can be refined

    if (num_points < min_cluster_size_) {
      return 0.0;
    }

    // Linear interpolation between min and max cluster size
    double normalized = static_cast<double>(num_points - min_cluster_size_) /
                       static_cast<double>(max_cluster_size_ - min_cluster_size_);

    // Clamp between 0.5 and 1.0
    double confidence = 0.5 + (normalized * 0.5);
    return std::min(1.0, std::max(0.5, confidence));
  }

  // ROS2 components
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_sub_;
  rclcpp::Publisher<sort_interfaces::msg::WeightEstimate>::SharedPtr weight_pub_;

  // Parameters
  double material_density_;
  int min_cluster_size_;
  int max_cluster_size_;
  double cluster_tolerance_;
  double voxel_size_;
  double workspace_min_x_, workspace_max_x_;
  double workspace_min_y_, workspace_max_y_;
  double workspace_min_z_, workspace_max_z_;
  double confidence_threshold_;

  // State
  uint32_t next_object_id_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RecognitionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
