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
#include <pcl/common/common.h>

#include <vector>
#include <string>
#include <map>
#include <set>
#include <cmath>
#include <limits>

// Struct to store tracked object information
struct TrackedObject
{
  uint32_t id;
  float weight;
  Eigen::Vector3f position;
  rclcpp::Time last_seen;
  int consecutive_misses;
};

// Struct for detected objects in current frame
struct DetectedObject
{
  Eigen::Vector3f position;
  float weight;
  float confidence;
  geometry_msgs::msg::Pose pose;
  float volume;
};

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

    // Object tracking parameters
    this->declare_parameter("tracking_distance_threshold", 0.05);  // 5cm
    this->declare_parameter("tracking_weight_threshold", 20.0);    // 20g
    this->declare_parameter("tracking_timeout", 5.0);              // 5 seconds
    this->declare_parameter("tracking_max_misses", 3);             // frames

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

    // Get tracking parameters
    tracking_distance_threshold_ = this->get_parameter("tracking_distance_threshold").as_double();
    tracking_weight_threshold_ = this->get_parameter("tracking_weight_threshold").as_double();
    tracking_timeout_ = this->get_parameter("tracking_timeout").as_double();
    tracking_max_misses_ = this->get_parameter("tracking_max_misses").as_int();

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
    RCLCPP_INFO(this->get_logger(), "Object tracking: distance=%.2fcm, weight=%.1fg",
                tracking_distance_threshold_ * 100.0, tracking_weight_threshold_);
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

    if (cluster_indices.empty()) {
      RCLCPP_DEBUG(this->get_logger(), "No objects detected");
      // Increment miss counter for all tracked objects
      update_tracked_objects_misses();
      cleanup_stale_objects();
      return;
    }

    RCLCPP_INFO(this->get_logger(), "Detected %zu object(s)", cluster_indices.size());

    // 4. Process each cluster with tracking
    process_clusters_with_tracking(cloud_downsampled, cluster_indices, msg->header);

    // 5. Clean up stale tracked objects
    cleanup_stale_objects();
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

    // Calculate bounding box (dimensions)
    pcl::PointXYZRGB min_pt, max_pt;
    pcl::getMinMax3D(*cluster, min_pt, max_pt);

    double width = max_pt.x - min_pt.x;   // X dimension
    double height = max_pt.y - min_pt.y;  // Y dimension
    double depth = max_pt.z - min_pt.z;   // Z dimension (height above table)

    // Calculate volume using convex hull
    double volume = calculate_volume(cluster);

    if (volume <= 0.0) {
      RCLCPP_WARN(this->get_logger(), "Invalid volume, skipping object");
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
    weight_msg.volume = volume * 1e6;  // Convert m³ to cm³

    // Set position (centroid)
    weight_msg.pose.position.x = centroid[0];
    weight_msg.pose.position.y = centroid[1];
    weight_msg.pose.position.z = centroid[2];

    // Default orientation (upright)
    weight_msg.pose.orientation.x = 0.0;
    weight_msg.pose.orientation.y = 0.0;
    weight_msg.pose.orientation.z = 0.0;
    weight_msg.pose.orientation.w = 1.0;

    // Publish to sort_node
    weight_pub_->publish(weight_msg);

    // Simple, concise log output
    RCLCPP_INFO(this->get_logger(),
                "  Object %u: %.1fg, %.1f×%.1f×%.1fcm @ (%.2f, %.2f, %.2f)",
                weight_msg.object_id,
                weight_grams,
                width * 100.0, height * 100.0, depth * 100.0,
                centroid[0], centroid[1], centroid[2]);
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

  // Object tracking functions
  void process_clusters_with_tracking(
    const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const std::vector<pcl::PointIndices>& cluster_indices,
    const std_msgs::msg::Header& header)
  {
    // Store detected objects this frame (position, weight, cluster data)
    std::vector<DetectedObject> detected_objects;

    // First pass: Extract all cluster information
    for (const auto& indices : cluster_indices) {
      // Extract cluster points
      pcl::PointCloud<pcl::PointXYZRGB>::Ptr cluster(new pcl::PointCloud<pcl::PointXYZRGB>);
      for (const auto& idx : indices.indices) {
        cluster->points.push_back(cloud->points[idx]);
      }
      cluster->width = cluster->points.size();
      cluster->height = 1;
      cluster->is_dense = true;

      // Calculate centroid
      Eigen::Vector4f centroid;
      pcl::compute3DCentroid(*cluster, centroid);

      // Calculate volume and weight
      double volume = calculate_volume(cluster);
      if (volume <= 0.0) {
        continue;
      }

      double weight_kg = volume * material_density_;
      double weight_grams = weight_kg * 1000.0;

      // Calculate confidence
      double confidence = calculate_confidence(cluster->points.size());
      if (confidence < confidence_threshold_) {
        continue;
      }

      // Store detected object
      DetectedObject obj;
      obj.position = Eigen::Vector3f(centroid[0], centroid[1], centroid[2]);
      obj.weight = weight_grams;
      obj.confidence = confidence;
      obj.volume = volume * 1e6;  // m³ to cm³

      // Create pose
      obj.pose.position.x = centroid[0];
      obj.pose.position.y = centroid[1];
      obj.pose.position.z = centroid[2];
      obj.pose.orientation.x = 0.0;
      obj.pose.orientation.y = 0.0;
      obj.pose.orientation.z = 0.0;
      obj.pose.orientation.w = 1.0;

      detected_objects.push_back(obj);
    }

    // Second pass: Match to tracked objects
    std::set<uint32_t> matched_ids;
    std::vector<bool> detection_matched(detected_objects.size(), false);

    for (size_t i = 0; i < detected_objects.size(); ++i) {
      const auto& detected = detected_objects[i];

      // Find best matching tracked object
      uint32_t best_match_id = find_matching_object(detected.position, detected.weight, matched_ids);

      if (best_match_id != UINT32_MAX) {
        // Matched to existing object - update it
        auto& tracked = tracked_objects_[best_match_id];
        tracked.position = detected.position;
        tracked.weight = detected.weight;
        tracked.last_seen = this->now();
        tracked.consecutive_misses = 0;
        matched_ids.insert(best_match_id);
        detection_matched[i] = true;

        // Publish with existing ID
        publish_weight_estimate(best_match_id, detected, header);

        RCLCPP_DEBUG(this->get_logger(), "Matched object %u (updated)", best_match_id);
      }
    }

    // Third pass: Create new tracked objects for unmatched detections
    for (size_t i = 0; i < detected_objects.size(); ++i) {
      if (!detection_matched[i]) {
        const auto& detected = detected_objects[i];

        // Create new tracked object
        uint32_t new_id = next_object_id_++;
        TrackedObject new_tracked;
        new_tracked.id = new_id;
        new_tracked.position = detected.position;
        new_tracked.weight = detected.weight;
        new_tracked.last_seen = this->now();
        new_tracked.consecutive_misses = 0;

        tracked_objects_[new_id] = new_tracked;

        // Publish with new ID
        publish_weight_estimate(new_id, detected, header);

        RCLCPP_INFO(this->get_logger(), "New object %u detected", new_id);
      }
    }

    // Fourth pass: Increment miss counter for unmatched tracked objects
    for (auto& [id, tracked] : tracked_objects_) {
      if (matched_ids.find(id) == matched_ids.end()) {
        tracked.consecutive_misses++;
      }
    }
  }

  uint32_t find_matching_object(const Eigen::Vector3f& position, float weight,
                                  const std::set<uint32_t>& already_matched)
  {
    uint32_t best_match = UINT32_MAX;
    double best_score = std::numeric_limits<double>::max();

    for (const auto& [id, tracked] : tracked_objects_) {
      // Skip already matched objects
      if (already_matched.find(id) != already_matched.end()) {
        continue;
      }

      // Calculate distance
      float distance = (position - tracked.position).norm();

      // Calculate weight difference
      float weight_diff = std::abs(weight - tracked.weight);

      // Check thresholds
      if (distance > tracking_distance_threshold_) {
        continue;
      }
      if (weight_diff > tracking_weight_threshold_) {
        continue;
      }

      // Combined score (weighted sum of normalized distance and weight difference)
      double score = distance / tracking_distance_threshold_ +
                     weight_diff / tracking_weight_threshold_;

      if (score < best_score) {
        best_score = score;
        best_match = id;
      }
    }

    return best_match;
  }

  void publish_weight_estimate(uint32_t object_id,
                                const DetectedObject& detected,
                                const std_msgs::msg::Header& header)
  {
    auto weight_msg = sort_interfaces::msg::WeightEstimate();
    weight_msg.header = header;
    weight_msg.header.stamp = this->now();
    weight_msg.object_id = object_id;
    weight_msg.estimated_weight = detected.weight;
    weight_msg.confidence = detected.confidence;
    weight_msg.volume = detected.volume;
    weight_msg.pose = detected.pose;

    weight_pub_->publish(weight_msg);

    RCLCPP_INFO(this->get_logger(),
                "  Object %u: %.1fg @ (%.2f, %.2f, %.2f)",
                object_id, detected.weight,
                detected.position[0], detected.position[1], detected.position[2]);
  }

  void update_tracked_objects_misses()
  {
    for (auto& [id, tracked] : tracked_objects_) {
      tracked.consecutive_misses++;
    }
  }

  void cleanup_stale_objects()
  {
    auto now = this->now();
    std::vector<uint32_t> to_remove;

    for (const auto& [id, tracked] : tracked_objects_) {
      // Check timeout
      double time_since_seen = (now - tracked.last_seen).seconds();

      if (time_since_seen > tracking_timeout_ ||
          tracked.consecutive_misses > tracking_max_misses_) {
        to_remove.push_back(id);
        RCLCPP_INFO(this->get_logger(),
                   "Removing stale object %u (%.1fs since last seen, %d misses)",
                   id, time_since_seen, tracked.consecutive_misses);
      }
    }

    for (uint32_t id : to_remove) {
      tracked_objects_.erase(id);
    }
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

  // Tracking parameters
  double tracking_distance_threshold_;
  double tracking_weight_threshold_;
  double tracking_timeout_;
  int tracking_max_misses_;

  // State
  uint32_t next_object_id_;
  std::map<uint32_t, TrackedObject> tracked_objects_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<RecognitionNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
