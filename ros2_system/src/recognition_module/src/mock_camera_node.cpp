/**
 * @file mock_camera_node.cpp
 * @brief Mock RGBD camera node for testing recognition module
 *
 * Simulates Lenovo 510 RGBD webcam by generating synthetic point clouds
 * with objects of known sizes and positions.
 */

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/point_cloud2.hpp>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>

#include <random>
#include <vector>

class MockCameraNode : public rclcpp::Node
{
public:
  MockCameraNode() : Node("mock_camera_node")
  {
    // Declare parameters
    this->declare_parameter("publish_rate", 1.0);  // Hz
    this->declare_parameter("frame_id", "camera_link");
    this->declare_parameter("num_objects", 3);
    this->declare_parameter("add_noise", true);
    this->declare_parameter("noise_stddev", 0.002);  // 2mm

    // Get parameters
    double publish_rate = this->get_parameter("publish_rate").as_double();
    frame_id_ = this->get_parameter("frame_id").as_string();
    num_objects_ = this->get_parameter("num_objects").as_int();
    add_noise_ = this->get_parameter("add_noise").as_bool();
    noise_stddev_ = this->get_parameter("noise_stddev").as_double();

    // Publisher
    pointcloud_pub_ = this->create_publisher<sensor_msgs::msg::PointCloud2>(
      "/camera/pointcloud",
      10
    );

    // Timer for publishing
    auto period = std::chrono::duration<double>(1.0 / publish_rate);
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(period),
      std::bind(&MockCameraNode::publish_pointcloud, this)
    );

    // Random number generator for noise
    rng_ = std::mt19937(std::random_device{}());

    RCLCPP_INFO(this->get_logger(), "Mock camera node initialized");
    RCLCPP_INFO(this->get_logger(), "Publishing %d objects at %.1f Hz",
                num_objects_, publish_rate);
    RCLCPP_INFO(this->get_logger(), "Noise: %s (stddev=%.4f)",
                add_noise_ ? "enabled" : "disabled", noise_stddev_);
  }

private:
  struct MockObject {
    double x, y, z;           // Position (meters)
    double width, height, depth;  // Dimensions (meters)
    uint8_t r, g, b;          // Color
    double weight_grams;      // Ground truth weight
  };

  void publish_pointcloud()
  {
    // Create mock objects (representing stainless steel weights)
    std::vector<MockObject> objects = create_mock_objects();

    // Generate point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (const auto& obj : objects) {
      add_object_to_cloud(cloud, obj);
    }

    // Add background/table points (optional)
    add_table_surface(cloud);

    // Convert to ROS message
    sensor_msgs::msg::PointCloud2 msg;
    pcl::toROSMsg(*cloud, msg);
    msg.header.stamp = this->now();
    msg.header.frame_id = frame_id_;

    // Publish
    pointcloud_pub_->publish(msg);

    RCLCPP_DEBUG(this->get_logger(), "Published point cloud with %zu points", cloud->size());
  }

  std::vector<MockObject> create_mock_objects()
  {
    std::vector<MockObject> objects;

    // Define mock objects with known dimensions
    // Stainless steel density = 8000 kg/m³

    if (num_objects_ >= 1) {
      // Small weight: 50g (volume = 50/1000/8000 = 6.25e-6 m³)
      // Approximate as 2.5cm x 2.5cm x 1cm cube
      MockObject obj1;
      obj1.x = 0.3; obj1.y = 0.15; obj1.z = 0.1;
      obj1.width = 0.025; obj1.height = 0.025; obj1.depth = 0.01;
      obj1.r = 192; obj1.g = 192; obj1.b = 192;  // Silver
      obj1.weight_grams = 50.0;
      objects.push_back(obj1);
    }

    if (num_objects_ >= 2) {
      // Medium weight: 100g (volume = 1.25e-5 m³)
      // Approximate as 3cm x 3cm x 1.5cm cube
      MockObject obj2;
      obj2.x = 0.35; obj2.y = -0.1; obj2.z = 0.12;
      obj2.width = 0.03; obj2.height = 0.03; obj2.depth = 0.015;
      obj2.r = 180; obj2.g = 180; obj2.b = 180;  // Silver
      obj2.weight_grams = 100.0;
      objects.push_back(obj2);
    }

    if (num_objects_ >= 3) {
      // Large weight: 200g (volume = 2.5e-5 m³)
      // Approximate as 4cm x 4cm x 1.5cm cube
      MockObject obj3;
      obj3.x = 0.25; obj3.y = 0.0; obj3.z = 0.115;
      obj3.width = 0.04; obj3.height = 0.04; obj3.depth = 0.015;
      obj3.r = 170; obj3.g = 170; obj3.b = 170;  // Silver
      obj3.weight_grams = 200.0;
      objects.push_back(obj3);
    }

    if (num_objects_ >= 4) {
      // Extra weight: 150g
      MockObject obj4;
      obj4.x = 0.4; obj4.y = 0.2; obj4.z = 0.11;
      obj4.width = 0.035; obj4.height = 0.035; obj4.depth = 0.012;
      obj4.r = 175; obj4.g = 175; obj4.b = 175;
      obj4.weight_grams = 150.0;
      objects.push_back(obj4);
    }

    return objects;
  }

  void add_object_to_cloud(
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud,
    const MockObject& obj)
  {
    // Generate points in a cuboid shape
    double resolution = 0.002;  // 2mm point spacing

    std::normal_distribution<double> noise_dist(0.0, noise_stddev_);

    for (double x = -obj.width/2; x <= obj.width/2; x += resolution) {
      for (double y = -obj.height/2; y <= obj.height/2; y += resolution) {
        for (double z = -obj.depth/2; z <= obj.depth/2; z += resolution) {
          pcl::PointXYZRGB point;

          // Apply position and optional noise
          double noise_x = add_noise_ ? noise_dist(rng_) : 0.0;
          double noise_y = add_noise_ ? noise_dist(rng_) : 0.0;
          double noise_z = add_noise_ ? noise_dist(rng_) : 0.0;

          point.x = obj.x + x + noise_x;
          point.y = obj.y + y + noise_y;
          point.z = obj.z + z + noise_z;

          // Color
          point.r = obj.r;
          point.g = obj.g;
          point.b = obj.b;

          cloud->points.push_back(point);
        }
      }
    }
  }

  void add_table_surface(pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud)
  {
    // Add a sparse table surface for context
    double resolution = 0.01;  // 1cm spacing for table

    for (double x = -0.5; x <= 0.5; x += resolution) {
      for (double y = -0.5; y <= 0.5; y += resolution) {
        // Only add 10% of table points (sparse)
        if (std::rand() % 10 != 0) continue;

        pcl::PointXYZRGB point;
        point.x = x;
        point.y = y;
        point.z = 0.0;  // Table at z=0

        // Brown color for table
        point.r = 139;
        point.g = 90;
        point.b = 43;

        cloud->points.push_back(point);
      }
    }
  }

  // ROS2 components
  rclcpp::Publisher<sensor_msgs::msg::PointCloud2>::SharedPtr pointcloud_pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  // Parameters
  std::string frame_id_;
  int num_objects_;
  bool add_noise_;
  double noise_stddev_;

  // Random number generator
  std::mt19937 rng_;
};

int main(int argc, char** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<MockCameraNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
