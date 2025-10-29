#include <rclcpp/rclcpp.hpp>
#include "tf2_ros/static_transform_broadcaster.h"
#include "tf2/LinearMath/Quaternion.h"
#include <geometry_msgs/msg/transform_stamped.hpp>

class StaticTFBroadcaster : public rclcpp::Node
{
public:
  StaticTFBroadcaster() : Node("static_tf_broadcaster")
  {
    // Create a static transform broadcaster
    tf_static_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);


    // Fill out the transformation stamped message


    // Broadcaster static transformation
    tf_static_broadcaster_->sendTransform(transformStamped_);
  }

private:
  std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_static_broadcaster_;
  geometry_msgs::msg::TransformStamped transformStamped_;
  rclcpp::TimerBase::SharedPtr publish_timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<StaticTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}
