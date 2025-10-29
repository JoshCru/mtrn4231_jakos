#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <cmath>

class DynamicTFBroadcaster : public rclcpp::Node
{
  public:
    DynamicTFBroadcaster() : Node("dynamic_tf_broadcaster")
    {
      timer_ = create_wall_timer(std::chrono::milliseconds(100), std::bind(&DynamicTFBroadcaster::publishDynamicTransform, this));
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }


  private:
    void publishDynamicTransform()
    {
      double t = rclcpp::Clock().now().seconds();
      double sin_value = std::sin(t);
      geometry_msgs::msg::TransformStamped transform_msg;

      //TODO: Place transformation broadcaster code here

      tf_broadcaster_->sendTransform(transform_msg);
    }

    rclcpp::TimerBase::SharedPtr timer_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DynamicTFBroadcaster>());
    rclcpp::shutdown();
    return 0;
}