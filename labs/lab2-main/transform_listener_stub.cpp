#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

class TF2Listener : public rclcpp::Node
{
public:
  TF2Listener() : Node("tf2_listener_node")
  {
    tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
    timer_ = this->create_wall_timer( std::chrono::milliseconds(200), std::bind(&TF2Listener::tfCallback, this));
  }

private:
  void tfCallback()
  {
    std::string fromFrameRel = "";
    std::string toFrameRel = "";
    geometry_msgs::msg::TransformStamped t;

    //TODO: PLACE LISTENER CODE HERE

    std::cout << "transformation found" <<std::endl;
    std::cout << " [ " << t.transform.translation.x << " , " << t.transform.translation.y << " , " << t.transform.translation.z << std::endl;
  }

  std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
  std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
  rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TF2Listener>());
  rclcpp::shutdown();
  return 0;
}