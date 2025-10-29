// int_addition_server.cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class IntAdditionServer : public rclcpp::Node
{
public:
  IntAdditionServer() : Node("int_addition_server")
  {
    srv_ = create_service<example_interfaces::srv::AddTwoInts>("add_two_ints",
                                                               std::bind(&IntAdditionServer::add, this, std::placeholders::_1, std::placeholders::_2));
  }

private:
  void add(const std::shared_ptr<example_interfaces::srv::AddTwoInts::Request> request,
           std::shared_ptr<example_interfaces::srv::AddTwoInts::Response> response)
  {
    response->sum = request->a + request->b;
    RCLCPP_INFO(get_logger(), "Received request: %ld + %ld = %ld", request->a, request->b, response->sum);
  }

  rclcpp::Service<example_interfaces::srv::AddTwoInts>::SharedPtr srv_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto server = std::make_shared<IntAdditionServer>();
  rclcpp::spin(server);
  rclcpp::shutdown();
  return 0;
}
