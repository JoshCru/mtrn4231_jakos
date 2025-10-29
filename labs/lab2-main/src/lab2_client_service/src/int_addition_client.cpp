// int_addition_client.cpp
#include "rclcpp/rclcpp.hpp"
#include "example_interfaces/srv/add_two_ints.hpp"

class IntAdditionClient : public rclcpp::Node
{
public:
  IntAdditionClient() : Node("int_addition_client")
  {
    client_ = create_client<example_interfaces::srv::AddTwoInts>("add_two_ints");
    while (!client_->wait_for_service(std::chrono::seconds(1)))
    {
      RCLCPP_INFO(get_logger(), "Service not available, waiting...");
    }
  }

  void call_service(int a, int b)
  {
    auto request = std::make_shared<example_interfaces::srv::AddTwoInts::Request>();
    request->a = a;
    request->b = b;

    auto future_result = client_->async_send_request(
      request,
      std::bind(&IntAdditionClient::handle_response, this, std::placeholders::_1)
    );
  }

private:
  void handle_response(rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedFuture future)
  {

    if (future.get()) {
        RCLCPP_INFO(get_logger(), "Result:  %ld", future.get()->sum);
    } else {
        RCLCPP_ERROR(get_logger(), "Service call failed");
    }
    

  }

  rclcpp::Client<example_interfaces::srv::AddTwoInts>::SharedPtr client_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto client = std::make_shared<IntAdditionClient>();
  client->call_service(5, 7);
  rclcpp::spin(client);
  rclcpp::shutdown();
  return 0;
}
