#include "arduinobot_msgs/srv/add_two_ints.hpp"
#include <rclcpp/rclcpp.hpp>

#include <memory>

using namespace std::placeholders;

class SimpleServiceServer : public rclcpp::Node {
public:
  SimpleServiceServer() : Node("simple_service_server") {
    service_ = create_service<arduinobot_msgs::srv::AddTwoInts>(
        "add_two_ints",                                  // service name
        std::bind(&SimpleServiceServer::serviceCallback, // callback is member
                                                         // of class SSS
                  this, // declared in current object
                  _1,
                  _2)); // will require 2 inputs = request & response messages
    RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Service add_two_ints Ready");
  }

private:
  rclcpp::Service<arduinobot_msgs::srv::AddTwoInts>::SharedPtr service_;

  void serviceCallback(
      const std::shared_ptr<arduinobot_msgs::srv::AddTwoInts::Request> req,
      const std::shared_ptr<arduinobot_msgs::srv::AddTwoInts::Response> res) {
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "New Request Received a: " << req->a
                                                  << " b: " << req->b);
    res->sum = req->a + req->b;
    RCLCPP_INFO_STREAM(rclcpp::get_logger("rclcpp"),
                       "Returning sum: " << res->sum);
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  auto node = std::make_shared<SimpleServiceServer>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}