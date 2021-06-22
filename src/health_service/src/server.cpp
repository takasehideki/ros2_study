#include "rclcpp/rclcpp.hpp"
#include "health_msgs/srv/health.hpp"

#include <memory>

void calc_bmi(const std::shared_ptr<health_msgs::srv::Health::Request> request,
    std::shared_ptr<health_msgs::srv::Health::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("health_server"), 
      "Incoming request.\n height: %u / weight: %.2f", request->height, request->weight);

  response->bmi = request->weight / (request->height / 100.0) / (request->height / 100.0);

  RCLCPP_INFO(rclcpp::get_logger("health_server"), 
      "sending back response.\n %s's BMI is [%.2f]", request->name.c_str(), response->bmi);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = 
      rclcpp::Node::make_shared("health_server");

  rclcpp::Service<health_msgs::srv::Health>::SharedPtr service =
      node->create_service<health_msgs::srv::Health>("health_service",
      &calc_bmi);

  RCLCPP_INFO(rclcpp::get_logger("health_server"), "Ready to calculate BMI.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
