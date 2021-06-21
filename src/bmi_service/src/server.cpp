#include "rclcpp/rclcpp.hpp"
#include "ros2_study_types/srv/health.hpp"

#include <memory>

void bmi(const std::shared_ptr<ros2_study_types::srv::Health::Request> request,
    std::shared_ptr<ros2_study_types::srv::Health::Response> response)
{
  RCLCPP_INFO(rclcpp::get_logger("bmi_server"), 
      "Incoming request.\n height: %u / weight: %.2f", request->height, request->weight);

  response->bmi = request->weight / (request->height / 100.0) / (request->height / 100.0);

  RCLCPP_INFO(rclcpp::get_logger("bmi_server"), 
      "sending back response.\n %s's BMI is [%.2f]", request->name.c_str(), response->bmi);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("bmi_server");

  rclcpp::Service<ros2_study_types::srv::Health>::SharedPtr service =
      node->create_service<ros2_study_types::srv::Health>("health", &bmi);

  RCLCPP_INFO(rclcpp::get_logger("bmi_server"), "Ready to calculate BMI.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}
