#include "rclcpp/rclcpp.hpp"
#include "ros2_study_types/srv/health.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 4) {
    RCLCPP_INFO(rclcpp::get_logger("bmi_client"), 
        "usage: bmi_client Name[str] Height[int/cm] Weight[float/kg]");
    return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("bmi_client");
  rclcpp::Client<ros2_study_types::srv::Health>::SharedPtr client =
      node->create_client<ros2_study_types::srv::Health>("health");

  auto request = std::make_shared<ros2_study_types::srv::Health::Request>();
  request->name = argv[1];
  request->height = atoi(argv[2]);
  request->weight = atof(argv[3]);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("bmi_client"), 
          "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("bmi_client"), 
        "service not available, waiting again...");
  }

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("bmi_client"), 
        "My BMI is %.2f", result.get()->bmi);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("bmi_client"), 
        "Failed to call bmi service");
  }

  rclcpp::shutdown();
  return 0;
}
