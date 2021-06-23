#include "rclcpp/rclcpp.hpp"
#include "velpose_msgs/srv/vel_pose.hpp"

#include <chrono>
#include <cstdlib>
#include <memory>

using namespace std::chrono_literals;

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  if (argc != 3) {
    RCLCPP_INFO(rclcpp::get_logger("vlp_client"), 
        "usage: linear[m/s] angular[rad/s]");
    return 1;
  }

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("vlp_client");
  rclcpp::Client<velpose_msgs::srv::VelPose>::SharedPtr client =
      node->create_client<velpose_msgs::srv::VelPose>("velpose_service");

  auto request = std::make_shared<velpose_msgs::srv::VelPose::Request>();
  request->linear  = atof(argv[1]);
  request->angular = atof(argv[2]);

  if( request->linear > 0.22 || request->linear < -0.22) {
    RCLCPP_ERROR(rclcpp::get_logger("vlp_client"), 
        "linear %.2f is invalid value", request->linear);
    return 0;
  }
  if( request->angular > 2.84 || request->angular < -2.84) {
    RCLCPP_ERROR(rclcpp::get_logger("vlp_client"), 
        "angular %.2f is invalid value", request->angular);
    return 0;
  }

  RCLCPP_INFO(rclcpp::get_logger("vlp_client"), 
      "Request to send: linear %.2lf / angular %.2lf",
      request->linear, request->angular);

  while (!client->wait_for_service(1s)) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(rclcpp::get_logger("vlp_client"), 
          "Interrupted while waiting for the service. Exiting.");
      return 0;
    }
    RCLCPP_INFO(rclcpp::get_logger("vlp_client"), 
        "service not available, waiting again...");
  }

  RCLCPP_INFO(rclcpp::get_logger("vlp_client"), 
      "service is now available, sending the request,,,");

  auto result = client->async_send_request(request);

  if (rclcpp::spin_until_future_complete(node, result) ==
    rclcpp::FutureReturnCode::SUCCESS)
  {
    RCLCPP_INFO(rclcpp::get_logger("vlp_client"), 
        "Successfully called VelPose service.\n");
    RCLCPP_INFO(rclcpp::get_logger("vlp_client"), 
        "Current position of TB3: x=%.2f y=%.2f", 
        result.get()->position_x, result.get()->position_y);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("vlp_client"), 
        "Failed to call VelPose service");
  }

  rclcpp::shutdown();
  return 0;
}
