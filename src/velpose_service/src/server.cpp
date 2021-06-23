#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "velpose_msgs/srv/vel_pose.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <chrono>
using namespace std::chrono_literals;

class Server : public rclcpp::Node
{
public:
  Server()
  : Node("vlp_server")
  {
    auto vlp_server = [this](
      const std::shared_ptr<velpose_msgs::srv::VelPose::Request> request,
      std::shared_ptr<velpose_msgs::srv::VelPose::Response> response)
    {
      RCLCPP_INFO(this->get_logger(), 
          "Incoming request: linear %.2f / angular %.2f", 
          request->linear, request->angular);

      geometry_msgs::msg::Twist vel;
      vel.linear.x  = request->linear;
      vel.angular.z = request->angular;
  
      RCLCPP_INFO(this->get_logger(), "Publishing velocity to TB3,,,");

      rclcpp::WallRate loop_rate(100ms);
      for(int i=0; i<100; i++){
        publisher_->publish(vel);
        loop_rate.sleep();
      }

      response->position_x = odom_position_x_;
      response->position_y = odom_position_y_;
      RCLCPP_INFO(this->get_logger(), 
          "Position after operating : x %.2f / y %.2f", 
          response->position_x, response->position_y);

      RCLCPP_INFO(this->get_logger(), "Operation on VelPose server finished");
    };
    service_ = this->create_service<velpose_msgs::srv::VelPose>(  
        "velpose_service", vlp_server);
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    RCLCPP_INFO(this->get_logger(), "Ready to run VelPose server");
  }

private:
  rclcpp::Service<velpose_msgs::srv::VelPose>::SharedPtr service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;

  float_t odom_position_x_;
  float_t odom_position_y_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Server>());
  rclcpp::shutdown();
  return 0;
}
