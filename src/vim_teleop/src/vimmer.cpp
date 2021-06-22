#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

using namespace std;
using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
  Publisher()
  : Node("imvimmer")
  {
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
      500ms, std::bind(&Publisher::timer_callback, this));
  }

private:
  void timer_callback()
  {
    RCLCPP_INFO(this->get_logger(), "Enter Key(+Enter) as Vimmer");
    RCLCPP_INFO(this->get_logger(), "j:forward, k:backward, h:left, l:right"); 

    geometry_msgs::msg::Twist vel;

    char key;
    cin >> key;

    switch(key)
    {
      case 'j': /* forward */
        RCLCPP_INFO(this->get_logger(), "forwarding,,,"); 
        vel.linear.x = 0.20;
        break;
      case 'k': /* backward */
        RCLCPP_INFO(this->get_logger(), "backwarding,,,"); 
        vel.linear.x = -0.20;
        break;
      case 'h': /* left */
        RCLCPP_INFO(this->get_logger(), "turning left,,,"); 
        vel.angular.z = 0.50;
        break;
      case 'l': /* right */
        RCLCPP_INFO(this->get_logger(), "turning right,,,"); 
        vel.angular.z = -0.50;
        break;
      default:  /* boo! */
        RCLCPP_INFO(this->get_logger(), "Input key is invalid");
        RCLCPP_INFO(this->get_logger(), "Are you really Vimmer?");
        vel.linear.x = 0.00;
        vel.angular.z = 0.00;
        break;
    }
    RCLCPP_INFO(this->get_logger(),
        "Publishing: linear = %.2lf angular = %.2lf",
        vel.linear.x, vel.angular.z);
    publisher_->publish(vel);
  }
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Publisher>());
  rclcpp::shutdown();
  return 0;
}