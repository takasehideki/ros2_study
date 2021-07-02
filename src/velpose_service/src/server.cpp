#include <memory>

#include "rclcpp/rclcpp.hpp"
/* 独自に定義したVelPose型のインクルード */
#include "velpose_msgs/srv/vel_pose.hpp"
/* Twist型, Odometry型のインクルード */
#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"

#include <chrono>
using namespace std::chrono_literals;

/* /odom から購読する {x,y} 座標を保持する変数 */
float_t g_odom_position_x;
float_t g_odom_position_y;

/* ノードのオブジェクトを操作するためのスマートポインタ */
using std::placeholders::_1;

class Server : public rclcpp::Node
{
public:
  /* service server の生成時に実行される処理 */
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

      /* client から受信した request を格納 */
      geometry_msgs::msg::Twist vel;
      vel.linear.x  = request->linear;
      vel.angular.z = request->angular;
  
      RCLCPP_INFO(this->get_logger(), "Publishing velocity to TB3,,,");

      /* linear,angular を /cmd_vel に対して100msごとに出版 */
      rclcpp::WallRate loop_rate(100ms);
      for(int i=0; i<100; i++){
        publisher_->publish(vel);
        loop_rate.sleep();
      }

      /* client に返送する {x,y} 座標を response に格納 */
      response->position_x = g_odom_position_x;
      response->position_y = g_odom_position_y;
      RCLCPP_INFO(this->get_logger(), 
          "Position after operating : x %.2f / y %.2f", 
          response->position_x, response->position_y);

      RCLCPP_INFO(this->get_logger(), "Operation on VelPose server finished");
    };
    /* service, publisher, subscriber をそれぞれ生成 */
    service_ = this->create_service<velpose_msgs::srv::VelPose>(  
        "velpose_service", vlp_server);
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 10, std::bind(&Server::odom_callback, this, _1));

    RCLCPP_INFO(this->get_logger(), "Ready to run VelPose server");
  }

private:
  /* /odom の購読時に実行されるコールバック関数 */
  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) const
  {
    g_odom_position_x = msg->pose.pose.position.x;
    g_odom_position_y = msg->pose.pose.position.y;
    RCLCPP_DEBUG(this->get_logger(), 
        "Position after operating : x %.2f / y %.2f", 
        g_odom_position_x, g_odom_position_y);
  }
  /* service, publisher, subscriber のためのスマートポインタ */
  rclcpp::Service<velpose_msgs::srv::VelPose>::SharedPtr service_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Server>());
  rclcpp::shutdown();
  return 0;
}
