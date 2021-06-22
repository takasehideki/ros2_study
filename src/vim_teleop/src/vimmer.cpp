#include <chrono>
#include <memory>

#include "rclcpp/rclcpp.hpp"
/* Twist型のインクルード */
#include "geometry_msgs/msg/twist.hpp"

/* キー入力cin用 */
using namespace std;
using namespace std::chrono_literals;

class Publisher : public rclcpp::Node
{
public:
  Publisher()
  : Node("imvimmer")
  {
    /* Twist型の /cmd_vel をtopicとして出版ノードを生成 */
    publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    /* コールバック関数をタイマイベントとして登録 */
    /* キー入力待ちでタイマのスケジューラが破綻するため，本来はRate.sleepで実現すべき */
    timer_ = this->create_wall_timer(
      500ms, std::bind(&Publisher::timer_callback, this));
  }

private:
  /* コールバック関数の定義 */
  void timer_callback()
  {
    /* キーマップの表示 */
    /* h-左 j-前 k-後 l-右 u-停止 */
    RCLCPP_INFO(this->get_logger(), "Enter Key(+Enter) as Vimmer");
    RCLCPP_INFO(this->get_logger(), "j:forward, k:backward, h:left, l:right"); 

    /* 出版用のTwist型変数の宣言 */
    geometry_msgs::msg::Twist vel;

    /* キー入力を受け付ける */
    char key;
    cin >> key;

    /* キー入力に応じて linear or angular を設定する */
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
    /* 出版データのログ出力 */
    RCLCPP_INFO(this->get_logger(),
        "Publishing: linear = %.2lf angular = %.2lf",
        vel.linear.x, vel.angular.z);
    /* /cmd_vel に出版 */
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