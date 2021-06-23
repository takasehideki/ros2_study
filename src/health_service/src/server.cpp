#include "rclcpp/rclcpp.hpp"
#include "health_msgs/srv/health.hpp"

#include <memory>

class Server : public rclcpp::Node
{
public:
  Server()
  : Node("health_server")
  {    
    auto calc_bmi = [this](            
      const std::shared_ptr<health_msgs::srv::Health::Request> request,
      std::shared_ptr<health_msgs::srv::Health::Response> response) -> 
    void {      
      RCLCPP_INFO(this->get_logger(), 
          "Incoming request.\n height: %u / weight: %.2f", 
          request->height, request->weight);

      response->bmi = request->weight / (request->height / 100.0) / (request->height / 100.0);

      RCLCPP_INFO(this->get_logger(), 
          "sending back response.\n %s's BMI is [%.2f]", 
          request->name.c_str(), response->bmi);
    };     
    service_ = this->create_service<health_msgs::srv::Health>(
        "health_service",calc_bmi);
    RCLCPP_INFO(this->get_logger(), "Ready to calculate BMI.");
  } 
  private:  
  rclcpp::Service<health_msgs::srv::Health>::SharedPtr service_;
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Server>());
  rclcpp::shutdown();
  return 0;
}
