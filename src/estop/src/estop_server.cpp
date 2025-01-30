#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "estop_interfaces/srv/estop.hpp"

class EstopServer : public rclcpp::Node
{
  private:
    rclcpp::Service<estop_interfaces::srv::Estop>::SharedPtr service_;
    void handle_estop_request(
      [[maybe_unused]] const std::shared_ptr<estop_interfaces::srv::Estop::Request>  request ,
      [[maybe_unused]]       std::shared_ptr<estop_interfaces::srv::Estop::Response> response)
    {      
      RCLCPP_INFO(this->get_logger(), "Starting Estop Service.");
      RCLCPP_INFO(this->get_logger(), "Finishing Estop Service.");
    }

  public:
    EstopServer() : Node {"estop_server"}
    {
      // make service available across server
      service_ = this->create_service<estop_interfaces::srv::Estop>("estop", 
        std::bind(&EstopServer::handle_estop_request, this, std::placeholders::_1, std::placeholders::_2)
      );
      RCLCPP_INFO(this->get_logger(), "Ready to respond to Estop calls.");
    }
};

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<EstopServer>());
  rclcpp::shutdown();
  return 0;
}