#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "estop_interfaces/srv/estop.hpp"

void handle_estop_request(const std::shared_ptr<estop_interfaces::srv::Estop::Request>  request,
                 std::shared_ptr<estop_interfaces::srv::Estop::Response> response)
{
  response->start_estop_received = request->start_estop;
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Incoming request\nstart estop: %d", request->start_estop);
  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "sending back response: [%d]", response->start_estop_received);
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);

  std::shared_ptr<rclcpp::Node> node = rclcpp::Node::make_shared("estop_server");

  // makes service available across network
  rclcpp::Service<estop_interfaces::srv::Estop>::SharedPtr service = node->create_service<estop_interfaces::srv::Estop>("estop_server", &handle_estop_request);

  RCLCPP_INFO(rclcpp::get_logger("rclcpp"), "Ready to respond to Estop calls.");

  rclcpp::spin(node);
  rclcpp::shutdown();
}