#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "estop_interfaces/srv/estop.hpp"

class Listener : public rclcpp::Node
{
  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Client<estop_interfaces::srv::Estop>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_ping_;

    void topic_callback(const std_msgs::msg::String & msg) // updates last ping time if keep-alive-estop talker publishes to topic
    {
      auto curr_ping = this->now();
      RCLCPP_INFO(this->get_logger(), "Heard: '%s' at '%f', last message was '%f's ago", msg.data.c_str(), curr_ping.seconds(), last_ping_.seconds());
      last_ping_ = curr_ping;
    }

    void timer_callback()
    {
      double diff = this->now().seconds() - last_ping_.seconds();
      RCLCPP_INFO(this->get_logger(), "Last message was '%f's ago.", diff);
      if (diff > 5)
        request_estop();
    }

    void request_estop()
    {
      auto request = std::make_shared<estop_interfaces::srv::Estop::Request>();
      request->start_estop = true;

      // wait for service to be available
      while (!client_->wait_for_service(std::chrono::seconds(1))) {
        if (!rclcpp::ok()) {
          RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
          return;
        }
        RCLCPP_INFO(this->get_logger(), "service not available, waiting again...");
      }

      // send result
      auto future = client_->async_send_request(request, 
        [this](rclcpp::Client<estop_interfaces::srv::Estop>::SharedFuture future) {
          try {
            auto response = future.get();
            RCLCPP_INFO(this->get_logger(), "Success: %d", response->start_estop_received);
          } catch (const std::exception &e) {
            RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
          }
        }
      );
    }
    
  public:
    Listener() : Node {"keep_alive_estop_listener"}
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, std::bind(&Listener::topic_callback, this, std::placeholders::_1));
      client_ = this->create_client<estop_interfaces::srv::Estop>("estop");
      timer_ = this->create_wall_timer(
          std::chrono::seconds(5), 
          std::bind(&Listener::timer_callback, this)
      );
      last_ping_ = this->now();
      RCLCPP_INFO(this->get_logger(), "Rover ready to receive base station, starting timer at '%f'.", last_ping_.seconds());
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}