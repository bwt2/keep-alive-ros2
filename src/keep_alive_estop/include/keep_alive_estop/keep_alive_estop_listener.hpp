#ifndef KEEP_ALIVE_ESTOP_LISTENER_HPP
#define KEEP_ALIVE_ESTOP_LISTENER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "estop_interfaces/srv/estop.hpp"

class KeepAliveEstopListener : public rclcpp::Node
{
  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    rclcpp::Client<estop_interfaces::srv::Estop>::SharedPtr client_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Time last_ping_;
    double timeout_; // for timer_callback()

    // updates last ping time if keep-alive-estop talker publishes to topic
    void topic_callback(const std_msgs::msg::String & msg);

    // checks keep alive topic every timeout_ seconds, calling request_estop() if no updates occur in time period
    void timer_callback(); 

    // attempts to call estop service
    void request_estop();
    
  public:
    KeepAliveEstopListener();
};

#endif // KEEP_ALIVE_ESTOP_LISTENER_HPP