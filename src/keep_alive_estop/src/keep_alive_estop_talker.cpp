#include "keep_alive_estop/keep_alive_estop_talker.hpp"

KeepAliveEstopTalker::KeepAliveEstopTalker() 
    : Node {"keep_alive_estop_talker"}
{
    auto qos {rclcpp::SystemDefaultsQoS()};
    qos.keep_last(1);
    qos.reliable();

    publisher_ = this->create_publisher<std_msgs::msg::String>("keep_alive_estop", qos);
    timer_ = this->create_wall_timer(
        std::chrono::milliseconds(2500), 
        std::bind(&KeepAliveEstopTalker::timer_callback, this)
    );
    RCLCPP_INFO(this->get_logger(), "Base station ready to ping rover.");
}

void KeepAliveEstopTalker::timer_callback()
{
    std_msgs::msg::String message = std_msgs::msg::String();
    message.data = std::to_string(this->now().seconds());
#ifdef DEBUG
    RCLCPP_INFO(this->get_logger(), "[%s]", message.data.c_str());
#endif // DEBUG
    publisher_->publish(message);
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeepAliveEstopTalker>());
  rclcpp::shutdown();
  return 0;
}