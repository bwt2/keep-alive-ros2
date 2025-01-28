#include <memory>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Listener : public rclcpp::Node
{
  private:
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    void topic_callback(const std_msgs::msg::String & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%s'", msg.data.c_str());
    }
    
  public:
    Listener()
      : Node {"minimal_subscriber"}
    {
      subscription_ = this->create_subscription<std_msgs::msg::String>("topic", 10, 
        std::bind(&Listener::topic_callback, this, std::placeholders::_1)
      );
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Listener>());
  rclcpp::shutdown();
  return 0;
}