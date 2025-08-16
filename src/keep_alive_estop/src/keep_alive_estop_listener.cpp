#include "keep_alive_estop/keep_alive_estop_listener.hpp"

KeepAliveEstopListener::KeepAliveEstopListener()
  : Node {"keep_alive_estop_listener"}
{
  this->declare_parameter("timeout", 5.0f);
  timeout_ = this->get_parameter("timeout").as_double();

  auto qos {rclcpp::SystemDefaultsQoS()};
  qos.keep_last(1);
  qos.reliable();

  rmw_qos_profile_t estop_service_qos {};
  estop_service_qos.reliability = RMW_QOS_POLICY_RELIABILITY_RELIABLE;

  subscription_ = this->create_subscription<std_msgs::msg::String>("keep_alive_estop", qos, std::bind(&KeepAliveEstopListener::topic_callback, this, std::placeholders::_1));
  client_ = this->create_client<estop_interfaces::srv::Estop>("estop", estop_service_qos);

  RCLCPP_INFO(this->get_logger(), "Waiting for keep alive talker ...");

  rclcpp::WaitSet wait_set;
  wait_set.add_subscription(subscription_);  
  const auto wait_result {wait_set.wait()};
  if (wait_result.kind() != rclcpp::WaitResultKind::Ready){ 
    RCLCPP_ERROR(this->get_logger(), "Keep alive talker not found!");
    return;
  }

  timer_ = this->create_wall_timer(
      std::chrono::seconds(5), 
      std::bind(&KeepAliveEstopListener::timer_callback, this)
  );
  last_ping_ = this->now();
  RCLCPP_INFO(this->get_logger(), "Rover ready to receive base station, starting timer at '%f'.", last_ping_.seconds());
}

void KeepAliveEstopListener::topic_callback([[maybe_unused]] const std_msgs::msg::String & msg) // updates last ping time if keep-alive-estop talker publishes to topic
{
  auto curr_ping = this->now();
#ifdef DEBUG
  RCLCPP_INFO(this->get_logger(), "Heard: '%s' at '%f', last message was '%f's ago", msg.data.c_str(), curr_ping.seconds(), last_ping_.seconds());
#endif // DEBUG
  last_ping_ = curr_ping;
}

void KeepAliveEstopListener::timer_callback()
{
  double diff = this->now().seconds() - last_ping_.seconds();
#ifdef DEBUG
  RCLCPP_INFO(this->get_logger(), "Last message was '%f's ago.", diff);
#endif // DEBUG
  if (diff > timeout_)
  {
    RCLCPP_WARN(this->get_logger(), "Last message was [%f]s ago. Starting Estop procedure.", diff);
    request_estop();
  }
}

void KeepAliveEstopListener::request_estop()
{
  auto request = std::make_shared<estop_interfaces::srv::Estop::Request>();
  int update_freq = 5; // how often "service not available" message is printed in seconds
  int update_timeout = update_freq + 1;

  // wait for service to be available
  while (!client_->wait_for_service(std::chrono::seconds(1))) {
    if (!rclcpp::ok()) {
      RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
      return;
    }
    if (update_timeout > update_freq)
    {
      RCLCPP_ERROR(this->get_logger(), "Estop service not available, trying again ...");
      update_timeout = 1;
    }
    update_timeout += 1;
  }

  // send estop request and wait for response
  auto future = client_->async_send_request(request, 
    [this](const rclcpp::Client<estop_interfaces::srv::Estop>::SharedFuture future){
      try 
      {
        auto response = future.get();
        if (response)
        {
          RCLCPP_WARN(this->get_logger(), "Estop procedure completed successfully.");
          rclcpp::shutdown();
        } 
        else
        {
          RCLCPP_ERROR(this->get_logger(), "Estop procedure failed.");
        }
      } 
      catch (const std::exception &e)
      {
        RCLCPP_ERROR(this->get_logger(), "Service call failed: %s", e.what());
      }        
    }
  ); 
}

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<KeepAliveEstopListener>());
  rclcpp::shutdown();
  return 0;
}