#ifndef KEEP_ALIVE_ESTOP_TALKER_HPP
#define KEEP_ALIVE_ESTOP_TALKER_HPP

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class KeepAliveEstopTalker: public rclcpp::Node {
    private:
        rclcpp::TimerBase::SharedPtr timer_;
        rclcpp::Publisher<std_msgs::msg::String>::SharedPtr publisher_;

        void timer_callback();

    public:
        KeepAliveEstopTalker();
};

#endif // KEEP_ALIVE_ESTOP_TALKER_HPP