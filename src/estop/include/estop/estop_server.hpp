#ifndef ESTOP_SERVER_HPP
#define ESTOP_SERVER_HPP

#include "rclcpp/rclcpp.hpp"
#include "estop_interfaces/srv/estop.hpp"

class EstopServer : public rclcpp::Node
{
    private:
        rclcpp::Service<estop_interfaces::srv::Estop>::SharedPtr service_;
        
        void handle_estop_request(
            const std::shared_ptr<estop_interfaces::srv::Estop::Request> request,
                  std::shared_ptr<estop_interfaces::srv::Estop::Response> response
        );

    public: 
        EstopServer();
};

#endif // ESTOP_SERVER_HPP