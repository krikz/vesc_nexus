#ifndef VESC_DRIVER_NODE_HPP_
#define VESC_DRIVER_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "vesc_manager.hpp"

class VescNexusNode : public rclcpp::Node
{
public:
    explicit VescNexusNode(const rclcpp::NodeOptions& options = rclcpp::NodeOptions());

private:
    std::unique_ptr<VescManager> vesc_manager_;
};

#endif // VESC_DRIVER_NODE_HPP_