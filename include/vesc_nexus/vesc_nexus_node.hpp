#ifndef VESC_NEXUS_NODE_HPP_
#define VESC_NEXUS_NODE_HPP_

#include <rclcpp/rclcpp.hpp>
#include "vesc_manager.hpp"

class VescNexusNode : public rclcpp::Node
{
public:
  VescNexusNode();

private:
  std::unique_ptr<VescManager> vesc_manager_;
};

#endif // VESC_NEXUS_NODE_HPP_