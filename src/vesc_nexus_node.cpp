#include "vesc_nexus/vesc_nexus_node.hpp"
#include "vesc_manager.hpp"

VescNexusNode::VescNexusNode() : Node("vesc_nexus_node")
{
  RCLCPP_INFO(this->get_logger(), "VescNexusNode started.");
}

int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<VescNexusNode>());
  rclcpp::shutdown();
  return 0;
}