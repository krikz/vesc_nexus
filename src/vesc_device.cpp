#include "vesc_nexus/vesc_device.hpp"

VescDevice::VescDevice(uint8_t id, const std::string &can_interface, rclcpp::Node *node)
    : id_(id), can_interface_(can_interface), node_(node)
{
    state_pub_ = node_->create_publisher<vesc_msgs::msg::VescStateStamped>(
        "vesc/" + std::to_string(id) + "/state", 10);
}

void VescDevice::sendDutyCycle(float duty)
{
    // TODO: ????????? ??????? ????? CAN
    RCLCPP_INFO(node_->get_logger(), "VESC %d: set duty %.2f", id_, duty);
}

void VescDevice::updateState(const vesc_nexus::msg::VescState &state)
{
    auto msg = std::make_shared<vesc_msgs::msg::VescStateStamped>();
    msg->header.stamp = node_->now();
    msg->header.frame_id = "vesc_" + std::to_string(id_);
    msg->state = state;
    state_pub_->publish(*msg);
}
