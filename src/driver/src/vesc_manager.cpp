#include "driver/vesc_manager.hpp"

VescManager::VescManager(rclcpp::Node *node) : node_(node) {}

void VescManager::addVesc(const std::string &can_if, uint8_t vesc_id)
{
  if (!can_interfaces_.count(can_if)) {
    can_interfaces_[can_if] = std::make_unique<CanInterface>(can_if);
    RCLCPP_INFO(node_->get_logger(), "Created CAN interface: %s", can_if.c_str());
  }

  vesc_devices_[{can_if, vesc_id}] = std::make_unique<VescDevice>(vesc_id, can_if, node_);
}

void VescManager::start()
{
    for (auto &pair : can_interfaces_) {
        pair.second->startListening([this](const can_frame &frame) {
            // TODO: ??????? ? ???????? ????????? VESC
        });
    }
}
