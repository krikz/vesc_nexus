#ifndef VESC_DEVICE_HPP_
#define VESC_DEVICE_HPP_

#include <rclcpp/rclcpp.hpp>
#include <memory>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>

class VescDevice
{
public:
  VescDevice(uint8_t id, const std::string &can_interface, rclcpp::Node *node);
  void sendDutyCycle(float duty);
  void updateState(const vesc_nexus::msg::VescState &state);

private:
  uint8_t id_;
  std::string can_interface_;
  rclcpp::Publisher<vesc_msgs::msg::VescStateStamped>::SharedPtr state_pub_;
  rclcpp::Node *node_;
};

#endif // VESC_DEVICE_HPP_