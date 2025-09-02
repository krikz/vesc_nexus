#ifndef VESC_MANAGER_HPP_
#define VESC_MANAGER_HPP_

#include "vesc_device.hpp"
#include "can_interface.hpp"
#include <map>
#include <memory>

class VescManager
{
public:
  VescManager(rclcpp::Node *node);
  void addVesc(const std::string& interface_name, uint8_t id, const std::string& label);
  void start();

private:
  rclcpp::Node *node_;
  std::map<std::pair<std::string, uint8_t>, std::unique_ptr<VescDevice>> vesc_devices_;
  std::map<std::string, std::unique_ptr<CanInterface>> can_interfaces_;
};

#endif // VESC_MANAGER_HPP_
