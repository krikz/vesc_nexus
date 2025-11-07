// include/vesc_nexus/vesc_system_hardware_interface.hpp
#pragma once

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include <memory>
#include <vector>

#include "vesc_nexus/can_interface.hpp"
#include "vesc_nexus/vesc_handler.hpp"

namespace vesc_nexus {

class VescSystemHardwareInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VescSystemHardwareInterface);

  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo& info) override;
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_shutdown(const rclcpp_lifecycle::State& previous_state) override;
  hardware_interface::CallbackReturn on_error(const rclcpp_lifecycle::State& previous_state) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::return_type read(const rclcpp::Time& time, const rclcpp::Duration& period) override;
  hardware_interface::return_type write(const rclcpp::Time& time, const rclcpp::Duration& period) override;

private:
  std::unique_ptr<CanInterface> can_interface_;
  std::vector<std::shared_ptr<VescHandler>> vesc_handlers_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;
  std::vector<double> cmd_velocities_;
  const hardware_interface::HardwareInfo * info_ = nullptr;

  std::string can_interface_name_;
  double publish_rate_ = 50.0;
  double wheel_radius_ = 0.115;
  
  // Timeout mechanism for motor relaxation
  std::vector<rclcpp::Time> last_nonzero_cmd_time_;
  double command_timeout_ = 0.5;  // seconds - configurable via URDF
  bool motors_relaxed_ = false;
  
  // Battery voltage monitoring (sensor interface)
  double hw_battery_voltage_ = 0.0;  // Aggregated battery voltage (V) - minimum non-zero
};

}  // namespace vesc_nexus