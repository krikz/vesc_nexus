// ros2_control SystemInterface для привода на VESC по CAN.
#pragma once

#include <memory>
#include <vector>

#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "vesc_nexus/can_interface.hpp"
#include "vesc_nexus/vesc_handler.hpp"

namespace vesc_nexus {

class VescSystemHardwareInterface : public hardware_interface::SystemInterface {
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(VescSystemHardwareInterface)

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
  void stopAllMotors();

  std::unique_ptr<CanInterface> can_;
  std::vector<std::unique_ptr<VescHandler>> handlers_;

  // Буферы интерфейсов, по одному элементу на joint. Адреса экспортируются, размер после on_init не меняется.
  std::vector<double> hw_positions_;    // рад
  std::vector<double> hw_velocities_;   // рад/с
  std::vector<double> hw_efforts_;      // ток мотора, А
  std::vector<double> cmd_velocities_;  // рад/с
  double hw_battery_voltage_{0.0};      // минимальное ненулевое напряжение среди VESC, В

  // Таймаут команд: после command_timeout_ секунд нулевых команд фреймы не отправляются, мотор расслабляется.
  std::vector<rclcpp::Time> last_nonzero_cmd_time_;
  double command_timeout_{0.5};
  bool motors_relaxed_{false};
};

}  // namespace vesc_nexus
