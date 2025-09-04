#include "vesc_nexus/vesc_hardware_interface.hpp"
#include "vesc_nexus/can_interface.hpp"
#include "vesc_nexus/vesc_handler.hpp"
#include <rclcpp/rclcpp.hpp>

namespace vesc_nexus {

hardware_interface::return_type VescHardwareInterface::configure(
  const hardware_interface::HardwareInfo & info) {
  RCLCPP_INFO(rclcpp::get_logger("VescHardwareInterface"), "Configuring...");

  can_interface_name_ = info.hardware_parameters.at("can_interface");
  publish_rate_ = std::stod(info.hardware_parameters.at("publish_rate"));
  wheel_radius_ = std::stod(info.hardware_parameters.at("wheel_radius"));

  // Инициализация CAN
  auto can_interface = std::make_shared<CanInterface>(can_interface_name_);
  if (!can_interface->open()) {
    RCLCPP_FATAL(rclcpp::get_logger("VescHardwareInterface"), "Failed to open CAN interface");
    return hardware_interface::return_type::ERROR;
  }

  // Создание обработчиков VESC из параметров
  for (const auto & joint : info.joints) {
    uint8_t can_id = static_cast<uint8_t>(std::stoi(joint.parameters.at("can_id")));
    double radius = wheel_radius_;
    if (joint.parameters.find("wheel_radius") != joint.parameters.end()) {
      radius = std::stod(joint.parameters.at("wheel_radius"));
    }
    int poles = std::stoi(joint.parameters.at("poles"));
    int64_t min_erpm = std::stoll(joint.parameters.at("min_erpm"));

    auto handler = std::make_shared<VescHandler>(
      can_id, joint.name, radius, poles, min_erpm, CommandLimits{});
    handler->setSendCanFunc([can_interface](const auto& f) {
      return can_interface->sendFrame(f);
    });
    vesc_handlers_.push_back(handler);

    // Инициализация буферов
    hw_positions_.push_back(0.0);
    hw_velocities_.push_back(0.0);
    hw_efforts_.push_back(0.0);
    cmd_velocities_.push_back(0.0);
  }

  return hardware_interface::return_type::OK;
}

std::vector<hardware_interface::StateInterface> VescHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < vesc_handlers_.size(); ++i) {
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      vesc_handlers_[i]->getLabel(), hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
    state_interfaces.emplace_back(hardware_interface::StateInterface(
      vesc_handlers_[i]->getLabel(), hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VescHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < vesc_handlers_.size(); ++i) {
    command_interfaces.emplace_back(hardware_interface::CommandInterface(
      vesc_handlers_[i]->getLabel(), hardware_interface::HW_IF_VELOCITY, &cmd_velocities_[i]));
  }
  return command_interfaces;
}

hardware_interface::return_type VescHardwareInterface::start() {
  RCLCPP_INFO(rclcpp::get_logger("VescHardwareInterface"), "Starting VESC hardware interface");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VescHardwareInterface::stop() {
  RCLCPP_INFO(rclcpp::get_logger("VescHardwareInterface"), "Stopping VESC hardware interface");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VescHardwareInterface::read() {
  for (size_t i = 0; i < vesc_handlers_.size(); ++i) {
    const auto& state = vesc_handlers_[i]->getLastState();
    hw_velocities_[i] = state.rpm * (2.0 * M_PI / 60.0);  // RPM → rad/s
    hw_positions_[i] += hw_velocities_[i] * (1.0 / publish_rate_);
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VescHardwareInterface::write() {
  for (size_t i = 0; i < vesc_handlers_.size(); ++i) {
    double mps = cmd_velocities_[i] * wheel_radius_;
    vesc_handlers_[i]->sendSpeed(mps);  // sendSpeed ожидает м/с
  }
  return hardware_interface::return_type::OK;
}

}  // namespace vesc_nexus

// Макрос для регистрации интерфейса
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vesc_nexus::VescHardwareInterface,
  hardware_interface::SystemInterface)