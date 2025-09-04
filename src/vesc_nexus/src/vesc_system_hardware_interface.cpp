// src/vesc_system_hardware_interface.cpp
#include "vesc_nexus/vesc_system_hardware_interface.hpp"
#include <rclcpp/rclcpp.hpp>
#include <memory>

namespace vesc_nexus {

hardware_interface::CallbackReturn VescSystemHardwareInterface::on_init(
  const hardware_interface::HardwareInfo& info) {
  RCLCPP_INFO(rclcpp::get_logger("VescSystemHardwareInterface"), "Initializing hardware interface...");

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS) {
    return hardware_interface::CallbackReturn::ERROR;
  }
  
  info_ = &info;

  can_interface_name_ = info.hardware_parameters.at("can_interface");
  publish_rate_ = std::stod(info.hardware_parameters.at("publish_rate"));
  wheel_radius_ = std::stod(info.hardware_parameters.at("wheel_radius"));

  // Инициализация CAN
  can_interface_ = std::make_unique<CanInterface>(can_interface_name_);
  if (!can_interface_->open()) {
    RCLCPP_FATAL(rclcpp::get_logger("VescSystemHardwareInterface"), "Failed to open CAN interface: %s", can_interface_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Подписка на CAN-фреймы
  can_interface_->setReceiveCallback([this](const can_frame& frame) {
    uint8_t sender_id = frame.can_id & 0xFF;
    for (auto& handler : vesc_handlers_) {
      if (handler->getCanId() == sender_id) {
        handler->processCanFrame(frame);
        break;
      }
    }
  });

  // Создание VescHandler'ов из описания соединений
  for (const auto& joint : info.joints) {
    uint8_t can_id = static_cast<uint8_t>(std::stoi(joint.parameters.at("can_id")));
    double radius = wheel_radius_;
    if (joint.parameters.find("wheel_radius") != joint.parameters.end()) {
      radius = std::stod(joint.parameters.at("wheel_radius"));
    }
    int poles = std::stoi(joint.parameters.at("poles"));
    int64_t min_erpm = std::stoll(joint.parameters.at("min_erpm"));

    auto handler = std::make_shared<VescHandler>(
      can_id, joint.name, radius, poles, min_erpm, CommandLimits{});
    handler->setSendCanFunc([this](const auto& f) {
      return can_interface_->sendFrame(f);
    });
    vesc_handlers_.push_back(handler);

    // Инициализация буферов
    hw_positions_.push_back(0.0);
    hw_velocities_.push_back(0.0);
    hw_efforts_.push_back(0.0);
    cmd_velocities_.push_back(0.0);
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescSystemHardwareInterface::on_configure(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("VescSystemHardwareInterface"), "Configured.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescSystemHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("VescSystemHardwareInterface"), "Cleaned up.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescSystemHardwareInterface::on_activate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("VescSystemHardwareInterface"), "Activated. Motors ready.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescSystemHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("VescSystemHardwareInterface"), "Deactivated.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescSystemHardwareInterface::on_shutdown(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_INFO(rclcpp::get_logger("VescSystemHardwareInterface"), "Shutdown.");
  if (can_interface_) {
    can_interface_->close();
  }
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn VescSystemHardwareInterface::on_error(
  const rclcpp_lifecycle::State& /*previous_state*/) {
  RCLCPP_ERROR(rclcpp::get_logger("VescSystemHardwareInterface"), "Error occurred.");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> VescSystemHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> state_interfaces;
  for (size_t i = 0; i < vesc_handlers_.size(); ++i) {
    const auto & joint = info_->joints[i];  // info_ нужно сохранить в on_init
    state_interfaces.emplace_back(joint.name, "position", &hw_positions_[i]);
    state_interfaces.emplace_back(joint.name, "velocity", &hw_velocities_[i]);
    state_interfaces.emplace_back(joint.name, "effort", &hw_efforts_[i]);
  }
  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> VescSystemHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> command_interfaces;
  for (size_t i = 0; i < vesc_handlers_.size(); ++i) {
    const auto & joint = info_->joints[i];
    command_interfaces.emplace_back(joint.name, "velocity", &cmd_velocities_[i]);
  }
  return command_interfaces;
}

hardware_interface::return_type VescSystemHardwareInterface::read(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  for (size_t i = 0; i < vesc_handlers_.size(); ++i) {
    const auto& state = vesc_handlers_[i]->getLastState();
    double rpm = state.speed_rpm;
    hw_velocities_[i] = rpm * (2.0 * M_PI / 60.0);  // RPM → rad/s
    hw_positions_[i] += hw_velocities_[i] * (1.0 / publish_rate_);
    hw_efforts_[i] = state.current_motor;  // Пример: ток как "усилие"
  }
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type VescSystemHardwareInterface::write(
  const rclcpp::Time& /*time*/, const rclcpp::Duration& /*period*/) {
  for (size_t i = 0; i < vesc_handlers_.size(); ++i) {
    double linear_speed = cmd_velocities_[i] * wheel_radius_;  // rad/s → m/s
    vesc_handlers_[i]->sendSpeed(linear_speed);
  }
  return hardware_interface::return_type::OK;
}

}  // namespace vesc_nexus

// Регистрация через pluginlib
#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  vesc_nexus::VescSystemHardwareInterface,
  hardware_interface::SystemInterface)