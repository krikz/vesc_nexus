#include "vesc_nexus/vesc_system_hardware_interface.hpp"

#include <algorithm>
#include <cmath>
#include <initializer_list>
#include <limits>
#include <stdexcept>
#include <string>
#include <unordered_map>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <pluginlib/class_list_macros.hpp>

#include "vesc_nexus/message_translator.hpp"

namespace vesc_nexus {
namespace {

using hardware_interface::CallbackReturn;
using hardware_interface::return_type;
using Params = std::unordered_map<std::string, std::string>;

rclcpp::Logger logger() { return rclcpp::get_logger("VescSystemHardwareInterface"); }

/// Команды по модулю меньше этого считаются нулём (шум контроллера).
constexpr double kZeroVelocityThreshold = 1e-3;  // рад/с

const char* modeName(ControlMode mode) { return mode == ControlMode::RPM ? "rpm" : "duty"; }

[[noreturn]] void fail(const std::string& scope, const std::string& key, const std::string& why) {
  throw std::invalid_argument(scope + ": parameter " + key + " " + why);
}

const std::string& require(const Params& params, const std::string& key, const std::string& scope) {
  const auto it = params.find(key);
  if (it == params.end()) fail(scope, key, "is required");
  return it->second;
}

double toDouble(const std::string& text, const std::string& key, const std::string& scope) {
  try {
    return std::stod(text);
  } catch (const std::exception&) {
    fail(scope, key, "is not a number: " + text);
  }
}

int toInt(const std::string& text, const std::string& key, const std::string& scope) {
  try {
    return std::stoi(text);
  } catch (const std::exception&) {
    fail(scope, key, "is not an integer: " + text);
  }
}

/// Положительное конечное число; при отсутствии параметра — fallback (NaN означает «обязателен»).
double positiveDouble(const Params& params, const std::string& key, const std::string& scope, double fallback) {
  const auto it = params.find(key);
  if (it == params.end() && std::isnan(fallback)) fail(scope, key, "is required");
  const double value = (it == params.end()) ? fallback : toDouble(it->second, key, scope);
  if (!std::isfinite(value) || value <= 0.0) fail(scope, key, "must be a positive finite number");
  return value;
}

/// Параметры прежних версий, которые больше ни на что не влияют.
void warnIgnored(const Params& params, const std::string& scope, std::initializer_list<const char*> keys) {
  for (const char* key : keys) {
    if (params.count(key) != 0) {
      RCLCPP_WARN(logger(), "%s: parameter %s is no longer used and is ignored", scope.c_str(), key);
    }
  }
}

}  // namespace

// ─── Жизненный цикл ─────────────────────────────────────────────────────────

CallbackReturn VescSystemHardwareInterface::on_init(const hardware_interface::HardwareInfo& info) {
  if (SystemInterface::on_init(info) != CallbackReturn::SUCCESS) return CallbackReturn::ERROR;

  try {
    const Params& hw = info.hardware_parameters;
    const std::string scope = "<hardware>";
    warnIgnored(hw, scope, {"publish_rate", "wheel_radius"});

    const std::string can_name = require(hw, "can_interface", scope);

    ControlMode mode = ControlMode::DUTY;
    if (const auto it = hw.find("control_mode"); it != hw.end()) {
      if (it->second == "rpm" || it->second == "RPM") {
        mode = ControlMode::RPM;
      } else if (it->second != "duty" && it->second != "DUTY") {
        fail(scope, "control_mode", "must be duty or rpm, got " + it->second);
      }
    }

    const double gear_ratio = positiveDouble(hw, "gear_ratio", scope, 1.0);
    command_timeout_ = positiveDouble(hw, "command_timeout", scope, 0.5);

    double min_duty = 0.0;
    if (const auto it = hw.find("min_duty"); it != hw.end()) {
      min_duty = toDouble(it->second, "min_duty", scope);
      if (!std::isfinite(min_duty) || min_duty < 0.0 || min_duty >= 1.0) fail(scope, "min_duty", "must be in [0, 1)");
    }

    RCLCPP_INFO(logger(), "can_interface=%s control_mode=%s gear_ratio=%.3f min_duty=%.3f command_timeout=%.2fs",
                can_name.c_str(), modeName(mode), gear_ratio, min_duty, command_timeout_);

    can_ = std::make_unique<CanInterface>(can_name);
    can_->setReceiveCallback([this](const can_frame& frame) {
      const uint8_t sender = vescIdFromCanId(frame.can_id);
      for (auto& handler : handlers_) {
        if (handler->getCanId() == sender) {
          handler->processCanFrame(frame);
          return;
        }
      }
    });

    const size_t n = info.joints.size();
    if (n == 0) throw std::invalid_argument("no joints defined for VescSystemHardwareInterface");
    handlers_.reserve(n);
    hw_positions_.assign(n, 0.0);
    hw_velocities_.assign(n, 0.0);
    hw_efforts_.assign(n, 0.0);
    cmd_velocities_.assign(n, 0.0);
    last_nonzero_cmd_time_.assign(n, rclcpp::Time(0, 0, RCL_ROS_TIME));

    for (const auto& joint : info.joints) {
      const Params& jp = joint.parameters;
      const std::string jscope = "joint " + joint.name;
      warnIgnored(jp, jscope, {"min_erpm", "wheel_radius"});

      const int can_id = toInt(require(jp, "can_id", jscope), "can_id", jscope);
      if (can_id < 0 || can_id > 254) fail(jscope, "can_id", "must be in [0, 254]");

      const int poles = toInt(require(jp, "poles", jscope), "poles", jscope);
      if (poles < 2) fail(jscope, "poles", "must be >= 2");

      auto handler = std::make_unique<VescHandler>(static_cast<uint8_t>(can_id), joint.name, poles);
      handler->setSendFrameFunc([this](const can_frame& frame) { return can_->sendFrame(frame); });
      handler->setControlMode(mode);
      handler->setGearRatio(gear_ratio);
      handler->setMinDuty(min_duty);

      if (mode == ControlMode::DUTY) {
        // В режиме duty скорость задаётся через калибровку: duty = motor_rps / max_rps.
        handler->setMaxRps(positiveDouble(jp, "max_rps", jscope, std::numeric_limits<double>::quiet_NaN()));
      } else if (jp.count("max_rps") != 0) {
        RCLCPP_INFO(logger(), "%s: max_rps is not used in rpm mode", jscope.c_str());
      }

      RCLCPP_INFO(logger(), "%s: can_id=%d poles=%d (pole_pairs=%d) max_rps=%.2f", jscope.c_str(), can_id, poles,
                  handler->getPolePairs(), handler->getMaxRps());
      handlers_.push_back(std::move(handler));
    }
  } catch (const std::exception& e) {
    RCLCPP_FATAL(logger(), "Invalid ros2_control configuration: %s", e.what());
    return CallbackReturn::ERROR;
  }

  return CallbackReturn::SUCCESS;
}

CallbackReturn VescSystemHardwareInterface::on_configure(const rclcpp_lifecycle::State&) {
  if (!can_->open()) {
    RCLCPP_FATAL(logger(), "Failed to open CAN interface %s", can_->name().c_str());
    return CallbackReturn::ERROR;
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn VescSystemHardwareInterface::on_cleanup(const rclcpp_lifecycle::State&) {
  can_->close();
  return CallbackReturn::SUCCESS;
}

CallbackReturn VescSystemHardwareInterface::on_activate(const rclcpp_lifecycle::State&) {
  std::fill(cmd_velocities_.begin(), cmd_velocities_.end(), 0.0);
  std::fill(last_nonzero_cmd_time_.begin(), last_nonzero_cmd_time_.end(), rclcpp::Time(0, 0, RCL_ROS_TIME));
  motors_relaxed_ = false;
  RCLCPP_INFO(logger(), "Activated: %zu VESC on %s", handlers_.size(), can_->name().c_str());
  return CallbackReturn::SUCCESS;
}

CallbackReturn VescSystemHardwareInterface::on_deactivate(const rclcpp_lifecycle::State&) {
  stopAllMotors();
  return CallbackReturn::SUCCESS;
}

CallbackReturn VescSystemHardwareInterface::on_shutdown(const rclcpp_lifecycle::State&) {
  if (can_ && can_->isOpen()) {
    stopAllMotors();
    can_->close();
  }
  return CallbackReturn::SUCCESS;
}

CallbackReturn VescSystemHardwareInterface::on_error(const rclcpp_lifecycle::State&) {
  RCLCPP_ERROR(logger(), "Entering error state, closing CAN");
  if (can_) can_->close();
  return CallbackReturn::SUCCESS;
}

void VescSystemHardwareInterface::stopAllMotors() {
  for (auto& handler : handlers_) handler->sendVelocity(0.0);
}

// ─── Интерфейсы ─────────────────────────────────────────────────────────────

std::vector<hardware_interface::StateInterface> VescSystemHardwareInterface::export_state_interfaces() {
  std::vector<hardware_interface::StateInterface> interfaces;
  for (size_t i = 0; i < handlers_.size(); ++i) {
    const std::string& joint = info_.joints[i].name;
    interfaces.emplace_back(joint, hardware_interface::HW_IF_POSITION, &hw_positions_[i]);
    interfaces.emplace_back(joint, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]);
    interfaces.emplace_back(joint, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]);
  }
  interfaces.emplace_back("battery", "voltage", &hw_battery_voltage_);
  return interfaces;
}

std::vector<hardware_interface::CommandInterface> VescSystemHardwareInterface::export_command_interfaces() {
  std::vector<hardware_interface::CommandInterface> interfaces;
  for (size_t i = 0; i < handlers_.size(); ++i) {
    interfaces.emplace_back(info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &cmd_velocities_[i]);
  }
  return interfaces;
}

// ─── Цикл управления ────────────────────────────────────────────────────────

return_type VescSystemHardwareInterface::read(const rclcpp::Time&, const rclcpp::Duration&) {
  double min_voltage = std::numeric_limits<double>::infinity();

  for (size_t i = 0; i < handlers_.size(); ++i) {
    const auto& handler = handlers_[i];
    hw_velocities_[i] = finite_or(handler->getVelocityRadPerSec());
    hw_positions_[i] = finite_or(handler->getAccumulatedPosition());

    const auto state = handler->getLastState();
    hw_efforts_[i] = finite_or(state.current_motor);
    if (std::isfinite(state.voltage_input) && state.voltage_input > 0.0) {
      min_voltage = std::min(min_voltage, state.voltage_input);
    }
  }

  // Нули — это VESC, которые ещё не прислали STATUS_5; пока никто не прислал, держим прошлое значение.
  if (std::isfinite(min_voltage)) hw_battery_voltage_ = min_voltage;
  return return_type::OK;
}

return_type VescSystemHardwareInterface::write(const rclcpp::Time& time, const rclcpp::Duration&) {
  bool all_idle = true;

  for (size_t i = 0; i < handlers_.size(); ++i) {
    double& cmd = cmd_velocities_[i];
    if (!std::isfinite(cmd)) {
      RCLCPP_ERROR_ONCE(logger(), "[%s] non-finite velocity command, treating as zero",
                        handlers_[i]->getLabel().c_str());
      cmd = 0.0;
    }

    if (std::abs(cmd) > kZeroVelocityThreshold) {
      last_nonzero_cmd_time_[i] = time;
      handlers_[i]->sendVelocity(cmd);
      all_idle = false;
    } else if ((time - last_nonzero_cmd_time_[i]).seconds() < command_timeout_) {
      // Недавно двигались: шлём явный ноль, чтобы VESC тормозил, а не ждал своего таймаута.
      handlers_[i]->sendVelocity(0.0);
      all_idle = false;
    }
    // Иначе таймаут истёк: ничего не отправляем, VESC по своему таймауту отпускает мотор.
  }

  if (all_idle != motors_relaxed_) {
    motors_relaxed_ = all_idle;
    if (motors_relaxed_) {
      RCLCPP_INFO(logger(), "All motors relaxed after %.2fs without commands", command_timeout_);
    } else {
      RCLCPP_INFO(logger(), "Motors active (%.2fs command timeout armed)", command_timeout_);
    }
  }
  return return_type::OK;
}

}  // namespace vesc_nexus

PLUGINLIB_EXPORT_CLASS(vesc_nexus::VescSystemHardwareInterface, hardware_interface::SystemInterface)
