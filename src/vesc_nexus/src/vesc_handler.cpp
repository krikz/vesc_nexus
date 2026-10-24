#include "vesc_nexus/vesc_handler.hpp"

#include <algorithm>
#include <cmath>
#include <utility>

#include <rclcpp/rclcpp.hpp>

#include "vesc_nexus/message_translator.hpp"

namespace vesc_nexus {
namespace {

rclcpp::Logger logger() { return rclcpp::get_logger("VescHandler"); }

/// Интервалы длиннее этого (пауза, потеря связи) в позицию не интегрируются.
constexpr double kMaxIntegrationDt = 0.5;  // с

constexpr double kTwoPi = 2.0 * M_PI;

}  // namespace

VescHandler::VescHandler(uint8_t can_id, std::string label, int poles)
    : can_id_(can_id), label_(std::move(label)), pole_pairs_(std::max(1, poles / 2)) {
  state_.label = label_;
  state_.alive = false;
  if (poles < 2) {
    RCLCPP_WARN(logger(), "[%s] poles=%d is invalid, using pole_pairs=1", label_.c_str(), poles);
  }
}

void VescHandler::setSendFrameFunc(SendFrameFunc func) { send_frame_ = std::move(func); }

void VescHandler::setGearRatio(double gear_ratio) {
  if (!std::isfinite(gear_ratio) || gear_ratio <= 0.0) {
    RCLCPP_WARN(logger(), "[%s] ignoring invalid gear_ratio=%g, keeping %g", label_.c_str(), gear_ratio, gear_ratio_);
    return;
  }
  gear_ratio_ = gear_ratio;
}

void VescHandler::setMaxRps(double max_rps) {
  if (!std::isfinite(max_rps) || max_rps <= 0.0) {
    RCLCPP_WARN(logger(), "[%s] ignoring invalid max_rps=%g, keeping %g", label_.c_str(), max_rps, max_rps_);
    return;
  }
  max_rps_ = max_rps;
}

void VescHandler::setMinDuty(double min_duty) {
  if (!std::isfinite(min_duty) || min_duty < 0.0 || min_duty >= 1.0) {
    RCLCPP_WARN(logger(), "[%s] ignoring invalid min_duty=%g, keeping %g", label_.c_str(), min_duty, min_duty_);
    return;
  }
  min_duty_ = min_duty;
}

// ─── Команды ────────────────────────────────────────────────────────────────

double VescHandler::motorRpsFor(double wheel_rad_s) const { return wheel_rad_s / kTwoPi * gear_ratio_; }

double VescHandler::dutyFor(double wheel_rad_s) const {
  double duty = std::clamp(motorRpsFor(wheel_rad_s) / max_rps_, -kDutyLimit, kDutyLimit);
  // Компенсация мёртвой зоны: |duty| из (0, 1] отображается в [min_duty, 1], ноль остаётся нулём.
  if (min_duty_ > 0.0 && duty != 0.0) {
    duty = std::copysign(min_duty_ + std::abs(duty) * (1.0 - min_duty_), duty);
  }
  return duty;
}

double VescHandler::erpmFor(double wheel_rad_s) const { return motorRpsFor(wheel_rad_s) * pole_pairs_ * 60.0; }

bool VescHandler::sendVelocity(double wheel_rad_s) {
  if (!send_frame_) return false;
  if (!std::isfinite(wheel_rad_s)) {
    RCLCPP_ERROR_ONCE(logger(), "[%s] non-finite velocity command (%g), not sending", label_.c_str(), wheel_rad_s);
    return false;
  }

  const can_frame frame = (control_mode_ == ControlMode::RPM)
                              ? createSetErpmFrame(can_id_, erpmFor(wheel_rad_s))
                              : createSetDutyCycleFrame(can_id_, dutyFor(wheel_rad_s));
  return send_frame_(frame);
}

// ─── Телеметрия ─────────────────────────────────────────────────────────────

void VescHandler::processCanFrame(const can_frame& frame) {
  if (!(frame.can_id & CAN_EFF_FLAG)) return;  // VESC использует только 29-битные ID
  if (vescIdFromCanId(frame.can_id) != can_id_) return;

  std::lock_guard<std::mutex> lock(state_mutex_);
  switch (packetIdFromCanId(frame.can_id)) {
    case CAN_PACKET_STATUS:
      if (!parseStatusPacket(frame, state_)) return;
      updateMotion();
      break;
    case CAN_PACKET_STATUS_2:
      if (!parseStatus2Packet(frame, state_)) return;
      break;
    case CAN_PACKET_STATUS_4:
      if (!parseStatus4Packet(frame, state_)) return;
      break;
    case CAN_PACKET_STATUS_5:
      if (!parseStatus5Packet(frame, state_)) return;
      break;
    default:
      return;
  }
  state_.alive = true;
}

void VescHandler::updateMotion() {
  const auto now = std::chrono::steady_clock::now();

  // ERPM → механические об/мин вала мотора → рад/с колеса.
  state_.speed_rpm = finite_or(state_.erpm / pole_pairs_);
  velocity_rad_s_ = finite_or(state_.speed_rpm * kTwoPi / 60.0 / gear_ratio_);

  if (have_status_) {
    const double dt = std::chrono::duration<double>(now - last_status_time_).count();
    if (dt > 0.0 && dt < kMaxIntegrationDt) {
      position_rad_ = finite_or(position_rad_ + velocity_rad_s_ * dt);
    }
  }
  last_status_time_ = now;
  have_status_ = true;
}

vesc_msgs::msg::VescState VescHandler::getLastState() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return state_;
}

double VescHandler::getVelocityRadPerSec() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return velocity_rad_s_;
}

double VescHandler::getAccumulatedPosition() const {
  std::lock_guard<std::mutex> lock(state_mutex_);
  return position_rad_;
}

}  // namespace vesc_nexus
