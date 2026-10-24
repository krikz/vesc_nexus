// Один VESC: преобразование скорости колеса в CAN-команду и разбор телеметрии.
#pragma once

#include <linux/can.h>

#include <chrono>
#include <cstdint>
#include <functional>
#include <mutex>
#include <string>

#include <vesc_msgs/msg/vesc_state.hpp>

namespace vesc_nexus {

enum class ControlMode {
  DUTY,  ///< Открытый контур: duty cycle. Скорость задаётся через калибровку max_rps.
  RPM    ///< Замкнутый контур: целевой ERPM, скорость держит PID внутри VESC.
};

/// processCanFrame() вызывается из потока приёма CAN, остальные методы — из потока controller_manager.
class VescHandler {
public:
  using SendFrameFunc = std::function<bool(const can_frame&)>;

  /// @param can_id  VESC ID на шине (0..254).
  /// @param label   Имя joint, используется в логах.
  /// @param poles   Число полюсов мотора; pole_pairs = max(1, poles / 2).
  VescHandler(uint8_t can_id, std::string label, int poles);

  void setSendFrameFunc(SendFrameFunc func);
  void setControlMode(ControlMode mode) { control_mode_ = mode; }

  /// Передаточное число редуктора: wheel_rps = motor_rps / gear_ratio. Невалидное значение игнорируется.
  void setGearRatio(double gear_ratio);
  /// Обороты мотора в секунду при duty = 100 % (только режим DUTY). Невалидное значение игнорируется.
  void setMaxRps(double max_rps);
  /// Минимальный duty для преодоления мёртвой зоны, 0..1 (только режим DUTY). Невалидное значение игнорируется.
  void setMinDuty(double min_duty);

  /// Разобрать входящий фрейм. Фреймы других VESC, 11-битные ID и неизвестные пакеты игнорируются.
  void processCanFrame(const can_frame& frame);

  /// Отправить скорость колеса (рад/с) в текущем режиме. NaN/Inf не отправляется, возвращает false.
  bool sendVelocity(double wheel_rad_s);

  uint8_t getCanId() const { return can_id_; }
  const std::string& getLabel() const { return label_; }
  int getPolePairs() const { return pole_pairs_; }
  ControlMode getControlMode() const { return control_mode_; }
  double getGearRatio() const { return gear_ratio_; }
  double getMaxRps() const { return max_rps_; }
  double getMinDuty() const { return min_duty_; }

  /// Снимок телеметрии (потокобезопасно). speed_rpm — механические обороты вала мотора.
  vesc_msgs::msg::VescState getLastState() const;
  /// Угловая скорость колеса, рад/с (с учётом gear_ratio).
  double getVelocityRadPerSec() const;
  /// Накопленный угол колеса, рад: интеграл скорости по реальным интервалам между STATUS-пакетами.
  double getAccumulatedPosition() const;

private:
  /// Обороты мотора в секунду для заданной скорости колеса.
  double motorRpsFor(double wheel_rad_s) const;
  double dutyFor(double wheel_rad_s) const;
  double erpmFor(double wheel_rad_s) const;
  /// Обновить скорость и позицию после CAN_PACKET_STATUS. Вызывать под state_mutex_.
  void updateMotion();

  const uint8_t can_id_;
  const std::string label_;
  const int pole_pairs_;

  SendFrameFunc send_frame_;
  ControlMode control_mode_{ControlMode::DUTY};
  double gear_ratio_{1.0};
  double max_rps_{15.0};
  double min_duty_{0.0};

  mutable std::mutex state_mutex_;
  vesc_msgs::msg::VescState state_;
  double velocity_rad_s_{0.0};
  double position_rad_{0.0};
  bool have_status_{false};
  std::chrono::steady_clock::time_point last_status_time_;
};

}  // namespace vesc_nexus
