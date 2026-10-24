// Протокол VESC по CAN: кодирование команд и разбор статусных пакетов.
// Источник: прошивка bldc (comm_can.c, datatypes.h — CAN_PACKET_ID).
#pragma once

#include <linux/can.h>

#include <cmath>
#include <cstdint>

#include <vesc_msgs/msg/vesc_state.hpp>

namespace vesc_nexus {

/// Возвращает value, если оно конечно (не NaN/Inf), иначе fallback.
inline double finite_or(double value, double fallback = 0.0) {
  return std::isfinite(value) ? value : fallback;
}

/// Идентификаторы пакетов VESC. CAN ID фрейма = (packet_id << 8) | vesc_id, 29-битный (EFF).
enum CanPacketId : uint8_t {
  CAN_PACKET_SET_DUTY = 0,
  CAN_PACKET_SET_CURRENT = 1,
  CAN_PACKET_SET_CURRENT_BRAKE = 2,
  CAN_PACKET_SET_RPM = 3,
  CAN_PACKET_SET_POS = 4,
  CAN_PACKET_STATUS = 9,     ///< ERPM, ток мотора, duty cycle
  CAN_PACKET_STATUS_2 = 14,  ///< Ah израсходовано / Ah заряжено
  CAN_PACKET_STATUS_3 = 15,  ///< Wh израсходовано / Wh заряжено (не разбирается)
  CAN_PACKET_STATUS_4 = 16,  ///< Температура FET и мотора, входной ток, PID-позиция
  CAN_PACKET_STATUS_5 = 27,  ///< Тахометр, входное напряжение
  CAN_PACKET_STATUS_6 = 58   ///< ADC1-3, PPM (не разбирается)
};

/// Пределы команд, которые уходят на шину.
constexpr double kDutyLimit = 1.0;      ///< ±100 %
constexpr double kErpmLimit = 23250.0;  ///< ±ERPM, защитный потолок (унаследован от vesc_driver)

inline uint8_t vescIdFromCanId(canid_t can_id) { return static_cast<uint8_t>(can_id & 0xFF); }
inline uint8_t packetIdFromCanId(canid_t can_id) { return static_cast<uint8_t>((can_id >> 8) & 0xFF); }

/// CAN_PACKET_SET_DUTY: duty ∈ [-1, 1], на шине умножается на 100000. NaN/Inf → 0.
can_frame createSetDutyCycleFrame(uint8_t vesc_id, double duty);

/// CAN_PACKET_SET_RPM: электрические обороты в минуту, клампится в ±kErpmLimit. NaN/Inf → 0.
can_frame createSetErpmFrame(uint8_t vesc_id, double erpm);

/// Разбор статусных пакетов. Возвращают false (и не трогают state), если фрейм короче ожидаемого.
bool parseStatusPacket(const can_frame& frame, vesc_msgs::msg::VescState& state);   ///< erpm, current_motor, duty_cycle
bool parseStatus2Packet(const can_frame& frame, vesc_msgs::msg::VescState& state);  ///< charge_drawn, charge_regen
bool parseStatus4Packet(const can_frame& frame, vesc_msgs::msg::VescState& state);  ///< temp_controller, temp_motor, current_input, pid_pos_now
bool parseStatus5Packet(const can_frame& frame, vesc_msgs::msg::VescState& state);  ///< tachometer, voltage_input

}  // namespace vesc_nexus
