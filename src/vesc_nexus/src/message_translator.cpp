#include "vesc_nexus/message_translator.hpp"

#include <algorithm>

namespace vesc_nexus {
namespace {

canid_t makeCanId(CanPacketId packet, uint8_t vesc_id) {
  return ((static_cast<canid_t>(packet) << 8) | vesc_id) | CAN_EFF_FLAG;
}

can_frame makeInt32Frame(CanPacketId packet, uint8_t vesc_id, int32_t value) {
  const auto u = static_cast<uint32_t>(value);
  can_frame frame{};
  frame.can_id = makeCanId(packet, vesc_id);
  frame.can_dlc = 4;
  frame.data[0] = static_cast<uint8_t>((u >> 24) & 0xFF);
  frame.data[1] = static_cast<uint8_t>((u >> 16) & 0xFF);
  frame.data[2] = static_cast<uint8_t>((u >> 8) & 0xFF);
  frame.data[3] = static_cast<uint8_t>(u & 0xFF);
  return frame;
}

int32_t readInt32BE(const uint8_t* p) {
  return static_cast<int32_t>((static_cast<uint32_t>(p[0]) << 24) | (static_cast<uint32_t>(p[1]) << 16) |
                              (static_cast<uint32_t>(p[2]) << 8) | static_cast<uint32_t>(p[3]));
}

int16_t readInt16BE(const uint8_t* p) {
  return static_cast<int16_t>((static_cast<uint16_t>(p[0]) << 8) | static_cast<uint16_t>(p[1]));
}

}  // namespace

can_frame createSetDutyCycleFrame(uint8_t vesc_id, double duty) {
  const double safe = std::clamp(finite_or(duty), -kDutyLimit, kDutyLimit);
  return makeInt32Frame(CAN_PACKET_SET_DUTY, vesc_id, static_cast<int32_t>(safe * 100000.0));
}

can_frame createSetErpmFrame(uint8_t vesc_id, double erpm) {
  const double safe = std::clamp(finite_or(erpm), -kErpmLimit, kErpmLimit);
  return makeInt32Frame(CAN_PACKET_SET_RPM, vesc_id, static_cast<int32_t>(safe));
}

// CAN_PACKET_STATUS: int32 erpm | int16 current×10 | int16 duty×1000
bool parseStatusPacket(const can_frame& frame, vesc_msgs::msg::VescState& state) {
  if (frame.can_dlc < 8) return false;
  state.erpm = readInt32BE(&frame.data[0]);
  state.current_motor = readInt16BE(&frame.data[4]) / 10.0;
  state.duty_cycle = readInt16BE(&frame.data[6]) / 1000.0;
  return true;
}

// CAN_PACKET_STATUS_2: int32 Ah×1e4 | int32 Ah_charged×1e4
bool parseStatus2Packet(const can_frame& frame, vesc_msgs::msg::VescState& state) {
  if (frame.can_dlc < 8) return false;
  state.charge_drawn = readInt32BE(&frame.data[0]) / 10000.0;
  state.charge_regen = readInt32BE(&frame.data[4]) / 10000.0;
  return true;
}

// CAN_PACKET_STATUS_4: int16 temp_fet×10 | int16 temp_motor×10 | int16 current_in×10 | int16 pid_pos×50
bool parseStatus4Packet(const can_frame& frame, vesc_msgs::msg::VescState& state) {
  if (frame.can_dlc < 8) return false;
  state.temp_controller = readInt16BE(&frame.data[0]) / 10.0;
  state.temp_motor = readInt16BE(&frame.data[2]) / 10.0;
  state.current_input = readInt16BE(&frame.data[4]) / 10.0;
  state.pid_pos_now = readInt16BE(&frame.data[6]) / 50.0;
  return true;
}

// CAN_PACKET_STATUS_5: int32 tachometer (шаги; 6 шагов = 1 электрический оборот) | int16 voltage_in×10
bool parseStatus5Packet(const can_frame& frame, vesc_msgs::msg::VescState& state) {
  if (frame.can_dlc < 6) return false;
  state.tachometer = static_cast<uint32_t>(readInt32BE(&frame.data[0]) / 6);
  state.voltage_input = readInt16BE(&frame.data[4]) / 10.0;
  return true;
}

}  // namespace vesc_nexus
