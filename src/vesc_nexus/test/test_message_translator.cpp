// Юнит-тесты кодирования команд и разбора статусов VESC (без железа).
#include <gtest/gtest.h>

#include <cstring>
#include <limits>

#include "vesc_nexus/message_translator.hpp"

namespace {

using vesc_nexus::CAN_PACKET_SET_DUTY;
using vesc_nexus::CAN_PACKET_SET_RPM;
using vesc_nexus::CAN_PACKET_STATUS;
using vesc_nexus::CAN_PACKET_STATUS_2;
using vesc_nexus::CAN_PACKET_STATUS_4;
using vesc_nexus::CAN_PACKET_STATUS_5;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();

int32_t decodeInt32BE(const can_frame& frame) {
  return static_cast<int32_t>((static_cast<uint32_t>(frame.data[0]) << 24) |
                              (static_cast<uint32_t>(frame.data[1]) << 16) |
                              (static_cast<uint32_t>(frame.data[2]) << 8) | static_cast<uint32_t>(frame.data[3]));
}

void putInt32BE(uint8_t* p, int32_t value) {
  const auto u = static_cast<uint32_t>(value);
  p[0] = (u >> 24) & 0xFF;
  p[1] = (u >> 16) & 0xFF;
  p[2] = (u >> 8) & 0xFF;
  p[3] = u & 0xFF;
}

void putInt16BE(uint8_t* p, int16_t value) {
  const auto u = static_cast<uint16_t>(value);
  p[0] = (u >> 8) & 0xFF;
  p[1] = u & 0xFF;
}

can_frame makeFrame(uint8_t packet_id, uint8_t vesc_id, uint8_t dlc) {
  can_frame frame{};
  frame.can_id = ((static_cast<canid_t>(packet_id) << 8) | vesc_id) | CAN_EFF_FLAG;
  frame.can_dlc = dlc;
  return frame;
}

}  // namespace

// ─── Кодирование команд ─────────────────────────────────────────────────────

TEST(SetDutyFrame, HeaderIsExtendedIdWithPacketAndVescId) {
  const auto frame = vesc_nexus::createSetDutyCycleFrame(49, 0.0);
  EXPECT_TRUE(frame.can_id & CAN_EFF_FLAG);
  EXPECT_EQ(vesc_nexus::vescIdFromCanId(frame.can_id), 49);
  EXPECT_EQ(vesc_nexus::packetIdFromCanId(frame.can_id), CAN_PACKET_SET_DUTY);
  EXPECT_EQ(frame.can_dlc, 4u);
}

TEST(SetDutyFrame, ScalesBy100000AndClamps) {
  EXPECT_EQ(decodeInt32BE(vesc_nexus::createSetDutyCycleFrame(1, 0.5)), 50000);
  EXPECT_EQ(decodeInt32BE(vesc_nexus::createSetDutyCycleFrame(1, -0.25)), -25000);
  EXPECT_EQ(decodeInt32BE(vesc_nexus::createSetDutyCycleFrame(1, 1.0)), 100000);
  EXPECT_EQ(decodeInt32BE(vesc_nexus::createSetDutyCycleFrame(1, 5.0)), 100000);
  EXPECT_EQ(decodeInt32BE(vesc_nexus::createSetDutyCycleFrame(1, -5.0)), -100000);
}

TEST(SetDutyFrame, NonFiniteBecomesZero) {
  EXPECT_EQ(decodeInt32BE(vesc_nexus::createSetDutyCycleFrame(1, kNaN)), 0);
  EXPECT_EQ(decodeInt32BE(vesc_nexus::createSetDutyCycleFrame(1, kInf)), 0);
  EXPECT_EQ(decodeInt32BE(vesc_nexus::createSetDutyCycleFrame(1, -kInf)), 0);
}

TEST(SetErpmFrame, HeaderAndValue) {
  const auto frame = vesc_nexus::createSetErpmFrame(124, 1800.0);
  EXPECT_EQ(vesc_nexus::vescIdFromCanId(frame.can_id), 124);
  EXPECT_EQ(vesc_nexus::packetIdFromCanId(frame.can_id), CAN_PACKET_SET_RPM);
  EXPECT_EQ(frame.can_dlc, 4u);
  EXPECT_EQ(decodeInt32BE(frame), 1800);
  EXPECT_EQ(decodeInt32BE(vesc_nexus::createSetErpmFrame(1, -750.0)), -750);
}

TEST(SetErpmFrame, ClampsToLimitAndSanitizes) {
  EXPECT_EQ(decodeInt32BE(vesc_nexus::createSetErpmFrame(1, 1e6)), 23250);
  EXPECT_EQ(decodeInt32BE(vesc_nexus::createSetErpmFrame(1, -1e6)), -23250);
  EXPECT_EQ(decodeInt32BE(vesc_nexus::createSetErpmFrame(1, kNaN)), 0);
  EXPECT_EQ(decodeInt32BE(vesc_nexus::createSetErpmFrame(1, kInf)), 0);
}

// ─── Разбор статусов ────────────────────────────────────────────────────────

TEST(ParseStatus, DecodesErpmCurrentDuty) {
  auto frame = makeFrame(CAN_PACKET_STATUS, 49, 8);
  putInt32BE(&frame.data[0], -15000);  // ERPM
  putInt16BE(&frame.data[4], 123);     // 12.3 A
  putInt16BE(&frame.data[6], -456);    // -0.456

  vesc_msgs::msg::VescState state;
  ASSERT_TRUE(vesc_nexus::parseStatusPacket(frame, state));
  EXPECT_DOUBLE_EQ(state.erpm, -15000.0);
  EXPECT_DOUBLE_EQ(state.current_motor, 12.3);
  EXPECT_DOUBLE_EQ(state.duty_cycle, -0.456);
}

TEST(ParseStatus, ShortFrameIsRejectedAndStateUntouched) {
  auto frame = makeFrame(CAN_PACKET_STATUS, 49, 4);
  vesc_msgs::msg::VescState state;
  state.erpm = 42.0;
  EXPECT_FALSE(vesc_nexus::parseStatusPacket(frame, state));
  EXPECT_DOUBLE_EQ(state.erpm, 42.0);
}

TEST(ParseStatus2, DecodesAmpereHours) {
  auto frame = makeFrame(CAN_PACKET_STATUS_2, 1, 8);
  putInt32BE(&frame.data[0], 12345);  // 1.2345 Ah
  putInt32BE(&frame.data[4], 678);    // 0.0678 Ah
  vesc_msgs::msg::VescState state;
  ASSERT_TRUE(vesc_nexus::parseStatus2Packet(frame, state));
  EXPECT_DOUBLE_EQ(state.charge_drawn, 1.2345);
  EXPECT_DOUBLE_EQ(state.charge_regen, 0.0678);
}

TEST(ParseStatus4, DecodesTemperaturesCurrentAndPidPos) {
  auto frame = makeFrame(CAN_PACKET_STATUS_4, 1, 8);
  putInt16BE(&frame.data[0], 415);  // 41.5 °C FET
  putInt16BE(&frame.data[2], 380);  // 38.0 °C motor
  putInt16BE(&frame.data[4], -25);  // -2.5 A input
  putInt16BE(&frame.data[6], 9000); // 180.0 deg
  vesc_msgs::msg::VescState state;
  ASSERT_TRUE(vesc_nexus::parseStatus4Packet(frame, state));
  EXPECT_DOUBLE_EQ(state.temp_controller, 41.5);
  EXPECT_DOUBLE_EQ(state.temp_motor, 38.0);
  EXPECT_DOUBLE_EQ(state.current_input, -2.5);
  EXPECT_DOUBLE_EQ(state.pid_pos_now, 180.0);
}

TEST(ParseStatus5, DecodesTachometerAndVoltage) {
  auto frame = makeFrame(CAN_PACKET_STATUS_5, 1, 8);
  putInt32BE(&frame.data[0], 600);  // 600 шагов = 100 эл. оборотов
  putInt16BE(&frame.data[4], 482);  // 48.2 V
  vesc_msgs::msg::VescState state;
  ASSERT_TRUE(vesc_nexus::parseStatus5Packet(frame, state));
  EXPECT_EQ(state.tachometer, 100u);
  EXPECT_DOUBLE_EQ(state.voltage_input, 48.2);
}

TEST(FiniteOr, ReturnsFallbackForNonFinite) {
  EXPECT_DOUBLE_EQ(vesc_nexus::finite_or(kNaN, 3.5), 3.5);
  EXPECT_DOUBLE_EQ(vesc_nexus::finite_or(kInf, 3.5), 3.5);
  EXPECT_DOUBLE_EQ(vesc_nexus::finite_or(-kInf, 3.5), 3.5);
  EXPECT_DOUBLE_EQ(vesc_nexus::finite_or(2.0, 3.5), 2.0);
  EXPECT_DOUBLE_EQ(vesc_nexus::finite_or(kNaN), 0.0);
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
