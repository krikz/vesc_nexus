// Юнит-тесты VescHandler: пересчёт команд, разбор телеметрии, защита от NaN (без железа).
#include <gtest/gtest.h>

#include <chrono>
#include <cmath>
#include <limits>
#include <thread>
#include <vector>

#include "vesc_nexus/message_translator.hpp"
#include "vesc_nexus/vesc_handler.hpp"

namespace {

using vesc_nexus::ControlMode;
using vesc_nexus::VescHandler;

constexpr double kNaN = std::numeric_limits<double>::quiet_NaN();
constexpr double kInf = std::numeric_limits<double>::infinity();
constexpr double kTwoPi = 2.0 * M_PI;
constexpr uint8_t kId = 49;
constexpr int kPoles = 30;  // pole_pairs = 15

int32_t decodeInt32BE(const can_frame& frame) {
  return static_cast<int32_t>((static_cast<uint32_t>(frame.data[0]) << 24) |
                              (static_cast<uint32_t>(frame.data[1]) << 16) |
                              (static_cast<uint32_t>(frame.data[2]) << 8) | static_cast<uint32_t>(frame.data[3]));
}

can_frame makeStatusFrame(uint8_t vesc_id, int32_t erpm, int16_t current_x10 = 0, bool extended = true) {
  can_frame frame{};
  frame.can_id = (static_cast<canid_t>(vesc_nexus::CAN_PACKET_STATUS) << 8) | vesc_id;
  if (extended) frame.can_id |= CAN_EFF_FLAG;
  frame.can_dlc = 8;
  const auto u = static_cast<uint32_t>(erpm);
  frame.data[0] = (u >> 24) & 0xFF;
  frame.data[1] = (u >> 16) & 0xFF;
  frame.data[2] = (u >> 8) & 0xFF;
  frame.data[3] = u & 0xFF;
  const auto c = static_cast<uint16_t>(current_x10);
  frame.data[4] = (c >> 8) & 0xFF;
  frame.data[5] = c & 0xFF;
  return frame;
}

class VescHandlerTest : public ::testing::Test {
protected:
  void SetUp() override {
    handler_ = std::make_unique<VescHandler>(kId, "left_front_wheel_joint", kPoles);
    handler_->setSendFrameFunc([this](const can_frame& frame) {
      sent_.push_back(frame);
      return true;
    });
  }

  std::unique_ptr<VescHandler> handler_;
  std::vector<can_frame> sent_;
};

}  // namespace

// ─── Конструктор и сеттеры ──────────────────────────────────────────────────

TEST_F(VescHandlerTest, ConstructorDerivesPolePairs) {
  EXPECT_EQ(handler_->getCanId(), kId);
  EXPECT_EQ(handler_->getLabel(), "left_front_wheel_joint");
  EXPECT_EQ(handler_->getPolePairs(), 15);
  EXPECT_FALSE(handler_->getLastState().alive);
}

TEST_F(VescHandlerTest, InvalidPolesFallBackToOnePolePair) {
  VescHandler h(1, "x", 0);
  EXPECT_EQ(h.getPolePairs(), 1);
}

TEST_F(VescHandlerTest, SettersIgnoreInvalidValues) {
  handler_->setMaxRps(12.2);
  handler_->setGearRatio(2.17);
  handler_->setMinDuty(0.05);

  handler_->setMaxRps(kNaN);
  handler_->setMaxRps(0.0);
  handler_->setMaxRps(-1.0);
  handler_->setGearRatio(kInf);
  handler_->setGearRatio(0.0);
  handler_->setMinDuty(kNaN);
  handler_->setMinDuty(1.0);
  handler_->setMinDuty(-0.1);

  EXPECT_DOUBLE_EQ(handler_->getMaxRps(), 12.2);
  EXPECT_DOUBLE_EQ(handler_->getGearRatio(), 2.17);
  EXPECT_DOUBLE_EQ(handler_->getMinDuty(), 0.05);
}

// ─── Режим DUTY ─────────────────────────────────────────────────────────────

TEST_F(VescHandlerTest, DutyModeScalesByMaxRps) {
  handler_->setControlMode(ControlMode::DUTY);
  handler_->setMaxRps(10.0);  // duty 1.0 = 10 об/с мотора

  ASSERT_TRUE(handler_->sendVelocity(kTwoPi));  // 1 об/с колеса, редуктор 1:1
  ASSERT_EQ(sent_.size(), 1u);
  EXPECT_EQ(vesc_nexus::packetIdFromCanId(sent_[0].can_id), vesc_nexus::CAN_PACKET_SET_DUTY);
  EXPECT_EQ(vesc_nexus::vescIdFromCanId(sent_[0].can_id), kId);
  EXPECT_EQ(decodeInt32BE(sent_[0]), 10000);  // duty 0.1
}

TEST_F(VescHandlerTest, DutyModeAppliesGearRatio) {
  handler_->setMaxRps(10.0);
  handler_->setGearRatio(2.0);  // мотор крутится в 2 раза быстрее колеса
  handler_->sendVelocity(kTwoPi);
  EXPECT_EQ(decodeInt32BE(sent_.at(0)), 20000);  // duty 0.2
}

TEST_F(VescHandlerTest, DutyModeDeadzoneCompensation) {
  handler_->setMaxRps(10.0);
  handler_->setMinDuty(0.05);

  handler_->sendVelocity(kTwoPi);   // duty 0.1 → 0.05 + 0.1 × 0.95 = 0.145
  handler_->sendVelocity(-kTwoPi);  // симметрично
  handler_->sendVelocity(0.0);      // ноль остаётся нулём
  ASSERT_EQ(sent_.size(), 3u);
  EXPECT_EQ(decodeInt32BE(sent_[0]), 14500);
  EXPECT_EQ(decodeInt32BE(sent_[1]), -14500);
  EXPECT_EQ(decodeInt32BE(sent_[2]), 0);
}

TEST_F(VescHandlerTest, DutyModeClampsToFullDuty) {
  handler_->setMaxRps(10.0);
  handler_->sendVelocity(1000.0);
  EXPECT_EQ(decodeInt32BE(sent_.at(0)), 100000);
}

// ─── Режим RPM ──────────────────────────────────────────────────────────────

TEST_F(VescHandlerTest, RpmModeConvertsWheelVelocityToErpm) {
  handler_->setControlMode(ControlMode::RPM);
  handler_->setGearRatio(2.0);

  handler_->sendVelocity(kTwoPi);  // 1 об/с колеса → 2 об/с мотора → 2 × 15 × 60 = 1800 ERPM
  ASSERT_EQ(sent_.size(), 1u);
  EXPECT_EQ(vesc_nexus::packetIdFromCanId(sent_[0].can_id), vesc_nexus::CAN_PACKET_SET_RPM);
  EXPECT_EQ(decodeInt32BE(sent_[0]), 1800);
}

TEST_F(VescHandlerTest, RpmModeIgnoresMaxRpsAndMinDuty) {
  handler_->setControlMode(ControlMode::RPM);
  handler_->setMaxRps(1.0);
  handler_->setMinDuty(0.5);
  handler_->sendVelocity(-kTwoPi);
  EXPECT_EQ(decodeInt32BE(sent_.at(0)), -900);
}

// ─── Защита от NaN/Inf ──────────────────────────────────────────────────────

TEST_F(VescHandlerTest, NonFiniteVelocityIsNotSentInEitherMode) {
  handler_->setControlMode(ControlMode::DUTY);
  EXPECT_FALSE(handler_->sendVelocity(kNaN));
  EXPECT_FALSE(handler_->sendVelocity(kInf));
  handler_->setControlMode(ControlMode::RPM);
  EXPECT_FALSE(handler_->sendVelocity(kNaN));
  EXPECT_FALSE(handler_->sendVelocity(-kInf));
  EXPECT_TRUE(sent_.empty());
}

TEST_F(VescHandlerTest, WithoutSendFunctionNothingIsSent) {
  VescHandler h(1, "x", kPoles);
  EXPECT_FALSE(h.sendVelocity(1.0));
}

// ─── Телеметрия ─────────────────────────────────────────────────────────────

TEST_F(VescHandlerTest, StatusFrameUpdatesSpeedAndVelocity) {
  handler_->processCanFrame(makeStatusFrame(kId, 1500, 87));  // 1500 ERPM / 15 = 100 об/мин, 8.7 A

  const auto state = handler_->getLastState();
  EXPECT_TRUE(state.alive);
  EXPECT_DOUBLE_EQ(state.erpm, 1500.0);
  EXPECT_DOUBLE_EQ(state.speed_rpm, 100.0);
  EXPECT_DOUBLE_EQ(state.current_motor, 8.7);
  EXPECT_NEAR(handler_->getVelocityRadPerSec(), 100.0 * kTwoPi / 60.0, 1e-9);
}

TEST_F(VescHandlerTest, VelocityIsReducedByGearRatio) {
  handler_->setGearRatio(2.0);
  handler_->processCanFrame(makeStatusFrame(kId, 1500));
  EXPECT_NEAR(handler_->getVelocityRadPerSec(), 100.0 * kTwoPi / 60.0 / 2.0, 1e-9);
}

TEST_F(VescHandlerTest, FramesFromOtherVescOrStandardIdAreIgnored) {
  handler_->processCanFrame(makeStatusFrame(kId + 1, 1500));
  handler_->processCanFrame(makeStatusFrame(kId, 1500, 0, /*extended=*/false));
  EXPECT_FALSE(handler_->getLastState().alive);
  EXPECT_DOUBLE_EQ(handler_->getVelocityRadPerSec(), 0.0);
}

TEST_F(VescHandlerTest, UnknownPacketAndShortFrameAreIgnored) {
  can_frame unknown = makeStatusFrame(kId, 1500);
  unknown.can_id = ((static_cast<canid_t>(vesc_nexus::CAN_PACKET_STATUS_6) << 8) | kId) | CAN_EFF_FLAG;
  handler_->processCanFrame(unknown);

  can_frame short_frame = makeStatusFrame(kId, 1500);
  short_frame.can_dlc = 3;
  handler_->processCanFrame(short_frame);

  EXPECT_FALSE(handler_->getLastState().alive);
}

TEST_F(VescHandlerTest, PositionIntegratesBetweenStatusFrames) {
  handler_->processCanFrame(makeStatusFrame(kId, 1500));  // первый кадр только фиксирует время
  EXPECT_DOUBLE_EQ(handler_->getAccumulatedPosition(), 0.0);

  std::this_thread::sleep_for(std::chrono::milliseconds(20));
  handler_->processCanFrame(makeStatusFrame(kId, 1500));

  const double omega = 100.0 * kTwoPi / 60.0;
  const double pos = handler_->getAccumulatedPosition();
  EXPECT_GT(pos, omega * 0.015);
  EXPECT_LT(pos, omega * 0.5);
}

TEST_F(VescHandlerTest, ExtremeErpmStaysFinite) {
  handler_->processCanFrame(makeStatusFrame(kId, std::numeric_limits<int32_t>::max()));
  std::this_thread::sleep_for(std::chrono::milliseconds(2));
  handler_->processCanFrame(makeStatusFrame(kId, std::numeric_limits<int32_t>::min()));
  EXPECT_TRUE(std::isfinite(handler_->getVelocityRadPerSec()));
  EXPECT_TRUE(std::isfinite(handler_->getAccumulatedPosition()));
}

int main(int argc, char** argv) {
  ::testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
