// test_message_translator.cpp
// Unit tests for message_translator NaN/Infinity guards (FA-3).
#include <gtest/gtest.h>
#include <cmath>
#include <limits>

#include "vesc_nexus/message_translator.hpp"

namespace {

// Декодирует int32 big-endian из первых 4 байт фрейма.
int32_t decodeInt32BE(const can_frame& frame) {
    int32_t value = 0;
    value |= (static_cast<int32_t>(frame.data[0]) & 0xFF) << 24;
    value |= (static_cast<int32_t>(frame.data[1]) & 0xFF) << 16;
    value |= (static_cast<int32_t>(frame.data[2]) & 0xFF) << 8;
    value |= (static_cast<int32_t>(frame.data[3]) & 0xFF);
    return value;
}

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

}  // namespace

TEST(MessageTranslator, createSetDutyCycleFrameNaNBecomesZero) {
    auto frame = vesc_nexus::createSetDutyCycleFrame(42, kNaN);
    EXPECT_EQ(frame.can_dlc, 4u);
    EXPECT_EQ(decodeInt32BE(frame), 0);
}

TEST(MessageTranslator, createSetDutyCycleFrameInfBecomesZero) {
    auto frame = vesc_nexus::createSetDutyCycleFrame(42, kInf);
    EXPECT_EQ(decodeInt32BE(frame), 0);
}

TEST(MessageTranslator, createSetDutyCycleFrameNegativeInfBecomesZero) {
    auto frame = vesc_nexus::createSetDutyCycleFrame(42, -kInf);
    EXPECT_EQ(decodeInt32BE(frame), 0);
}

TEST(MessageTranslator, createSetDutyCycleFrameClampsFinite) {
    // 1.0 → 100000 (максимум)
    auto frame = vesc_nexus::createSetDutyCycleFrame(42, 1.0);
    EXPECT_EQ(decodeInt32BE(frame), 100000);
    // -1.0 → -100000 (минимум)
    frame = vesc_nexus::createSetDutyCycleFrame(42, -1.0);
    EXPECT_EQ(decodeInt32BE(frame), -100000);
    // 0.5 → 50000
    frame = vesc_nexus::createSetDutyCycleFrame(42, 0.5);
    EXPECT_EQ(decodeInt32BE(frame), 50000);
    // Превышение максимума → клампится, а не ломается
    frame = vesc_nexus::createSetDutyCycleFrame(42, 5.0);
    EXPECT_EQ(decodeInt32BE(frame), 100000);
}

TEST(MessageTranslator, createSetSpeedFrameNaNBecomesZero) {
    auto frame = vesc_nexus::createSetSpeedFrame(7, kNaN);
    EXPECT_EQ(frame.can_dlc, 4u);
    EXPECT_EQ(decodeInt32BE(frame), 0);
}

TEST(MessageTranslator, createSetSpeedFrameInfBecomesZero) {
    auto frame = vesc_nexus::createSetSpeedFrame(7, kInf);
    EXPECT_EQ(decodeInt32BE(frame), 0);
}

TEST(MessageTranslator, createSetSpeedFrameClampsFinite) {
    auto frame = vesc_nexus::createSetSpeedFrame(7, 23250.0);
    EXPECT_EQ(decodeInt32BE(frame), 23250);
    frame = vesc_nexus::createSetSpeedFrame(7, 100000.0);
    EXPECT_EQ(decodeInt32BE(frame), 23250);
    frame = vesc_nexus::createSetSpeedFrame(7, -100000.0);
    EXPECT_EQ(decodeInt32BE(frame), -23250);
}

TEST(MessageTranslator, createSetCurrentFrameNaNBecomesZero) {
    auto frame = vesc_nexus::createSetCurrentFrame(7, kNaN);
    EXPECT_EQ(decodeInt32BE(frame), 0);
}

TEST(MessageTranslator, createSetBrakeFrameNaNBecomesZero) {
    auto frame = vesc_nexus::createSetBrakeFrame(7, kNaN);
    EXPECT_EQ(decodeInt32BE(frame), 0);
}

TEST(MessageTranslator, createSetPositionFrameNaNBecomesZero) {
    auto frame = vesc_nexus::createSetPositionFrame(7, kNaN);
    EXPECT_EQ(decodeInt32BE(frame), 0);
}

TEST(MessageTranslator, finiteOrReturnsFallbackForNaN) {
    EXPECT_EQ(vesc_nexus::finite_or(kNaN, 3.5), 3.5);
    EXPECT_EQ(vesc_nexus::finite_or(kInf, 3.5), 3.5);
    EXPECT_EQ(vesc_nexus::finite_or(-kInf, 3.5), 3.5);
    EXPECT_EQ(vesc_nexus::finite_or(2.0, 3.5), 2.0);
    EXPECT_EQ(vesc_nexus::finite_or(0.0), 0.0);
}

TEST(MessageTranslator, parseStatusPacketCorruptedFrameNoCrash) {
    // Короткий фрейм (can_dlc < 8) не должен ничего менять/падать.
    vesc_msgs::msg::VescState state;
    state.speed_rpm = 123.0;
    can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_dlc = 4;
    vesc_nexus::parseStatusPacket(frame, state);
    EXPECT_EQ(state.speed_rpm, 123.0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
