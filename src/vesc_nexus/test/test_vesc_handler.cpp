// test_vesc_handler.cpp
// Unit tests for VescHandler NaN/Infinity guards (FA-3).
#include <gtest/gtest.h>
#include <cmath>
#include <cstring>
#include <limits>

#include "vesc_nexus/vesc_handler.hpp"

namespace {

const double kNaN = std::numeric_limits<double>::quiet_NaN();
const double kInf = std::numeric_limits<double>::infinity();

// Записывает int32 big-endian в буфер.
void putInt32BE(uint8_t* buf, int32_t value) {
    buf[0] = (value >> 24) & 0xFF;
    buf[1] = (value >> 16) & 0xFF;
    buf[2] = (value >> 8) & 0xFF;
    buf[3] = value & 0xFF;
}

// Строит CAN_PACKET_STATUS фрейм (ID 9) с заданным ERPM.
can_frame makeStatusFrame(uint8_t can_id, int32_t erpm) {
    can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = ((vesc_nexus::CAN_PACKET_STATUS << 8) | can_id) | CAN_EFF_FLAG;
    frame.can_dlc = 8;
    putInt32BE(frame.data, erpm);
    // current (int16 BE) = 0, duty (int16 BE) = 0
    frame.data[4] = 0; frame.data[5] = 0;
    frame.data[6] = 0; frame.data[7] = 0;
    return frame;
}

class VescHandlerTest : public ::testing::Test {
protected:
    void SetUp() override {
        handler_ = std::make_shared<VescHandler>(
            42, "test_wheel", 0.115, 14, 0, vesc_nexus::CommandLimits{});
        sent_frames_ = 0;
        handler_->setSendCanFunc([this](const can_frame&) {
            sent_frames_++;
            return true;
        });
    }

    std::shared_ptr<VescHandler> handler_;
    int sent_frames_ = 0;
};

}  // namespace

TEST_F(VescHandlerTest, ConstructorSanitizesNaNWheelRadius) {
    auto handler = std::make_shared<VescHandler>(
        1, "nan_wheel", kNaN, 14, 0, vesc_nexus::CommandLimits{});
    EXPECT_TRUE(std::isfinite(handler->getWheelRadius()));
    EXPECT_GT(handler->getWheelRadius(), 0.0);
    EXPECT_TRUE(std::isfinite(handler->getMaxSpeed()));
}

TEST_F(VescHandlerTest, ConstructorSanitizesZeroPoles) {
    // poles=0 → pole_pairs_ должен стать >= 1, деления на ноль быть не должно.
    auto handler = std::make_shared<VescHandler>(
        2, "zero_poles", 0.115, 0, 0, vesc_nexus::CommandLimits{});
    auto frame = makeStatusFrame(2, 30000);
    handler->processCanFrame(frame);
    // Скорость должна быть конечной (не inf от деления на 0).
    EXPECT_TRUE(std::isfinite(handler->getVelocityRadPerSec()));
}

TEST_F(VescHandlerTest, SendSpeedNaNDoesNotSendFrame) {
    handler_->sendSpeed(kNaN);
    handler_->sendSpeed(-kInf);
    EXPECT_EQ(sent_frames_, 0);
}

TEST_F(VescHandlerTest, SendSpeedRpmNaNDoesNotSendFrame) {
    handler_->sendSpeedRpm(kNaN);
    handler_->sendSpeedRpm(kInf);
    EXPECT_EQ(sent_frames_, 0);
}

TEST_F(VescHandlerTest, SendSpeedFiniteSendsFrame) {
    handler_->sendSpeed(0.5);
    EXPECT_EQ(sent_frames_, 1);
}

TEST_F(VescHandlerTest, ProcessCorruptedStatusKeepsVelocityFinite) {
    // Экстремальный ERPM = INT32_MAX → скорость должна остаться конечной.
    auto frame = makeStatusFrame(42, std::numeric_limits<int32_t>::max());
    handler_->processCanFrame(frame);
    EXPECT_TRUE(std::isfinite(handler_->getVelocityRadPerSec()));
    EXPECT_TRUE(std::isfinite(handler_->getAccumulatedPosition()));
}

TEST_F(VescHandlerTest, ProcessGarbageFrameNoCrash) {
    can_frame frame;
    std::memset(&frame, 0, sizeof(frame));
    frame.can_id = CAN_EFF_FLAG;  // чужой/мусорный ID
    frame.can_dlc = 8;
    // Не должен упасть и не должен изменить состояние.
    handler_->processCanFrame(frame);
    EXPECT_TRUE(std::isfinite(handler_->getVelocityRadPerSec()));
    EXPECT_TRUE(std::isfinite(handler_->getAccumulatedPosition()));
}

TEST_F(VescHandlerTest, SetMaxRpsSanitizesNaN) {
    handler_->setMaxRps(kNaN);
    EXPECT_TRUE(std::isfinite(handler_->getMaxSpeed()));
    EXPECT_GT(handler_->getMaxSpeed(), 0.0);
}

TEST_F(VescHandlerTest, SetGearRatioSanitizesNaN) {
    handler_->setGearRatio(kNaN);
    EXPECT_TRUE(std::isfinite(handler_->getMaxSpeed()));
    EXPECT_GT(handler_->getMaxSpeed(), 0.0);
}

int main(int argc, char** argv) {
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
