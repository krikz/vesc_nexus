// message_translator.hpp
#pragma once

#include <linux/can.h>
#include <vesc_msgs/msg/vesc_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <vesc_msgs/msg/vesc_state.hpp>

namespace vesc_nexus {

struct CommandLimits {
    double duty_cycle_min = -1.0;
    double duty_cycle_max = 1.0;
    double current_min = -10.0;
    double current_max = 10.0;
    double speed_min = -23250.0;
    double speed_max = 23250.0;
    double brake_min = -20000.0;
    double brake_max = 200000.0;
    double position_min = 0.0;
    double position_max = 360.0;
    double servo_min = 0.15;
    double servo_max = 0.85;
};

// CAN Packet IDs (из спецификации)
enum CanPacketId : uint8_t {
    CAN_PACKET_SET_DUTY = 0,
    CAN_PACKET_SET_CURRENT = 1,
    CAN_PACKET_SET_CURRENT_BRAKE = 2,
    CAN_PACKET_SET_RPM = 3,
    CAN_PACKET_SET_POS = 4,
    // ... другие, если нужно
    CAN_PACKET_STATUS = 9,
    CAN_PACKET_STATUS_2 = 14,
    CAN_PACKET_STATUS_3 = 15,
    CAN_PACKET_STATUS_4 = 16,
    CAN_PACKET_STATUS_5 = 27,
    CAN_PACKET_STATUS_6 = 58
};

// Утилита: big-endian запись
void buffer_append_int32(uint8_t* buffer, int32_t number, int& index);
void buffer_append_int16(uint8_t* buffer, int16_t number, int& index);

// Команды
can_frame createSetDutyCycleFrame(uint8_t can_id, double duty);
can_frame createSetCurrentFrame(uint8_t can_id, double current);
can_frame createSetSpeedFrame(uint8_t can_id, double rpm);
can_frame createSetBrakeFrame(uint8_t can_id, double brake);
can_frame createSetPositionFrame(uint8_t can_id, double pos);

// Парсинг входящих статусов
void parseStatusPacket(const can_frame& frame, vesc_msgs::msg::VescState& state);
void parseStatus2Packet(const can_frame& frame, vesc_msgs::msg::VescState& state);
void parseStatus4Packet(const can_frame& frame, vesc_msgs::msg::VescState& state);
void parseStatus5Packet(const can_frame& frame, vesc_msgs::msg::VescState& state);

} // namespace vesc_nexus