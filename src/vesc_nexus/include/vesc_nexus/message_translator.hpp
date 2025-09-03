// message_translator.hpp
#pragma once

#include <linux/can.h>
#include <vesc_msgs/msg/vesc_state.hpp>
#include <rclcpp/rclcpp.hpp>
#include <cstdint>
#include <vesc_msgs/msg/vesc_state.hpp>

namespace vesc_nexus {

struct CommandLimits {
    double duty_cycle_min = -0.5;
    double duty_cycle_max = 0.5;
    double current_min = 0.0;
    double current_max = 10.0;
    double speed_min = -23250.0;
    double speed_max = 23250.0;
    double brake_min = -20000.0;
    double brake_max = 200000.0;
    double position_min = 0.0;
    double position_max = 0.0;
    double servo_min = 0.15;
    double servo_max = 0.85;
};

// Команды
can_frame createSetDutyCycleFrame(uint8_t can_id, double duty, const CommandLimits& limits);
can_frame createSetCurrentFrame(uint8_t can_id, double current, const CommandLimits& limits);
can_frame createSetSpeedFrame(uint8_t can_id, double rpm, const CommandLimits& limits);
can_frame createSetBrakeFrame(uint8_t can_id, double brake, const CommandLimits& limits);
can_frame createSetPositionFrame(uint8_t can_id, double position, const CommandLimits& limits);
can_frame createRequestValuesFrame(uint8_t can_id);  // COMM_GET_VALUES (0x04)

// Парсинг ответов
void parseValuesReply(const struct can_frame& frame, vesc_msgs::msg::VescState& state_out);

} // namespace vesc_nexus