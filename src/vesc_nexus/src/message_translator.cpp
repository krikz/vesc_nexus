// message_translator.cpp
#include "vesc_nexus/message_translator.hpp"
#include <cstring>
#include <algorithm>

void vesc_nexus::buffer_append_int32(uint8_t* buffer, int32_t number, int& index) {
    buffer[index++] = (number >> 24) & 0xFF;
    buffer[index++] = (number >> 16) & 0xFF;
    buffer[index++] = (number >> 8) & 0xFF;
    buffer[index++] = number & 0xFF;
}

void vesc_nexus::buffer_append_int16(uint8_t* buffer, int16_t number, int& index) {
    buffer[index++] = (number >> 8) & 0xFF;
    buffer[index++] = number & 0xFF;
}

can_frame vesc_nexus::createSetDutyCycleFrame(uint8_t can_id, double duty) {
    can_frame frame;
    frame.can_id = ((CAN_PACKET_SET_DUTY << 8) | can_id) | CAN_EFF_FLAG;
    frame.can_dlc = 4;

    int32_t value = static_cast<int32_t>(std::clamp(duty, -1.0, 1.0) * 100000.0);
    frame.data[0] = (value >> 24) & 0xFF;
    frame.data[1] = (value >> 16) & 0xFF;
    frame.data[2] = (value >> 8)  & 0xFF;
    frame.data[3] = value        & 0xFF;

    return frame;
}

can_frame vesc_nexus::createSetCurrentFrame(uint8_t can_id, double current) {
    can_frame frame;
    frame.can_id = (can_id) | (CAN_PACKET_SET_CURRENT << 8);
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 4;
    int index = 0;
    int32_t value = static_cast<int32_t>(std::clamp(current, -10.0, 10.0) * 1000.0);
    buffer_append_int32(frame.data, value, index);
    return frame;
}

// message_translator.cpp
can_frame vesc_nexus::createSetSpeedFrame(uint8_t can_id, double rpm) {
    can_frame frame;
    // Правильный 29-битный ID: (CAN_PACKET_SET_RPM << 8) | can_id
    frame.can_id = ((CAN_PACKET_SET_RPM << 8) | can_id) | CAN_EFF_FLAG;
    frame.can_dlc = 4;  // Только 4 байта данных!

    int32_t value = static_cast<int32_t>(std::clamp(rpm, -23250.0, 23250.0));
    // Big-endian запись
    frame.data[0] = (value >> 24) & 0xFF;
    frame.data[1] = (value >> 16) & 0xFF;
    frame.data[2] = (value >> 8)  & 0xFF;
    frame.data[3] = value        & 0xFF;

    return frame;
}

can_frame vesc_nexus::createSetBrakeFrame(uint8_t can_id, double brake) {
    can_frame frame;
    frame.can_id = (can_id) | (CAN_PACKET_SET_CURRENT_BRAKE << 8);
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 4;
    int index = 0;
    int32_t value = static_cast<int32_t>(std::clamp(brake, -20000.0, 200000.0) * 1000.0);
    buffer_append_int32(frame.data, value, index);
    return frame;
}

can_frame vesc_nexus::createSetPositionFrame(uint8_t can_id, double pos) {
    can_frame frame;
    frame.can_id = (can_id) | (CAN_PACKET_SET_POS << 8);
    frame.can_id |= CAN_EFF_FLAG;
    frame.can_dlc = 4;
    int index = 0;
    int32_t value = static_cast<int32_t>(std::clamp(pos, 0.0, 360.0) * 1000000.0);
    buffer_append_int32(frame.data, value, index);
    return frame;
}

// Парсинг CAN_PACKET_STATUS (ID 9)
void vesc_nexus::parseStatusPacket(const can_frame& frame, vesc_msgs::msg::VescState& state) {
    if (frame.can_dlc < 8) return;
    int32_t erpm; int16_t current, duty;
    std::memcpy(&erpm,   &frame.data[0], 4); erpm = __builtin_bswap32(erpm);
    std::memcpy(&current, &frame.data[4], 2); current = __builtin_bswap16(current);
    std::memcpy(&duty,    &frame.data[6], 2); duty = __builtin_bswap16(duty);

    state.speed_rpm = static_cast<double>(erpm);
    state.current_motor = static_cast<double>(current) / 10.0;
    state.duty_cycle = static_cast<double>(duty) / 1000.0;
}

// CAN_PACKET_STATUS_4: Temp FET, Temp Motor, Current In, PID Pos
void vesc_nexus::parseStatus4Packet(const can_frame& frame, vesc_msgs::msg::VescState& state) {
    if (frame.can_dlc < 8) return;
    int16_t temp_fet, temp_motor, current_in, pid_pos;
    std::memcpy(&temp_fet,    &frame.data[0], 2); temp_fet = __builtin_bswap16(temp_fet);
    std::memcpy(&temp_motor,  &frame.data[2], 2); temp_motor = __builtin_bswap16(temp_motor);
    std::memcpy(&current_in,  &frame.data[4], 2); current_in = __builtin_bswap16(current_in);
    std::memcpy(&pid_pos,     &frame.data[6], 2); pid_pos = __builtin_bswap16(pid_pos);

    state.temp_controller = static_cast<double>(temp_fet) / 10.0;
    state.temp_motor = static_cast<double>(temp_motor) / 10.0;
    state.current_input = static_cast<double>(current_in) / 10.0;
    state.pid_pos_now = static_cast<double>(pid_pos) / 50.0;
}

// CAN_PACKET_STATUS_5: Tachometer, Voltage In
void vesc_nexus::parseStatus5Packet(const can_frame& frame, vesc_msgs::msg::VescState& state) {
    if (frame.can_dlc < 6) return;
    int32_t tachometer; int16_t voltage_in;
    std::memcpy(&tachometer, &frame.data[0], 4); tachometer = __builtin_bswap32(tachometer);
    std::memcpy(&voltage_in, &frame.data[4], 2); voltage_in = __builtin_bswap16(voltage_in);

    state.tachometer = static_cast<uint32_t>(tachometer / 6);
    state.voltage_input = static_cast<double>(voltage_in) / 10.0;
}

// CAN_PACKET_STATUS_2: Ah Used, Ah Charged
void vesc_nexus::parseStatus2Packet(const can_frame& frame, vesc_msgs::msg::VescState& state) {
    if (frame.can_dlc < 8) return;
    int32_t ah_used, ah_charged;
    std::memcpy(&ah_used,   &frame.data[0], 4); ah_used = __builtin_bswap32(ah_used);
    std::memcpy(&ah_charged, &frame.data[4], 4); ah_charged = __builtin_bswap32(ah_charged);

    state.charge_drawn = static_cast<double>(ah_used) / 10000.0;
    state.charge_regen = static_cast<double>(ah_charged) / 10000.0;
}