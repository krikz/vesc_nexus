// message_translator.cpp
#include "vesc_nexus/message_translator.hpp"
#include <cstring>
#include <algorithm>

namespace vesc_nexus {

can_frame createSetDutyCycleFrame(uint8_t can_id, double duty, const CommandLimits& limits) {
    can_frame frame;
    frame.can_id = 0x000 | can_id;
    frame.can_dlc = 5;
    frame.data[0] = 3;  // COMM_SET_DUTY_CYCLE

    duty = std::clamp(duty, limits.duty_cycle_min, limits.duty_cycle_max);
    float f_duty = static_cast<float>(duty);
    std::memcpy(&frame.data[1], &f_duty, 4);

    return frame;
}

can_frame createSetCurrentFrame(uint8_t can_id, double current, const CommandLimits& limits) {
    can_frame frame;
    frame.can_id = 0x000 | can_id;
    frame.can_dlc = 5;
    frame.data[0] = 4;  // COMM_SET_CURRENT

    current = std::clamp(current, limits.current_min, limits.current_max);
    float f_current = static_cast<float>(current);
    std::memcpy(&frame.data[1], &f_current, 4);

    return frame;
}

can_frame createSetSpeedFrame(uint8_t can_id, double rpm, const CommandLimits& limits) {
    can_frame frame;
    frame.can_id = 0x000 | can_id;
    frame.can_dlc = 5;
    frame.data[0] = 5;  // COMM_SET_RPM

    rpm = std::clamp(rpm, limits.speed_min, limits.speed_max);
    float f_rpm = static_cast<float>(rpm);
    std::memcpy(&frame.data[1], &f_rpm, 4);

    return frame;
}

can_frame createSetBrakeFrame(uint8_t can_id, double brake, const CommandLimits& limits) {
    can_frame frame;
    frame.can_id = 0x000 | can_id;
    frame.can_dlc = 5;
    frame.data[0] = 6;  // COMM_SET_CURRENT_BRAKE

    brake = std::clamp(brake, limits.brake_min, limits.brake_max);
    float f_brake = static_cast<float>(brake);
    std::memcpy(&frame.data[1], &f_brake, 4);

    return frame;
}

can_frame createSetPositionFrame(uint8_t can_id, double position, const CommandLimits& limits) {
    can_frame frame;
    frame.can_id = 0x000 | can_id;
    frame.can_dlc = 5;
    frame.data[0] = 18;  // COMM_SET_SERVO_POSITION

    position = std::clamp(position, limits.servo_min, limits.servo_max);
    float f_pos = static_cast<float>(position);
    std::memcpy(&frame.data[1], &f_pos, 4);

    return frame;
}

can_frame createRequestValuesFrame(uint8_t can_id) {
    can_frame frame;
    frame.can_id = 0x000 | can_id;
    frame.can_dlc = 1;
    frame.data[0] = 4;  // COMM_GET_VALUES
    return frame;
}

void parseValuesReply(const struct can_frame& frame, vesc_msgs::msg::VescState& state_out) {
    if (frame.can_dlc < 50 || frame.data[0] != 4) return;

    int offset = 1;

    float temp_motor, temp_controller, current_motor, current_in, id, iq;
    float duty, rpm, v_in, amp_hours, amp_hours_charged;
    float tachometer, tachometer_abs;
    uint8_t fault_code;

    std::memcpy(&temp_motor,        &frame.data[offset], sizeof(float)); offset += 4;
    std::memcpy(&temp_controller,   &frame.data[offset], sizeof(float)); offset += 4;
    std::memcpy(&current_motor,     &frame.data[offset], sizeof(float)); offset += 4;
    std::memcpy(&current_in,        &frame.data[offset], sizeof(float)); offset += 4;
    std::memcpy(&id,                &frame.data[offset], sizeof(float)); offset += 4;
    std::memcpy(&iq,                &frame.data[offset], sizeof(float)); offset += 4;
    std::memcpy(&duty,              &frame.data[offset], sizeof(float)); offset += 4;
    std::memcpy(&rpm,               &frame.data[offset], sizeof(float)); offset += 4;
    std::memcpy(&v_in,              &frame.data[offset], sizeof(float)); offset += 4;
    std::memcpy(&amp_hours,         &frame.data[offset], sizeof(float)); offset += 4;
    std::memcpy(&amp_hours_charged, &frame.data[offset], sizeof(float)); offset += 4;
    std::memcpy(&tachometer,        &frame.data[offset], sizeof(uint32_t)); offset += 4;
    std::memcpy(&tachometer_abs,    &frame.data[offset], sizeof(uint32_t)); offset += 4;
    fault_code = frame.data[offset];  // uint8_t

    state_out.speed_rpm = static_cast<double>(rpm);
    state_out.current_motor = static_cast<double>(current_motor);
    state_out.current_input = static_cast<double>(current_in);
    state_out.duty_cycle = static_cast<double>(duty);
    state_out.erpm = static_cast<double>(rpm);  // приближённо
    state_out.power_w = static_cast<double>(v_in * current_in);
    state_out.temp_motor = static_cast<double>(temp_motor);
    state_out.temp_controller = static_cast<double>(temp_controller);
    state_out.voltage_input = static_cast<double>(v_in);
    state_out.tachometer = static_cast<uint32_t>(tachometer);
    state_out.tachometer_abs = static_cast<uint32_t>(tachometer_abs);
    state_out.charge_drawn = static_cast<double>(amp_hours);
    state_out.charge_regen = static_cast<double>(amp_hours_charged);
    state_out.fault_code = static_cast<float>(fault_code);
    state_out.alive = true;
}

} // namespace vesc_nexus