// vesc_handler.cpp
#include "vesc_nexus/vesc_handler.hpp"
#include "vesc_nexus/message_translator.hpp"
#include <rclcpp/logging.hpp>

VescHandler::VescHandler(uint8_t can_id, const std::string& label,
                         const vesc_nexus::CommandLimits& limits)
    : can_id_(can_id), label_(label), limits_(limits)
{
    last_state_.label = label;
    last_state_.alive = false;
}

void VescHandler::setSendCanFunc(SendCanFrameFunc func) {
    send_can_func_ = func;
}

void VescHandler::setStateUpdateCallback(StateUpdateCallback cb) {
    state_update_cb_ = cb;
}

// vesc_handler.cpp
void VescHandler::processCanFrame(const struct can_frame& frame) {
    uint8_t sender_id = frame.can_id & 0xFF;
    uint8_t command_id = (frame.can_id >> 8) & 0xFF;

    if (!frame.can_id & CAN_EFF_FLAG) return;  // Только 29-bit
    if (sender_id != can_id_) return;

    switch (command_id) {
        case vesc_nexus::CAN_PACKET_STATUS:
            vesc_nexus::parseStatusPacket(frame, last_state_);
            break;
        case vesc_nexus::CAN_PACKET_STATUS_2:
            vesc_nexus::parseStatus2Packet(frame, last_state_);
            break;
        case vesc_nexus::CAN_PACKET_STATUS_4:
            vesc_nexus::parseStatus4Packet(frame, last_state_);
            break;
        case vesc_nexus::CAN_PACKET_STATUS_5:
            vesc_nexus::parseStatus5Packet(frame, last_state_);
            break;
        default:
            return;
    }

    last_state_.alive = true;
    if (state_update_cb_) {
        state_update_cb_(last_state_);
    }
}

// Удали метод requestState() или оставь пустым
void VescHandler::requestState() {
    // Ничего не делаем — VESC сам шлёт статус
}

void VescHandler::sendDutyCycle(double duty) {
    if (send_can_func_) {
        auto frame = vesc_nexus::createSetDutyCycleFrame(can_id_, duty, limits_);
        send_can_func_(frame);
    }
}

void VescHandler::sendCurrent(double current) {
    if (send_can_func_) {
        auto frame = vesc_nexus::createSetCurrentFrame(can_id_, current, limits_);
        send_can_func_(frame);
    }
}

void VescHandler::sendSpeed(double rpm) {
    if (send_can_func_) {
        auto frame = vesc_nexus::createSetSpeedFrame(can_id_, rpm, limits_);
        send_can_func_(frame);
    }
}

void VescHandler::sendBrake(double brake) {
    if (send_can_func_) {
        auto frame = vesc_nexus::createSetBrakeFrame(can_id_, brake, limits_);
        send_can_func_(frame);
    }
}

void VescHandler::sendPosition(double pos) {
    if (send_can_func_) {
        auto frame = vesc_nexus::createSetPositionFrame(can_id_, pos, limits_);
        send_can_func_(frame);
    }
}

uint8_t VescHandler::getCanId() const {
    return can_id_;
}

std::string VescHandler::getLabel() const {
    return label_;
}

vesc_msgs::msg::VescState VescHandler::getLastState() const {
    return last_state_;
}