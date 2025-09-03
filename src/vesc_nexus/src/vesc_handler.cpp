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

void VescHandler::processCanFrame(const struct can_frame& frame) {
    if ((frame.can_id & 0xFF) != can_id_) return;

    if (frame.data[0] == 4) {  // COMM_GET_VALUES
        vesc_nexus::parseValuesReply(frame, last_state_);
        last_state_.label = label_;
        last_state_.alive = true;

        if (state_update_cb_) {
            state_update_cb_(last_state_);
        }
    }
}

void VescHandler::requestState() {
    if (send_can_func_) {
        auto frame = vesc_nexus::createRequestValuesFrame(can_id_);
        send_can_func_(frame);
    }
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