// vesc_handler.cpp
#include "vesc_nexus/vesc_handler.hpp"
#include "vesc_nexus/message_translator.hpp"
#include <rclcpp/logging.hpp>

VescHandler::VescHandler(uint8_t can_id, const std::string& label,
                         rclcpp::Node::SharedPtr node,
                         const vesc_nexus::CommandLimits& limits)
    : can_id_(can_id), label_(label), node_(node), limits_(limits)
{
    state_.label = label;
    state_.alive = false;

    state_pub_ = node_->create_publisher<vesc_msgs::msg::VescStateStamped>(
        "sensors/motor_state/" + label, 10);
}

void VescHandler::setSendCanFunc(SendCanFrameFunc func) {
    send_can_func_ = func;
}

void VescHandler::processCanFrame(const struct can_frame& frame) {
    if ((frame.can_id & 0xFF) != can_id_) return;

    if (frame.data[0] == 4) {  // COMM_GET_VALUES
        vesc_nexus::parseValuesReply(frame, state_);
        state_.label = label_;

        vesc_msgs::msg::VescStateStamped stamped;
        stamped.header.stamp = node_->now();
        stamped.header.frame_id = label_;
        stamped.state = state_;

        state_pub_->publish(stamped);
    }
}

void VescHandler::sendDutyCycle(double duty) {
    if (send_can_func_) {
        auto frame = createSetDutyCycleFrame(can_id_, duty, limits_);
        send_can_func_(frame);
    }
}

void VescHandler::sendCurrent(double current) {
    if (send_can_func_) {
        auto frame = createSetCurrentFrame(can_id_, current, limits_);
        send_can_func_(frame);
    }
}

void VescHandler::sendSpeed(double rpm) {
    if (send_can_func_) {
        auto frame = createSetSpeedFrame(can_id_, rpm, limits_);
        send_can_func_(frame);
    }
}

void VescHandler::sendBrake(double brake) {
    if (send_can_func_) {
        auto frame = createSetBrakeFrame(can_id_, brake, limits_);
        send_can_func_(frame);
    }
}

void VescHandler::sendPosition(double pos) {
    if (send_can_func_) {
        auto frame = createSetPositionFrame(can_id_, pos, limits_);
        send_can_func_(frame);
    }
}

void VescHandler::requestState() {
    if (send_can_func_) {
        auto frame = vesc_nexus::createRequestValuesFrame(can_id_);
        send_can_func_(frame);
    }
}

vesc_msgs::msg::VescState VescHandler::getState() const {
    return state_;
}

uint8_t VescHandler::getCanId() const {
    return can_id_;
}

std::string VescHandler::getLabel() const {
    return label_;
}