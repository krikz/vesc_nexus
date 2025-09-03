// vesc_handler.hpp
#pragma once

#include <rclcpp/rclcpp.hpp>
#include <vesc_msgs/msg/vesc_state_stamped.hpp>
#include <linux/can.h>
#include "message_translator.hpp"

class VescHandler {
public:
    using SendCanFrameFunc = std::function<bool(const struct can_frame&)>;

    VescHandler(uint8_t can_id,
                const std::string& label,
                rclcpp::Node::SharedPtr node,
                const vesc_nexus::CommandLimits& limits);

    void processCanFrame(const struct can_frame& frame);
    void sendDutyCycle(double duty);
    void sendCurrent(double current);
    void sendSpeed(double rpm);
    void sendBrake(double brake);
    void sendPosition(double pos);
    void requestState();  // Отправить запрос COMM_GET_VALUES

    vesc_msgs::msg::VescState getState() const;
    uint8_t getCanId() const;
    std::string getLabel() const;

    void setSendCanFunc(SendCanFrameFunc func);

private:
    uint8_t can_id_;
    std::string label_;
    rclcpp::Node::SharedPtr node_;
    vesc_msgs::msg::VescState state_;
    vesc_nexus::CommandLimits limits_;
    SendCanFrameFunc send_can_func_;

    rclcpp::Publisher<vesc_msgs::msg::VescStateStamped>::SharedPtr state_pub_;
};