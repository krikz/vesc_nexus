// vesc_handler.hpp
#pragma once

#include <linux/can.h>
#include <functional>
#include <string>
#include <chrono>
#include "message_translator.hpp"
#include <vesc_msgs/msg/vesc_state.hpp>

class VescHandler {
public:
    using SendCanFrameFunc = std::function<bool(const struct can_frame&)>;
    using StateUpdateCallback = std::function<void(const vesc_msgs::msg::VescState&)>;

    VescHandler(uint8_t can_id, const std::string& label,
                double wheel_radius, int poles, int64_t min_erpm,
                const vesc_nexus::CommandLimits& limits);

    void setSendCanFunc(SendCanFrameFunc func);
    void setStateUpdateCallback(StateUpdateCallback cb);

    void processCanFrame(const struct can_frame& frame);
    void requestState();  // Отправить COMM_GET_VALUES
    void sendDutyCycle(double duty);
    void sendCurrent(double current);
    void sendSpeed(double linear_speed);
    void sendBrake(double brake);
    void sendPosition(double pos);

    uint8_t getCanId() const;
    std::string getLabel() const;
    vesc_msgs::msg::VescState getLastState() const;
    double getWheelRadius() const;
    int getPolePairs() const;

private:
    uint8_t can_id_;
    std::string label_;
    vesc_nexus::CommandLimits limits_;
    vesc_msgs::msg::VescState last_state_;

    SendCanFrameFunc send_can_func_;
    StateUpdateCallback state_update_cb_;
    
    double wheel_radius_;     // радиус колеса
    int pole_pairs_;          // количество пар полюсов (poles / 2)
    int64_t min_erpm_;         // минимальный ERPM для движения
    
    // Для подсчёта частоты отправки команд
    size_t send_speed_count_;
    std::chrono::steady_clock::time_point last_freq_log_time_;
    
    // Для отслеживания изменений значений
    double last_linear_speed_;  // последняя линейная скорость (м/с)
    double last_erpm_;          // последний ERPM для логирования изменений
};