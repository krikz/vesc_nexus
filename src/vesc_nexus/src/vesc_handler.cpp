// vesc_handler.cpp
#include "vesc_nexus/vesc_handler.hpp"
#include "vesc_nexus/message_translator.hpp"
#include <rclcpp/logging.hpp>

VescHandler::VescHandler(uint8_t can_id, const std::string& label,
                         double wheel_radius, int poles, int64_t min_erpm,
                         const vesc_nexus::CommandLimits& limits)
    : can_id_(can_id), label_(label), limits_(limits), wheel_radius_(wheel_radius),
     pole_pairs_(poles / 2), min_erpm_(min_erpm),
     max_rps_(15.0),  // По умолчанию 15 об/сек при duty=100%
     send_speed_count_(0), last_freq_log_time_(std::chrono::steady_clock::now()),
     last_linear_speed_(0.0), last_duty_(0.0)
{
    updateMaxSpeed();
    
    RCLCPP_INFO(rclcpp::get_logger("VescHandler"), "Initialized VescHandler: "
        "label='%s', can_id=%d, wheel_radius=%.3fm, poles=%d (%d pole pairs), min_erpm=%ld",
        label_.c_str(), can_id_, wheel_radius_, poles, pole_pairs_, min_erpm_);
    last_state_.label = label;
    last_state_.alive = false;
}

void VescHandler::updateMaxSpeed() {
    // max_speed = 2π × max_rps × wheel_radius
    max_speed_mps_ = 2.0 * M_PI * max_rps_ * wheel_radius_;
}

double VescHandler::clamp(double value, double lo, double hi) const {
    return (value < lo) ? lo : (value > hi) ? hi : value;
}

void VescHandler::setSendCanFunc(SendCanFrameFunc func) {
    send_can_func_ = func;
}

void VescHandler::setStateUpdateCallback(StateUpdateCallback cb) {
    state_update_cb_ = cb;
}

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

void VescHandler::requestState() {
    // Ничего не делаем — VESC сам шлёт статус
}

void VescHandler::sendDutyCycle(double duty) {
    if (send_can_func_) {
        auto frame = vesc_nexus::createSetDutyCycleFrame(can_id_, duty);
        send_can_func_(frame);
    }
}

void VescHandler::sendCurrent(double current) {
    if (send_can_func_) {
        auto frame = vesc_nexus::createSetCurrentFrame(can_id_, current);
        send_can_func_(frame);
    }
}

void VescHandler::sendSpeed(double linear_speed) {
    if (!send_can_func_) return;

    // Счётчик для подсчёта частоты отправки
    send_speed_count_++;
    
    auto now = std::chrono::steady_clock::now();
    auto elapsed = std::chrono::duration_cast<std::chrono::milliseconds>(now - last_freq_log_time_).count();
    
    // Логируем частоту каждые 2 секунды
    if (elapsed >= 2000) {
        double frequency = (send_speed_count_ * 1000.0) / elapsed;
        RCLCPP_INFO(rclcpp::get_logger("VescHandler"), 
                    "[%s] sendSpeed: freq=%.1f Hz, max_rps=%.1f, max_speed=%.2f m/s", 
                    label_.c_str(), frequency, max_rps_, max_speed_mps_);
        send_speed_count_ = 0;
        last_freq_log_time_ = now;
    }

    // Конвертируем линейную скорость (м/с) в duty cycle
    // Формула: duty = target_speed / max_speed
    // где max_speed = 2π × max_rps × wheel_radius (расчитано в updateMaxSpeed)
    double duty_cycle = 0.0;
    if (max_speed_mps_ > 0.0) {
        duty_cycle = linear_speed / max_speed_mps_;
        duty_cycle = clamp(duty_cycle, -1.0, 1.0);
    }

    // Логируем изменения значений
    double speed_delta = std::abs(linear_speed - last_linear_speed_);
    double duty_delta = std::abs(duty_cycle - last_duty_);
    
    if (speed_delta > 0.001 || duty_delta > 0.001) {  // Логируем только значимые изменения
        RCLCPP_INFO(rclcpp::get_logger("VescHandler"),
                    "[%s] speed=%.4f m/s → duty=%.4f (Δspeed=%.4f, Δduty=%.4f)",
                    label_.c_str(), linear_speed, duty_cycle, 
                    speed_delta, duty_delta);
    }
    
    last_linear_speed_ = linear_speed;
    last_duty_ = duty_cycle;

    // Отправляем duty cycle
    auto frame = vesc_nexus::createSetDutyCycleFrame(can_id_, duty_cycle);
    send_can_func_(frame);
}

void VescHandler::sendBrake(double brake) {
    if (send_can_func_) {
        auto frame = vesc_nexus::createSetBrakeFrame(can_id_, brake);
        send_can_func_(frame);
    }
}

void VescHandler::sendPosition(double pos) {
    if (send_can_func_) {
        auto frame = vesc_nexus::createSetPositionFrame(can_id_, pos);
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

double VescHandler::getWheelRadius() const {
    return wheel_radius_;
}

int VescHandler::getPolePairs() const {
    return pole_pairs_;
}

void VescHandler::setMaxRps(double max_rps) {
    max_rps_ = max_rps;
    updateMaxSpeed();
    RCLCPP_INFO(rclcpp::get_logger("VescHandler"),
                "[%s] Калибровка: max_rps=%.1f об/сек → max_speed=%.2f м/с",
                label_.c_str(), max_rps_, max_speed_mps_);
}