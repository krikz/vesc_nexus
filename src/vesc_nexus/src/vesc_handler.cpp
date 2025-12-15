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
        case vesc_nexus::CAN_PACKET_STATUS: {
            vesc_nexus::parseStatusPacket(frame, last_state_);
            // Сохраняем raw ERPM для логирования
            double raw_erpm = last_state_.speed_rpm;
            // Конвертируем ERPM → механический RPM
            last_state_.speed_rpm = last_state_.speed_rpm / pole_pairs_;
            
            // Конвертируем RPM → rad/s
            velocity_rad_s_ = last_state_.speed_rpm * (2.0 * M_PI / 60.0);
            
            // Накапливаем позицию по РЕАЛЬНОМУ интервалу между CAN пакетами
            auto now = std::chrono::steady_clock::now();
            if (first_status_received_) {
                double dt = std::chrono::duration<double>(now - last_status_time_).count();
                // Защита от слишком больших dt (например, после паузы)
                if (dt > 0.0 && dt < 0.5) {
                    accumulated_position_rad_ += velocity_rad_s_ * dt;
                }
            }
            last_status_time_ = now;
            first_status_received_ = true;
            
            // DEBUG: логируем ERPM → RPM конвертацию (раз в секунду)
            static auto last_log = std::chrono::steady_clock::now();
            if (std::chrono::duration_cast<std::chrono::milliseconds>(now - last_log).count() > 1000) {
                if (std::abs(raw_erpm) > 10) {  // Только если есть движение
                    RCLCPP_INFO(rclcpp::get_logger("VescHandler"), 
                        "[%s] ERPM=%.0f, RPM=%.1f, vel=%.2f rad/s, pos=%.2f rad",
                        label_.c_str(), raw_erpm, last_state_.speed_rpm, 
                        velocity_rad_s_, accumulated_position_rad_);
                    last_log = now;
                }
            }
            break;
        }
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

    // Deadzone compensation: масштабируем duty из [0,1] в [min_duty, 1]
    // Это нужно потому что VESC не крутит мотор при очень малых duty (мёртвая зона)
    // При duty=0 остаётся 0, при duty=0.01 становится ~min_duty, при duty=1 остаётся 1
    if (min_duty_ > 0.0 && duty_cycle != 0.0) {
        double abs_duty = std::abs(duty_cycle);
        // Масштабируем: real_duty = min_duty + abs_duty * (1 - min_duty)
        double scaled_duty = min_duty_ + abs_duty * (1.0 - min_duty_);
        duty_cycle = (duty_cycle > 0) ? scaled_duty : -scaled_duty;
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