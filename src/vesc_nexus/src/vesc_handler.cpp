// vesc_handler.cpp
#include "vesc_nexus/vesc_handler.hpp"
#include "vesc_nexus/message_translator.hpp"
#include <rclcpp/logging.hpp>

VescHandler::VescHandler(uint8_t can_id, const std::string& label,
                         double wheel_radius, int poles, int64_t min_erpm,
                         const vesc_nexus::CommandLimits& limits)
    : can_id_(can_id), label_(label), limits_(limits), wheel_radius_(wheel_radius),
     pole_pairs_(poles / 2), min_erpm_(min_erpm),
     send_speed_count_(0), last_freq_log_time_(std::chrono::steady_clock::now()),
     last_linear_speed_(0.0), last_erpm_(0.0)
{
  RCLCPP_INFO(rclcpp::get_logger("VescHandler"), "Initialized VescHandler: "
        "label='%s', can_id=%d, wheel_radius=%.3fm, poles=%d (%d pole pairs), min_erpm=%ld",
        label_.c_str(), can_id_, wheel_radius_, poles, pole_pairs_, min_erpm_);
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
                    "[%s] sendSpeed frequency: %.1f Hz (sent %lu packets in %ld ms)", 
                    label_.c_str(), frequency, send_speed_count_, elapsed);
        send_speed_count_ = 0;
        last_freq_log_time_ = now;
    }

    // Конвертируем линейную скорость в duty cycle
    // Duty cycle: -1.0 (полный назад) до +1.0 (полный вперед)
    
    // Определяем максимальную скорость (для нормализации)
    const double max_linear_speed = 1.0;  // м/с, настраиваемое значение
    
    // Нормализуем скорость в диапазон -1.0 до +1.0
    double duty_cycle = linear_speed / max_linear_speed;
    
    // Ограничиваем duty cycle в допустимых пределах
    duty_cycle = std::clamp(duty_cycle, -1.0, 1.0);

    // Логируем изменения значений
    double speed_delta = std::abs(linear_speed - last_linear_speed_);
    int32_t duty_vesc = static_cast<int32_t>(duty_cycle * 100000.0);  // Для логов
    double duty_delta = std::abs(duty_vesc - last_erpm_) / 100000.0;
    
    if (speed_delta > 0.001 || duty_delta > 0.001) {  // Логируем только значимые изменения
        RCLCPP_INFO(rclcpp::get_logger("VescHandler"),
                    "[%s] Speed: %.4f m/s → Duty: %.4f (%d) (Δspeed: %.4f, ΔDuty: %.4f)",
                    label_.c_str(), linear_speed, duty_cycle, duty_vesc, 
                    speed_delta, duty_delta);
    }
    
    last_linear_speed_ = linear_speed;
    last_erpm_ = duty_vesc;  // Используем для хранения duty cycle (в формате VESC)

    // Отправляем duty cycle (функция сама конвертирует в VESC формат)
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