// vesc_handler.cpp
#include "vesc_nexus/vesc_handler.hpp"
#include "vesc_nexus/message_translator.hpp"
#include <rclcpp/logging.hpp>

VescHandler::VescHandler(uint8_t can_id, const std::string& label,
                         double wheel_radius, int poles, int64_t min_erpm,
                         const vesc_nexus::CommandLimits& limits)
    : can_id_(can_id), label_(label), limits_(limits), wheel_radius_(wheel_radius),
     pole_pairs_(poles / 2), min_erpm_(min_erpm)
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

    // Логируем каждый пришедший фрейм (опционально — можно убрать в релизе)
    RCLCPP_DEBUG_STREAM(rclcpp::get_logger("VescHandler" ),
        "[" << label_ << "] CAN RX: ID=0x" << std::hex << frame.can_id
        << " CMD=" << std::dec << (int)command_id
        << " DLC=" << (int)frame.can_dlc
        << " Data=" << std::vector<uint8_t>(frame.data, frame.data + frame.can_dlc));

    // Фильтр: только 29-битные фреймы (расширенный формат)
    if (!(frame.can_id & CAN_EFF_FLAG)) {
        RCLCPP_WARN_STREAM(rclcpp::get_logger("VescHandler" ), "[" << label_ << "] Non-EFF frame received (ignored): ID=0x" << std::hex << frame.can_id);
        return;
    }

    // Проверка ID
    if (sender_id != can_id_) {
        RCLCPP_DEBUG_STREAM(rclcpp::get_logger("VescHandler" ),
            "[" << label_ << "] Frame from wrong sender: expected=0x" << std::hex << (int)can_id_
            << ", got=0x" << (int)sender_id);
        return;
    }

    // Обработка пакетов статуса
    switch (command_id) {
        case vesc_nexus::CAN_PACKET_STATUS: {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("VescHandler" ), "[" << label_ << "] ✅ STATUS Packet received");
            vesc_nexus::parseStatusPacket(frame, last_state_);
            break;
        }
        case vesc_nexus::CAN_PACKET_STATUS_2: {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("VescHandler" ), "[" << label_ << "] ✅ STATUS_2 Packet received");
            vesc_nexus::parseStatus2Packet(frame, last_state_);
            break;
        }
        case vesc_nexus::CAN_PACKET_STATUS_4: {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("VescHandler" ), "[" << label_ << "] ✅ STATUS_4 Packet received");
            vesc_nexus::parseStatus4Packet(frame, last_state_);
            break;
        }
        case vesc_nexus::CAN_PACKET_STATUS_5: {
            RCLCPP_INFO_STREAM(rclcpp::get_logger("VescHandler" ), "[" << label_ << "] ✅ STATUS_5 Packet received");
            vesc_nexus::parseStatus5Packet(frame, last_state_);
            break;
        }
        default:
            RCLCPP_DEBUG_STREAM(rclcpp::get_logger("VescHandler" ),
                "[" << label_ << "] Unknown command ID: 0x" << std::hex << (int)command_id);
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

    // Конвертируем линейную скорость в ERPM
    double circumference = 2.0 * M_PI * wheel_radius_;
    double rps = linear_speed / circumference;
    double rpm = rps * 60.0;
    double erpm = rpm * pole_pairs_;

    // Ограничиваем по минимальному ERPM (чтобы преодолеть статическое трение)
    if (std::abs(erpm) > 0 && std::abs(erpm) < min_erpm_) {
        erpm = (erpm > 0) ? min_erpm_ : -min_erpm_;
    }

    // Отправляем ERPM
    auto frame = vesc_nexus::createSetSpeedFrame(can_id_, erpm);
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