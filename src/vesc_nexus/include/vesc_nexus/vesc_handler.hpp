// vesc_handler.hpp
#pragma once

#include <linux/can.h>
#include <functional>
#include <string>
#include <chrono>
#include <vector>
#include "message_translator.hpp"
#include "duty_calibration.hpp"
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

    /**
     * @brief Установить максимальную скорость для калибровки
     * @param max_speed_mps Максимальная линейная скорость при duty=1.0 (м/с)
     */
    void setMaxSpeed(double max_speed_mps);

    /**
     * @brief Установить калибровочную таблицу для duty cycle → скорость
     * @param duty_values Вектор значений duty cycle
     * @param speed_values Вектор соответствующих скоростей (м/с)
     * 
     * Оба вектора должны иметь одинаковую длину.
     * Калибровочные точки будут автоматически отсортированы.
     */
    void setCalibrationTable(const std::vector<double>& duty_values,
                             const std::vector<double>& speed_values);

    /**
     * @brief Добавить одну калибровочную точку
     * @param duty Значение duty cycle
     * @param speed_mps Измеренная скорость (м/с)
     */
    void addCalibrationPoint(double duty, double speed_mps);

    /**
     * @brief Получить калибровочную таблицу
     */
    const vesc_nexus::DutyCalibrationTable& getCalibrationTable() const;

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
    
    // Калибровочная таблица для duty cycle → скорость
    vesc_nexus::DutyCalibrationTable calibration_table_;
    
    // Для подсчёта частоты отправки команд
    size_t send_speed_count_;
    std::chrono::steady_clock::time_point last_freq_log_time_;
    
    // Для отслеживания изменений значений
    double last_linear_speed_;
    double last_erpm_;
};