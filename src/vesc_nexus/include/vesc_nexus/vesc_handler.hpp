// vesc_handler.hpp
#pragma once

#include <linux/can.h>
#include <functional>
#include <string>
#include <chrono>
#include <cmath>
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

    /**
     * @brief Установить максимальные обороты в секунду при duty = 100%
     * @param max_rps Обороты в секунду (замерить при калибровке: RPM / 60)
     * 
     * КАЛИБРОВКА:
     * Используйте скрипт tools/calibrate_max_rpm.py для автоматической калибровки
     * или вручную:
     * 1. Поставить робота на подставку (колёса должны свободно крутиться)
     * 2. Подать duty = 100% на колесо
     * 3. Замерить RPM из телеметрии VESC
     * 4. max_rps = RPM / 60
     */
    void setMaxRps(double max_rps);

    /**
     * @brief Получить максимальные обороты в секунду
     */
    double getMaxRps() const { return max_rps_; }

    /**
     * @brief Получить максимальную линейную скорость (м/с)
     */
    double getMaxSpeed() const { return max_speed_mps_; }

    /**
     * @brief Установить минимальный duty cycle для преодоления мёртвой зоны
     * @param min_duty Минимальный duty (0.0-1.0), типично 0.05-0.10
     */
    void setMinDuty(double min_duty) { min_duty_ = min_duty; }

    /**
     * @brief Включить/выключить PWM модуляцию для низких скоростей
     * @param enabled true - использовать PWM модуляцию при duty < min_duty
     */
    void setPwmModulationEnabled(bool enabled) { pwm_modulation_enabled_ = enabled; }

    /**
     * @brief Установить параметры PWM модуляции
     * @param duty_low Низкий duty cycle (типично 0.01-0.03)
     * @param duty_high Высокий duty cycle (типично min_duty)
     * @param frequency Частота PWM модуляции в Гц (типично 50 Hz)
     */
    void setPwmModulationParams(double duty_low, double duty_high, double frequency) {
        pwm_duty_low_ = duty_low;
        pwm_duty_high_ = duty_high;
        pwm_frequency_hz_ = frequency;
    }

    /**
     * @brief Получить накопленную позицию колеса (rad)
     * Интегрируется по реальным интервалам между CAN пакетами
     */
    double getAccumulatedPosition() const { return accumulated_position_rad_; }

    /**
     * @brief Получить текущую угловую скорость (rad/s)
     */
    double getVelocityRadPerSec() const { return velocity_rad_s_; }

private:
    uint8_t can_id_;
    std::string label_;
    vesc_nexus::CommandLimits limits_;
    vesc_msgs::msg::VescState last_state_;

    SendCanFrameFunc send_can_func_;
    StateUpdateCallback state_update_cb_;
    
    double wheel_radius_;     // радиус колеса
    int pole_pairs_;          // количество пар полюсов (poles / 2)
    int64_t min_erpm_;        // минимальный ERPM для движения
    
    // Калибровка: максимальные обороты при duty=100%
    // Используйте tools/calibrate_max_rpm.py для измерения
    double max_rps_ = 15.0;           // обороты в секунду при duty=100%
    double max_speed_mps_ = 0.0;      // расчётная макс. скорость (м/с)
    double min_duty_ = 0.0;           // минимальный duty для преодоления мёртвой зоны
    
    // PWM модуляция для ультра-низких скоростей
    bool pwm_modulation_enabled_ = false;  // включить PWM модуляцию
    double pwm_duty_low_ = 0.01;          // низкий duty для PWM
    double pwm_duty_high_ = 0.1;          // высокий duty для PWM (обычно = min_duty_)
    double pwm_frequency_hz_ = 50.0;      // частота PWM в Гц
    size_t pwm_cycle_counter_ = 0;        // счётчик для PWM цикла
    
    // Для подсчёта частоты отправки команд
    size_t send_speed_count_;
    std::chrono::steady_clock::time_point last_freq_log_time_;
    
    // Для отслеживания изменений значений
    double last_linear_speed_;
    double last_duty_;
    
    // Накопление позиции по реальным интервалам CAN пакетов
    std::chrono::steady_clock::time_point last_status_time_;
    double accumulated_position_rad_ = 0.0;
    double velocity_rad_s_ = 0.0;
    bool first_status_received_ = false;
    
    // Вспомогательные функции
    void updateMaxSpeed();
    double clamp(double value, double lo, double hi) const;
    
    /**
     * @brief Вычислить duty cycle с PWM модуляцией
     * @param target_duty Желаемый duty cycle (может быть < min_duty)
     * @return Фактический duty для отправки (LOW или HIGH)
     */
    double calculatePwmDuty(double target_duty);
};