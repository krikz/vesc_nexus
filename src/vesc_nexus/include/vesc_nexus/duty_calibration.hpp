// duty_calibration.hpp
// Калибровка соотношения duty cycle → скорость (обороты/сек) для VESC
//
// ПРИНЦИП РАБОТЫ:
// 1. При duty = 100% мотор крутится с максимальной скоростью (max_rps об/сек)
// 2. При duty = 50% мотор крутится с половиной скорости (предполагаем линейную зависимость)
// 3. Для расчёта duty из требуемой скорости: duty = target_speed / max_speed
//
// ПРОЦЕДУРА КАЛИБРОВКИ:
// 1. Поставить робота на подставку (колёса должны свободно крутиться)
// 2. Подать duty = 100% на колесо
// 3. Замерить обороты в секунду (через VESC телеметрию: RPM / 60)
// 4. Записать в конфиг wheel_max_rps: [15.0, 15.0, 15.0, 15.0] — пример для 15 об/сек
// 5. При необходимости проверить duty = -100% (обратное направление)
//
#pragma once

#include <cmath>
#include <string>

namespace vesc_nexus {

namespace detail {
// Собственная реализация clamp для совместимости с C++14
// (std::clamp доступен только начиная с C++17)
template<typename T>
constexpr const T& clamp(const T& value, const T& lo, const T& hi) {
    return (value < lo) ? lo : (value > hi) ? hi : value;
}
}  // namespace detail

/**
 * @brief Калибровка duty cycle → скорость для одного колеса VESC
 * 
 * Использует линейную модель: duty = target_rps / max_rps
 * где max_rps — замеренные обороты в секунду при duty = 100%
 */
class DutyCalibrationTable {
public:
    /**
     * @brief Конструктор
     * @param max_rps Максимальные обороты в секунду при duty = 100%
     *                (измеряется при калибровке: RPM / 60)
     * @param wheel_radius Радиус колеса в метрах (для конвертации м/с ↔ об/сек)
     * @param label Метка колеса для логирования
     */
    explicit DutyCalibrationTable(double max_rps = 15.0, 
                                   double wheel_radius = 0.1,
                                   const std::string& label = "")
        : max_rps_(max_rps)
        , wheel_radius_(wheel_radius)
        , label_(label) 
    {
        // Расчёт максимальной линейной скорости: v = ω * r = 2π * rps * r
        max_speed_mps_ = 2.0 * M_PI * max_rps_ * wheel_radius_;
    }

    /**
     * @brief Установить максимальные обороты (из калибровки)
     * @param max_rps Обороты в секунду при duty = 100%
     */
    void setMaxRps(double max_rps) {
        max_rps_ = max_rps;
        max_speed_mps_ = 2.0 * M_PI * max_rps_ * wheel_radius_;
    }

    /**
     * @brief Установить радиус колеса
     * @param wheel_radius Радиус в метрах
     */
    void setWheelRadius(double wheel_radius) {
        wheel_radius_ = wheel_radius;
        max_speed_mps_ = 2.0 * M_PI * max_rps_ * wheel_radius_;
    }

    /**
     * @brief Преобразовать линейную скорость (м/с) в duty cycle
     * @param target_speed_mps Целевая скорость в м/с (из контроллера)
     * @return Duty cycle от -1.0 до 1.0
     * 
     * Формула: duty = target_speed / max_speed
     *          где max_speed = 2π * max_rps * wheel_radius
     */
    double speedToDuty(double target_speed_mps) const {
        if (max_speed_mps_ <= 0.0) {
            return 0.0;  // Защита от деления на ноль
        }
        
        // Линейная зависимость: duty = speed / max_speed
        double duty = target_speed_mps / max_speed_mps_;
        
        // Ограничиваем в допустимых пределах
        return detail::clamp(duty, -1.0, 1.0);
    }

    /**
     * @brief Преобразовать обороты в секунду в duty cycle
     * @param target_rps Целевые обороты в секунду
     * @return Duty cycle от -1.0 до 1.0
     */
    double rpsToDuty(double target_rps) const {
        if (max_rps_ <= 0.0) {
            return 0.0;
        }
        double duty = target_rps / max_rps_;
        return detail::clamp(duty, -1.0, 1.0);
    }

    /**
     * @brief Преобразовать duty cycle в ожидаемую скорость (м/с)
     * @param duty_cycle Duty cycle от -1.0 до 1.0
     * @return Ожидаемая линейная скорость в м/с
     */
    double dutyToSpeed(double duty_cycle) const {
        return duty_cycle * max_speed_mps_;
    }

    /**
     * @brief Преобразовать duty cycle в ожидаемые обороты в секунду
     * @param duty_cycle Duty cycle от -1.0 до 1.0
     * @return Ожидаемые обороты в секунду
     */
    double dutyToRps(double duty_cycle) const {
        return duty_cycle * max_rps_;
    }

    // Геттеры
    double getMaxRps() const { return max_rps_; }
    double getMaxSpeed() const { return max_speed_mps_; }
    double getWheelRadius() const { return wheel_radius_; }
    const std::string& getLabel() const { return label_; }
    
    // Для совместимости со старым API
    bool isLinearModel() const { return true; }  // Всегда линейная модель
    size_t getCalibrationPointCount() const { return 0; }

private:
    double max_rps_;         // Максимальные обороты в секунду при duty = 100%
    double wheel_radius_;    // Радиус колеса в метрах
    double max_speed_mps_;   // Расчётная максимальная линейная скорость (м/с)
    std::string label_;      // Метка колеса
};

} // namespace vesc_nexus
