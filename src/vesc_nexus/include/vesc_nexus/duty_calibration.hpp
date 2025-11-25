// duty_calibration.hpp
// Класс для калибровки соотношения duty cycle → скорость для каждого колеса VESC
// Позволяет учитывать нелинейности и индивидуальные особенности каждого мотора
#pragma once

#include <vector>
#include <algorithm>
#include <cmath>
#include <string>
#include <stdexcept>

namespace vesc_nexus {

namespace detail {
// Собственная реализация clamp для совместимости с C++14
// (std::clamp доступен только начиная с C++17)
// Проект использует C++14 согласно CMakeLists.txt
template<typename T>
constexpr const T& clamp(const T& value, const T& lo, const T& hi) {
    return (value < lo) ? lo : (value > hi) ? hi : value;
}
}  // namespace detail

/**
 * @brief Калибровочная таблица для преобразования скорости в duty cycle.
 * 
 * Решает проблему рывков при движении робота за счёт:
 * 1. Использования таблицы соответствия duty_cycle → реальная_скорость
 * 2. Линейной интерполяции между точками калибровки
 * 3. Индивидуальной настройки для каждого колеса
 * 
 * Рекомендуемый процесс калибровки:
 * 1. Подать серию значений duty_cycle (например, 0.1, 0.2, ..., 1.0)
 * 2. Измерить реальную скорость колеса (через одометр или тахометр)
 * 3. Заполнить таблицу duty_to_speed_pairs_
 * 4. При работе использовать обратный поиск: speed → duty
 */
class DutyCalibrationTable {
public:
    /**
     * @brief Точка калибровки: пара (duty_cycle, скорость)
     */
    struct CalibrationPoint {
        double duty_cycle;  // Значение duty cycle (-1.0 до 1.0)
        double speed_mps;   // Линейная скорость колеса (м/с)
        
        CalibrationPoint(double d, double s) : duty_cycle(d), speed_mps(s) {}
        
        bool operator<(const CalibrationPoint& other) const {
            return speed_mps < other.speed_mps;
        }
    };

    /**
     * @brief Конструктор с линейной моделью по умолчанию
     * @param max_speed_mps Максимальная скорость при duty=1.0
     * @param label Метка колеса для логирования
     */
    explicit DutyCalibrationTable(double max_speed_mps = 1.0, const std::string& label = "")
        : max_speed_mps_(max_speed_mps), label_(label), use_linear_model_(true) {}

    /**
     * @brief Конструктор с калибровочной таблицей
     * @param points Вектор точек калибровки (duty, speed)
     * @param label Метка колеса для логирования
     */
    DutyCalibrationTable(const std::vector<CalibrationPoint>& points, const std::string& label = "")
        : label_(label), use_linear_model_(false) {
        setCalibrationPoints(points);
    }

    /**
     * @brief Установить точки калибровки
     * @param points Вектор точек калибровки
     * 
     * Точки автоматически сортируются по скорости для корректной интерполяции
     */
    void setCalibrationPoints(const std::vector<CalibrationPoint>& points) {
        calibration_points_ = points;
        
        // Сортируем по скорости для корректного поиска
        std::sort(calibration_points_.begin(), calibration_points_.end());
        
        use_linear_model_ = calibration_points_.size() < 2;
        
        if (!use_linear_model_ && !calibration_points_.empty()) {
            // Находим максимальную скорость из калибровки
            max_speed_mps_ = std::max(
                std::abs(calibration_points_.front().speed_mps),
                std::abs(calibration_points_.back().speed_mps)
            );
        }
    }

    /**
     * @brief Добавить точку калибровки
     * @param duty Значение duty cycle
     * @param speed_mps Измеренная скорость
     */
    void addCalibrationPoint(double duty, double speed_mps) {
        calibration_points_.emplace_back(duty, speed_mps);
        std::sort(calibration_points_.begin(), calibration_points_.end());
        
        if (calibration_points_.size() >= 2) {
            use_linear_model_ = false;
            max_speed_mps_ = std::max(
                std::abs(calibration_points_.front().speed_mps),
                std::abs(calibration_points_.back().speed_mps)
            );
        }
    }

    /**
     * @brief Преобразовать желаемую скорость в duty cycle
     * @param target_speed_mps Целевая линейная скорость (м/с)
     * @return Значение duty cycle для достижения этой скорости
     * 
     * Использует линейную интерполяцию между точками калибровки
     * или линейную модель, если калибровка не задана
     */
    double speedToDuty(double target_speed_mps) const {
        if (use_linear_model_) {
            // Простая линейная модель: duty = speed / max_speed
            double duty = target_speed_mps / max_speed_mps_;
            return detail::clamp(duty, -1.0, 1.0);
        }

        if (calibration_points_.empty()) {
            return 0.0;
        }

        // Находим ближайшие точки для интерполяции
        // Для отрицательных скоростей инвертируем знак
        double sign = (target_speed_mps >= 0) ? 1.0 : -1.0;
        double abs_speed = std::abs(target_speed_mps);

        // Ищем две точки для интерполяции
        const CalibrationPoint* lower = nullptr;
        const CalibrationPoint* upper = nullptr;

        for (const auto& point : calibration_points_) {
            if (point.speed_mps <= abs_speed) {
                lower = &point;
            }
            if (point.speed_mps >= abs_speed && upper == nullptr) {
                upper = &point;
            }
        }

        double duty;
        if (lower == nullptr && upper != nullptr) {
            // Ниже минимальной калибровки — экстраполируем от нуля
            duty = upper->duty_cycle * (abs_speed / upper->speed_mps);
        } else if (upper == nullptr && lower != nullptr) {
            // Выше максимальной калибровки — используем максимум
            duty = lower->duty_cycle;
        } else if (lower != nullptr && upper != nullptr) {
            if (lower == upper || std::abs(upper->speed_mps - lower->speed_mps) < 1e-9) {
                duty = lower->duty_cycle;
            } else {
                // Линейная интерполяция
                double t = (abs_speed - lower->speed_mps) / (upper->speed_mps - lower->speed_mps);
                duty = lower->duty_cycle + t * (upper->duty_cycle - lower->duty_cycle);
            }
        } else {
            duty = 0.0;
        }

        return detail::clamp(sign * duty, -1.0, 1.0);
    }

    /**
     * @brief Преобразовать duty cycle в ожидаемую скорость
     * @param duty_cycle Значение duty cycle
     * @return Ожидаемая линейная скорость (м/с)
     * 
     * Полезно для оценки скорости по текущему duty cycle
     */
    double dutyToSpeed(double duty_cycle) const {
        if (use_linear_model_) {
            return duty_cycle * max_speed_mps_;
        }

        if (calibration_points_.empty()) {
            return 0.0;
        }

        double sign = (duty_cycle >= 0) ? 1.0 : -1.0;
        double abs_duty = std::abs(duty_cycle);

        const CalibrationPoint* lower = nullptr;
        const CalibrationPoint* upper = nullptr;

        // Ищем по duty_cycle
        for (const auto& point : calibration_points_) {
            if (point.duty_cycle <= abs_duty) {
                lower = &point;
            }
            if (point.duty_cycle >= abs_duty && upper == nullptr) {
                upper = &point;
            }
        }

        double speed;
        if (lower == nullptr && upper != nullptr) {
            speed = upper->speed_mps * (abs_duty / upper->duty_cycle);
        } else if (upper == nullptr && lower != nullptr) {
            speed = lower->speed_mps;
        } else if (lower != nullptr && upper != nullptr) {
            if (lower == upper || std::abs(upper->duty_cycle - lower->duty_cycle) < 1e-9) {
                speed = lower->speed_mps;
            } else {
                double t = (abs_duty - lower->duty_cycle) / (upper->duty_cycle - lower->duty_cycle);
                speed = lower->speed_mps + t * (upper->speed_mps - lower->speed_mps);
            }
        } else {
            speed = 0.0;
        }

        return sign * speed;
    }

    /**
     * @brief Получить максимальную скорость
     */
    double getMaxSpeed() const { return max_speed_mps_; }

    /**
     * @brief Установить максимальную скорость для линейной модели
     */
    void setMaxSpeed(double max_speed_mps) { max_speed_mps_ = max_speed_mps; }

    /**
     * @brief Получить метку колеса
     */
    const std::string& getLabel() const { return label_; }

    /**
     * @brief Проверить используется ли линейная модель
     */
    bool isLinearModel() const { return use_linear_model_; }

    /**
     * @brief Получить количество точек калибровки
     */
    size_t getCalibrationPointCount() const { return calibration_points_.size(); }

    /**
     * @brief Получить калибровочные точки (для отладки)
     */
    const std::vector<CalibrationPoint>& getCalibrationPoints() const {
        return calibration_points_;
    }

private:
    std::vector<CalibrationPoint> calibration_points_;
    double max_speed_mps_ = 1.0;
    std::string label_;
    bool use_linear_model_ = true;
};

} // namespace vesc_nexus
