#!/usr/bin/env python3
"""
Скрипт для исследования линейности зависимости duty cycle → RPM

ИСПОЛЬЗОВАНИЕ:
    python3 linearity_test.py --vesc-id 49 --can-interface can0

ОПИСАНИЕ:
    1. Постепенно увеличивает duty cycle с заданным шагом (по умолчанию 1%)
    2. Замеряет RPM на каждом шаге
    3. Сохраняет результаты в CSV и JSON
    4. Генерирует график зависимости

ТРЕБОВАНИЯ:
    pip install python-can matplotlib pandas
"""

import argparse
import can
import struct
import time
import json
import csv
import sys
from dataclasses import dataclass, asdict
from typing import List, Optional
from datetime import datetime

try:
    import matplotlib.pyplot as plt
    import matplotlib
    matplotlib.use('Agg')  # Для работы без дисплея
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False
    print("⚠️  matplotlib не установлен. Графики не будут генерироваться.")
    print("   Установите: pip install matplotlib")


# VESC CAN Protocol Constants
CAN_PACKET_SET_DUTY = 0
CAN_PACKET_STATUS = 9

# Настройки теста
DEFAULT_DUTY_STEP = 0.01   # Шаг 1%
DEFAULT_DUTY_MAX = 1.0     # Максимум 100%
SETTLE_TIME = 0.8          # Время стабилизации
SAMPLES_PER_POINT = 5      # Количество измерений для усреднения
MAX_CONSECUTIVE_CAN_ERRORS = 10  # Порог подряд идущих ошибок CAN — после него тест прерывается


@dataclass
class MeasurementPoint:
    """Одна точка измерения."""
    duty_cycle: float
    rpm: float
    rps: float
    current: float
    timestamp: str


@dataclass
class LinearityTestResult:
    """Результат теста линейности."""
    vesc_id: int
    direction: str  # "forward" или "backward"
    points: List[MeasurementPoint]
    r_squared: float  # Коэффициент детерминации R²
    slope: float      # Наклон линии регрессии
    intercept: float  # Пересечение с осью Y


class VescLinearityTester:
    """Тестер линейности VESC."""

    def __init__(self, can_interface: str):
        self.can_interface = can_interface
        self.bus: Optional[can.Bus] = None
        self.consecutive_can_errors = 0  # подряд идущие ошибки CAN (read/send)

    def connect(self) -> bool:
        """Подключение к CAN шине."""
        try:
            self.bus = can.Bus(channel=self.can_interface, interface='socketcan')
            print(f"✅ Подключено к CAN: {self.can_interface}")
            return True
        except Exception as e:
            print(f"❌ Ошибка подключения: {e}")
            return False

    def disconnect(self):
        """Отключение."""
        if self.bus:
            self.bus.shutdown()

    def send_duty_cycle(self, vesc_id: int, duty: float):
        """Отправка команды duty cycle."""
        duty_scaled = int(duty * 100000)
        data = struct.pack('>i', duty_scaled)
        can_id = (CAN_PACKET_SET_DUTY << 8) | vesc_id
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        try:
            self.bus.send(msg)
            self.consecutive_can_errors = 0
        except can.CanError as e:
            self._register_can_error(f"отправка duty={duty:+.2f} на VESC {vesc_id}", e)

    def _register_can_error(self, operation: str, error: Exception):
        """Учесть ошибку CAN: предупредить и прервать тест после порога подряд идущих сбоев."""
        self.consecutive_can_errors += 1
        print(f"⚠️  Ошибка CAN ({operation}): {error} "
              f"[{self.consecutive_can_errors}/{MAX_CONSECUTIVE_CAN_ERRORS} подряд]")
        if self.consecutive_can_errors >= MAX_CONSECUTIVE_CAN_ERRORS:
            raise RuntimeError(
                f"CAN-шина нестабильна: {self.consecutive_can_errors} ошибок подряд "
                f"({operation}: {error}). Прерываю тест — проверьте подключение CAN."
            ) from error

    def read_status(self, timeout: float = 0.1) -> Optional[tuple]:
        """Чтение статуса VESC."""
        try:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                return None

            vesc_id = msg.arbitration_id & 0xFF
            cmd_id = (msg.arbitration_id >> 8) & 0xFF

            if cmd_id == CAN_PACKET_STATUS and len(msg.data) >= 8:
                erpm = struct.unpack('>i', msg.data[0:4])[0]
                current = struct.unpack('>h', msg.data[4:6])[0] / 10.0
                self.consecutive_can_errors = 0
                return (vesc_id, erpm, current)
        except (can.CanError, struct.error, ValueError) as e:
            self._register_can_error("чтение статуса", e)
        return None

    def measure_point(self, vesc_id: int, duty: float) -> MeasurementPoint:
        """Измерение одной точки с усреднением."""
        self.send_duty_cycle(vesc_id, duty)
        time.sleep(SETTLE_TIME)

        rpm_values = []
        current_values = []

        # Собираем несколько измерений
        start = time.time()
        while len(rpm_values) < SAMPLES_PER_POINT and time.time() - start < 2.0:
            result = self.read_status(timeout=0.1)
            if result and result[0] == vesc_id:
                rpm_values.append(result[1])
                current_values.append(result[2])

        avg_rpm = sum(rpm_values) / len(rpm_values) if rpm_values else 0.0
        avg_current = sum(current_values) / len(current_values) if current_values else 0.0

        if not rpm_values:
            # Ноль измерений за окно — это не «мотор стоит», а признак проблем
            # с шиной/ID. Предупреждаем, чтобы линейность не строилась по мусору.
            print(f"⚠️  VESC {vesc_id} не прислал ни одного статуса при duty={duty:+.2f} "
                  f"(0 из {SAMPLES_PER_POINT} измерений) — проверьте CAN и VESC ID")

        return MeasurementPoint(
            duty_cycle=duty,
            rpm=avg_rpm,
            rps=avg_rpm / 60.0,
            current=avg_current,
            timestamp=datetime.now().isoformat()
        )

    def run_linearity_test(
        self,
        vesc_id: int,
        duty_step: float = DEFAULT_DUTY_STEP,
        duty_max: float = DEFAULT_DUTY_MAX,
        direction: int = 1
    ) -> LinearityTestResult:
        """
        Запуск теста линейности

        Args:
            vesc_id: ID VESC
            duty_step: Шаг duty cycle
            duty_max: Максимальный duty
            direction: 1 для вперёд, -1 для назад
        """
        dir_name = "forward" if direction > 0 else "backward"
        dir_ru = "ВПЕРЁД" if direction > 0 else "НАЗАД"

        print(f"\n📊 Тест линейности {dir_ru} для VESC {vesc_id}")
        print(f"   Шаг: {duty_step*100:.1f}%, Максимум: {duty_max*100:.0f}%")
        print("-" * 50)

        points = []
        duty = 0.0

        while duty <= duty_max:
            actual_duty = duty * direction
            point = self.measure_point(vesc_id, actual_duty)
            points.append(point)

            print(f"   duty={actual_duty:+.2f} → RPM={point.rpm:8.1f}, "
                  f"RPS={point.rps:6.2f}, I={point.current:5.1f}A")

            duty += duty_step

        # Остановка
        self.send_duty_cycle(vesc_id, 0.0)
        time.sleep(0.5)

        # Расчёт линейной регрессии
        r_squared, slope, intercept = self._calculate_regression(points)

        return LinearityTestResult(
            vesc_id=vesc_id,
            direction=dir_name,
            points=points,
            r_squared=r_squared,
            slope=slope,
            intercept=intercept
        )

    def _calculate_regression(self, points: List[MeasurementPoint]) -> tuple:
        """Расчёт линейной регрессии и R²."""
        if len(points) < 2:
            return (0.0, 0.0, 0.0)

        x = [abs(p.duty_cycle) for p in points]
        y = [abs(p.rpm) for p in points]

        n = len(x)
        sum_x = sum(x)
        sum_y = sum(y)
        sum_xy = sum(xi * yi for xi, yi in zip(x, y))
        sum_x2 = sum(xi ** 2 for xi in x)

        # Наклон и пересечение
        denom = n * sum_x2 - sum_x ** 2
        if abs(denom) < 1e-10:
            return (0.0, 0.0, sum_y / n if n > 0 else 0.0)

        slope = (n * sum_xy - sum_x * sum_y) / denom
        intercept = (sum_y - slope * sum_x) / n

        # R²
        y_mean = sum_y / n
        ss_tot = sum((yi - y_mean) ** 2 for yi in y)
        ss_res = sum((yi - (slope * xi + intercept)) ** 2 for xi, yi in zip(x, y))

        r_squared = 1 - (ss_res / ss_tot) if ss_tot > 0 else 0.0

        return (r_squared, slope, intercept)


def save_results_csv(result: LinearityTestResult, filename: str):
    """Сохранение в CSV."""
    with open(filename, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(['duty_cycle', 'rpm', 'rps', 'current', 'timestamp'])
        for p in result.points:
            writer.writerow([p.duty_cycle, p.rpm, p.rps, p.current, p.timestamp])
    print(f"📄 CSV сохранён: {filename}")


def save_results_json(result: LinearityTestResult, filename: str):
    """Сохранение в JSON."""
    data = {
        'vesc_id': result.vesc_id,
        'direction': result.direction,
        'r_squared': result.r_squared,
        'slope': result.slope,
        'intercept': result.intercept,
        'linearity': 'ХОРОШАЯ' if result.r_squared > 0.95 else 'ПЛОХАЯ',
        'points': [asdict(p) for p in result.points]
    }
    with open(filename, 'w') as f:
        json.dump(data, f, indent=2)
    print(f"📄 JSON сохранён: {filename}")


def generate_plot(results: List[LinearityTestResult], filename: str):
    """Генерация графика."""
    if not HAS_MATPLOTLIB:
        return

    fig, axes = plt.subplots(1, 2, figsize=(14, 6))

    for i, result in enumerate(results):
        ax = axes[i] if len(results) > 1 else axes[0]

        duties = [abs(p.duty_cycle) * 100 for p in result.points]
        rpms = [abs(p.rpm) for p in result.points]

        # Точки данных
        ax.scatter(duties, rpms, label='Измерения', alpha=0.7, s=50)

        # Линия регрессии
        if result.slope != 0:
            x_line = [0, max(duties)]
            y_line = [result.intercept, result.slope * max(duties)/100 + result.intercept]
            ax.plot(x_line, y_line, 'r--', label=f'Регрессия (R²={result.r_squared:.4f})')

        ax.set_xlabel('Duty Cycle (%)')
        ax.set_ylabel('RPM')
        ax.set_title(f'VESC {result.vesc_id} - {result.direction.upper()}\n'
                     f'R² = {result.r_squared:.4f} '
                     f'({"Линейно ✓" if result.r_squared > 0.95 else "Нелинейно ✗"})')
        ax.legend()
        ax.grid(True, alpha=0.3)

    plt.tight_layout()
    plt.savefig(filename, dpi=150)
    print(f"📊 График сохранён: {filename}")


def main():
    parser = argparse.ArgumentParser(
        description='Тест линейности зависимости duty cycle → RPM'
    )
    parser.add_argument(
        '--vesc-id',
        type=int,
        required=True,
        help='VESC ID для тестирования'
    )
    parser.add_argument(
        '--can-interface',
        type=str,
        default='can0',
        help='CAN интерфейс (по умолчанию: can0)'
    )
    parser.add_argument(
        '--duty-step',
        type=float,
        default=DEFAULT_DUTY_STEP,
        help=f'Шаг duty cycle (по умолчанию: {DEFAULT_DUTY_STEP})'
    )
    parser.add_argument(
        '--duty-max',
        type=float,
        default=DEFAULT_DUTY_MAX,
        help=f'Максимальный duty cycle (по умолчанию: {DEFAULT_DUTY_MAX})'
    )
    parser.add_argument(
        '--output-prefix',
        type=str,
        default='linearity_test',
        help='Префикс для выходных файлов'
    )
    parser.add_argument(
        '--both-directions',
        action='store_true',
        help='Тестировать оба направления'
    )

    args = parser.parse_args()

    print("🔬 VESC Тест линейности duty → RPM")
    print("=" * 50)
    print(f"VESC ID: {args.vesc_id}")
    print(f"CAN: {args.can_interface}")
    print(f"Шаг: {args.duty_step * 100:.1f}%")
    print(f"Максимум: {args.duty_max * 100:.0f}%")
    print("=" * 50)

    print("\n⚠️  Убедитесь что колесо может свободно вращаться!")
    input("Нажмите Enter для начала теста...")

    tester = VescLinearityTester(args.can_interface)

    if not tester.connect():
        sys.exit(1)

    try:
        results = []

        # Тест вперёд
        result_fwd = tester.run_linearity_test(
            args.vesc_id,
            args.duty_step,
            args.duty_max,
            direction=1
        )
        results.append(result_fwd)

        # Тест назад (если запрошено)
        if args.both_directions:
            print("\n⏸️  Пауза 2 сек перед тестом назад...")
            time.sleep(2.0)

            result_bwd = tester.run_linearity_test(
                args.vesc_id,
                args.duty_step,
                args.duty_max,
                direction=-1
            )
            results.append(result_bwd)

        # Сохранение результатов
        for result in results:
            prefix = f"{args.output_prefix}_vesc{result.vesc_id}_{result.direction}"
            save_results_csv(result, f"{prefix}.csv")
            save_results_json(result, f"{prefix}.json")

        # Генерация графика
        plot_filename = f"{args.output_prefix}_vesc{args.vesc_id}_plot.png"
        generate_plot(results, plot_filename)

        # Итоговый отчёт
        print("\n" + "=" * 50)
        print("📋 ИТОГОВЫЙ ОТЧЁТ")
        print("=" * 50)
        for result in results:
            linearity = "✅ ЛИНЕЙНО" if result.r_squared > 0.95 else "⚠️ НЕЛИНЕЙНО"
            print(f"\n{result.direction.upper()}:")
            print(f"  R² = {result.r_squared:.4f} → {linearity}")
            print(f"  Наклон: {result.slope:.2f} RPM/duty")
            print(f"  Пересечение: {result.intercept:.2f} RPM")

    finally:
        tester.disconnect()

    print("\n✅ Тест завершён!")


if __name__ == '__main__':
    main()
