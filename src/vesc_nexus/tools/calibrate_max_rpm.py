#!/usr/bin/env python3
"""
Скрипт автоматической калибровки максимальных оборотов VESC при duty=100%

ИСПОЛЬЗОВАНИЕ:
    python3 calibrate_max_rpm.py --vesc-ids 49 124 81 94 --pole-pairs 15

ОПИСАНИЕ:
    Для каждого VESC ID:
    1. Разгоняет колесо постепенно увеличивая duty cycle
    2. Останавливается когда RPM перестаёт значительно расти (насыщение)
    3. Замеряет максимальные обороты в обоих направлениях (вперёд/назад)
    4. Конвертирует ERPM в механический RPM через pole_pairs
    5. Сохраняет результаты в YAML файл

ВАЖНО:
    --pole-pairs должен быть количество_полюсов / 2
    Например: если у мотора 30 полюсов, то --pole-pairs 15

ТРЕБОВАНИЯ:
    pip install python-can pyyaml
"""

import argparse
import can
import struct
import time
import yaml
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional
from datetime import datetime


# VESC CAN Protocol Constants
CAN_PACKET_SET_DUTY = 0
CAN_PACKET_STATUS = 9

# Настройки калибровки
DUTY_STEP = 0.05          # Шаг увеличения duty (5%)
DUTY_MAX = 1.0            # Максимальный duty cycle
SETTLE_TIME = 1.0         # Время стабилизации оборотов (сек)
RPM_CHANGE_THRESHOLD = 5  # Порог изменения механического RPM для детекции насыщения
MAX_ITERATIONS = 25       # Максимум итераций разгона
MAX_CONSECUTIVE_CAN_ERRORS = 10  # Порог подряд идущих ошибок CAN — после него калибровка прерывается


@dataclass
class CalibrationResult:
    """Результат калибровки одного колеса."""
    vesc_id: int
    max_erpm_forward: float
    max_erpm_backward: float
    max_rpm_forward: float
    max_rpm_backward: float
    max_duty_forward: float
    max_duty_backward: float
    max_rps_forward: float
    max_rps_backward: float
    timestamp: str


class VescCalibrator:
    """Калибратор для VESC моторов."""

    def __init__(self, can_interface: str, pole_pairs: int = 15):
        self.can_interface = can_interface
        self.pole_pairs = pole_pairs  # количество пар полюсов (poles / 2)
        self.bus: Optional[can.Bus] = None
        self.current_rpm: Dict[int, float] = {}  # vesc_id → rpm
        self.consecutive_can_errors = 0  # подряд идущие ошибки CAN (read/send)

    def connect(self) -> bool:
        """Подключение к CAN шине."""
        try:
            self.bus = can.Bus(channel=self.can_interface, interface='socketcan')
            print(f"✅ Подключено к CAN интерфейсу: {self.can_interface}")
            return True
        except Exception as e:
            print(f"❌ Ошибка подключения к CAN: {e}")
            return False

    def disconnect(self):
        """Отключение от CAN шины."""
        if self.bus:
            self.bus.shutdown()
            print("Отключено от CAN шины")

    def send_duty_cycle(self, vesc_id: int, duty: float):
        """Отправка команды duty cycle на VESC."""
        duty_scaled = int(duty * 100000)  # VESC формат: duty * 100000
        data = struct.pack('>i', duty_scaled)

        # CAN ID: command_id << 8 | vesc_id
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
        """Учесть ошибку CAN: предупредить и прервать калибровку после порога подряд идущих сбоев."""
        self.consecutive_can_errors += 1
        print(f"⚠️  Ошибка CAN ({operation}): {error} "
              f"[{self.consecutive_can_errors}/{MAX_CONSECUTIVE_CAN_ERRORS} подряд]")
        if self.consecutive_can_errors >= MAX_CONSECUTIVE_CAN_ERRORS:
            raise RuntimeError(
                f"CAN-шина нестабильна: {self.consecutive_can_errors} ошибок подряд "
                f"({operation}: {error}). Прерываю калибровку — проверьте подключение CAN."
            ) from error

    def read_status(self, timeout: float = 0.1) -> Optional[tuple]:
        """Чтение статуса VESC (ERPM, current, duty)."""
        try:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                return None

            vesc_id = msg.arbitration_id & 0xFF
            cmd_id = (msg.arbitration_id >> 8) & 0xFF

            if cmd_id == CAN_PACKET_STATUS and len(msg.data) >= 8:
                erpm = struct.unpack('>i', msg.data[0:4])[0]
                current = struct.unpack('>h', msg.data[4:6])[0] / 10.0
                duty = struct.unpack('>h', msg.data[6:8])[0] / 1000.0
                # Конвертируем ERPM в механический RPM
                mechanical_rpm = erpm / self.pole_pairs
                self.consecutive_can_errors = 0
                return (vesc_id, mechanical_rpm, current, duty)
        except (can.CanError, struct.error, ValueError) as e:
            self._register_can_error("чтение статуса", e)
        return None

    def get_stable_rpm(self, vesc_id: int, duty: float, settle_time: float = SETTLE_TIME) -> float:
        """Подать duty и дождаться стабильного RPM."""
        self.send_duty_cycle(vesc_id, duty)
        time.sleep(settle_time)

        # Считываем несколько значений и усредняем
        rpm_values = []
        start_time = time.time()
        while time.time() - start_time < 0.5:
            result = self.read_status(timeout=0.1)
            if result and result[0] == vesc_id:
                rpm_values.append(result[1])

        if rpm_values:
            return sum(rpm_values) / len(rpm_values)

        # Ни одного пакета статуса от VESC за окно измерений — это не «0 RPM»,
        # а признак проблем с шиной/ID. Предупреждаем, чтобы калибровка не
        # записала мусор как «максимальные обороты = 0».
        print(f"⚠️  VESC {vesc_id} не прислал ни одного статуса при duty={duty:+.2f} "
              f"(0 из ~5 измерений) — проверьте CAN и VESC ID")
        return 0.0

    def find_max_rpm(self, vesc_id: int, direction: int = 1) -> tuple:
        """
        Найти максимальные обороты разгоняя колесо до насыщения

        Args:
            vesc_id: ID VESC контроллера
            direction: 1 для вперёд, -1 для назад

        Returns:
            (max_erpm, max_rpm, duty_at_max_rpm)
        """
        direction_name = "ВПЕРЁД" if direction > 0 else "НАЗАД"
        print(f"\n  📊 Калибровка {direction_name}...")

        prev_rpm = 0.0
        max_rpm = 0.0
        max_erpm = 0.0
        duty_at_max = 0.0

        for i in range(MAX_ITERATIONS):
            duty = min(DUTY_STEP * (i + 1), DUTY_MAX) * direction

            current_rpm = self.get_stable_rpm(vesc_id, duty)
            rpm_change = abs(current_rpm) - abs(prev_rpm)

            print(f"    duty={duty:+.2f} → RPM={current_rpm:.0f} (Δ={rpm_change:.0f})")

            # Обновляем максимум
            if abs(current_rpm) > abs(max_rpm):
                max_rpm = current_rpm
                max_erpm = current_rpm * self.pole_pairs  # сохраняем ERPM для справки
                duty_at_max = duty

            # Детекция насыщения: если RPM почти не растёт
            if i > 2 and rpm_change < RPM_CHANGE_THRESHOLD:
                print(f"    ✓ Насыщение достигнуто при duty={duty:.2f}")
                break

            prev_rpm = current_rpm

            if abs(duty) >= DUTY_MAX:
                print(f"    ✓ Достигнут максимальный duty={DUTY_MAX}")
                break

        # Остановка
        self.send_duty_cycle(vesc_id, 0.0)
        time.sleep(0.5)

        return (abs(max_erpm), abs(max_rpm), abs(duty_at_max))

    def calibrate_wheel(self, vesc_id: int) -> CalibrationResult:
        """Полная калибровка одного колеса."""
        print(f"\n{'='*50}")
        print(f"🔧 Калибровка VESC ID: {vesc_id}")
        print(f"{'='*50}")

        # Калибровка вперёд
        max_erpm_fwd, max_rpm_fwd, duty_fwd = self.find_max_rpm(vesc_id, direction=1)

        # Пауза между направлениями
        print("\n  ⏸️  Пауза 2 сек...")
        time.sleep(2.0)

        # Калибровка назад
        max_erpm_bwd, max_rpm_bwd, duty_bwd = self.find_max_rpm(vesc_id, direction=-1)

        # Результаты
        max_rps_fwd = max_rpm_fwd / 60.0
        max_rps_bwd = max_rpm_bwd / 60.0

        result = CalibrationResult(
            vesc_id=vesc_id,
            max_erpm_forward=max_erpm_fwd,
            max_erpm_backward=max_erpm_bwd,
            max_rpm_forward=max_rpm_fwd,
            max_rpm_backward=max_rpm_bwd,
            max_duty_forward=duty_fwd,
            max_duty_backward=duty_bwd,
            max_rps_forward=max_rps_fwd,
            max_rps_backward=max_rps_bwd,
            timestamp=datetime.now().isoformat()
        )

        print(f"\n  📋 Результаты:")
        print(f"    Вперёд:  {max_erpm_fwd:.0f} ERPM = {max_rpm_fwd:.0f} RPM = {max_rps_fwd:.2f} об/сек при duty={duty_fwd:.2f}")
        print(f"    Назад:   {max_erpm_bwd:.0f} ERPM = {max_rpm_bwd:.0f} RPM = {max_rps_bwd:.2f} об/сек при duty={duty_bwd:.2f}")

        return result


def save_results(results: List[CalibrationResult], output_file: str, pole_pairs: int):
    """Сохранение результатов в YAML файл."""
    data = {
        'calibration': {
            'timestamp': datetime.now().isoformat(),
            'description': 'Результаты автокалибровки VESC',
            'pole_pairs': pole_pairs,
            'note': 'ERPM конвертирован в механический RPM через pole_pairs'
        },
        'wheels': {}
    }

    # Для конфига драйвера
    wheel_max_rps = []

    for r in results:
        # Используем среднее от forward и backward
        avg_rps = (r.max_rps_forward + r.max_rps_backward) / 2.0
        wheel_max_rps.append(round(avg_rps, 2))

        data['wheels'][f'vesc_{r.vesc_id}'] = {
            'vesc_id': r.vesc_id,
            'max_erpm_forward': round(r.max_erpm_forward, 1),
            'max_erpm_backward': round(r.max_erpm_backward, 1),
            'max_rpm_forward': round(r.max_rpm_forward, 1),
            'max_rpm_backward': round(r.max_rpm_backward, 1),
            'max_rps_forward': round(r.max_rps_forward, 2),
            'max_rps_backward': round(r.max_rps_backward, 2),
            'avg_max_rps': round(avg_rps, 2),
            'timestamp': r.timestamp
        }

    data['config_snippet'] = {
        'wheel_max_rps': wheel_max_rps,
        'description': 'Скопируйте в vesc_nexus_config.yaml'
    }

    with open(output_file, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)

    print(f"\n✅ Результаты сохранены в: {output_file}")
    print(f"\n📋 Для конфига драйвера добавьте:")
    print(f"   wheel_max_rps: {wheel_max_rps}")


def main():
    parser = argparse.ArgumentParser(
        description='Автокалибровка максимальных оборотов VESC моторов'
    )
    parser.add_argument(
        '--vesc-ids',
        type=int,
        nargs='+',
        required=True,
        help='Список VESC ID для калибровки (например: 49 124 81 94)'
    )
    parser.add_argument(
        '--can-interface',
        type=str,
        default='can0',
        help='CAN интерфейс (по умолчанию: can0)'
    )
    parser.add_argument(
        '--pole-pairs',
        type=int,
        default=15,
        help='Количество пар полюсов мотора (poles/2, по умолчанию: 15)'
    )
    parser.add_argument(
        '--output',
        type=str,
        default='calibration_results.yaml',
        help='Файл для сохранения результатов'
    )

    args = parser.parse_args()

    print("🚀 VESC Калибровка максимальных оборотов")
    print("="*50)
    print(f"VESC IDs: {args.vesc_ids}")
    print(f"CAN интерфейс: {args.can_interface}")
    print(f"Пар полюсов: {args.pole_pairs} (конвертирует ERPM → RPM)")
    print(f"Выходной файл: {args.output}")
    print("="*50)

    print("\n⚠️  ВНИМАНИЕ!")
    print("   Убедитесь что робот стоит на подставке")
    print("   и колёса могут свободно вращаться!")
    input("\nНажмите Enter для начала калибровки...")

    calibrator = VescCalibrator(args.can_interface, pole_pairs=args.pole_pairs)

    if not calibrator.connect():
        sys.exit(1)

    try:
        results = []
        for vesc_id in args.vesc_ids:
            result = calibrator.calibrate_wheel(vesc_id)
            results.append(result)

        save_results(results, args.output, args.pole_pairs)

    finally:
        calibrator.disconnect()

    print("\n✅ Калибровка завершена!")


if __name__ == '__main__':
    main()
