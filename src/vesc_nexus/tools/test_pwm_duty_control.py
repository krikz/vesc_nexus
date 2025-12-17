#!/usr/bin/env python3
"""
Экспериментальный скрипт для тестирования ШИМ управления низкими скоростями

ИСПОЛЬЗОВАНИЕ:
    python3 test_pwm_duty_control.py --vesc-id 49 --pole-pairs 15

ОПИСАНИЕ:
    Проводит серию экспериментов с чередованием duty cycle 0.05 и 0.1
    для управления низкими скоростями колеса.
    
    10 экспериментов по 5 секунд каждый:
    - Эксп 1: 45 раз duty=0.05, 5 раз duty=0.1 (10% времени на 0.1)
    - Эксп 2: 40 раз duty=0.05, 10 раз duty=0.1 (20% времени на 0.1)
    - Эксп 3: 35 раз duty=0.05, 15 раз duty=0.1 (30% времени на 0.1)
    - ...
    - Эксп 10: 0 раз duty=0.05, 50 раз duty=0.1 (100% времени на 0.1)
    
    Частота обновления: 50 Гц (каждые 20 мс)
    Замеряет среднюю скорость (RPM) и строит зависимость.

ВАЖНО:
    --pole-pairs должен быть количество_полюсов / 2
    Например: если у мотора 30 полюсов, то --pole-pairs 15

ТРЕБОВАНИЯ:
    pip install python-can
"""

import argparse
import can
import struct
import time
import sys
from dataclasses import dataclass, asdict
from typing import Dict, List, Optional
from datetime import datetime


# VESC CAN Protocol Constants
CAN_PACKET_SET_DUTY = 0
CAN_PACKET_STATUS = 9

# Настройки эксперимента
UPDATE_RATE_HZ = 50                    # Частота отправки команд (50 Гц = каждые 20 мс)
UPDATE_PERIOD = 1.0 / UPDATE_RATE_HZ   # Период обновления в секундах (0.02 сек)
EXPERIMENT_DURATION = 5.0              # Длительность каждого эксперимента (секунд)
SETTLE_TIME = 1.0                      # Время разгона перед началом измерений
MEASUREMENTS_PER_EXPERIMENT = int(EXPERIMENT_DURATION * UPDATE_RATE_HZ)  # 250 итераций

# Duty cycle значения
DUTY_LOW = 0.05   # Низкий duty cycle
DUTY_HIGH = 0.1   # Высокий duty cycle


@dataclass
class ExperimentResult:
    """Результат одного эксперимента"""
    experiment_number: int
    duty_low_count: int       # Сколько раз послали DUTY_LOW
    duty_high_count: int      # Сколько раз послали DUTY_HIGH
    duty_high_percentage: int # Процент времени на DUTY_HIGH
    average_rpm: float        # Средняя скорость (RPM)
    min_rpm: float            # Минимальная скорость
    max_rpm: float            # Максимальная скорость
    std_rpm: float            # Стандартное отклонение RPM
    average_current: float    # Средний ток (А)
    timestamp: str


class VescPWMTester:
    """Тестер ШИМ управления для VESC моторов"""
    
    def __init__(self, can_interface: str, vesc_id: int, pole_pairs: int = 15):
        self.can_interface = can_interface
        self.vesc_id = vesc_id
        self.pole_pairs = pole_pairs  # количество пар полюсов (poles / 2)
        self.bus: Optional[can.Bus] = None
        
    def connect(self) -> bool:
        """Подключение к CAN шине"""
        try:
            self.bus = can.Bus(channel=self.can_interface, interface='socketcan')
            print(f"✅ Подключено к CAN интерфейсу: {self.can_interface}")
            return True
        except Exception as e:
            print(f"❌ Ошибка подключения к CAN: {e}")
            return False
    
    def disconnect(self):
        """Отключение от CAN шины"""
        if self.bus:
            # Остановка мотора
            self.send_duty_cycle(0.0)
            time.sleep(0.1)
            self.bus.shutdown()
            print("Отключено от CAN шины")
    
    def send_duty_cycle(self, duty: float):
        """Отправка команды duty cycle на VESC"""
        duty_scaled = int(duty * 100000)  # VESC формат: duty * 100000
        data = struct.pack('>i', duty_scaled)
        
        # CAN ID: command_id << 8 | vesc_id
        can_id = (CAN_PACKET_SET_DUTY << 8) | self.vesc_id
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"Ошибка отправки CAN: {e}")
    
    def read_status(self, timeout: float = 0.01) -> Optional[tuple]:
        """Чтение статуса VESC (RPM, current, duty)"""
        try:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                return None
            
            vesc_id = msg.arbitration_id & 0xFF
            cmd_id = (msg.arbitration_id >> 8) & 0xFF
            
            if cmd_id == CAN_PACKET_STATUS and len(msg.data) >= 8 and vesc_id == self.vesc_id:
                erpm = struct.unpack('>i', msg.data[0:4])[0]
                current = struct.unpack('>h', msg.data[4:6])[0] / 10.0
                duty = struct.unpack('>h', msg.data[6:8])[0] / 1000.0
                # Конвертируем ERPM в механический RPM
                mechanical_rpm = erpm / self.pole_pairs
                return (mechanical_rpm, current, duty)
        except Exception:
            pass
        return None
    
    def run_experiment(self, experiment_num: int, duty_high_count: int) -> ExperimentResult:
        """
        Запуск одного эксперимента с заданным количеством HIGH импульсов
        
        Args:
            experiment_num: Номер эксперимента (1-10)
            duty_high_count: Сколько раз за цикл из 50 посылать DUTY_HIGH
        """
        duty_low_count = 50 - duty_high_count
        duty_high_percentage = (duty_high_count / 50) * 100
        
        print(f"\n{'='*60}")
        print(f"Эксперимент {experiment_num}/10")
        print(f"  Duty LOW ({DUTY_LOW}):  {duty_low_count} раз")
        print(f"  Duty HIGH ({DUTY_HIGH}): {duty_high_count} раз")
        print(f"  Процент HIGH: {duty_high_percentage:.0f}%")
        print(f"{'='*60}")
        
        # Создаём паттерн чередования: равномерно распределяем HIGH импульсы
        pattern = []
        if duty_high_count > 0:
            step = 50 / duty_high_count
            for i in range(50):
                # Позиция для HIGH импульса
                high_positions = [int(j * step) for j in range(duty_high_count)]
                if i in high_positions:
                    pattern.append(DUTY_HIGH)
                else:
                    pattern.append(DUTY_LOW)
        else:
            pattern = [DUTY_LOW] * 50
        
        # Разгон
        print(f"Разгон мотора... ({SETTLE_TIME} сек)")
        start_settle = time.time()
        while time.time() - start_settle < SETTLE_TIME:
            idx = int((time.time() - start_settle) / UPDATE_PERIOD) % 50
            self.send_duty_cycle(pattern[idx])
            time.sleep(UPDATE_PERIOD)
        
        # Измерения
        print("Запись измерений...")
        rpm_values = []
        current_values = []
        
        start_time = time.time()
        iteration = 0
        
        while time.time() - start_time < EXPERIMENT_DURATION:
            # Отправка команды по паттерну
            pattern_idx = iteration % 50
            self.send_duty_cycle(pattern[pattern_idx])
            
            # Чтение статуса
            status = self.read_status(timeout=0.005)
            if status:
                rpm, current, _ = status
                rpm_values.append(rpm)
                current_values.append(current)
            
            # Ожидание до следующего цикла
            next_time = start_time + (iteration + 1) * UPDATE_PERIOD
            sleep_time = next_time - time.time()
            if sleep_time > 0:
                time.sleep(sleep_time)
            
            iteration += 1
        
        # Остановка
        self.send_duty_cycle(0.0)
        time.sleep(0.5)
        
        # Анализ результатов
        if not rpm_values:
            print("⚠️  Нет данных!")
            return ExperimentResult(
                experiment_number=experiment_num,
                duty_low_count=duty_low_count,
                duty_high_count=duty_high_count,
                duty_high_percentage=int(duty_high_percentage),
                average_rpm=0.0,
                min_rpm=0.0,
                max_rpm=0.0,
                std_rpm=0.0,
                average_current=0.0,
                timestamp=datetime.now().isoformat()
            )
        
        avg_rpm = sum(rpm_values) / len(rpm_values)
        min_rpm = min(rpm_values)
        max_rpm = max(rpm_values)
        
        # Стандартное отклонение
        mean_rpm = avg_rpm
        variance = sum((x - mean_rpm) ** 2 for x in rpm_values) / len(rpm_values)
        std_rpm = variance ** 0.5
        
        avg_current = sum(current_values) / len(current_values) if current_values else 0.0
        
        print(f"\nРезультаты:")
        print(f"  Средний RPM: {avg_rpm:.1f}")
        print(f"  Диапазон: {min_rpm:.1f} - {max_rpm:.1f}")
        print(f"  Стд. откл.: {std_rpm:.1f}")
        print(f"  Средний ток: {avg_current:.2f} А")
        print(f"  Измерений: {len(rpm_values)}")
        
        return ExperimentResult(
            experiment_number=experiment_num,
            duty_low_count=duty_low_count,
            duty_high_count=duty_high_count,
            duty_high_percentage=int(duty_high_percentage),
            average_rpm=round(avg_rpm, 1),
            min_rpm=round(min_rpm, 1),
            max_rpm=round(max_rpm, 1),
            std_rpm=round(std_rpm, 1),
            average_current=round(avg_current, 2),
            timestamp=datetime.now().isoformat()
        )
    
    def print_results(self, results: List[ExperimentResult]):
        """Вывести результаты в консоль"""
        print("\n" + "="*80)
        print("РЕЗУЛЬТАТЫ ТЕСТИРОВАНИЯ ШИМ УПРАВЛЕНИЯ")
        print("="*80)
        print(f"VESC ID: {self.vesc_id} | Пары полюсов: {self.pole_pairs}")
        print(f"Duty LOW: {DUTY_LOW} | Duty HIGH: {DUTY_HIGH}")
        print(f"Частота: {UPDATE_RATE_HZ} Гц | Длительность: {EXPERIMENT_DURATION} сек")
        print("="*80)
        print(f"{'№':<4} {'HIGH%':<8} {'RPM ср':<12} {'RPM мин':<12} {'RPM макс':<12} {'RPM σ':<10} {'Ток':<8}")
        print("-"*80)
        
        for r in results:
            print(f"{r.experiment_number:<4} "
                  f"{r.duty_high_percentage:<8} "
                  f"{r.average_rpm:<12.1f} "
                  f"{r.min_rpm:<12.1f} "
                  f"{r.max_rpm:<12.1f} "
                  f"{r.std_rpm:<10.1f} "
                  f"{r.average_current:<8.2f}")
        
        print("="*80)
        print(f"Всего экспериментов: {len(results)}")
        print("="*80 + "\n")
    
    def run_all_experiments(self) -> List[ExperimentResult]:
        """Запуск всех 10 экспериментов"""
        results = []
        
        # 10 экспериментов с разным соотношением
        # Эксп 1: 10% HIGH, Эксп 2: 20% HIGH, ..., Эксп 10: 100% HIGH
        for i in range(1, 11):
            duty_high_count = i * 5  # 5, 10, 15, ..., 50
            result = self.run_experiment(i, duty_high_count)
            results.append(result)
            
            # Пауза между экспериментами
            if i < 10:
                print(f"\nПауза перед следующим экспериментом... (2 сек)")
                time.sleep(2.0)
        
        return results


def main():
    parser = argparse.ArgumentParser(
        description='Экспериментальное тестирование ШИМ управления низкими скоростями VESC',
        formatter_class=argparse.RawDescriptionHelpFormatter
    )
    
    parser.add_argument('--vesc-id', type=int, required=True,
                        help='ID VESC контроллера (например, 49 для переднего левого)')
    parser.add_argument('--pole-pairs', type=int, default=15,
                        help='Количество пар полюсов мотора (poles/2, по умолчанию 15)')
    parser.add_argument('--can-interface', type=str, default='can0',
                        help='CAN интерфейс (по умолчанию can0)')
    
    args = parser.parse_args()
    
    print("\n" + "="*60)
    print("ЭКСПЕРИМЕНТАЛЬНОЕ ТЕСТИРОВАНИЕ ШИМ УПРАВЛЕНИЯ")
    print("="*60)
    print(f"VESC ID: {args.vesc_id}")
    print(f"Пары полюсов: {args.pole_pairs}")
    print(f"CAN интерфейс: {args.can_interface}")
    print(f"Duty LOW: {DUTY_LOW}")
    print(f"Duty HIGH: {DUTY_HIGH}")
    print(f"Частота обновления: {UPDATE_RATE_HZ} Гц")
    print(f"Длительность эксперимента: {EXPERIMENT_DURATION} сек")
    print("="*60)
    
    # Инициализация
    tester = VescPWMTester(args.can_interface, args.vesc_id, args.pole_pairs)
    
    if not tester.connect():
        sys.exit(1)
    
    try:
        # Запуск экспериментов
        results = tester.run_all_experiments()
        
        # Вывод результатов
        tester.print_results(results)
        
    except KeyboardInterrupt:
        print("\n\n⚠️  Прервано пользователем")
    finally:
        tester.disconnect()


if __name__ == '__main__':
    main()
