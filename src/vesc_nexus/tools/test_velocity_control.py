#!/usr/bin/env python3
"""
Скрипт тестирования VESC Velocity Control Mode через CAN шину

ИСПОЛЬЗОВАНИЕ:
    # Интерактивный режим - управление с клавиатуры
    python3 test_velocity_control.py --vesc-id 49 --pole-pairs 15 --interactive
    
    # Тестовая последовательность
    python3 test_velocity_control.py --vesc-id 49 --pole-pairs 15 --test-sequence
    
    # Задать конкретную скорость
    python3 test_velocity_control.py --vesc-id 49 --pole-pairs 15 --rpm 100

ОПИСАНИЕ:
    Отправляет команды velocity (RPM) на VESC вместо duty cycle.
    Позволяет проверить:
    - Как VESC реагирует на команды RPM
    - Насколько точно держит заданную скорость
    - Время выхода на режим
    - Стабильность регулирования

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
import sys
from typing import Optional, Dict
from dataclasses import dataclass
from datetime import datetime


# VESC CAN Protocol Constants
CAN_PACKET_SET_DUTY = 0
CAN_PACKET_SET_RPM = 3
CAN_PACKET_STATUS = 9

# Настройки
STATUS_UPDATE_RATE = 0.1  # Частота опроса статуса (сек)


@dataclass
class VescStatus:
    """Статус VESC контроллера"""
    vesc_id: int
    erpm: float
    rpm: float
    current: float
    duty: float
    timestamp: float


class VescVelocityController:
    """Контроллер для управления VESC в режиме velocity"""
    
    def __init__(self, can_interface: str, vesc_id: int, pole_pairs: int = 15):
        self.can_interface = can_interface
        self.vesc_id = vesc_id
        self.pole_pairs = pole_pairs  # количество пар полюсов (poles / 2)
        self.bus: Optional[can.Bus] = None
        self.last_status: Optional[VescStatus] = None
        
    def connect(self) -> bool:
        """Подключение к CAN шине"""
        try:
            self.bus = can.Bus(channel=self.can_interface, interface='socketcan')
            print(f"✅ Подключено к CAN интерфейсу: {self.can_interface}")
            print(f"   VESC ID: {self.vesc_id}, Pole pairs: {self.pole_pairs}")
            return True
        except Exception as e:
            print(f"❌ Ошибка подключения к CAN: {e}")
            return False
    
    def disconnect(self):
        """Отключение от CAN шины"""
        if self.bus:
            # Отправляем stop перед отключением
            self.set_rpm(0)
            time.sleep(0.2)
            self.bus.shutdown()
            print("\n✅ Отключено от CAN шины")
    
    def set_rpm(self, rpm: float):
        """
        Отправка команды RPM на VESC
        
        Args:
            rpm: Механический RPM (положительный - вперёд, отрицательный - назад)
        """
        # Конвертируем механический RPM в ERPM
        erpm = int(rpm * self.pole_pairs)
        
        # Пакуем в формат VESC: 4 байта signed int
        data = struct.pack('>i', erpm)
        
        # CAN ID: command_id << 8 | vesc_id
        can_id = (CAN_PACKET_SET_RPM << 8) | self.vesc_id
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        
        try:
            self.bus.send(msg)
            print(f"📤 Sent: RPM={rpm:.0f} (ERPM={erpm})")
        except can.CanError as e:
            print(f"❌ Ошибка отправки CAN: {e}")
    
    def set_duty(self, duty: float):
        """
        Отправка команды duty cycle на VESC (для сравнения)
        
        Args:
            duty: Duty cycle от -1.0 до 1.0
        """
        duty_scaled = int(duty * 100000)  # VESC формат: duty * 100000
        data = struct.pack('>i', duty_scaled)
        
        can_id = (CAN_PACKET_SET_DUTY << 8) | self.vesc_id
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        
        try:
            self.bus.send(msg)
            print(f"📤 Sent: Duty={duty:.3f}")
        except can.CanError as e:
            print(f"❌ Ошибка отправки CAN: {e}")
    
    def read_status(self, timeout: float = 0.1) -> Optional[VescStatus]:
        """
        Чтение статуса VESC (ERPM, current, duty)
        
        Returns:
            VescStatus или None если нет данных
        """
        try:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                return None
            
            vesc_id = msg.arbitration_id & 0xFF
            cmd_id = (msg.arbitration_id >> 8) & 0xFF
            
            # Проверяем что это наш VESC и это статус
            if vesc_id != self.vesc_id:
                return None
                
            if cmd_id == CAN_PACKET_STATUS and len(msg.data) >= 8:
                erpm = struct.unpack('>i', msg.data[0:4])[0]
                current = struct.unpack('>h', msg.data[4:6])[0] / 10.0
                duty = struct.unpack('>h', msg.data[6:8])[0] / 1000.0
                
                # Конвертируем ERPM в механический RPM
                mechanical_rpm = erpm / self.pole_pairs
                
                status = VescStatus(
                    vesc_id=vesc_id,
                    erpm=erpm,
                    rpm=mechanical_rpm,
                    current=current,
                    duty=duty,
                    timestamp=time.time()
                )
                
                self.last_status = status
                return status
                
        except Exception as e:
            print(f"⚠️  Ошибка чтения статуса: {e}")
        
        return None
    
    def monitor_status(self, duration: float, target_rpm: Optional[float] = None):
        """
        Мониторинг статуса VESC в течение заданного времени
        
        Args:
            duration: Длительность мониторинга (сек)
            target_rpm: Целевой RPM для отображения ошибки (опционально)
        """
        print(f"\n📊 Мониторинг статуса ({duration:.1f} сек)...")
        print(f"{'Time':>6} | {'RPM':>8} | {'Current':>8} | {'Duty':>7} | {'Error':>8}")
        print("-" * 60)
        
        start_time = time.time()
        rpm_samples = []
        
        while time.time() - start_time < duration:
            status = self.read_status(timeout=0.1)
            
            if status:
                elapsed = time.time() - start_time
                error_str = ""
                
                if target_rpm is not None:
                    error = status.rpm - target_rpm
                    error_str = f"{error:+7.1f}"
                    rpm_samples.append(status.rpm)
                else:
                    error_str = "---"
                
                print(f"{elapsed:6.2f} | {status.rpm:8.1f} | {status.current:8.2f} | {status.duty:7.3f} | {error_str}")
            
            time.sleep(0.1)
        
        # Статистика если был target
        if target_rpm is not None and rpm_samples:
            avg_rpm = sum(rpm_samples) / len(rpm_samples)
            avg_error = avg_rpm - target_rpm
            max_rpm = max(rpm_samples)
            min_rpm = min(rpm_samples)
            
            print("\n📈 Статистика:")
            print(f"   Target RPM:  {target_rpm:.1f}")
            print(f"   Actual RPM:  {avg_rpm:.1f} (avg)")
            print(f"   Error:       {avg_error:+.1f} ({abs(avg_error/target_rpm)*100:.1f}%)")
            print(f"   Range:       {min_rpm:.1f} ... {max_rpm:.1f}")


def interactive_mode(controller: VescVelocityController):
    """Интерактивный режим управления с клавиатуры"""
    print("\n" + "="*60)
    print("🎮 ИНТЕРАКТИВНЫЙ РЕЖИМ")
    print("="*60)
    print("\nКоманды:")
    print("  r <rpm>    - Задать RPM (например: r 100)")
    print("  d <duty>   - Задать Duty cycle (например: d 0.1)")
    print("  s          - Стоп (RPM=0)")
    print("  m [сек]    - Мониторинг статуса (по умолчанию 5 сек)")
    print("  q          - Выход")
    print("\nПримеры:")
    print("  r 50       - Разогнать до 50 RPM")
    print("  r -30      - Назад 30 RPM")
    print("  d 0.15     - Duty cycle 15%")
    print("  m 10       - Мониторить 10 секунд")
    print("-"*60)
    
    try:
        while True:
            try:
                cmd = input("\n> ").strip().lower()
                
                if not cmd:
                    continue
                
                parts = cmd.split()
                command = parts[0]
                
                if command == 'q':
                    print("👋 Выход...")
                    break
                
                elif command == 's':
                    controller.set_rpm(0)
                    print("⏹️  Остановлено")
                
                elif command == 'r' and len(parts) >= 2:
                    try:
                        rpm = float(parts[1])
                        controller.set_rpm(rpm)
                        
                        # Короткий мониторинг
                        time.sleep(0.5)
                        status = controller.read_status(timeout=0.2)
                        if status:
                            print(f"📊 Status: RPM={status.rpm:.1f}, Current={status.current:.2f}A, Duty={status.duty:.3f}")
                    except ValueError:
                        print("❌ Неверный формат RPM")
                
                elif command == 'd' and len(parts) >= 2:
                    try:
                        duty = float(parts[1])
                        if -1.0 <= duty <= 1.0:
                            controller.set_duty(duty)
                            
                            # Короткий мониторинг
                            time.sleep(0.5)
                            status = controller.read_status(timeout=0.2)
                            if status:
                                print(f"📊 Status: RPM={status.rpm:.1f}, Current={status.current:.2f}A, Duty={status.duty:.3f}")
                        else:
                            print("❌ Duty должен быть от -1.0 до 1.0")
                    except ValueError:
                        print("❌ Неверный формат Duty")
                
                elif command == 'm':
                    duration = 5.0
                    if len(parts) >= 2:
                        try:
                            duration = float(parts[1])
                        except ValueError:
                            print("❌ Неверный формат длительности, используем 5 сек")
                    
                    controller.monitor_status(duration)
                
                else:
                    print("❌ Неизвестная команда. Используйте: r, d, s, m, q")
            
            except KeyboardInterrupt:
                print("\n\n⚠️  Ctrl+C нажат, выход...")
                break
    
    finally:
        controller.set_rpm(0)
        print("⏹️  Мотор остановлен")


def test_sequence(controller: VescVelocityController):
    """Тестовая последовательность для проверки velocity control"""
    print("\n" + "="*60)
    print("🧪 ТЕСТОВАЯ ПОСЛЕДОВАТЕЛЬНОСТЬ")
    print("="*60)
    
    test_rpms = [30, 60, 100, 150, 100, 50, 0, -50, -100, -50, 0]
    monitor_time = 3.0
    
    print(f"\nБудет протестировано {len(test_rpms)} значений RPM")
    print(f"Каждое значение держится {monitor_time} сек\n")
    
    input("Нажмите Enter для старта...")
    
    for i, target_rpm in enumerate(test_rpms, 1):
        print(f"\n{'='*60}")
        print(f"Тест {i}/{len(test_rpms)}: Target RPM = {target_rpm}")
        print(f"{'='*60}")
        
        controller.set_rpm(target_rpm)
        time.sleep(0.5)  # Даём время на разгон
        
        controller.monitor_status(monitor_time, target_rpm=target_rpm)
        
        time.sleep(0.5)
    
    # Финальная остановка
    print("\n✅ Тест завершён, остановка...")
    controller.set_rpm(0)
    time.sleep(1.0)


def single_rpm_test(controller: VescVelocityController, rpm: float):
    """Тест одного значения RPM"""
    print("\n" + "="*60)
    print(f"🎯 ТЕСТ ОДНОГО ЗНАЧЕНИЯ: {rpm} RPM")
    print("="*60)
    
    controller.set_rpm(rpm)
    time.sleep(1.0)  # Время на разгон
    
    controller.monitor_status(10.0, target_rpm=rpm)
    
    print("\n⏹️  Остановка...")
    controller.set_rpm(0)
    time.sleep(1.0)


def main():
    parser = argparse.ArgumentParser(
        description='Тестирование VESC Velocity Control Mode',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Примеры использования:
  # Интерактивный режим
  %(prog)s --vesc-id 49 --interactive
  
  # Автоматический тест
  %(prog)s --vesc-id 49 --test-sequence
  
  # Одно значение RPM
  %(prog)s --vesc-id 49 --rpm 100
        """
    )
    
    parser.add_argument('--can-interface', default='can0',
                        help='CAN интерфейс (по умолчанию: can0)')
    parser.add_argument('--vesc-id', type=int, required=True,
                        help='ID VESC контроллера')
    parser.add_argument('--pole-pairs', type=int, default=15,
                        help='Количество пар полюсов мотора (по умолчанию: 15)')
    
    # Режимы работы
    mode_group = parser.add_mutually_exclusive_group(required=True)
    mode_group.add_argument('--interactive', action='store_true',
                            help='Интерактивный режим с клавиатуры')
    mode_group.add_argument('--test-sequence', action='store_true',
                            help='Автоматическая тестовая последовательность')
    mode_group.add_argument('--rpm', type=float,
                            help='Задать конкретное значение RPM')
    
    args = parser.parse_args()
    
    # Создаём контроллер
    controller = VescVelocityController(
        can_interface=args.can_interface,
        vesc_id=args.vesc_id,
        pole_pairs=args.pole_pairs
    )
    
    # Подключаемся
    if not controller.connect():
        sys.exit(1)
    
    try:
        # Выбираем режим
        if args.interactive:
            interactive_mode(controller)
        elif args.test_sequence:
            test_sequence(controller)
        elif args.rpm is not None:
            single_rpm_test(controller, args.rpm)
    
    except KeyboardInterrupt:
        print("\n\n⚠️  Программа прервана (Ctrl+C)")
    
    finally:
        controller.disconnect()


if __name__ == '__main__':
    main()
