#!/usr/bin/env python3
"""
Скрипт тестирования VESC Position Control Mode через CAN шину

ИСПОЛЬЗОВАНИЕ:
    # Интерактивный режим - управление с клавиатуры
    python3 test_position_control.py --vesc-id 49 --pole-pairs 15 --interactive

    # Задать конкретную позицию (в градусах)
    python3 test_position_control.py --vesc-id 49 --pole-pairs 15 --degrees 360

    # Задать позицию в оборотах
    python3 test_position_control.py --vesc-id 49 --pole-pairs 15 --revolutions 5

ОПИСАНИЕ:
    Отправляет команды position (угол в градусах) на VESC.
    Позволяет проверить:
    - Точность позиционирования
    - Время отклика
    - Behaviour в endless режиме (накопление оборотов)
    - Стабильность удержания позиции

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
from typing import Optional
from dataclasses import dataclass
from datetime import datetime
import math


# VESC CAN Protocol Constants
CAN_PACKET_SET_POS = 4
CAN_PACKET_STATUS = 9

# Настройки
STATUS_UPDATE_RATE = 0.1  # Частота опроса статуса (сек)


@dataclass
class VescStatus:
    """Статус VESC контроллера."""
    vesc_id: int
    erpm: float
    rpm: float
    current: float
    duty: float
    timestamp: float


class VescPositionController:
    """Контроллер для управления VESC в режиме position."""

    def __init__(self, can_interface: str, vesc_id: int, pole_pairs: int = 15):
        self.can_interface = can_interface
        self.vesc_id = vesc_id
        self.pole_pairs = pole_pairs  # количество пар полюсов (poles / 2)
        self.bus: Optional[can.Bus] = None
        self.last_status: Optional[VescStatus] = None
        self.current_position_degrees: float = 0.0  # Текущая позиция в градусах

    def connect(self) -> bool:
        """Подключение к CAN шине."""
        try:
            self.bus = can.Bus(channel=self.can_interface, interface='socketcan')
            print(f"✅ Подключено к CAN интерфейсу: {self.can_interface}")
            print(f"   VESC ID: {self.vesc_id}, Pole pairs: {self.pole_pairs}")
            return True
        except Exception as e:
            print(f"❌ Ошибка подключения к CAN: {e}")
            return False

    def disconnect(self):
        """Отключение от CAN шины."""
        if self.bus:
            self.bus.shutdown()
            print("\n✅ Отключено от CAN шины")

    def set_position_degrees(self, degrees: float):
        """
        Отправка команды position на VESC (в градусах)

        Args:
            degrees: Позиция в градусах (бесконечная, накапливается)
        """
        # VESC position = degrees * 1000000
        position_int = int(degrees * 1000000)

        # Пакуем в формат VESC: 4 байта signed int
        data = struct.pack('>i', position_int)

        # CAN ID: command_id << 8 | vesc_id
        can_id = (CAN_PACKET_SET_POS << 8) | self.vesc_id
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )

        try:
            self.bus.send(msg)
            self.current_position_degrees = degrees
            print(f"📤 Sent: Position={degrees:.1f}° ({degrees/360:.2f} rev)")
        except can.CanError as e:
            print(f"❌ Ошибка отправки CAN: {e}")

    def rotate_relative(self, degrees: float):
        """
        Повернуть на заданный угол относительно текущей позиции

        Args:
            degrees: Угол поворота в градусах (положительный - вперёд)
        """
        new_position = self.current_position_degrees + degrees
        self.set_position_degrees(new_position)

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

    def monitor_movement(self, duration: float, target_position: Optional[float] = None):
        """
        Мониторинг движения к целевой позиции

        Args:
            duration: Максимальная длительность мониторинга (сек)
            target_position: Целевая позиция в градусах (опционально)
        """
        print(f"\n📊 Мониторинг движения ({duration:.1f} сек)...")
        print(f"{'Time':>6} | {'RPM':>8} | {'Current':>8} | {'Duty':>7}")
        print("-" * 50)

        start_time = time.time()

        while time.time() - start_time < duration:
            status = self.read_status(timeout=0.1)

            if status:
                elapsed = time.time() - start_time
                print(f"{elapsed:6.2f} | {status.rpm:8.1f} | {status.current:8.2f} | {status.duty:7.3f}")

            # Проверяем достигли ли цели (RPM близок к 0)
            if status and abs(status.rpm) < 5 and elapsed > 0.5:
                print(f"\n✅ Позиция достигнута за {elapsed:.2f} сек")
                break

            time.sleep(0.1)


def interactive_mode(controller: VescPositionController):
    """Интерактивный режим управления с клавиатуры."""
    print("\n" + "="*60)
    print("🎮 ИНТЕРАКТИВНЫЙ РЕЖИМ - POSITION CONTROL")
    print("="*60)
    print("\nКоманды:")
    print("  p <degrees>  - Задать абсолютную позицию в градусах")
    print("  r <degrees>  - Повернуть относительно (+ вперёд, - назад)")
    print("  rev <n>      - Задать позицию в оборотах")
    print("  z            - Сброс позиции в 0")
    print("  m [сек]      - Мониторинг движения (по умолчанию 5 сек)")
    print("  q            - Выход")
    print("\nПримеры:")
    print("  p 360        - Переместиться на 360° (1 оборот)")
    print("  r 90         - Повернуть на 90° вперёд")
    print("  r -180       - Повернуть на 180° назад")
    print("  rev 5        - Сделать 5 оборотов")
    print("  z            - Вернуться в начальную позицию")
    print("-"*60)

    try:
        while True:
            try:
                cmd = input(f"\n[Pos: {controller.current_position_degrees:.1f}°] > ").strip().lower()

                if not cmd:
                    continue

                parts = cmd.split()
                command = parts[0]

                if command == 'q':
                    print("👋 Выход...")
                    break

                elif command == 'z':
                    controller.set_position_degrees(0)
                    print("⏺️  Позиция сброшена в 0°")

                elif command == 'p' and len(parts) >= 2:
                    try:
                        degrees = float(parts[1])
                        controller.set_position_degrees(degrees)

                        # Короткий мониторинг
                        time.sleep(0.5)
                        status = controller.read_status(timeout=0.2)
                        if status:
                            print(f"📊 Status: RPM={status.rpm:.1f}, Current={status.current:.2f}A, Duty={status.duty:.3f}")
                    except ValueError:
                        print("❌ Неверный формат градусов")

                elif command == 'r' and len(parts) >= 2:
                    try:
                        degrees = float(parts[1])
                        controller.rotate_relative(degrees)

                        # Короткий мониторинг
                        time.sleep(0.5)
                        status = controller.read_status(timeout=0.2)
                        if status:
                            print(f"📊 Status: RPM={status.rpm:.1f}, Current={status.current:.2f}A, Duty={status.duty:.3f}")
                    except ValueError:
                        print("❌ Неверный формат градусов")

                elif command == 'rev' and len(parts) >= 2:
                    try:
                        revolutions = float(parts[1])
                        degrees = revolutions * 360.0
                        controller.set_position_degrees(degrees)

                        print(f"🔄 Задано {revolutions} оборотов ({degrees:.0f}°)")

                        # Короткий мониторинг
                        time.sleep(0.5)
                        status = controller.read_status(timeout=0.2)
                        if status:
                            print(f"📊 Status: RPM={status.rpm:.1f}, Current={status.current:.2f}A, Duty={status.duty:.3f}")
                    except ValueError:
                        print("❌ Неверный формат оборотов")

                elif command == 'm':
                    duration = 5.0
                    if len(parts) >= 2:
                        try:
                            duration = float(parts[1])
                        except ValueError:
                            print("❌ Неверный формат длительности, используем 5 сек")

                    controller.monitor_movement(duration, target_position=controller.current_position_degrees)

                else:
                    print("❌ Неизвестная команда. Используйте: p, r, rev, z, m, q")

            except KeyboardInterrupt:
                print("\n\n⚠️  Ctrl+C нажат, выход...")
                break

    finally:
        # Возврат в начальную позицию перед выходом
        print("\n⏺️  Возврат в начальную позицию...")
        controller.set_position_degrees(0)
        time.sleep(2.0)


def test_position(controller: VescPositionController, degrees: float):
    """Тест позиционирования на заданный угол."""
    print("\n" + "="*60)
    print(f"🎯 ТЕСТ ПОЗИЦИИ: {degrees}° ({degrees/360:.2f} оборотов)")
    print("="*60)

    # Сбрасываем в 0
    print("\n1️⃣  Сброс в начальную позицию...")
    controller.set_position_degrees(0)
    time.sleep(2.0)

    # Едем к целевой позиции
    print(f"\n2️⃣  Движение к {degrees}°...")
    controller.set_position_degrees(degrees)
    controller.monitor_movement(10.0, target_position=degrees)

    # Возврат в 0
    print("\n3️⃣  Возврат в начальную позицию...")
    controller.set_position_degrees(0)
    controller.monitor_movement(10.0, target_position=0)

    print("\n✅ Тест завершён")


def test_revolutions(controller: VescPositionController, revolutions: float):
    """Тест вращения на заданное количество оборотов."""
    degrees = revolutions * 360.0
    print("\n" + "="*60)
    print(f"🔄 ТЕСТ ОБОРОТОВ: {revolutions} оборотов ({degrees:.0f}°)")
    print("="*60)

    # Сбрасываем в 0
    print("\n1️⃣  Сброс в начальную позицию...")
    controller.set_position_degrees(0)
    time.sleep(2.0)

    # Делаем обороты
    print(f"\n2️⃣  Вращение {revolutions} оборотов...")
    controller.set_position_degrees(degrees)
    controller.monitor_movement(15.0, target_position=degrees)

    # Возврат в 0
    print("\n3️⃣  Возврат в начальную позицию...")
    controller.set_position_degrees(0)
    controller.monitor_movement(15.0, target_position=0)

    print("\n✅ Тест завершён")


def main():
    parser = argparse.ArgumentParser(
        description='Тестирование VESC Position Control Mode',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Примеры использования:
  # Интерактивный режим
  %(prog)s --vesc-id 49 --interactive

  # Тест конкретного угла
  %(prog)s --vesc-id 49 --degrees 360

  # Тест вращения оборотов
  %(prog)s --vesc-id 49 --revolutions 5
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
    mode_group.add_argument('--degrees', type=float,
                            help='Задать конкретную позицию в градусах')
    mode_group.add_argument('--revolutions', type=float,
                            help='Задать количество оборотов')

    args = parser.parse_args()

    # Создаём контроллер
    controller = VescPositionController(
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
        elif args.degrees is not None:
            test_position(controller, args.degrees)
        elif args.revolutions is not None:
            test_revolutions(controller, args.revolutions)

    except KeyboardInterrupt:
        print("\n\n⚠️  Программа прервана (Ctrl+C)")

    finally:
        controller.disconnect()


if __name__ == '__main__':
    main()
