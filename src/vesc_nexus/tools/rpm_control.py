#!/usr/bin/env python3
"""Ручная проверка VESC в режиме RPM (замкнутый контур, PID внутри VESC).

Отправляет CAN_PACKET_SET_RPM напрямую через python-can, минуя ROS, и показывает,
насколько точно VESC держит заданные обороты. Полезно перед включением
control_mode=rpm в ros2_control и для поиска минимальных стабильных оборотов.

    python3 rpm_control.py --vesc-id 49 --pole-pairs 15 --interactive
    python3 rpm_control.py --vesc-id 49 --pole-pairs 15 --test-sequence
    python3 rpm_control.py --vesc-id 49 --pole-pairs 15 --rpm 100

Обороты везде механические (вал мотора); ERPM = rpm × pole_pairs.
Робот на подставке, E-STOP под рукой. Ctrl+C останавливает мотор.

Зависимости: pip install python-can
"""
from __future__ import annotations

import argparse
import sys
import time

from vesc_can import CanBusError, VescBus, configure_console

TEST_SEQUENCE_RPM = [30, 60, 100, 150, 100, 50, 0, -50, -100, -50, 0]
TEST_SEQUENCE_HOLD = 3.0  # с на каждую ступень


def print_status(bus: VescBus, vesc_id: int) -> None:
    time.sleep(0.5)
    status = bus.read_status(timeout=0.3, vesc_id=vesc_id)
    if status is None:
        print("📊 статус не получен")
    else:
        print(f"📊 {status.rpm:7.1f} об/мин   I={status.current:5.2f} А   duty={status.duty:+.3f}")


def monitor(bus: VescBus, vesc_id: int, duration: float, target_rpm: float | None = None) -> None:
    """Печатать статусы duration секунд; при target_rpm — ошибку и итоговую статистику."""
    print(f"\n📊 Мониторинг {duration:.1f} с")
    print(f"{'t, с':>6} | {'об/мин':>8} | {'I, А':>6} | {'duty':>7} | {'ошибка':>8}")
    print("-" * 50)
    samples: list[float] = []
    start = time.monotonic()
    while time.monotonic() - start < duration:
        status = bus.read_status(timeout=0.1, vesc_id=vesc_id)
        if status is None:
            continue
        error = f"{status.rpm - target_rpm:+8.1f}" if target_rpm is not None else "     ---"
        print(f"{time.monotonic() - start:6.2f} | {status.rpm:8.1f} | {status.current:6.2f} | {status.duty:+7.3f} | {error}")
        samples.append(status.rpm)

    if target_rpm and samples:
        avg = sum(samples) / len(samples)
        print(f"\n📈 цель {target_rpm:.1f}   факт {avg:.1f} (среднее)   "
              f"ошибка {avg - target_rpm:+.1f} ({abs(avg - target_rpm) / abs(target_rpm) * 100:.1f} %)   "
              f"разброс {min(samples):.1f} … {max(samples):.1f}")
    elif not samples:
        print("⚠️  статусов не получено — проверьте CAN и VESC ID")


def interactive(bus: VescBus, vesc_id: int) -> None:
    print("\n🎮 ИНТЕРАКТИВНЫЙ РЕЖИМ (RPM)")
    print("  r <об/мин>   задать обороты (r 50, r -30)")
    print("  d <duty>     задать duty для сравнения (d 0.1)")
    print("  s            стоп")
    print("  m [с]        мониторинг (по умолчанию 5 с)")
    print("  q            выход")

    while True:
        try:
            parts = input("\n> ").strip().lower().split()
        except EOFError:
            return
        if not parts:
            continue
        cmd, arg = parts[0], parts[1] if len(parts) > 1 else None
        try:
            if cmd == "q":
                return
            elif cmd == "s":
                bus.send_rpm(vesc_id, 0)
                print("⏹️  стоп")
            elif cmd == "r" and arg is not None:
                bus.send_rpm(vesc_id, float(arg))
                print_status(bus, vesc_id)
            elif cmd == "d" and arg is not None:
                duty = float(arg)
                if not -1.0 <= duty <= 1.0:
                    print("❌ duty должен быть в [-1, 1]")
                    continue
                bus.send_duty(vesc_id, duty)
                print_status(bus, vesc_id)
            elif cmd == "m":
                monitor(bus, vesc_id, float(arg) if arg else 5.0)
            else:
                print("❌ команды: r, d, s, m, q")
        except ValueError:
            print("❌ неверное число")


def test_sequence(bus: VescBus, vesc_id: int) -> None:
    print(f"\n🧪 ПОСЛЕДОВАТЕЛЬНОСТЬ: {TEST_SEQUENCE_RPM} об/мин, по {TEST_SEQUENCE_HOLD} с")
    input("Enter — старт, Ctrl+C — отмена... ")
    for i, target in enumerate(TEST_SEQUENCE_RPM, 1):
        print(f"\n[{i}/{len(TEST_SEQUENCE_RPM)}] цель {target} об/мин")
        bus.send_rpm(vesc_id, target)
        time.sleep(0.5)
        monitor(bus, vesc_id, TEST_SEQUENCE_HOLD, target_rpm=target or None)


def single_rpm(bus: VescBus, vesc_id: int, rpm: float) -> None:
    print(f"\n🎯 {rpm} об/мин, 10 с")
    bus.send_rpm(vesc_id, rpm)
    time.sleep(1.0)
    monitor(bus, vesc_id, 10.0, target_rpm=rpm or None)


def main() -> int:
    configure_console()
    parser = argparse.ArgumentParser(description="Ручная проверка VESC в режиме RPM через CAN")
    parser.add_argument("--vesc-id", type=int, required=True, help="VESC ID")
    parser.add_argument("--can-interface", default="can0", help="SocketCAN интерфейс (по умолчанию can0)")
    parser.add_argument("--pole-pairs", type=int, default=15, help="пар полюсов мотора = полюсов / 2 (по умолчанию 15)")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--interactive", action="store_true", help="ручное управление с клавиатуры")
    mode.add_argument("--test-sequence", action="store_true", help="автоматическая последовательность оборотов")
    mode.add_argument("--rpm", type=float, help="удерживать заданные обороты 10 с")
    args = parser.parse_args()

    try:
        with VescBus(args.can_interface, pole_pairs=args.pole_pairs) as bus:
            print(f"✅ {args.can_interface}: VESC {args.vesc_id}, pole_pairs {args.pole_pairs}")
            try:
                if args.interactive:
                    interactive(bus, args.vesc_id)
                elif args.test_sequence:
                    test_sequence(bus, args.vesc_id)
                else:
                    single_rpm(bus, args.vesc_id, args.rpm)
            except KeyboardInterrupt:
                print("\n⚠️  Ctrl+C")
            finally:
                bus.send_rpm(args.vesc_id, 0)
                time.sleep(0.2)
                print("⏹️  мотор остановлен")
    except CanBusError as e:
        print(f"❌ {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
