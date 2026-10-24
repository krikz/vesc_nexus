#!/usr/bin/env python3
"""Ручная проверка VESC в режиме позиционирования (CAN_PACKET_SET_POS).

Задаёт угол вала в градусах и следит за движением по статусам. В endless-режиме
VESC угол накапливается, поэтому «rev 5» означает пять полных оборотов.

    python3 position_control.py --vesc-id 49 --pole-pairs 15 --interactive
    python3 position_control.py --vesc-id 49 --pole-pairs 15 --degrees 360
    python3 position_control.py --vesc-id 49 --pole-pairs 15 --revolutions 5

Робот на подставке, E-STOP под рукой. При выходе вал возвращается в 0°.

Зависимости: pip install python-can
"""
from __future__ import annotations

import argparse
import sys
import time

from vesc_can import CanBusError, VescBus, configure_console

SETTLED_RPM = 5.0  # ниже этого считаем, что позиция достигнута


class PositionSession:
    """Текущая заданная позиция, чтобы поддерживать относительные повороты."""

    def __init__(self, bus: VescBus, vesc_id: int):
        self.bus = bus
        self.vesc_id = vesc_id
        self.target_deg = 0.0

    def goto(self, degrees: float) -> None:
        self.bus.send_position(self.vesc_id, degrees)
        self.target_deg = degrees
        print(f"📤 позиция {degrees:.1f}° ({degrees / 360:.2f} об)")

    def rotate(self, delta_deg: float) -> None:
        self.goto(self.target_deg + delta_deg)

    def print_status(self) -> None:
        time.sleep(0.5)
        status = self.bus.read_status(timeout=0.3, vesc_id=self.vesc_id)
        if status is None:
            print("📊 статус не получен")
        else:
            print(f"📊 {status.rpm:7.1f} об/мин   I={status.current:5.2f} А   duty={status.duty:+.3f}")

    def monitor(self, max_duration: float) -> None:
        """Печатать статусы до остановки вала или истечения max_duration."""
        print(f"\n📊 Движение к {self.target_deg:.1f}° (до {max_duration:.0f} с)")
        print(f"{'t, с':>6} | {'об/мин':>8} | {'I, А':>6} | {'duty':>7}")
        print("-" * 36)
        start = time.monotonic()
        while (elapsed := time.monotonic() - start) < max_duration:
            status = self.bus.read_status(timeout=0.1, vesc_id=self.vesc_id)
            if status is None:
                continue
            print(f"{elapsed:6.2f} | {status.rpm:8.1f} | {status.current:6.2f} | {status.duty:+7.3f}")
            if elapsed > 0.5 and abs(status.rpm) < SETTLED_RPM:
                print(f"✅ позиция достигнута за {elapsed:.2f} с")
                return
        print("⚠️  вал не остановился за отведённое время")


def interactive(session: PositionSession) -> None:
    print("\n🎮 ИНТЕРАКТИВНЫЙ РЕЖИМ (позиция)")
    print("  p <град>     абсолютная позиция")
    print("  r <град>     относительный поворот (+ вперёд, − назад)")
    print("  rev <n>      позиция в оборотах")
    print("  z            в 0°")
    print("  m [с]        мониторинг движения (по умолчанию 5 с)")
    print("  q            выход")

    while True:
        try:
            parts = input(f"\n[{session.target_deg:.1f}°] > ").strip().lower().split()
        except EOFError:
            return
        if not parts:
            continue
        cmd, arg = parts[0], parts[1] if len(parts) > 1 else None
        try:
            if cmd == "q":
                return
            elif cmd == "z":
                session.goto(0.0)
            elif cmd == "p" and arg is not None:
                session.goto(float(arg))
                session.print_status()
            elif cmd == "r" and arg is not None:
                session.rotate(float(arg))
                session.print_status()
            elif cmd == "rev" and arg is not None:
                session.goto(float(arg) * 360.0)
                session.print_status()
            elif cmd == "m":
                session.monitor(float(arg) if arg else 5.0)
            else:
                print("❌ команды: p, r, rev, z, m, q")
        except ValueError:
            print("❌ неверное число")


def goto_and_back(session: PositionSession, degrees: float, timeout: float) -> None:
    print(f"\n🎯 {degrees:.0f}° ({degrees / 360:.2f} об)")
    print("\n1️⃣  сброс в 0°")
    session.goto(0.0)
    time.sleep(2.0)
    print(f"\n2️⃣  движение к {degrees:.0f}°")
    session.goto(degrees)
    session.monitor(timeout)
    print("\n3️⃣  возврат в 0°")
    session.goto(0.0)
    session.monitor(timeout)


def main() -> int:
    configure_console()
    parser = argparse.ArgumentParser(description="Ручная проверка VESC в режиме позиционирования через CAN")
    parser.add_argument("--vesc-id", type=int, required=True, help="VESC ID")
    parser.add_argument("--can-interface", default="can0", help="SocketCAN интерфейс (по умолчанию can0)")
    parser.add_argument("--pole-pairs", type=int, default=15, help="пар полюсов мотора = полюсов / 2 (по умолчанию 15)")
    mode = parser.add_mutually_exclusive_group(required=True)
    mode.add_argument("--interactive", action="store_true", help="ручное управление с клавиатуры")
    mode.add_argument("--degrees", type=float, help="доехать до угла и вернуться")
    mode.add_argument("--revolutions", type=float, help="сделать N оборотов и вернуться")
    args = parser.parse_args()

    try:
        with VescBus(args.can_interface, pole_pairs=args.pole_pairs) as bus:
            print(f"✅ {args.can_interface}: VESC {args.vesc_id}, pole_pairs {args.pole_pairs}")
            session = PositionSession(bus, args.vesc_id)
            try:
                if args.interactive:
                    interactive(session)
                elif args.degrees is not None:
                    goto_and_back(session, args.degrees, timeout=10.0)
                else:
                    goto_and_back(session, args.revolutions * 360.0, timeout=15.0)
            except KeyboardInterrupt:
                print("\n⚠️  Ctrl+C")
            finally:
                print("⏺️  возврат в 0°")
                session.goto(0.0)
                time.sleep(2.0)
    except CanBusError as e:
        print(f"❌ {e}")
        return 1
    return 0


if __name__ == "__main__":
    sys.exit(main())
