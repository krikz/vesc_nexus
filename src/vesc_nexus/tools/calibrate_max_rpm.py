#!/usr/bin/env python3
"""Автокалибровка максимальных оборотов VESC при duty = 100 %.

Для каждого VESC ступенчато поднимает duty cycle, ждёт стабилизации и следит за
ростом механических об/мин. Останавливается, когда обороты перестают расти
(насыщение) или достигнут duty = 100 %. Повторяет в обратном направлении.

Результат — max_rps (обороты вала мотора в секунду) для параметра max_rps
в ros2_control xacro (режим control_mode=duty).

    python3 calibrate_max_rpm.py --vesc-ids 49 94 81 124 --pole-pairs 15

ВАЖНО: --pole-pairs = число_полюсов / 2 (мотор на 30 полюсов → 15).
Робот должен стоять на подставке, колёса — свободно вращаться.

Зависимости: pip install python-can pyyaml
"""
from __future__ import annotations

import argparse
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime

import yaml

from vesc_can import CanBusError, VescBus, configure_console

DUTY_STEP = 0.05          # шаг увеличения duty
DUTY_MAX = 1.0             # предел duty
SETTLE_TIME = 1.0          # с, ожидание стабилизации оборотов после смены duty
MEASURE_TIME = 0.5         # с, окно усреднения статусов
RPM_CHANGE_THRESHOLD = 5   # об/мин, меньший прирост считаем насыщением
MAX_ITERATIONS = 25


@dataclass
class WheelCalibration:
    vesc_id: int
    max_rpm_forward: float
    max_rpm_backward: float
    duty_at_max_forward: float
    duty_at_max_backward: float
    timestamp: str

    @property
    def max_rps(self) -> float:
        """Среднее по направлениям, об/с вала мотора."""
        return (self.max_rpm_forward + self.max_rpm_backward) / 2.0 / 60.0


def stable_rpm(bus: VescBus, vesc_id: int, duty: float) -> float:
    """Подать duty, дождаться стабилизации и усреднить об/мин за окно измерения."""
    bus.send_duty(vesc_id, duty)
    time.sleep(SETTLE_TIME)
    statuses = bus.collect_statuses(vesc_id, MEASURE_TIME)
    if not statuses:
        # Ноль статусов — это не «0 об/мин», а признак проблем с шиной или неверного ID.
        print(f"    ⚠️  VESC {vesc_id} не прислал ни одного статуса при duty={duty:+.2f} — проверьте CAN и ID")
        return 0.0
    return sum(s.rpm for s in statuses) / len(statuses)


def find_max_rpm(bus: VescBus, vesc_id: int, direction: int) -> tuple[float, float]:
    """Разогнать колесо до насыщения. Возвращает (max_rpm, duty_at_max), оба по модулю."""
    print(f"\n  📊 Калибровка {'ВПЕРЁД' if direction > 0 else 'НАЗАД'}...")
    prev_rpm = max_rpm = duty_at_max = 0.0

    for i in range(MAX_ITERATIONS):
        duty = min(DUTY_STEP * (i + 1), DUTY_MAX) * direction
        rpm = abs(stable_rpm(bus, vesc_id, duty))
        gain = rpm - prev_rpm
        print(f"    duty={duty:+.2f} → {rpm:7.1f} об/мин (Δ={gain:+.1f})")

        if rpm > max_rpm:
            max_rpm, duty_at_max = rpm, abs(duty)
        if i > 2 and gain < RPM_CHANGE_THRESHOLD:
            print(f"    ✓ Насыщение при duty={abs(duty):.2f}")
            break
        if abs(duty) >= DUTY_MAX:
            print(f"    ✓ Достигнут duty={DUTY_MAX}")
            break
        prev_rpm = rpm

    bus.send_duty(vesc_id, 0.0)
    time.sleep(0.5)
    return max_rpm, duty_at_max


def calibrate_wheel(bus: VescBus, vesc_id: int) -> WheelCalibration:
    print(f"\n{'=' * 50}\n🔧 VESC ID {vesc_id}\n{'=' * 50}")
    fwd_rpm, fwd_duty = find_max_rpm(bus, vesc_id, +1)
    print("\n  ⏸️  Пауза 2 с...")
    time.sleep(2.0)
    bwd_rpm, bwd_duty = find_max_rpm(bus, vesc_id, -1)

    result = WheelCalibration(vesc_id, fwd_rpm, bwd_rpm, fwd_duty, bwd_duty, datetime.now().isoformat())
    print(f"\n  📋 Вперёд: {fwd_rpm:.0f} об/мин при duty={fwd_duty:.2f}")
    print(f"     Назад:  {bwd_rpm:.0f} об/мин при duty={bwd_duty:.2f}")
    print(f"     max_rps = {result.max_rps:.2f} об/с")
    return result


def save_results(results: list[WheelCalibration], path: str, pole_pairs: int) -> None:
    data = {
        "calibration": {
            "timestamp": datetime.now().isoformat(),
            "pole_pairs": pole_pairs,
            "note": "max_rps — обороты вала мотора в секунду при duty=100 %, среднее по направлениям",
        },
        "wheels": {f"vesc_{r.vesc_id}": {**asdict(r), "max_rps": round(r.max_rps, 2)} for r in results},
        "ros2_control_params": {r.vesc_id: round(r.max_rps, 2) for r in results},
    }
    with open(path, "w", encoding="utf-8") as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)

    print(f"\n✅ Результаты сохранены: {path}")
    print("\n📋 В ros2_control xacro, для каждого joint:")
    for r in results:
        print(f'   can_id={r.vesc_id}: <param name="max_rps">{r.max_rps:.2f}</param>')


def main() -> int:
    configure_console()
    parser = argparse.ArgumentParser(description="Автокалибровка максимальных оборотов VESC при duty=100 %")
    parser.add_argument("--vesc-ids", type=int, nargs="+", required=True, help="VESC ID для калибровки, например 49 94 81 124")
    parser.add_argument("--can-interface", default="can0", help="SocketCAN интерфейс (по умолчанию can0)")
    parser.add_argument("--pole-pairs", type=int, default=15, help="пар полюсов мотора = полюсов / 2 (по умолчанию 15)")
    parser.add_argument("--output", default="calibration_results.yaml", help="файл результатов")
    args = parser.parse_args()

    print("🚀 Калибровка max_rps")
    print(f"   VESC: {args.vesc_ids}   CAN: {args.can_interface}   pole_pairs: {args.pole_pairs}")
    print("\n⚠️  Робот на подставке, колёса свободно вращаются?")
    input("Enter — начать, Ctrl+C — отмена... ")

    try:
        with VescBus(args.can_interface, pole_pairs=args.pole_pairs) as bus:
            results = [calibrate_wheel(bus, vesc_id) for vesc_id in args.vesc_ids]
            save_results(results, args.output, args.pole_pairs)
    except CanBusError as e:
        print(f"\n❌ {e}")
        return 1
    except KeyboardInterrupt:
        print("\n⚠️  Прервано пользователем")
        return 130

    print("\n✅ Калибровка завершена")
    return 0


if __name__ == "__main__":
    sys.exit(main())
