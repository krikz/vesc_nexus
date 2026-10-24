#!/usr/bin/env python3
"""Проверка линейности зависимости duty cycle → обороты.

Ступенчато поднимает duty с заданным шагом, на каждой ступени усредняет
механические об/мин, строит линейную регрессию и считает R². Сохраняет
CSV, JSON и график (если установлен matplotlib).

    python3 check_linearity.py --vesc-id 49 --pole-pairs 15 --both-directions

Режим duty в vesc_nexus предполагает линейность (duty = motor_rps / max_rps):
R² > 0.95 — допущение выполняется, иначе рассмотрите control_mode=rpm.

Зависимости: pip install python-can  (опционально matplotlib для графика)
"""
from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from dataclasses import asdict, dataclass
from datetime import datetime

from vesc_can import CanBusError, VescBus, configure_console

try:
    import matplotlib
    matplotlib.use("Agg")  # без дисплея
    import matplotlib.pyplot as plt
    HAS_MATPLOTLIB = True
except ImportError:
    HAS_MATPLOTLIB = False

SETTLE_TIME = 0.8       # с, стабилизация после смены duty
SAMPLES_PER_POINT = 5   # статусов на точку
SAMPLE_TIMEOUT = 2.0    # с, максимум ожидания выборки
LINEAR_R2 = 0.95


@dataclass
class Point:
    duty: float
    rpm: float
    current: float
    timestamp: str


@dataclass
class Result:
    vesc_id: int
    direction: str  # forward | backward
    points: list[Point]
    r_squared: float
    slope: float      # об/мин на единицу duty
    intercept: float  # об/мин

    @property
    def is_linear(self) -> bool:
        return self.r_squared > LINEAR_R2


def measure_point(bus: VescBus, vesc_id: int, duty: float) -> Point:
    bus.send_duty(vesc_id, duty)
    time.sleep(SETTLE_TIME)

    rpms: list[float] = []
    currents: list[float] = []
    deadline = time.monotonic() + SAMPLE_TIMEOUT
    while len(rpms) < SAMPLES_PER_POINT and time.monotonic() < deadline:
        status = bus.read_status(timeout=0.1, vesc_id=vesc_id)
        if status is not None:
            rpms.append(status.rpm)
            currents.append(status.current)

    if not rpms:
        print(f"   ⚠️  VESC {vesc_id} не прислал ни одного статуса при duty={duty:+.2f} — проверьте CAN и ID")
    mean = lambda xs: sum(xs) / len(xs) if xs else 0.0  # noqa: E731
    return Point(duty, mean(rpms), mean(currents), datetime.now().isoformat())


def linear_regression(points: list[Point]) -> tuple[float, float, float]:
    """(R², slope, intercept) по модулям duty и об/мин."""
    if len(points) < 2:
        return 0.0, 0.0, 0.0
    xs = [abs(p.duty) for p in points]
    ys = [abs(p.rpm) for p in points]
    n = len(xs)
    sx, sy = sum(xs), sum(ys)
    sxy = sum(x * y for x, y in zip(xs, ys))
    sxx = sum(x * x for x in xs)
    denominator = n * sxx - sx * sx
    if abs(denominator) < 1e-12:
        return 0.0, 0.0, sy / n
    slope = (n * sxy - sx * sy) / denominator
    intercept = (sy - slope * sx) / n
    y_mean = sy / n
    ss_tot = sum((y - y_mean) ** 2 for y in ys)
    ss_res = sum((y - (slope * x + intercept)) ** 2 for x, y in zip(xs, ys))
    r2 = 1.0 - ss_res / ss_tot if ss_tot > 0 else 0.0
    return r2, slope, intercept


def run_test(bus: VescBus, vesc_id: int, step: float, duty_max: float, direction: int) -> Result:
    name = "forward" if direction > 0 else "backward"
    print(f"\n📊 Тест {'ВПЕРЁД' if direction > 0 else 'НАЗАД'}, VESC {vesc_id}: шаг {step * 100:.1f} %, до {duty_max * 100:.0f} %")
    print("-" * 50)

    points: list[Point] = []
    duty = 0.0
    while duty <= duty_max + 1e-9:
        point = measure_point(bus, vesc_id, duty * direction)
        points.append(point)
        print(f"   duty={point.duty:+.2f} → {point.rpm:8.1f} об/мин, I={point.current:5.1f} А")
        duty += step

    bus.send_duty(vesc_id, 0.0)
    time.sleep(0.5)

    r2, slope, intercept = linear_regression(points)
    return Result(vesc_id, name, points, r2, slope, intercept)


def save_csv(result: Result, path: str) -> None:
    with open(path, "w", newline="", encoding="utf-8") as f:
        writer = csv.writer(f)
        writer.writerow(["duty", "rpm", "current", "timestamp"])
        writer.writerows([p.duty, p.rpm, p.current, p.timestamp] for p in result.points)
    print(f"📄 CSV: {path}")


def save_json(result: Result, path: str) -> None:
    data = {**asdict(result), "linear": result.is_linear}
    with open(path, "w", encoding="utf-8") as f:
        json.dump(data, f, indent=2, ensure_ascii=False)
    print(f"📄 JSON: {path}")


def save_plot(results: list[Result], path: str) -> None:
    if not HAS_MATPLOTLIB:
        print("ℹ️  matplotlib не установлен, график пропущен (pip install matplotlib)")
        return
    fig, axes = plt.subplots(1, len(results), figsize=(7 * len(results), 6), squeeze=False)
    for ax, result in zip(axes[0], results):
        duties = [abs(p.duty) * 100 for p in result.points]
        rpms = [abs(p.rpm) for p in result.points]
        ax.scatter(duties, rpms, label="измерения", alpha=0.7)
        if result.slope:
            x_max = max(duties)
            ax.plot([0, x_max], [result.intercept, result.slope * x_max / 100 + result.intercept],
                    "r--", label=f"регрессия, R²={result.r_squared:.4f}")
        ax.set_xlabel("duty, %")
        ax.set_ylabel("об/мин")
        ax.set_title(f"VESC {result.vesc_id} {result.direction}: {'линейно' if result.is_linear else 'нелинейно'}")
        ax.grid(True, alpha=0.3)
        ax.legend()
    fig.tight_layout()
    fig.savefig(path, dpi=150)
    print(f"📊 График: {path}")


def main() -> int:
    configure_console()
    parser = argparse.ArgumentParser(description="Проверка линейности duty → обороты для VESC")
    parser.add_argument("--vesc-id", type=int, required=True, help="VESC ID")
    parser.add_argument("--can-interface", default="can0", help="SocketCAN интерфейс (по умолчанию can0)")
    parser.add_argument("--pole-pairs", type=int, default=15, help="пар полюсов мотора = полюсов / 2 (по умолчанию 15)")
    parser.add_argument("--duty-step", type=float, default=0.01, help="шаг duty (по умолчанию 0.01)")
    parser.add_argument("--duty-max", type=float, default=1.0, help="максимальный duty (по умолчанию 1.0)")
    parser.add_argument("--both-directions", action="store_true", help="проверить также обратное направление")
    parser.add_argument("--output-prefix", default="linearity", help="префикс выходных файлов")
    args = parser.parse_args()

    print("🔬 Проверка линейности duty → обороты")
    print(f"   VESC: {args.vesc_id}   CAN: {args.can_interface}   шаг: {args.duty_step * 100:.1f} %   до: {args.duty_max * 100:.0f} %")
    print("\n⚠️  Колесо должно свободно вращаться.")
    input("Enter — начать, Ctrl+C — отмена... ")

    try:
        with VescBus(args.can_interface, pole_pairs=args.pole_pairs) as bus:
            results = [run_test(bus, args.vesc_id, args.duty_step, args.duty_max, +1)]
            if args.both_directions:
                print("\n⏸️  Пауза 2 с перед обратным направлением...")
                time.sleep(2.0)
                results.append(run_test(bus, args.vesc_id, args.duty_step, args.duty_max, -1))
    except CanBusError as e:
        print(f"\n❌ {e}")
        return 1
    except KeyboardInterrupt:
        print("\n⚠️  Прервано пользователем")
        return 130

    for result in results:
        prefix = f"{args.output_prefix}_vesc{result.vesc_id}_{result.direction}"
        save_csv(result, f"{prefix}.csv")
        save_json(result, f"{prefix}.json")
    save_plot(results, f"{args.output_prefix}_vesc{args.vesc_id}.png")

    print("\n" + "=" * 50 + "\n📋 ИТОГ\n" + "=" * 50)
    for result in results:
        verdict = "✅ линейно" if result.is_linear else "⚠️ нелинейно"
        print(f"{result.direction}: R²={result.r_squared:.4f} → {verdict}; "
              f"наклон {result.slope:.1f} об/мин на duty=1, смещение {result.intercept:.1f} об/мин")
    return 0


if __name__ == "__main__":
    sys.exit(main())
