# Инструменты калибровки и ручной проверки VESC

Скрипты работают **напрямую по CAN через python-can**, без ROS и без controller_manager.
Остановите контейнер `ros2-control` (или любой другой процесс, который шлёт команды в `can0`),
иначе команды будут конфликтовать.

```bash
pip install python-can pyyaml          # matplotlib — опционально, для графика линейности
sudo ip link set can0 up type can bitrate 500000
```

| Скрипт | Назначение | Результат |
|---|---|---|
| `calibrate_max_rpm.py` | Максимальные обороты вала при duty = 100 % | `max_rps` для каждого joint (режим `duty`) |
| `check_linearity.py` | Насколько линейна зависимость duty → обороты | R², CSV/JSON, график |
| `rpm_control.py` | Ручная проверка режима RPM (PID внутри VESC) | Точность и стабильность удержания оборотов |
| `position_control.py` | Ручная проверка позиционирования | Время отклика, удержание позиции |
| `vesc_can.py` | Общий модуль протокола и шины | Используется всеми скриптами, покрыт тестами |

Во всех скриптах обороты **механические** (вал мотора): `rpm = ERPM / pole_pairs`,
где `pole_pairs = число полюсов / 2`. Для моторов Rob Box (30 полюсов) `--pole-pairs 15`.

## Безопасность

- Робот на подставке, колёса не касаются пола.
- E-STOP под рукой; `Ctrl+C` останавливает мотор (скрипты шлют 0 при выходе).
- Начинайте с малых значений: duty ≤ 0.1, обороты 10–30 об/мин.

## calibrate_max_rpm.py

```bash
python3 calibrate_max_rpm.py --vesc-ids 49 94 81 124 --pole-pairs 15 [--can-interface can0] [--output calibration_results.yaml]
```

Для каждого VESC поднимает duty шагами по 5 %, ждёт 1 с стабилизации, усредняет статусы за 0.5 с и
останавливается, когда прирост оборотов меньше 5 об/мин (насыщение) или duty достиг 100 %.
Повторяет в обратном направлении. Результат — среднее по направлениям:

```
📋 В ros2_control xacro, для каждого joint:
   can_id=49: <param name="max_rps">12.20</param>
```

Если VESC не прислал ни одного статуса за окно измерения, скрипт предупреждает:
это не «0 об/мин», а неверный ID или проблема с шиной. После 10 ошибок CAN подряд работа прерывается.

## check_linearity.py

```bash
python3 check_linearity.py --vesc-id 49 --pole-pairs 15 [--duty-step 0.01] [--duty-max 1.0] [--both-directions]
```

Режим `duty` в драйвере предполагает линейность `duty = motor_rps / max_rps`.
`R² > 0.95` — допущение выполняется. Иначе либо используйте `control_mode=rpm`,
либо калибруйте `max_rps` в рабочем диапазоне скоростей, а не на насыщении.

Выход: `linearity_vesc49_forward.{csv,json}`, `linearity_vesc49.png`.

## rpm_control.py

```bash
python3 rpm_control.py --vesc-id 49 --pole-pairs 15 --interactive      # r <об/мин>, d <duty>, s, m [с], q
python3 rpm_control.py --vesc-id 49 --pole-pairs 15 --test-sequence    # 30→60→100→150→…→−100→0, по 3 с
python3 rpm_control.py --vesc-id 49 --pole-pairs 15 --rpm 100          # удерживать 10 с и показать статистику
```

Хорошие признаки: ошибка < 5 %, разброс < 10 % от цели, duty меняется плавно.
Ошибка > 10 % или рывки на низких оборотах — нужна настройка PID скорости в VESC Tool
(и для режима `rpm` в ros2_control тоже).

## position_control.py

```bash
python3 position_control.py --vesc-id 49 --pole-pairs 15 --interactive   # p <град>, r <град>, rev <n>, z, m [с], q
python3 position_control.py --vesc-id 49 --pole-pairs 15 --degrees 360
python3 position_control.py --vesc-id 49 --pole-pairs 15 --revolutions 5
```

Требует включённого позиционного режима в VESC (PID position). Драйвер ros2_control
позиционирование не использует; скрипт нужен для проверки механики и энкодера.

## Тесты

Чистые функции протокола (кодирование команд, разбор статуса) и учёт ошибок шины
покрыты тестами без железа и без python-can:

```bash
cd <repo>
python3 -m pytest -v
```
