# vesc_nexus

[![ROS 2 Humble](https://img.shields.io/badge/ROS_2-Humble-blue)](https://docs.ros.org/en/humble/)
[![ros2_control](https://img.shields.io/badge/ros2__control-SystemInterface-informational)](https://control.ros.org/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)

**Hardware interface ros2_control для моторных контроллеров [VESC](https://vesc-project.com/) по CAN (SocketCAN).**

Плагин загружается в `controller_manager`, принимает от контроллера (например `diff_drive_controller`)
угловую скорость каждого колеса и переводит её в команды VESC, а из статусных CAN-пакетов
собирает скорость, пройденный угол, ток мотора и напряжение батареи.
Разработан для робота [Rob Box](https://github.com/krikz/rob_box_project) (4 колеса, skid-steer, Raspberry Pi 5 + MCP2515),
но не содержит ничего специфичного для него.

## Возможности

- Несколько VESC на одной шине, адресация по VESC ID, 29-битные CAN ID.
- Два режима управления, переключаются одним параметром:
  - **`duty`** — открытый контур: duty cycle по калибровке `max_rps`, компенсация мёртвой зоны `min_duty`;
  - **`rpm`** — замкнутый контур: целевой ERPM, скорость держит PID внутри VESC.
- Учёт редуктора (`gear_ratio`) и в командах, и в обратной связи.
- Состояние по каждому колесу: `position` (рад), `velocity` (рад/с), `effort` (А); общее `battery/voltage` (В).
- Таймаут команд: после `command_timeout` секунд нулевой скорости фреймы не отправляются и VESC отпускает мотор.
- Строгая проверка конфигурации при загрузке: неверный параметр — понятная ошибка, а не тихая подмена значения.
- Защита от NaN/Inf в командах и телеметрии, потокобезопасный обмен между потоком CAN и циклом управления.
- Инструменты калибровки и ручной проверки на python-can, работающие без ROS.

## Как устроено

```
controller_manager (update_rate, например 50 Гц)
 └─ VescSystemHardwareInterface         read() / write()
     ├─ VescHandler  × N joint          рад/с ↔ duty | ERPM, разбор STATUS-пакетов
     └─ CanInterface                    SocketCAN RAW, поток приёма
          └─ can0 ──── VESC id 49, 94, 81, 124 …
```

Поток приёма CAN раскладывает статусы по обработчикам (по VESC ID); цикл управления читает
снимок состояния и отправляет команды. Пересчёт:

| Направление | Формула |
|---|---|
| Команда, об/с мотора | `motor_rps = ω / 2π × gear_ratio` |
| Режим `duty` | `duty = motor_rps / max_rps`, затем при `min_duty > 0`: `\|duty\| → min_duty + \|duty\| × (1 − min_duty)` |
| Режим `rpm` | `ERPM = motor_rps × pole_pairs × 60` |
| Обратная связь | `ω = ERPM / pole_pairs × 2π / 60 / gear_ratio`; `position = ∫ ω dt` по реальным интервалам между STATUS |

`ω` — угловая скорость колеса (рад/с), `pole_pairs = poles / 2`. Радиус колеса плагину не нужен:
он сокращается в формулах, а в метры скорость переводит контроллер (`diff_drive_controller/wheel_radius`).

## Состав репозитория

```
vesc_nexus/
├── src/vesc_msgs/                 сообщение VescState (телеметрия VESC)
├── src/vesc_nexus/
│   ├── include/vesc_nexus/
│   │   ├── can_interface.hpp      SocketCAN: сокет, поток приёма, отправка
│   │   ├── message_translator.hpp протокол VESC: кодирование команд, разбор статусов
│   │   ├── vesc_handler.hpp       один VESC: пересчёт команд и телеметрии
│   │   └── vesc_system_hardware_interface.hpp   плагин ros2_control
│   ├── src/                       реализация
│   ├── test/                      gtest: протокол и VescHandler, без железа
│   ├── tools/                     калибровка и ручная проверка (python-can), см. tools/README.md
│   ├── vesc_nexus_plugins.xml     описание плагина для pluginlib
│   ├── CMakeLists.txt
│   └── package.xml
├── pytest.ini                     тесты инструментов
├── LICENSE
└── README.md
```

## Требования

- ROS 2 Humble, `ros2_control` (`hardware_interface`, `controller_manager`), `pluginlib`.
- Linux с SocketCAN: MCP2515 (SPI), USB-CAN, встроенный контроллер; скорость шины у всех участников одна (на Rob Box — 500 кбит/с).
- VESC с включённой CAN-телеметрией. В VESC Tool → *App Settings → General*:
  - **CAN Status Message Mode** = `CAN_STATUS_1_2_3_4_5` — пакет STATUS_5 нужен для `battery/voltage`;
    без него напряжение останется 0 (на Rob Box на 2026-10-24 включены только 1–4);
  - **CAN Status Rate** ≥ `update_rate` контроллера (50 Гц достаточно);
  - уникальный **VESC ID** у каждого контроллера, **Send CAN status** включён.
- Для инструментов: Python 3.9+, `python-can`, `pyyaml`.

## Сборка и тесты

```bash
cd ~/ros2_ws/src && git clone https://github.com/krikz/vesc_nexus.git
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select vesc_msgs vesc_nexus
colcon test  --packages-select vesc_nexus && colcon test-result --verbose
```

Юнит-тесты не требуют CAN-устройства. Тесты инструментов:

```bash
cd src/vesc_nexus && python3 -m pytest -v
```

В Rob Box пакет собирается в образ `ros2-control` (см. `docker/main/ros2_control/Dockerfile` родительского проекта),
где уже стоят `ros-humble-ros2-control`, `ros-humble-hardware-interface` и `can-utils`.

## Настройка

Плагин конфигурируется в блоке `<ros2_control>` URDF/xacro. Полный рабочий пример для Rob Box —
`src/rob_box_description/urdf/rob_box_ros2_control.xacro` в родительском проекте.

```xml
<ros2_control name="VescSystem" type="system">
  <hardware>
    <plugin>vesc_nexus/VescSystemHardwareInterface</plugin>
    <param name="can_interface">can0</param>
    <param name="control_mode">duty</param>      <!-- duty | rpm -->
    <param name="gear_ratio">2.17</param>         <!-- motor_rps / wheel_rps -->
    <param name="min_duty">0.05</param>           <!-- только для duty -->
    <param name="command_timeout">0.5</param>     <!-- с -->
  </hardware>

  <joint name="left_front_wheel_joint">
    <command_interface name="velocity"/>
    <state_interface name="position"/>
    <state_interface name="velocity"/>
    <state_interface name="effort"/>
    <param name="can_id">49</param>
    <param name="poles">30</param>
    <param name="max_rps">12.2</param>            <!-- обязателен в режиме duty -->
  </joint>
  <!-- … остальные колёса -->
</ros2_control>
```

### Параметры `<hardware>`

| Параметр | Обязателен | По умолчанию | Описание |
|---|---|---|---|
| `can_interface` | да | — | Имя SocketCAN-интерфейса (`can0`). |
| `control_mode` | нет | `duty` | `duty` — открытый контур; `rpm` — PID скорости в VESC. |
| `gear_ratio` | нет | `1.0` | Передаточное число редуктора, `> 0`. Применяется к командам и обратной связи. |
| `min_duty` | нет | `0.0` | Минимальный duty для срыва с места, `[0, 1)`. Только `duty`. |
| `command_timeout` | нет | `0.5` | Секунды нулевой команды, после которых фреймы не отправляются. |

### Параметры `<joint>`

| Параметр | Обязателен | Описание |
|---|---|---|
| `can_id` | да | VESC ID, `0..254`. |
| `poles` | да | Число полюсов мотора, `≥ 2`; `pole_pairs = poles / 2`. |
| `max_rps` | в режиме `duty` | Обороты вала мотора в секунду при duty = 100 %. Измеряется `tools/calibrate_max_rpm.py`. В режиме `rpm` не используется. |

Параметры прежних версий `publish_rate`, `wheel_radius` (в `<hardware>` и `<joint>`) и `min_erpm`
принимаются, но ни на что не влияют; при загрузке об этом пишется предупреждение.
Ошибка в любом параметре (отсутствует, не число, вне диапазона) — `on_init` возвращает `ERROR`
с указанием параметра, компонент не стартует.

### Интерфейсы

| Интерфейс | Единицы | Источник |
|---|---|---|
| `<joint>/velocity` (command) | рад/с колеса | контроллер |
| `<joint>/position` (state) | рад колеса | интеграл скорости по CAN STATUS |
| `<joint>/velocity` (state) | рад/с колеса | CAN STATUS, ERPM |
| `<joint>/effort` (state) | А, ток мотора | CAN STATUS |
| `battery/voltage` (state) | В | минимум ненулевых `voltage_input` по всем VESC (CAN STATUS_5) |

### Контроллер

Минимальная конфигурация `diff_drive_controller`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 50
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
    diff_drive_controller:
      type: diff_drive_controller/DiffDriveController

diff_drive_controller:
  ros__parameters:
    left_wheel_names:  [left_front_wheel_joint, left_rear_wheel_joint]
    right_wheel_names: [right_front_wheel_joint, right_rear_wheel_joint]
    wheel_separation: 1.11     # эффективная колея skid-steer, калибруется по повороту
    wheel_radius: 0.1143       # м — единственное место, где задаётся радиус
    cmd_vel_timeout: 0.5
    open_loop: true            # одометрия от команд; false — от обратной связи VESC
    position_feedback: false
```

С `open_loop: true` (так настроен Rob Box) одометрия считается из команд, а `position`/`velocity`
от VESC идут только в `/joint_states`. Для одометрии по обратной связи поставьте `open_loop: false`,
`position_feedback: true` и убедитесь, что `gear_ratio` и `poles` заданы верно.

## Поведение

- **Старт.** `on_init` читает и проверяет параметры и создаёт обработчики; `on_configure` открывает CAN;
  `on_deactivate`/`on_shutdown` отправляют нулевую скорость и закрывают CAN.
- **Таймаут команд.** Пока от контроллера идёт ненулевая скорость — команды уходят каждый цикл.
  Когда скорость стала нулевой, ещё `command_timeout` секунд отправляется явный ноль (VESC тормозит),
  затем отправка прекращается и VESC по своему таймауту отпускает мотор. Переходы логируются один раз.
- **Напряжение батареи.** Минимум среди VESC, приславших ненулевое напряжение; пока не прислал никто — держится прошлое значение.
- **Позиция.** Интегрируется по реальным интервалам между STATUS-пакетами; интервалы длиннее 0.5 с (пауза, потеря связи) не учитываются.
- **Телеметрия.** Фреймы с 11-битным ID, от чужих VESC и неизвестных пакетов игнорируются; короткие фреймы отбрасываются.

## Калибровка (режим `duty`)

1. Робот на подставке, контейнер `ros2-control` остановлен, `can0` поднят.
2. `max_rps`: `tools/calibrate_max_rpm.py --vesc-ids 49 94 81 124 --pole-pairs 15` → значения в `<param name="max_rps">`.
3. `gear_ratio`: при `min_duty = 0` проехать 1 м по команде, измерить рулеткой;
   `gear_ratio_new = gear_ratio_old / (физ. расстояние / ожидаемое)`. Повторять до ±5 %.
4. `min_duty`: минимальный duty, при котором робот трогается с пола; обычно 0.03–0.08. Завышенное значение искажает малые скорости.
5. Проверить линейность: `tools/check_linearity.py --vesc-id 49 --pole-pairs 15`; при R² < 0.95 рассмотрите `control_mode=rpm`.

Подробности и остальные инструменты — [`src/vesc_nexus/tools/README.md`](src/vesc_nexus/tools/README.md).

## Диагностика

```bash
ip -details -statistics link show can0        # state UP, ERROR-ACTIVE, нет bus-off и роста errors
candump can0                                  # 00000931 = пакет 0x09 (STATUS) от VESC 0x31 (49)
ros2 control list_hardware_components -v      # VescSystem: active, интерфейсы available/claimed
ros2 topic echo /joint_states                 # velocity ≠ 0 при вращении колёс
```

| Симптом | Причина и решение |
|---|---|
| `Invalid ros2_control configuration: joint … parameter max_rps is required` | Режим `duty` без калибровки. Добавьте `max_rps` или переключитесь на `rpm`. |
| `CAN interface 'can0' not found` | Интерфейс не поднят: `sudo ip link set can0 up type can bitrate 500000`. |
| `write('can0') failed: No buffer space available` | Нет терминаторов 120 Ом, VESC обесточены или не отвечают (ACK). |
| `velocity` в `/joint_states` всегда 0 | VESC не шлют STATUS: включите CAN Status Message Mode и проверьте VESC ID в `candump`. |
| `battery/voltage` всегда 0 | Не включён STATUS_5 (режим `CAN_STATUS_1_2_3_4_5`). |
| Робот едет быстрее/медленнее команды | Неверный `gear_ratio` или `max_rps`; см. «Калибровка». |
| Дрожание на малых оборотах в режиме `rpm` | Настройте PID скорости в VESC Tool или вернитесь к `duty` с `min_duty`. |

## Протокол CAN

CAN ID = `(packet_id << 8) | vesc_id`, расширенный (29-битный) формат, все числа big-endian.

| Пакет | ID | Направление | Содержимое |
|---|---|---|---|
| `SET_DUTY` | 0 | → VESC | `int32 duty × 100000` |
| `SET_RPM` | 3 | → VESC | `int32 ERPM` (клампится ±23250) |
| `STATUS` | 9 | ← VESC | `int32 ERPM`, `int16 ток мотора × 10`, `int16 duty × 1000` |
| `STATUS_2` | 14 | ← VESC | `int32 Ah × 1e4`, `int32 Ah заряжено × 1e4` |
| `STATUS_4` | 16 | ← VESC | `int16 t° FET × 10`, `int16 t° мотора × 10`, `int16 входной ток × 10`, `int16 PID-позиция × 50` |
| `STATUS_5` | 27 | ← VESC | `int32 тахометр`, `int16 напряжение × 10` |

`STATUS_3` (Wh) и `STATUS_6` (ADC/PPM) принимаются и игнорируются. Инструменты дополнительно используют `SET_POS` (4).

## Разработка

- Код плагина — C++17, предупреждения `-Wall -Wextra -Wpedantic`; логика протокола и пересчёта вынесена в
  `message_translator` и `VescHandler` и покрыта gtest без железа.
- Инструменты — Python, общий модуль `tools/vesc_can.py`; чистые функции покрыты pytest.
- Перед PR: `colcon build && colcon test` и `python3 -m pytest`; к PR прикладывайте вывод.
- Версия пакета — в `src/vesc_nexus/package.xml`; изменения параметров конфигурации описывайте в README.

## Лицензия и благодарности

MIT, см. [LICENSE](LICENSE).

- Сообщение `VescState` основано на пакете `vesc_msgs` из [f1tenth/vesc](https://github.com/f1tenth/vesc).
- Протокол CAN — по прошивке [vedderb/bldc](https://github.com/vedderb/bldc) Бенджамина Веддера.
