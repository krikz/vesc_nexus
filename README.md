# VESC Nexus — Multi-CAN VESC Driver for ROS 2

[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](LICENSE)
[![ROS 2](https://img.shields.io/badge/ROS-Humble-brightgreen)](https://docs.ros.org/en/humble/)

**`vesc_nexus`** — это высокопроизводительный ROS 2-драйвер для управления **множеством VESC-регуляторов** по CAN-шине. Поддерживает до 4+ VESC на одном или нескольких CAN-интерфейсах, публикует состояние моторов и одометрию, принимает команды от `cmd_vel`.

Идеально подходит для:
- Роботов с дифференциальным приводом
- Четырёхколёсных платформ
- Проектов на Raspberry Pi, Jetson, x86

## 🚀 Основные возможности

- ✅ Управление **несколькими VESC** по CAN (ID: 1–254)
- ✅ Поддержка **нескольких CAN-интерфейсов** (`can0`, `can1`, ...)
- ✅ Приём команд от `/cmd_vel` → преобразование в RPM
- ✅ Публикация состояния каждого мотора: RPM, ток, температура, напряжение
- ✅ Расчёт и публикация `/odom` (одометрия)
- ✅ Гибкая конфигурация через YAML
- ✅ Запуск в Docker (включая Raspberry Pi)
- ✅ Поддержка ROS 2 Humble/Iron

## 📦 Установка

### 1. Клонирование

```bash
cd ~/ros2_ws/src
git clone https://github.com/your-org/vesc_nexus.git
```

### 2. Установка зависимостей

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

### 3. Сборка

```bash
colcon build --packages-select vesc_nexus vesc_msgs
source install/setup.bash
```

## ⚙️ Конфигурация

Настройка производится через `config/vesc_nexus_config.yaml`.

Пример для 4 VESC на `can0`:

```yaml
/**:
  ros__parameters:
    can_interface: "can0"
    vesc_ids: [49, 124, 81, 94]
    wheel_labels: ["front_left", "front_right", "rear_left", "rear_right"]
    publish_rate: 50.0
    speed_max: 23250.0
    speed_min: -23250.0
    current_max: 10.0
    brake_max: 200000.0
    brake_min: -20000.0
    servo_min: 0.15
    servo_max: 0.85
```

> 🔔 Убедитесь, что VESC прошиты с поддержкой CAN и имеют соответствующие ID.

## ▶️ Запуск

### 1. Поднимите CAN-интерфейс

```bash
sudo ip link set can0 up type can bitrate 500000
```

### 2. Запустите драйвер

```bash
ros2 launch vesc_nexus vesc_nexus_node.launch.py
```

## 📡 Топики

### Публикуемые топики

| Топик | Тип | Описание |
|-------|-----|--------|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | Принимает команды движения |
| `/odom` | `nav_msgs/msg/Odometry` | Публикует одометрию робота |
| `/sensors/motor_state/<label>` | `vesc_msgs/msg/VescStateStamped` | Состояние каждого мотора (RPM, ток, температура) |
| `/tf` | `tf2_msgs/msg/TFMessage` | Публикует преобразование `odom → base_link` |

### Пример: чтение состояния мотора

```bash
ros2 topic echo /sensors/motor_state/front_left
```

## 🐳 Запуск в Docker

### Сборка образа

```bash
docker build -t vesc_nexus .
```

### Запуск с доступом к CAN

```bash
docker run -it \
  --network=host \
  --cap-add=NET_ADMIN \
  --device=/dev/socketcan \
  --name vesc-nexus \
  --rm \
  vesc_nexus:latest
```

## 🔧 Диагностика

### Проверка CAN-трафика

```bash
# Просмотр всех CAN-фреймов
candump can0

# Отправка тестовой команды (например, запрос состояния VESC с ID 49)
cansend can0 031#04
```

### Проверка подключения узла

```bash
# Список узлов
ros2 node list

# Информация о подписках
ros2 node info /vesc_can_driver
```

## 🧪 Тестирование управления

### Движение вперёд и поворот

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: 0.3}}"
```

### Остановка

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{}"
```

## ⚠️ Распространённые проблемы

| Проблема | Решение |
|--------|--------|
| `errno=105 (No buffer space available)` | Нет терминатора 120 Ом, VESC не подключены, `can0` не поднят |
| Нет ответов от VESC | Проверьте CAN_H/CAN_L, питание, ID, прошивку |
| Не публикуются данные | Убедитесь, что `cmd_vel` отправляется, и `publish_rate > 0` |
| `bad_weak_ptr` при старте | Убедитесь, что `shared_from_this()` не вызывается в конструкторе |

## 📁 Структура проекта

```
vesc_nexus/
├── config/
│   └── vesc_nexus_config.yaml
├── launch/
│   └── vesc_nexus_node.launch.py
├── src/
│   ├── can_interface.cpp
│   ├── vesc_can_driver_node.cpp
│   ├── vesc_handler.cpp
│   ├── message_translator.cpp
│   └── odometry_publisher.cpp
├── include/vesc_nexus/
│   ├── can_interface.hpp
│   ├── vesc_handler.hpp
│   ├── message_translator.hpp
│   └── odometry_publisher.hpp
└── Dockerfile
```

## 📄 Лицензия

MIT

---

> Поддержка: [vesc.org](https://vesc.org/) | [GitHub: vedderb/bldc](https://github.com/vedderb/bldc)