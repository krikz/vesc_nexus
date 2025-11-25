# VESC Nexus Calibration Tools

Утилиты для калибровки VESC моторов.

## Требования

```bash
pip install python-can pyyaml matplotlib pandas
```

## Скрипты

### 1. calibrate_max_rpm.py — Автокалибровка максимальных оборотов

Автоматически определяет максимальные обороты для каждого колеса при duty=100%.

**Использование:**
```bash
# Калибровка 4 колёс
python3 calibrate_max_rpm.py --vesc-ids 49 124 81 94 --can-interface can0

# С указанием выходного файла
python3 calibrate_max_rpm.py --vesc-ids 49 124 --output my_calibration.yaml
```

**Алгоритм:**
1. Постепенно увеличивает duty cycle
2. Отслеживает рост RPM
3. Останавливается когда RPM перестаёт расти (насыщение)
4. Повторяет для обратного направления
5. Сохраняет результаты в YAML

**Выход:**
```yaml
calibration:
  timestamp: '2024-01-15T10:30:00'
wheels:
  vesc_49:
    max_rpm_forward: 900.0
    max_rpm_backward: 895.0
    avg_max_rps: 14.96

config_snippet:
  wheel_max_rps: [14.96, 15.02, 14.88, 15.10]
```

---

### 2. linearity_test.py — Тест линейности зависимости duty → RPM

Проверяет насколько линейна зависимость между duty cycle и оборотами.

**Использование:**
```bash
# Базовый тест с шагом 1%
python3 linearity_test.py --vesc-id 49

# Тест с шагом 5% до 80%
python3 linearity_test.py --vesc-id 49 --duty-step 0.05 --duty-max 0.8

# Тест в обоих направлениях
python3 linearity_test.py --vesc-id 49 --both-directions
```

**Выход:**
- `linearity_test_vesc49_forward.csv` — данные в CSV формате
- `linearity_test_vesc49_forward.json` — данные в JSON формате
- `linearity_test_vesc49_plot.png` — график зависимости

**Интерпретация R²:**
- R² > 0.95 — линейная зависимость ✅
- R² < 0.95 — нелинейная зависимость ⚠️

---

## Процедура калибровки

1. **Подготовка:**
   - Поставить робота на подставку (колёса должны свободно вращаться)
   - Подключить CAN адаптер
   - Поднять CAN интерфейс: `sudo ip link set can0 up type can bitrate 500000`

2. **Калибровка:**
   ```bash
   cd src/vesc_nexus/tools
   python3 calibrate_max_rpm.py --vesc-ids 49 124 81 94
   ```

3. **Применение результатов:**
   - Скопировать `wheel_max_rps` из результатов в `config/vesc_nexus_config.yaml`

4. **(Опционально) Проверка линейности:**
   ```bash
   python3 linearity_test.py --vesc-id 49 --both-directions
   ```

---

## Формат конфигурации

После калибровки добавьте в `config/vesc_nexus_config.yaml`:

```yaml
# Максимальные обороты при duty=100% (результат калибровки)
wheel_max_rps: [15.0, 15.0, 15.0, 15.0]
```

Формула расчёта duty из скорости:
```
max_speed = 2π × wheel_max_rps × wheel_radius
duty = target_speed / max_speed
```
