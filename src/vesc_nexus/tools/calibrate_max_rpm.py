#!/usr/bin/env python3
"""
Скрипт автоматической калибровки максимальных оборотов VESC при duty=100%

ИСПОЛЬЗОВАНИЕ:
    python3 calibrate_max_rpm.py --vesc-ids 49 124 81 94 --can-interface can0

ОПИСАНИЕ:
    Для каждого VESC ID:
    1. Разгоняет колесо постепенно увеличивая duty cycle
    2. Останавливается когда ERPM перестаёт значительно расти (насыщение)
    3. Замеряет максимальные обороты в обоих направлениях (вперёд/назад)
    4. Сохраняет результаты в YAML файл

ТРЕБОВАНИЯ:
    pip install python-can pyyaml
"""

import argparse
import can
import struct
import time
import yaml
import sys
from dataclasses import dataclass
from typing import Dict, List, Optional
from datetime import datetime


# VESC CAN Protocol Constants
CAN_PACKET_SET_DUTY = 0
CAN_PACKET_STATUS = 9

# Настройки калибровки
DUTY_STEP = 0.05          # Шаг увеличения duty (5%)
DUTY_MAX = 1.0            # Максимальный duty cycle
SETTLE_TIME = 1.0         # Время стабилизации оборотов (сек)
RPM_CHANGE_THRESHOLD = 50 # Порог изменения RPM для детекции насыщения
MAX_ITERATIONS = 25       # Максимум итераций разгона


@dataclass
class CalibrationResult:
    """Результат калибровки одного колеса"""
    vesc_id: int
    max_rpm_forward: float
    max_rpm_backward: float
    max_duty_forward: float
    max_duty_backward: float
    max_rps_forward: float
    max_rps_backward: float
    timestamp: str


class VescCalibrator:
    """Калибратор для VESC моторов"""
    
    def __init__(self, can_interface: str):
        self.can_interface = can_interface
        self.bus: Optional[can.Bus] = None
        self.current_rpm: Dict[int, float] = {}  # vesc_id → rpm
        
    def connect(self) -> bool:
        """Подключение к CAN шине"""
        try:
            self.bus = can.Bus(channel=self.can_interface, interface='socketcan')
            print(f"✅ Подключено к CAN интерфейсу: {self.can_interface}")
            return True
        except Exception as e:
            print(f"❌ Ошибка подключения к CAN: {e}")
            return False
    
    def disconnect(self):
        """Отключение от CAN шины"""
        if self.bus:
            self.bus.shutdown()
            print("Отключено от CAN шины")
    
    def send_duty_cycle(self, vesc_id: int, duty: float):
        """Отправка команды duty cycle на VESC"""
        duty_scaled = int(duty * 100000)  # VESC формат: duty * 100000
        data = struct.pack('>i', duty_scaled)
        
        # CAN ID: command_id << 8 | vesc_id
        can_id = (CAN_PACKET_SET_DUTY << 8) | vesc_id
        msg = can.Message(
            arbitration_id=can_id,
            data=data,
            is_extended_id=True
        )
        
        try:
            self.bus.send(msg)
        except can.CanError as e:
            print(f"Ошибка отправки CAN: {e}")
    
    def read_status(self, timeout: float = 0.1) -> Optional[tuple]:
        """Чтение статуса VESC (ERPM, current, duty)"""
        try:
            msg = self.bus.recv(timeout=timeout)
            if msg is None:
                return None
            
            vesc_id = msg.arbitration_id & 0xFF
            cmd_id = (msg.arbitration_id >> 8) & 0xFF
            
            if cmd_id == CAN_PACKET_STATUS and len(msg.data) >= 8:
                erpm = struct.unpack('>i', msg.data[0:4])[0]
                current = struct.unpack('>h', msg.data[4:6])[0] / 10.0
                duty = struct.unpack('>h', msg.data[6:8])[0] / 1000.0
                return (vesc_id, erpm, current, duty)
        except Exception:
            pass
        return None
    
    def get_stable_rpm(self, vesc_id: int, duty: float, settle_time: float = SETTLE_TIME) -> float:
        """Подать duty и дождаться стабильного RPM"""
        self.send_duty_cycle(vesc_id, duty)
        time.sleep(settle_time)
        
        # Считываем несколько значений и усредняем
        rpm_values = []
        start_time = time.time()
        while time.time() - start_time < 0.5:
            result = self.read_status(timeout=0.1)
            if result and result[0] == vesc_id:
                rpm_values.append(result[1])
        
        if rpm_values:
            return sum(rpm_values) / len(rpm_values)
        return 0.0
    
    def find_max_rpm(self, vesc_id: int, direction: int = 1) -> tuple:
        """
        Найти максимальные обороты разгоняя колесо до насыщения
        
        Args:
            vesc_id: ID VESC контроллера
            direction: 1 для вперёд, -1 для назад
            
        Returns:
            (max_rpm, duty_at_max_rpm)
        """
        direction_name = "ВПЕРЁД" if direction > 0 else "НАЗАД"
        print(f"\n  📊 Калибровка {direction_name}...")
        
        prev_rpm = 0.0
        max_rpm = 0.0
        duty_at_max = 0.0
        
        for i in range(MAX_ITERATIONS):
            duty = min(DUTY_STEP * (i + 1), DUTY_MAX) * direction
            
            current_rpm = self.get_stable_rpm(vesc_id, duty)
            rpm_change = abs(current_rpm) - abs(prev_rpm)
            
            print(f"    duty={duty:+.2f} → RPM={current_rpm:.0f} (Δ={rpm_change:.0f})")
            
            # Обновляем максимум
            if abs(current_rpm) > abs(max_rpm):
                max_rpm = current_rpm
                duty_at_max = duty
            
            # Детекция насыщения: если RPM почти не растёт
            if i > 2 and rpm_change < RPM_CHANGE_THRESHOLD:
                print(f"    ✓ Насыщение достигнуто при duty={duty:.2f}")
                break
            
            prev_rpm = current_rpm
            
            if abs(duty) >= DUTY_MAX:
                print(f"    ✓ Достигнут максимальный duty={DUTY_MAX}")
                break
        
        # Остановка
        self.send_duty_cycle(vesc_id, 0.0)
        time.sleep(0.5)
        
        return (abs(max_rpm), abs(duty_at_max))
    
    def calibrate_wheel(self, vesc_id: int) -> CalibrationResult:
        """Полная калибровка одного колеса"""
        print(f"\n{'='*50}")
        print(f"🔧 Калибровка VESC ID: {vesc_id}")
        print(f"{'='*50}")
        
        # Калибровка вперёд
        max_rpm_fwd, duty_fwd = self.find_max_rpm(vesc_id, direction=1)
        
        # Пауза между направлениями
        print("\n  ⏸️  Пауза 2 сек...")
        time.sleep(2.0)
        
        # Калибровка назад
        max_rpm_bwd, duty_bwd = self.find_max_rpm(vesc_id, direction=-1)
        
        # Результаты
        max_rps_fwd = max_rpm_fwd / 60.0
        max_rps_bwd = max_rpm_bwd / 60.0
        
        result = CalibrationResult(
            vesc_id=vesc_id,
            max_rpm_forward=max_rpm_fwd,
            max_rpm_backward=max_rpm_bwd,
            max_duty_forward=duty_fwd,
            max_duty_backward=duty_bwd,
            max_rps_forward=max_rps_fwd,
            max_rps_backward=max_rps_bwd,
            timestamp=datetime.now().isoformat()
        )
        
        print(f"\n  📋 Результаты:")
        print(f"    Вперёд:  {max_rpm_fwd:.0f} RPM = {max_rps_fwd:.2f} об/сек при duty={duty_fwd:.2f}")
        print(f"    Назад:   {max_rpm_bwd:.0f} RPM = {max_rps_bwd:.2f} об/сек при duty={duty_bwd:.2f}")
        
        return result


def save_results(results: List[CalibrationResult], output_file: str):
    """Сохранение результатов в YAML файл"""
    data = {
        'calibration': {
            'timestamp': datetime.now().isoformat(),
            'description': 'Результаты автокалибровки VESC',
        },
        'wheels': {}
    }
    
    # Для конфига драйвера
    wheel_max_rps = []
    
    for r in results:
        # Используем среднее от forward и backward
        avg_rps = (r.max_rps_forward + r.max_rps_backward) / 2.0
        wheel_max_rps.append(round(avg_rps, 2))
        
        data['wheels'][f'vesc_{r.vesc_id}'] = {
            'vesc_id': r.vesc_id,
            'max_rpm_forward': round(r.max_rpm_forward, 1),
            'max_rpm_backward': round(r.max_rpm_backward, 1),
            'max_rps_forward': round(r.max_rps_forward, 2),
            'max_rps_backward': round(r.max_rps_backward, 2),
            'avg_max_rps': round(avg_rps, 2),
            'timestamp': r.timestamp
        }
    
    data['config_snippet'] = {
        'wheel_max_rps': wheel_max_rps,
        'description': 'Скопируйте в vesc_nexus_config.yaml'
    }
    
    with open(output_file, 'w') as f:
        yaml.dump(data, f, default_flow_style=False, allow_unicode=True)
    
    print(f"\n✅ Результаты сохранены в: {output_file}")
    print(f"\n📋 Для конфига драйвера добавьте:")
    print(f"   wheel_max_rps: {wheel_max_rps}")


def main():
    parser = argparse.ArgumentParser(
        description='Автокалибровка максимальных оборотов VESC моторов'
    )
    parser.add_argument(
        '--vesc-ids', 
        type=int, 
        nargs='+', 
        required=True,
        help='Список VESC ID для калибровки (например: 49 124 81 94)'
    )
    parser.add_argument(
        '--can-interface', 
        type=str, 
        default='can0',
        help='CAN интерфейс (по умолчанию: can0)'
    )
    parser.add_argument(
        '--output', 
        type=str, 
        default='calibration_results.yaml',
        help='Файл для сохранения результатов'
    )
    
    args = parser.parse_args()
    
    print("🚀 VESC Калибровка максимальных оборотов")
    print("="*50)
    print(f"VESC IDs: {args.vesc_ids}")
    print(f"CAN интерфейс: {args.can_interface}")
    print(f"Выходной файл: {args.output}")
    print("="*50)
    
    print("\n⚠️  ВНИМАНИЕ!")
    print("   Убедитесь что робот стоит на подставке")
    print("   и колёса могут свободно вращаться!")
    input("\nНажмите Enter для начала калибровки...")
    
    calibrator = VescCalibrator(args.can_interface)
    
    if not calibrator.connect():
        sys.exit(1)
    
    try:
        results = []
        for vesc_id in args.vesc_ids:
            result = calibrator.calibrate_wheel(vesc_id)
            results.append(result)
        
        save_results(results, args.output)
        
    finally:
        calibrator.disconnect()
    
    print("\n✅ Калибровка завершена!")


if __name__ == '__main__':
    main()
