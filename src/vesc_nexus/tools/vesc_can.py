"""Общий слой протокола VESC по CAN для инструментов калибровки и ручного управления.

Чистые функции кодирования и разбора не требуют python-can и покрыты тестами (tests/).
VescBus — тонкая обёртка над python-can (SocketCAN) с учётом ошибок шины:
после MAX_CONSECUTIVE_CAN_ERRORS подряд идущих ошибок работа прерывается
исключением CanBusError, чтобы калибровка не записала мусор как результат.
"""
from __future__ import annotations

import struct
import time
from dataclasses import dataclass
from typing import Optional

# Идентификаторы пакетов VESC (bldc/datatypes.h). CAN ID = (packet_id << 8) | vesc_id, 29-битный.
CAN_PACKET_SET_DUTY = 0
CAN_PACKET_SET_RPM = 3
CAN_PACKET_SET_POS = 4
CAN_PACKET_STATUS = 9

DUTY_SCALE = 100_000          # duty × 100000
POSITION_SCALE = 1_000_000    # градусы × 1e6
STATUS_CURRENT_SCALE = 10.0   # int16 → А
STATUS_DUTY_SCALE = 1000.0    # int16 → доля 0..1

#: Сколько подряд идущих ошибок CAN считаем «шина нестабильна».
MAX_CONSECUTIVE_CAN_ERRORS = 10


def configure_console() -> None:
    """Не падать на эмодзи и кириллице в консолях без UTF-8 (Windows cp1252, голый TTY)."""
    import sys
    for stream in (sys.stdout, sys.stderr):
        if hasattr(stream, "reconfigure"):
            stream.reconfigure(errors="replace")


# ─── Чистые функции протокола ───────────────────────────────────────────────

def make_can_id(packet_id: int, vesc_id: int) -> int:
    """Собрать 29-битный CAN ID для пакета VESC."""
    if not 0 <= vesc_id <= 254:
        raise ValueError(f"vesc_id must be in 0..254, got {vesc_id}")
    return (packet_id << 8) | vesc_id


def split_can_id(can_id: int) -> tuple[int, int]:
    """Разобрать CAN ID на (packet_id, vesc_id)."""
    return (can_id >> 8) & 0xFF, can_id & 0xFF


def encode_duty(duty: float) -> bytes:
    """CAN_PACKET_SET_DUTY: duty ∈ [-1, 1], клампится."""
    duty = max(-1.0, min(1.0, duty))
    return struct.pack(">i", int(round(duty * DUTY_SCALE)))


def encode_erpm(erpm: float) -> bytes:
    """CAN_PACKET_SET_RPM: электрические об/мин."""
    return struct.pack(">i", int(round(erpm)))


def encode_position_deg(degrees: float) -> bytes:
    """CAN_PACKET_SET_POS: угол в градусах (в endless-режиме накапливается)."""
    return struct.pack(">i", int(round(degrees * POSITION_SCALE)))


@dataclass(frozen=True)
class VescStatus:
    """Разобранный CAN_PACKET_STATUS."""

    vesc_id: int
    erpm: float
    rpm: float      # механические об/мин вала мотора = erpm / pole_pairs
    current: float  # ток мотора, А
    duty: float     # -1..1
    timestamp: float


def parse_status(can_id: int, data: bytes, pole_pairs: int,
                 timestamp: Optional[float] = None) -> Optional[VescStatus]:
    """Разобрать CAN_PACKET_STATUS. None — если это другой пакет или фрейм короче 8 байт."""
    packet_id, vesc_id = split_can_id(can_id)
    if packet_id != CAN_PACKET_STATUS or len(data) < 8:
        return None
    if pole_pairs < 1:
        raise ValueError(f"pole_pairs must be >= 1, got {pole_pairs}")
    erpm = struct.unpack(">i", data[0:4])[0]
    current = struct.unpack(">h", data[4:6])[0] / STATUS_CURRENT_SCALE
    duty = struct.unpack(">h", data[6:8])[0] / STATUS_DUTY_SCALE
    return VescStatus(
        vesc_id=vesc_id,
        erpm=float(erpm),
        rpm=erpm / pole_pairs,
        current=current,
        duty=duty,
        timestamp=time.time() if timestamp is None else timestamp,
    )


# ─── Шина ───────────────────────────────────────────────────────────────────

class CanBusError(RuntimeError):
    """Шина недоступна или нестабильна (превышен порог подряд идущих ошибок)."""


class VescBus:
    """Соединение с SocketCAN через python-can.

    Использование::

        with VescBus("can0", pole_pairs=15) as bus:
            bus.send_duty(49, 0.1)
            status = bus.read_status(vesc_id=49)

    При выходе из контекста stop-команда НЕ отправляется — это ответственность инструмента.
    """

    def __init__(self, channel: str = "can0", pole_pairs: int = 15,
                 max_consecutive_errors: int = MAX_CONSECUTIVE_CAN_ERRORS):
        if pole_pairs < 1:
            raise ValueError(f"pole_pairs must be >= 1, got {pole_pairs}")
        self.channel = channel
        self.pole_pairs = pole_pairs
        self.max_consecutive_errors = max_consecutive_errors
        self.consecutive_errors = 0
        self._bus = None
        # Типы исключений python-can; подставляются в connect(), в тестах можно переопределить.
        self._error_types: tuple[type, ...] = (OSError,)

    # Жизненный цикл ---------------------------------------------------------

    def connect(self) -> "VescBus":
        try:
            import can  # импорт здесь, чтобы --help и тесты работали без python-can
        except ImportError as exc:
            raise CanBusError("python-can не установлен: pip install python-can") from exc
        try:
            self._bus = can.Bus(channel=self.channel, interface="socketcan")
        except Exception as exc:  # python-can бросает разные типы в зависимости от причины
            raise CanBusError(f"не удалось открыть {self.channel}: {exc}") from exc
        self._error_types = (can.CanError,)
        return self

    def close(self) -> None:
        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None

    def __enter__(self) -> "VescBus":
        return self.connect()

    def __exit__(self, *exc_info) -> None:
        self.close()

    @property
    def is_open(self) -> bool:
        return self._bus is not None

    # Отправка ---------------------------------------------------------------

    def send(self, packet_id: int, vesc_id: int, payload: bytes) -> bool:
        """Отправить пакет. False — если шина ответила ошибкой (учитывается в счётчике)."""
        if self._bus is None:
            raise CanBusError("шина не открыта, вызовите connect()")
        import can
        message = can.Message(arbitration_id=make_can_id(packet_id, vesc_id),
                              data=payload, is_extended_id=True)
        try:
            self._bus.send(message)
        except self._error_types as exc:
            self._register_error(f"отправка пакета {packet_id} на VESC {vesc_id}", exc)
            return False
        self.consecutive_errors = 0
        return True

    def send_duty(self, vesc_id: int, duty: float) -> bool:
        return self.send(CAN_PACKET_SET_DUTY, vesc_id, encode_duty(duty))

    def send_erpm(self, vesc_id: int, erpm: float) -> bool:
        return self.send(CAN_PACKET_SET_RPM, vesc_id, encode_erpm(erpm))

    def send_rpm(self, vesc_id: int, rpm: float) -> bool:
        """Механические об/мин вала мотора → ERPM через pole_pairs."""
        return self.send_erpm(vesc_id, rpm * self.pole_pairs)

    def send_position(self, vesc_id: int, degrees: float) -> bool:
        return self.send(CAN_PACKET_SET_POS, vesc_id, encode_position_deg(degrees))

    # Приём ------------------------------------------------------------------

    def read_status(self, timeout: float = 0.1, vesc_id: Optional[int] = None) -> Optional[VescStatus]:
        """Дождаться CAN_PACKET_STATUS (опционально — от конкретного VESC).

        Возвращает None по таймауту, для чужих пакетов и для пакетов других VESC.
        """
        if self._bus is None:
            raise CanBusError("шина не открыта, вызовите connect()")
        try:
            message = self._bus.recv(timeout=timeout)
        except self._error_types as exc:
            self._register_error("чтение статуса", exc)
            return None
        if message is None:
            return None
        try:
            status = parse_status(message.arbitration_id, bytes(message.data), self.pole_pairs)
        except (struct.error, ValueError) as exc:
            self._register_error("разбор статуса", exc)
            return None
        if status is None:
            return None
        self.consecutive_errors = 0
        if vesc_id is not None and status.vesc_id != vesc_id:
            return None
        return status

    def collect_statuses(self, vesc_id: int, duration: float, timeout: float = 0.1) -> list[VescStatus]:
        """Собрать все статусы указанного VESC за duration секунд."""
        statuses: list[VescStatus] = []
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            status = self.read_status(timeout=timeout, vesc_id=vesc_id)
            if status is not None:
                statuses.append(status)
        return statuses

    # Ошибки -----------------------------------------------------------------

    def _register_error(self, operation: str, error: Exception) -> None:
        self.consecutive_errors += 1
        print(f"⚠️  Ошибка CAN ({operation}): {error} "
              f"[{self.consecutive_errors}/{self.max_consecutive_errors} подряд]")
        if self.consecutive_errors >= self.max_consecutive_errors:
            raise CanBusError(
                f"CAN-шина нестабильна: {self.consecutive_errors} ошибок подряд "
                f"({operation}: {error}). Проверьте подключение CAN."
            ) from error
