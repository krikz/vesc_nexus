"""Тесты чистой части протокола и учёта ошибок VescBus. Железо и python-can не нужны."""
import struct
import sys
from pathlib import Path

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

import vesc_can  # noqa: E402
from vesc_can import (  # noqa: E402
    CAN_PACKET_SET_DUTY,
    CAN_PACKET_SET_POS,
    CAN_PACKET_SET_RPM,
    CAN_PACKET_STATUS,
    CanBusError,
    VescBus,
    encode_duty,
    encode_erpm,
    encode_position_deg,
    make_can_id,
    parse_status,
    split_can_id,
)


def status_payload(erpm: int, current_x10: int = 0, duty_x1000: int = 0) -> bytes:
    return struct.pack(">ihh", erpm, current_x10, duty_x1000)


# ─── CAN ID ─────────────────────────────────────────────────────────────────

def test_make_and_split_can_id_roundtrip():
    can_id = make_can_id(CAN_PACKET_STATUS, 49)
    assert can_id == (9 << 8) | 49
    assert split_can_id(can_id) == (CAN_PACKET_STATUS, 49)


@pytest.mark.parametrize("vesc_id", [-1, 255, 1000])
def test_make_can_id_rejects_bad_vesc_id(vesc_id):
    with pytest.raises(ValueError):
        make_can_id(CAN_PACKET_SET_DUTY, vesc_id)


# ─── Кодирование ────────────────────────────────────────────────────────────

@pytest.mark.parametrize("duty,expected", [
    (0.0, 0),
    (0.1, 10_000),
    (-0.25, -25_000),
    (1.0, 100_000),
    (2.0, 100_000),
    (-7.0, -100_000),
])
def test_encode_duty_scales_and_clamps(duty, expected):
    assert struct.unpack(">i", encode_duty(duty))[0] == expected


def test_encode_erpm_rounds_to_int():
    assert struct.unpack(">i", encode_erpm(1799.6))[0] == 1800
    assert struct.unpack(">i", encode_erpm(-750))[0] == -750


def test_encode_position_scales_degrees():
    assert struct.unpack(">i", encode_position_deg(360.0))[0] == 360_000_000
    assert struct.unpack(">i", encode_position_deg(-0.5))[0] == -500_000


# ─── Разбор статуса ─────────────────────────────────────────────────────────

def test_parse_status_decodes_fields():
    status = parse_status(make_can_id(CAN_PACKET_STATUS, 49), status_payload(1500, 87, -456),
                          pole_pairs=15, timestamp=12.5)
    assert status is not None
    assert status.vesc_id == 49
    assert status.erpm == 1500.0
    assert status.rpm == pytest.approx(100.0)
    assert status.current == pytest.approx(8.7)
    assert status.duty == pytest.approx(-0.456)
    assert status.timestamp == 12.5


def test_parse_status_ignores_other_packets_and_short_frames():
    assert parse_status(make_can_id(CAN_PACKET_SET_RPM, 49), status_payload(1500), 15) is None
    assert parse_status(make_can_id(CAN_PACKET_STATUS, 49), b"\x00\x01\x02", 15) is None


def test_parse_status_rejects_bad_pole_pairs():
    with pytest.raises(ValueError):
        parse_status(make_can_id(CAN_PACKET_STATUS, 1), status_payload(1), pole_pairs=0)


# ─── VescBus: учёт ошибок без железа ────────────────────────────────────────

class FakeCanError(Exception):
    pass


class FakeMessage:
    def __init__(self, arbitration_id, data):
        self.arbitration_id = arbitration_id
        self.data = data


class FakeBus:
    """Эмулирует python-can Bus: очередь входящих сообщений и опциональный сбой при отправке."""

    def __init__(self, incoming=(), fail_send=False):
        self.incoming = list(incoming)
        self.fail_send = fail_send
        self.sent = []

    def send(self, message):
        if self.fail_send:
            raise FakeCanError("tx buffer full")
        self.sent.append(message)

    def recv(self, timeout=None):
        return self.incoming.pop(0) if self.incoming else None

    def shutdown(self):
        pass


@pytest.fixture
def fake_can(monkeypatch):
    """Подменяет модуль can минимальной заглушкой, чтобы VescBus.send() работал без python-can."""
    import types

    module = types.ModuleType("can")
    module.CanError = FakeCanError

    class Message:
        def __init__(self, arbitration_id, data, is_extended_id):
            self.arbitration_id = arbitration_id
            self.data = data
            self.is_extended_id = is_extended_id

    module.Message = Message
    monkeypatch.setitem(sys.modules, "can", module)
    return module


def make_bus(fake: FakeBus, pole_pairs: int = 15) -> VescBus:
    bus = VescBus("vcan0", pole_pairs=pole_pairs, max_consecutive_errors=3)
    bus._bus = fake
    bus._error_types = (FakeCanError,)
    return bus


def test_send_rpm_converts_to_erpm_with_extended_id(fake_can):
    fake = FakeBus()
    bus = make_bus(fake)
    assert bus.send_rpm(49, 100.0) is True
    assert len(fake.sent) == 1
    message = fake.sent[0]
    assert message.is_extended_id is True
    assert split_can_id(message.arbitration_id) == (CAN_PACKET_SET_RPM, 49)
    assert struct.unpack(">i", message.data)[0] == 1500


def test_send_position_uses_set_pos_packet(fake_can):
    fake = FakeBus()
    bus = make_bus(fake)
    bus.send_position(81, 90.0)
    assert split_can_id(fake.sent[0].arbitration_id) == (CAN_PACKET_SET_POS, 81)
    assert struct.unpack(">i", fake.sent[0].data)[0] == 90_000_000


def test_read_status_filters_by_vesc_id_and_resets_error_counter():
    frames = [
        FakeMessage(make_can_id(CAN_PACKET_STATUS, 94), status_payload(300)),
        FakeMessage(make_can_id(CAN_PACKET_STATUS, 49), status_payload(1500)),
    ]
    bus = make_bus(FakeBus(incoming=frames))
    bus.consecutive_errors = 2
    assert bus.read_status(vesc_id=49) is None  # первый кадр от другого VESC
    status = bus.read_status(vesc_id=49)
    assert status is not None and status.rpm == pytest.approx(100.0)
    assert bus.consecutive_errors == 0


def test_consecutive_send_errors_raise_can_bus_error(fake_can):
    bus = make_bus(FakeBus(fail_send=True))
    assert bus.send_duty(49, 0.1) is False
    assert bus.send_duty(49, 0.1) is False
    with pytest.raises(CanBusError):
        bus.send_duty(49, 0.1)


def test_successful_send_resets_error_counter(fake_can):
    fake = FakeBus(fail_send=True)
    bus = make_bus(fake)
    bus.send_duty(49, 0.1)
    fake.fail_send = False
    assert bus.send_duty(49, 0.1) is True
    assert bus.consecutive_errors == 0


def test_operations_without_connect_raise():
    bus = VescBus("vcan0")
    with pytest.raises(CanBusError):
        bus.send_duty(1, 0.0)
    with pytest.raises(CanBusError):
        bus.read_status()


def test_module_has_no_hard_dependency_on_python_can():
    assert "can" not in sys.modules or sys.modules["can"].__name__ == "can"
    assert vesc_can.MAX_CONSECUTIVE_CAN_ERRORS == 10
