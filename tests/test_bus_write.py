import json
from pathlib import Path

import pytest

from so101.hw.bus import WRITABLE, BusError, MockBus, check_write, read
from so101.hw.registers import REGISTERS

FACTORY = json.loads((Path(__file__).parent / "fixtures" / "follower_factory_dump.json").read_text())


def owned(bus: MockBus, owner: str = "safearm") -> MockBus:
    bus.claim(owner)
    return bus


def test_only_sram_registers_are_writable():
    assert all(REGISTERS[n].area == "sram" for n in WRITABLE)


@pytest.mark.parametrize("name", ["Offset", "Min_Angle_Limit", "Max_Angle_Limit", "ID", "Mode", "Lock",
                                  "P_Coefficient", "Max_Torque_Limit", "Present_Position"])  # fmt: skip
def test_everything_else_is_refused(name):
    with pytest.raises(BusError, match="refused"):
        check_write(name, 1)


@pytest.mark.parametrize(
    "name, value",
    [
        ("Torque_Enable", 128),  # recalibrates the offset (EEPROM) per Feetech docs
        ("Torque_Enable", 2),
        ("Goal_Speed", 0),  # 0 = unlimited speed
        ("Acceleration", 0),  # 0 = unlimited acceleration
        ("Acceleration", 255),
        ("Goal_Position", -1),
        ("Goal_Position", 4096),
    ],
)
def test_dangerous_values_are_refused(name, value):
    with pytest.raises(BusError, match="refused"):
        check_write(name, value)


def test_write_requires_ownership():
    bus = MockBus.with_servos([1])
    with pytest.raises(BusError, match="claim it first"):
        bus.write("safearm", 1, "Goal_Position", 2000)
    bus.claim("poller-follower")
    with pytest.raises(BusError, match="owner is 'poller-follower'"):
        bus.write("safearm", 1, "Goal_Position", 2000)
    assert bus.write_log == []


def test_refused_write_never_reaches_memory():
    bus = owned(MockBus.with_servos([1]))
    before = dict(bus.memory[1])
    with pytest.raises(BusError):
        bus.write("safearm", 1, "Torque_Enable", 128)
    assert bus.memory[1] == before and bus.write_log == []


def test_write_lands_in_memory_and_log():
    bus = owned(MockBus.with_servos([1, 2]))
    bus.write("safearm", 2, "Goal_Position", 1500)
    bus.write("safearm", 2, "Goal_Speed", 200)
    bus.write("safearm", 2, "Torque_Enable", 1)
    assert read(bus, 2, "Goal_Position") == 1500
    assert read(bus, 2, "Goal_Speed") == 200
    assert read(bus, 2, "Torque_Enable") == 1
    assert [(w.servo_id, w.name, w.value) for w in bus.write_log] == [
        (2, "Goal_Position", 1500), (2, "Goal_Speed", 200), (2, "Torque_Enable", 1),
    ]  # fmt: skip
    assert bus.write_log[0].t_ns <= bus.write_log[-1].t_ns


def test_write_to_missing_servo_raises():
    bus = owned(MockBus.with_servos([1]))
    with pytest.raises(BusError, match="no status packet"):
        bus.write("safearm", 6, "Goal_Position", 2000)


def test_mock_from_factory_dump_has_the_real_power_up_state():
    bus = MockBus.from_dump(FACTORY)
    assert sorted(bus.memory) == [1, 2, 3, 4, 5, 6]
    for servo_id in range(1, 7):
        # The lunge setup the torque-enable sequence must defuse (notes 2026-10-26).
        assert read(bus, servo_id, "Goal_Position") == 0
        assert read(bus, servo_id, "Goal_Speed") == 0
        assert read(bus, servo_id, "Acceleration") == 0
        assert read(bus, servo_id, "Torque_Enable") == 0
        assert read(bus, servo_id, "Lock") == 1
    assert read(bus, 3, "Present_Position") == 2927  # elbow at its stop, outside the soft limit
    assert read(bus, 1, "Offset") == -1835  # factory homing offset, decoded


def test_torque_limit_is_writable_within_1_to_1000():
    check_write("Torque_Limit", 1)
    check_write("Torque_Limit", 1000)
    for bad in (0, 1001):
        with pytest.raises(BusError, match="refused"):
            check_write("Torque_Limit", bad)
