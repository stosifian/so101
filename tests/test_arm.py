import pytest

from so101.hw.arm import BLOCK_ADDR, BLOCK_SIZE, Arm, decode_block
from so101.hw.bus import MockBus
from so101.hw.registers import REGISTERS, encode

JOINTS = {"shoulder_pan": 1, "shoulder_lift": 2, "gripper": 6}


def test_block_spans_position_to_current():
    assert (BLOCK_ADDR, BLOCK_SIZE) == (56, 15)


def test_snapshot_decodes_all_fields():
    bus = MockBus.with_servos([1, 2, 6], voltage_dv=119, position=1500)
    mem = bus.memory[2]
    mem[REGISTERS["Present_Load"].addr] = encode(REGISTERS["Present_Load"], -100)
    mem[REGISTERS["Present_Speed"].addr] = encode(REGISTERS["Present_Speed"], -250)
    mem[REGISTERS["Present_Current"].addr] = 12
    mem[REGISTERS["Moving"].addr] = 1

    snap = Arm(bus, JOINTS).snapshot()

    assert snap.ok
    lift = snap.joints["shoulder_lift"]
    assert (lift.position, lift.speed, lift.load, lift.moving) == (1500, -250, -100, 1)
    assert lift.voltage_v == pytest.approx(11.9)
    assert lift.current_ma == pytest.approx(78.0)
    assert lift.temperature_c == 25
    assert snap.t_end_ns >= snap.t_start_ns
    assert bus.stats.transactions == 3  # one block read per servo


def test_block_read_matches_per_register_reads():
    bus = MockBus.with_servos([1], position=3000)
    bus.memory[1][REGISTERS["Present_Load"].addr] = encode(REGISTERS["Present_Load"], 321)
    state = decode_block(bus.read_block(1, BLOCK_ADDR, BLOCK_SIZE))
    assert state.position == bus.read_raw(1, REGISTERS["Present_Position"])
    assert state.load == 321


def test_missing_servo_is_none_not_crash():
    bus = MockBus.with_servos([1, 2])  # gripper (6) absent
    snap = Arm(bus, JOINTS).snapshot()
    assert not snap.ok
    assert snap.joints["gripper"] is None
    assert "gripper" in snap.errors
    assert snap.joints["shoulder_pan"] is not None


def test_decode_block_rejects_wrong_length():
    with pytest.raises(ValueError):
        decode_block(bytes(BLOCK_SIZE - 1))
