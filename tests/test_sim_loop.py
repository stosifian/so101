import dataclasses
import json
import threading
import time
from pathlib import Path

import pytest

from so101.config import load_arm_config
from so101.hw.bus import read
from so101.hw.sim import SimBus
from so101.motion.loop import Segment, control_loop, sequence
from so101.safety import SafeArm, SafetyConfig, SafetyTrip, State

FACTORY = json.loads((Path(__file__).parent / "fixtures" / "follower_factory_dump.json").read_text())
CFG = load_arm_config("follower")
IDS = CFG.joints
FAST = dataclasses.replace(SafetyConfig.from_arm_config(CFG), debounce_s=0.05, load_trip_s=0.05)


def test_sim_does_not_move_with_torque_off():
    b = SimBus.from_dump(FACTORY)
    b.claim("t")
    b.write("t", 1, "Goal_Position", 2500)
    b.write("t", 1, "Goal_Speed", 1000)
    time.sleep(0.05)
    assert read(b, 1, "Present_Position") == 2176


def test_sim_moves_toward_goal_at_goal_speed_with_torque_on():
    b = SimBus.from_dump(FACTORY)
    b.claim("t")
    for name, v in [("Goal_Position", 2576), ("Goal_Speed", 1000), ("Torque_Enable", 1)]:
        b.write("t", 1, name, v)
    time.sleep(0.2)
    pos = read(b, 1, "Present_Position")
    assert 2176 + 120 < pos < 2176 + 320  # ~200 ticks at 1000 ticks/s, loose for CI timing
    assert read(b, 1, "Moving") == 1
    time.sleep(0.4)
    assert read(b, 1, "Present_Position") == 2576 and read(b, 1, "Moving") == 0


def test_sim_glitches_only_ids_4_to_6_at_about_the_rate():
    from so101.hw.arm import BLOCK_ADDR, BLOCK_SIZE, decode_block

    b = SimBus.from_dump(FACTORY, glitch_rate=0.05, seed=1)
    bad = {sid: 0 for sid in IDS.values()}
    for _ in range(2000):
        for sid in IDS.values():
            s = decode_block(b.read_block(sid, BLOCK_ADDR, BLOCK_SIZE))
            bad[sid] += s.voltage_dv < 118 or s.temperature_c > 35
    assert bad[1] == bad[2] == bad[3] == 0
    assert all(40 < bad[sid] < 170 for sid in (4, 5, 6))  # ~100 expected


def test_segment_and_sequence_ramp_linearly():
    seg = Segment({"a": 0.0}, {"a": 100.0}, speed=50.0)
    assert seg.duration_s == 2.0
    assert seg.at(1.0) == {"a": 50.0} and seg.at(5.0) == {"a": 100.0}
    fn, total = sequence([seg, Segment({"a": 100.0}, {"a": 0.0}, 100.0)], settle_s=0.5)
    assert total == 2.0 + 0.5 + 1.0 + 0.5
    assert fn(2.2) == {"a": 100.0} and fn(3.0) == {"a": 50.0}


def test_full_run_reaches_target_and_parks_with_torque_off():
    b = SimBus.from_dump(FACTORY, glitch_rate=0.01)
    start = {"shoulder_pan": 2176.0}
    fn, total = sequence([Segment(start, {"shoulder_pan": 2376.0}, 150.0),
                          Segment({"shoulder_pan": 2376.0}, start, 150.0)], settle_s=0.2)  # fmt: skip
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        mid = {}
        stats = control_loop(arm, 100, fn, total, on_tick=lambda t: mid.setdefault(round(t, 1), None))
        assert arm.state is State.HOLDING
        assert abs(read(b, IDS["shoulder_pan"], "Present_Position") - 2176) <= 5
    assert stats.cycles > 0.8 * total * 100
    assert all(read(b, sid, "Torque_Enable") == 0 for sid in IDS.values())
    assert arm.clamps.get(("shoulder_pan", "max_step"), 0) == 0  # a 150 ticks/s ramp never needs clamping


@pytest.mark.parametrize(
    "fault, reason",
    [
        (lambda b: b.inject(5, "Present_Temperature", 60), "temperature"),
        (lambda b: b.inject(5, "Present_Voltage", 100), "voltage"),
        (lambda b: b.inject(5, "Present_Load", -700), "load"),
        (lambda b: b.inject(5, "Status", 0x20), "status"),
        (lambda b: b.dead.add(5), "comm"),
        (lambda b: b.inject(5, "Torque_Enable", 0), "torque_off"),
    ],
)
def test_injected_fault_mid_motion_trips_and_stops_everything(fault, reason):
    b = SimBus.from_dump(FACTORY)
    fn, total = sequence([Segment({"wrist_roll": 1197.0}, {"wrist_roll": 1797.0}, 150.0)])
    injected = threading.Event()

    def on_tick(t):
        if t > 0.3 and not injected.is_set():
            injected.set()
            fault(b)

    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        with pytest.raises(SafetyTrip) as e:
            control_loop(arm, 100, fn, total, on_tick=on_tick)
        assert e.value.trip.reason == reason
        assert arm.state is State.TRIPPED
    b.clear()
    others = [sid for sid in IDS.values() if sid != 5]
    assert all(read(b, sid, "Torque_Enable") == 0 for sid in others)
    pos = read(b, 5, "Present_Position")
    time.sleep(0.1)
    assert read(b, 5, "Present_Position") == pos  # no motion after the trip
