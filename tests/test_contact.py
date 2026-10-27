import dataclasses
import json
import threading
import time
from pathlib import Path

import pyarrow.parquet as pq
import pytest

from so101.config import load_arm_config
from so101.hw.arm import ArmSnapshot, JointState
from so101.hw.bus import BusError, FeetechBus, MockBus, read
from so101.hw.sim import SimBus
from so101.motion.loop import Segment, control_loop, sequence
from so101.safety import SafeArm, SafetyConfig, State
from so101.safety.config import _threshold
from so101.safety.contact import ContactChecker

FIX = Path(__file__).parent / "fixtures"
FACTORY = json.loads((FIX / "follower_factory_dump.json").read_text())
CFG = load_arm_config("follower")
IDS = CFG.joints
SAFETY = SafetyConfig.from_arm_config(CFG)


# --- config ----------------------------------------------------------------------------------


def test_config_has_the_approved_thresholds():
    s = SAFETY
    assert (s.contact_err_moving, s.contact_err_holding, s.contact_drift_idle, s.contact_window_s) == (40, 25, 10, 0.3)
    assert s.disabled_contact_checks() == []


def test_null_disables_and_bad_values_are_rejected():
    assert _threshold({"x": None}, "x") is None
    assert _threshold({"x": 12}, "x") == 12
    for bad in (0, -5, "ten", True):
        with pytest.raises(ValueError, match="positive number"):
            _threshold({"x": bad}, "x")
    with pytest.raises(ValueError, match="missing"):
        _threshold({}, "x")  # a deleted line must not silently disable a check


# --- checker logic ---------------------------------------------------------------------------


def snap(t_s: float, **pos) -> ArmSnapshot:
    t = int(t_s * 1e9)
    return ArmSnapshot(t, t, {j: JointState(p, 0, 0, 119, 30, 0, 0, 0) for j, p in pos.items()})


def test_roles_and_window():
    c = ContactChecker(SAFETY, {"a": 1000, "b": 2000}, t_ns=0)
    assert c.role("a", int(1e9)) == "idle"
    c.update({"a": 1100}, int(1e9))
    assert c.role("a", int(1.1e9)) == "moving" and c.role("a", int(1.5e9)) == "holding"
    # idle joint 'b' pushed 15 ticks: needs 0.3 s continuously
    assert c.check(snap(2.0, a=1100, b=2015)) is None
    assert c.check(snap(2.2, a=1100, b=2015)) is None
    hit = c.check(snap(2.31, a=1100, b=2015))
    assert hit and hit.joint == "b" and hit.role == "idle" and hit.error == 15
    # a good sample resets the window
    c2 = ContactChecker(SAFETY, {"b": 2000}, t_ns=0)
    for t, p in [(1.0, 2015), (1.2, 2015), (1.25, 2003), (1.4, 2015), (1.6, 2015)]:
        assert c2.check(snap(t, b=p)) is None


def test_disabled_check_never_fires():
    c = ContactChecker(dataclasses.replace(SAFETY, contact_drift_idle=None), {"b": 2000}, t_ns=0)
    for t in (1.0, 1.5, 2.0):
        assert c.check(snap(t, b=2500)) is None


# --- replay of today's real runs -------------------------------------------------------------


def replay(run: str) -> tuple[list, str]:
    meta = json.loads((FIX / "contact_replay_step6.json").read_text())[run]
    rows = [r for r in pq.read_table(FIX / "contact_replay_step6.parquet").to_pylist()
            if r["run"] == run and meta["t_on_ns"] <= r["t_start_ns"] < meta["t_off_ns"] and r["position"] is not None]  # fmt: skip
    by_seq: dict[int, dict] = {}
    for r in rows:
        by_seq.setdefault(r["seq"], {})[r["joint"]] = r
    seqs = sorted(by_seq)
    first = by_seq[seqs[0]]
    c = ContactChecker(SAFETY, {j: r["position"] for j, r in first.items()}, first[next(iter(first))]["t_start_ns"])
    hits = []
    for s in seqs:
        js = by_seq[s]
        t = next(iter(js.values()))["t_start_ns"]
        hit = c.check(ArmSnapshot(t, t, {j: JointState(r["position"], 0, 0, 119, 30, 0, 0, 0) for j, r in js.items()}))
        if hit:
            hits.append(hit)
            break  # SafeArm would freeze here
        c.update({j: r["goal"] for j, r in js.items() if r["goal"] is not None}, t)
    return hits, meta["kind"]


RUNS = list(json.loads((FIX / "contact_replay_step6.json").read_text()))


@pytest.mark.parametrize("run", RUNS)
def test_replay_today(run):
    hits, kind = replay(run)
    if kind == "normal":
        assert hits == [], f"false contact on a normal run: {hits}"
    elif kind == "table_contact":
        assert hits and hits[0].joint in ("shoulder_lift", "elbow_flex"), hits
    elif kind == "gentle_contact":
        assert hits == []  # documented limitation: gentle contact is not detectable; prevent it by geometry


def test_table_contact_is_caught_early():
    hits, _ = replay("20261026-162655_sweep_follower")
    meta = json.loads((FIX / "contact_replay_step6.json").read_text())["20261026-162655_sweep_follower"]
    t = (hits[0].t_ns - meta["t_on_ns"]) / 1e9
    assert t < 5.5, f"caught at {t:.1f} s into the sweep (contact began ~4.3 s, peak 39 % load ~7 s)"


# --- SafeArm end to end on the simulator -----------------------------------------------------


def test_safearm_freezes_on_contact_and_keeps_torque_on():
    b = SimBus.from_dump(FACTORY)
    fn, total = sequence([Segment({"shoulder_pan": 2176.0}, {"shoulder_pan": 2476.0}, 150.0)])
    stop = threading.Event()

    def obstacle(t):
        if t > 0.5:
            b.inject(IDS["shoulder_pan"], "Present_Position", 2240)  # blocked: position stops at 2240

    with SafeArm(b, CFG) as arm:
        arm.enable()
        threading.Timer(total + 0.5, stop.set).start()
        stats = control_loop(arm, 100, fn, total, stop, on_tick=obstacle)
        assert arm.frozen and stats.frozen and arm.state is State.HOLDING
        assert arm.contact_hit.joint == "shoulder_pan" and arm.contact_hit.role == "moving"
        assert all(read(b, sid, "Torque_Enable") == 1 for sid in IDS.values())
        assert read(b, IDS["shoulder_pan"], "Goal_Position") == 2240  # goal := where it was blocked
        assert any(e.kind == "contact" for e in arm.events)


def test_disabled_checks_are_logged_as_events():
    s = dataclasses.replace(SAFETY, contact_drift_idle=None)
    with SafeArm(MockBus.from_dump(FACTORY), CFG, s) as arm:
        pass
    assert [e.detail.split(" ")[0] for e in arm.events if e.kind == "check_disabled"] == ["contact_drift_idle"]


def test_shadow_mode_does_no_contact_checks():
    b = MockBus.from_dump(FACTORY)
    with SafeArm(b, CFG, shadow=True) as arm:
        arm.step({"shoulder_pan": 2300})
        assert arm.contact is None and not arm.frozen


# --- bus hardening ---------------------------------------------------------------------------


class FakePkt:
    def __init__(self, replies):
        self.replies = list(replies)

    def readTxRx(self, port, sid, addr, size):
        return self.replies.pop(0)

    def getTxRxResult(self, comm):
        return f"comm {comm}"


def fake_feetech(replies) -> FeetechBus:
    bus = object.__new__(FeetechBus)  # no serial port
    from so101.hw.bus import BusOwnership, BusStats

    BusOwnership.__init__(bus)
    bus._ok, bus.retries, bus.stats, bus.last_error, bus._port = 0, 2, BusStats(), {}, None
    bus._pkt = FakePkt(replies)
    return bus


def test_short_reply_counts_as_failure_and_is_retried():
    good = list(range(15))
    bus = fake_feetech([(good[:9], 0, 0), (good, 0, 0x04)])
    assert bus.read_block(5, 56, 15) == bytes(good)
    assert bus.stats.failures[(5, "short reply (9/15 bytes)")] == 1
    assert bus.last_error[5] == 0x04  # error byte of the successful read is kept


def test_three_short_replies_raise_bus_error_not_value_error():
    bus = fake_feetech([([1, 2], 0, 0)] * 3)
    with pytest.raises(BusError):
        bus.read_block(5, 56, 15)


def test_error_byte_reaches_the_snapshot():
    from so101.hw.arm import Arm

    b = MockBus.from_dump(FACTORY)
    b.last_error[6] = 0x04
    s = Arm(b, IDS).snapshot()
    assert s.joints["gripper"].error_byte == 0x04 and s.joints["shoulder_pan"].error_byte == 0


def test_exempt_joint_is_not_checked():
    c = ContactChecker(SAFETY, {"gripper": 2172}, t_ns=0)
    c.exempt = {"gripper"}
    for t in (1.0, 1.5, 2.0):
        assert c.check(snap(t, gripper=2300)) is None


def test_gripper_squeeze_does_not_freeze_when_exempt():
    b = MockBus.from_dump(FACTORY)
    with SafeArm(b, CFG) as arm:
        arm.set_contact_exempt("gripper")
        arm.enable()
        arm.step({"gripper": 2126 + 100})  # mock never moves: gripper stays 100 ticks short, like a squeeze
        t0 = time.monotonic()
        while time.monotonic() - t0 < 0.6:
            arm.step()
        assert not arm.frozen
        assert any(e.kind == "contact_exempt" for e in arm.events)


def test_load_trip_override_for_the_gripper():
    from so101.safety import TripChecker

    assert SAFETY.load_trip_overrides == {"gripper": 80.0}
    tc = TripChecker(SAFETY)
    st = lambda load: {"gripper": JointState(2200, 0, load, 119, 30, 0, 0, 0)}
    for t in (0.0, 0.3, 0.6):  # 60 % on the gripper for 0.6 s: below its 80 % override
        assert tc.feed(ArmSnapshot(int(t * 1e9), 0, st(600))) is None
    tc2 = TripChecker(SAFETY)
    trip = None
    for t in (0.0, 0.3, 0.6):
        trip = trip or tc2.feed(ArmSnapshot(int(t * 1e9), 0, st(850)))
    assert trip and trip.reason == "load" and "80" in trip.detail
