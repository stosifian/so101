import dataclasses
import json
import time
from pathlib import Path

import pyarrow.parquet as pq
import pytest

from so101.config import load_arm_config
from so101.hw.arm import ArmSnapshot, JointState
from so101.hw.bus import MockBus, read
from so101.hw.registers import REGISTERS, encode
from so101.runlog.run import Run
from so101.safety import SafeArm, SafetyConfig, SafetyError, SafetyTrip, State, TripChecker
from so101.telemetry.hub import Hub
from so101.telemetry.parquet_log import ParquetLogger

FIX = Path(__file__).parent / "fixtures"
FACTORY = json.loads((FIX / "follower_factory_dump.json").read_text())
CFG = load_arm_config("follower")
IDS = CFG.joints
# Short windows so trip tests run fast; same logic as the approved 0.5 s.
FAST = dataclasses.replace(SafetyConfig.from_arm_config(CFG), debounce_s=0.05, load_trip_s=0.05, torque_check_s=0.0)
ADDR = {n: REGISTERS[n].addr for n in ["Present_Position", "Present_Temperature", "Present_Voltage",
                                       "Present_Load", "Status", "Torque_Enable"]}  # fmt: skip


def bus() -> MockBus:
    return MockBus.from_dump(FACTORY)


def run_steps(arm: SafeArm, seconds: float, goals=None) -> int:
    n, t_end = 0, time.monotonic() + seconds
    while time.monotonic() < t_end:
        arm.step(goals)
        n += 1
        time.sleep(0.002)
    return n


# --- goal clamps ---------------------------------------------------------------------------


def test_goal_is_clamped_to_max_step_and_written():
    b = bus()  # shoulder_pan present 2176, limits [969, 3404]
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        r = arm.step({"shoulder_pan": 2176 + 500})
        assert r.commanded == {"shoulder_pan": 2276}
        assert read(b, IDS["shoulder_pan"], "Goal_Position") == 2276
        assert arm.clamps[("shoulder_pan", "max_step")] == 1


def test_goal_inside_limits_clamped_to_limit():
    b = bus()
    b.memory[IDS["shoulder_pan"]][ADDR["Present_Position"]] = 3380  # near max 3404
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        assert arm.step({"shoulder_pan": 3450}).commanded == {"shoulder_pan": 3404}
        assert arm.clamps[("shoulder_pan", "limit")] == 1


def test_outside_limits_only_moves_back_toward_range():
    b = bus()  # elbow present 2927, limit max 2878
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        assert arm.step({"elbow_flex": 3000}).commanded == {"elbow_flex": 2927}  # further out -> hold
        assert arm.step({"elbow_flex": 2850}).commanded == {"elbow_flex": 2850}  # inward is allowed
        assert arm.step({"elbow_flex": 2000}).commanded == {"elbow_flex": 2827}  # still max-step limited


def test_untouched_joints_keep_their_goal():
    b = bus()
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        before = len(b.write_log)
        arm.step({"gripper": 2200})
        assert [(w.servo_id, w.name) for w in b.write_log[before:]] == [(IDS["gripper"], "Goal_Position")]


def test_goals_refused_unless_holding():
    b = bus()
    with SafeArm(b, CFG, FAST) as arm:
        with pytest.raises(SafetyError, match="enable\\(\\) first"):
            arm.step({"gripper": 2200})
        arm.step()  # monitoring without goals is fine while disabled
        arm.enable()
        with pytest.raises(SafetyError, match="unknown joint"):
            arm.step({"elbo": 2000})


# --- trips -----------------------------------------------------------------------------------


def assert_tripped(b: MockBus, arm: SafeArm, reason: str, joint: str | None):
    assert arm.state is State.TRIPPED
    assert all(read(b, sid, "Torque_Enable") == 0 for sid in IDS.values())
    trip_events = [e for e in arm.events if e.kind == "trip"]
    assert trip_events and trip_events[-1].joint == joint and reason in trip_events[-1].detail or reason in str(
        trip_events[-1]
    )


@pytest.mark.parametrize(
    "register, value, reason",
    [
        ("Present_Temperature", 60, "temperature"),
        ("Present_Voltage", 100, "voltage"),  # 10.0 V
        ("Present_Load", encode(REGISTERS["Present_Load"], -700), "load"),  # -70 %
    ],
)
def test_sustained_out_of_range_trips_and_turns_torque_off(register, value, reason):
    b = bus()
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        b.memory[IDS["wrist_roll"]][ADDR[register]] = value
        with pytest.raises(SafetyTrip) as e:
            run_steps(arm, 1.0)
        assert e.value.trip.reason == reason and e.value.trip.joint == "wrist_roll"
        assert_tripped(b, arm, reason, "wrist_roll")


def test_single_sample_glitch_does_not_trip():
    b = bus()
    sid = IDS["gripper"]
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        for i in range(60):
            glitch = i % 10 == 5
            b.memory[sid][ADDR["Present_Temperature"]] = 63 if glitch else 31
            b.memory[sid][ADDR["Present_Voltage"]] = 104 if glitch else 119
            arm.step()
            time.sleep(0.003)  # 10 samples apart = longer than the 0.05 s window
        assert arm.state is State.HOLDING


def test_overload_status_trips_immediately():
    b = bus()
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        b.memory[IDS["shoulder_lift"]][ADDR["Status"]] = 0x20  # overload
        with pytest.raises(SafetyTrip, match="status"):
            arm.step()
        assert_tripped(b, arm, "status", "shoulder_lift")


def test_voltage_status_bit_is_debounced():
    b = bus()
    sid = IDS["gripper"]
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        b.memory[sid][ADDR["Status"]] = 0x01  # single-sample voltage bit, like the real leader gripper
        arm.step()
        b.memory[sid][ADDR["Status"]] = 0
        run_steps(arm, 0.1)
        assert arm.state is State.HOLDING
        b.memory[sid][ADDR["Status"]] = 0x01  # sustained -> trips
        with pytest.raises(SafetyTrip, match="voltage/overheat bit"):
            run_steps(arm, 1.0)


def test_torque_silently_cleared_trips():
    b = bus()
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        arm.step()
        b.memory[IDS["elbow_flex"]][ADDR["Torque_Enable"]] = 0
        with pytest.raises(SafetyTrip, match="torque_off on elbow_flex"):
            arm.step()


class Flaky(MockBus):
    """Gripper stops answering block reads for `bad` snapshots, then recovers."""

    bad = 0

    def read_block(self, servo_id, addr, size):
        if servo_id == 6 and self.bad > 0:
            self.bad -= 1
            return super().read_block(99, addr, size)  # missing id -> BusError
        return super().read_block(servo_id, addr, size)


def test_comm_failures_trip_after_n_consecutive():
    b = Flaky.from_dump(FACTORY)
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        b.bad = 4  # comm_fail_trip = 5: four in a row is tolerated
        for _ in range(6):
            arm.step({"gripper": 2200})
        assert arm.state is State.HOLDING
        assert arm.clamps[("gripper", "no_present")] == 4  # no goal written without a fresh position
        b.bad = 5
        with pytest.raises(SafetyTrip, match="comm"):
            for _ in range(6):
                arm.step()


def test_no_reenable_after_trip_without_reset():
    b = bus()
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        b.memory[IDS["shoulder_lift"]][ADDR["Status"]] = 0x20
        with pytest.raises(SafetyTrip):
            arm.step()
        with pytest.raises(SafetyError):
            arm.enable()
        with pytest.raises(SafetyError):
            arm.step({"gripper": 2200})


# --- shadow mode (step 5a) -------------------------------------------------------------------


def test_shadow_mode_never_writes_anything():
    b = bus()
    with SafeArm(b, CFG, FAST, shadow=True) as arm:
        with pytest.raises(SafetyError, match="shadow"):
            arm.enable()
        r = arm.step({"shoulder_pan": 2176 + 500, "elbow_flex": 3000})
        assert r.commanded == {"shoulder_pan": 2276, "elbow_flex": 2927}  # validated, logged, not written
        b.memory[IDS["wrist_roll"]][ADDR["Present_Temperature"]] = 60
        with pytest.raises(SafetyTrip, match="temperature"):
            run_steps(arm, 1.0)
    assert b.write_log == []  # including no torque-off on exit
    assert b.owner is None


# --- publishing + logging ------------------------------------------------------------------


def test_samples_carry_commanded_goals_into_parquet(tmp_path):
    b, hub = bus(), Hub()
    run = Run("test", "follower", root=tmp_path)
    logger = ParquetLogger(hub.subscribe("parquet"), run, flush_s=0.05).start()
    with SafeArm(b, CFG, FAST, hub=hub) as arm:
        arm.enable()
        arm.step()
        arm.step({"shoulder_pan": 2300})
    logger.stop()
    rows = pq.read_table(run.dir / "telemetry").to_pylist()
    pan = [r for r in rows if r["joint"] == "shoulder_pan"]
    assert [r["goal"] for r in pan] == [None, 2276]
    assert [r["seq"] for r in pan] == [0, 1]


def test_report_is_json_serialisable():
    with SafeArm(bus(), CFG, FAST) as arm:
        arm.enable()
        arm.step({"shoulder_pan": 9999})
    rep = json.loads(json.dumps(arm.report()))
    assert rep["clamps"] == {"shoulder_pan:max_step": 1}
    assert [e["kind"] for e in rep["events"]][:1] == ["outside_limits"]


# --- replay real telemetry -----------------------------------------------------------------


def replay(arm_name: str, safety: SafetyConfig):
    rows = pq.read_table(FIX / "telemetry_20261026-121235_both.parquet").to_pylist()
    by_seq: dict[int, dict] = {}
    for r in rows:
        if r["arm"] == arm_name:
            by_seq.setdefault(r["seq"], {})[r["joint"]] = r
    checker, n = TripChecker(safety), 0
    for seq in sorted(by_seq):
        joints = by_seq[seq]
        t = next(iter(joints.values()))["t_start_ns"]
        states = {j: JointState(r["position"], 0, r["load"], r["voltage_dv"], r["temperature_c"], r["status"], 0, 0)
                  for j, r in joints.items()}  # fmt: skip
        trip = checker.feed(ArmSnapshot(t, t, states))
        assert trip is None, f"false trip on real data at seq {seq}: {trip}"
        n += 1
    return n


def test_real_follower_telemetry_with_adc_glitches_never_trips():
    s = SafetyConfig.from_arm_config(CFG)  # the approved values, 0.5 s windows
    rows = [r for r in pq.read_table(FIX / "telemetry_20261026-121235_both.parquet").to_pylist()
            if r["arm"] == "follower"]  # fmt: skip
    assert min(r["voltage_dv"] for r in rows) < 110  # the fixture really contains below-trip glitches
    assert replay("follower", s) == 3001


def test_real_leader_telemetry_never_trips_with_leader_voltage_range():
    s = dataclasses.replace(SafetyConfig.from_arm_config(CFG), voltage_v=(4.5, 5.5))
    assert replay("leader", s) == 3001
