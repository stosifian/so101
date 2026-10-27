import json
from pathlib import Path

import pytest

from so101.config import load_arm_config
from so101.hw.bus import BusError, MockBus, read
from so101.safety import SafeArm, SafetyConfig, SafetyError, State

FACTORY = json.loads((Path(__file__).parent / "fixtures" / "follower_factory_dump.json").read_text())
CFG = load_arm_config("follower")
IDS = CFG.joints


def factory_bus(cls=MockBus) -> MockBus:
    bus = cls.from_dump(FACTORY)
    return bus


def writes(bus, servo_id=None):
    return [(w.servo_id, w.name, w.value) for w in bus.write_log if servo_id in (None, w.servo_id)]


def torque(bus):
    return {j: read(bus, sid, "Torque_Enable") for j, sid in IDS.items()}


def test_config_parses_approved_values():
    s = SafetyConfig.from_arm_config(CFG)
    assert (s.speed_cap, s.accel_cap, s.max_step, s.temp_trip_c) == (200, 10, 100, 55.0)
    assert s.limits["elbow_flex"] == (765, 2878)


def test_enable_sequence_order_per_joint_from_factory_state():
    bus = factory_bus()
    with SafeArm(bus, CFG) as arm:
        report = arm.enable()
        assert arm.state is State.HOLDING
        for joint, sid in IDS.items():
            present = read(bus, sid, "Present_Position")
            seq = [w for w in writes(bus, sid) if w[1] != "Torque_Enable" or w[2] == 1][:4]
            assert seq == [
                (sid, "Goal_Position", present),  # goal := present, before anything else
                (sid, "Goal_Speed", 200),
                (sid, "Acceleration", 10),
                (sid, "Torque_Enable", 1),  # torque last
            ]
            assert report.present[joint] == present
        assert all(v == 1 for v in torque(bus).values())


def test_torque_never_enabled_while_goal_is_zero():
    bus = factory_bus()
    with SafeArm(bus, CFG) as arm:
        arm.enable()
    goal = {}
    for sid, name, value in writes(bus):
        if name == "Goal_Position":
            goal[sid] = value
        if name == "Torque_Enable" and value == 1:
            assert goal.get(sid, 0) != 0, f"id {sid} enabled with Goal_Position 0"


def test_rest_pose_outside_limits_is_held_and_reported():
    bus = factory_bus()  # factory elbow at 2927 > limit 2878
    with SafeArm(bus, CFG) as arm:
        report = arm.enable()
        assert "elbow_flex" in report.outside_limits
        assert read(bus, IDS["elbow_flex"], "Goal_Position") == 2927  # held where it is, no clamp-jump
        assert any(e.kind == "outside_limits" and e.joint == "elbow_flex" for e in arm.events)


def test_exit_turns_torque_off_and_releases_bus_even_on_error():
    bus = factory_bus()
    with pytest.raises(ZeroDivisionError):
        with SafeArm(bus, CFG) as arm:
            arm.enable()
            1 / 0
    assert all(v == 0 for v in torque(bus).values())
    assert arm.state is State.DISABLED
    assert bus.owner is None


def test_cannot_coexist_with_a_poller():
    bus = factory_bus()
    bus.claim("poller-follower")
    with pytest.raises(BusError, match="already owned"):
        with SafeArm(bus, CFG):
            pass


def test_enable_outside_context_manager_is_refused():
    with pytest.raises(SafetyError, match="context manager"):
        SafeArm(factory_bus(), CFG).enable()


def test_preflight_refuses_arm_already_under_torque_without_dropping_it():
    bus = factory_bus()
    bus.memory[IDS["wrist_flex"]][40] = 1  # Torque_Enable already on
    with pytest.raises(SafetyError, match="already on"):
        with SafeArm(bus, CFG):
            pass
    assert writes(bus) == []  # nothing written: no torque-off that would drop the arm
    assert bus.owner is None


def test_preflight_refuses_missing_servo():
    bus = factory_bus()
    del bus.memory[IDS["gripper"]]
    with pytest.raises(SafetyError, match="gripper"):
        with SafeArm(bus, CFG):
            pass


class StickyGoal(MockBus):
    """Servo 3 ignores Goal_Position writes (e.g. a corrupted packet acked as success)."""

    def write(self, owner, servo_id, name, value):
        if servo_id == 3 and name == "Goal_Position":
            self._require_owner(owner)
            return
        super().write(owner, servo_id, name, value)


def test_readback_mismatch_aborts_and_turns_everything_off():
    bus = factory_bus(StickyGoal)
    with SafeArm(bus, CFG) as arm:
        with pytest.raises(SafetyError, match="elbow_flex: Goal_Position read back 0"):
            arm.enable()
        assert arm.state is State.TRIPPED
        assert (3, "Torque_Enable", 1) not in writes(bus)  # never enabled the bad joint
        assert all(v == 0 for v in torque(bus).values())  # joints 1-2 were on, now off
        with pytest.raises(SafetyError, match="needs state DISABLED"):
            arm.enable()  # no re-enable without reset()
        arm.reset()
        assert arm.state is State.DISABLED


class Drifting(MockBus):
    """Joint 2 sags 30 ticks between the first and second position read (e.g. gravity)."""

    def __init__(self, memory=None):
        super().__init__(memory)
        self._reads = 0

    def read_raw(self, servo_id, reg):
        if servo_id == 2 and reg.name == "Present_Position":
            self._reads += 1
            if self._reads == 2:
                self.memory[2][reg.addr] += 30
        return super().read_raw(servo_id, reg)


def test_drift_during_enable_aborts():
    bus = factory_bus(Drifting)
    with SafeArm(bus, CFG) as arm:
        with pytest.raises(SafetyError, match="shoulder_lift: moved 30 ticks"):
            arm.enable()
        assert (2, "Torque_Enable", 1) not in writes(bus)
        assert all(v == 0 for v in torque(bus).values())


class TorqueWontEnable(MockBus):
    def write(self, owner, servo_id, name, value):
        super().write(owner, servo_id, name, value)
        if servo_id == 5 and name == "Torque_Enable":
            self.memory[5][40] = 0  # firmware refuses / silently clears


def test_torque_readback_failure_aborts():
    bus = factory_bus(TorqueWontEnable)
    with SafeArm(bus, CFG) as arm:
        with pytest.raises(SafetyError, match="wrist_roll: Torque_Enable read back 0"):
            arm.enable()
        assert all(v == 0 for v in torque(bus).values())


class DeadOnDisable(MockBus):
    """Servo 4 stops answering after torque-on, so disable can't reach it."""

    def write(self, owner, servo_id, name, value):
        if servo_id == 4 and name == "Torque_Enable" and value == 0 and self.memory[4][40] == 1:
            raise BusError("id 4: no status packet")
        super().write(owner, servo_id, name, value)


def test_disable_tries_every_joint_and_reports_failures():
    bus = factory_bus(DeadOnDisable)
    with SafeArm(bus, CFG) as arm:
        arm.enable()
        failed = arm.disable()
        assert failed == ["wrist_flex"]
        others = {j: v for j, v in torque(bus).items() if j != "wrist_flex"}
        assert all(v == 0 for v in others.values())  # the rest still went off
        assert any(e.kind == "disable_failed" for e in arm.events)


def test_torque_limit_is_set_read_back_and_restored_on_exit():
    bus = factory_bus()
    with SafeArm(bus, CFG) as arm:
        arm.enable()
        arm.set_torque_limit("gripper", 50)
        assert read(bus, IDS["gripper"], "Torque_Limit") == 500
        with pytest.raises(SafetyError, match="outside"):
            arm.set_torque_limit("gripper", 0)
    assert read(bus, IDS["gripper"], "Torque_Limit") == 1000  # restored even though torque is off
    assert any(e.kind == "torque_limit" and "restored" in e.detail for e in arm.events)
