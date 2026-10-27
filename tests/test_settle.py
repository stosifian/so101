import dataclasses
import json
from pathlib import Path

import pytest

from so101.config import load_arm_config
from so101.hw.bus import MockBus, read
from so101.hw.sim import SimBus
from so101.motion.loop import Segment
from so101.motion.settle import approach, wait_until_settled
from so101.safety import SafeArm, SafetyConfig

FACTORY = json.loads((Path(__file__).parent / "fixtures" / "follower_factory_dump.json").read_text())
CFG = load_arm_config("follower")
IDS = CFG.joints


def test_minjerk_segment_is_smooth_and_peak_speed_bounded():
    seg = Segment({"a": 0.0}, {"a": 100.0}, speed=100.0, easing="minjerk")
    assert seg.duration_s == pytest.approx(1.875)
    assert seg.at(0)["a"] == 0 and seg.at(seg.duration_s)["a"] == pytest.approx(100)
    assert seg.at(seg.duration_s / 2)["a"] == pytest.approx(50)
    dt = 0.001
    v = [(seg.at(t + dt)["a"] - seg.at(t)["a"]) / dt for t in [i * 0.01 for i in range(188)]]
    assert max(v) <= 100.5 and v[0] < 1 and v[-1] < 1  # peak <= speed; starts and ends at rest


def test_settles_on_sim_after_move_and_waits_the_dwell():
    b = SimBus.from_dump(FACTORY)
    with SafeArm(b, CFG) as arm:
        arm.enable()
        arm.step({"shoulder_pan": 2196})  # 20 ticks away; sim moves at 200 t/s
        r = wait_until_settled(arm, ["shoulder_pan"], dwell_s=0.2)
        assert r.settled and r.errors["shoulder_pan"] == 0 and r.positions["shoulder_pan"] == 2196
        assert r.t_total_s >= r.t_encoder_s + 0.2 - 0.02


class Creeping(MockBus):
    """Pan keeps creeping 1 tick per read: never steady."""

    def read_block(self, servo_id, addr, size):
        if servo_id == 1:
            self.memory[1][56] += 1
        return super().read_block(servo_id, addr, size)


def test_times_out_when_joint_never_steadies():
    b = Creeping.from_dump(FACTORY)
    with SafeArm(b, CFG) as arm:
        arm.enable()
        r = wait_until_settled(arm, ["shoulder_pan", "gripper"], dwell_s=0.1, timeout_s=0.5)
        assert not r.settled and r.not_settled == ["shoulder_pan"]


def test_moving_flag_blocks_settle():
    b = MockBus.from_dump(FACTORY)
    with SafeArm(b, CFG) as arm:
        arm.enable()
        b.memory[IDS["gripper"]][66] = 1  # Moving stuck at 1
        r = wait_until_settled(arm, ["gripper"], dwell_s=0.0, timeout_s=0.4)
        assert not r.settled


def test_steady_but_short_of_goal_counts_as_settled_unless_tol_given():
    b = MockBus.from_dump(FACTORY)  # mock never moves: goal 2196, position stays 2176
    with SafeArm(b, CFG) as arm:
        arm.enable()
        arm.step({"shoulder_pan": 2196})
        r = wait_until_settled(arm, ["shoulder_pan"], dwell_s=0.0, timeout_s=0.6)
        assert r.settled and r.errors["shoulder_pan"] == -20  # stopped; error is a measurement
        r2 = wait_until_settled(arm, ["shoulder_pan"], tol=3, dwell_s=0.0, timeout_s=0.4)
        assert not r2.settled


@pytest.mark.parametrize("direction", [1, -1])
def test_approach_arrives_from_the_requested_side(direction):
    b = SimBus.from_dump(FACTORY)
    with SafeArm(b, CFG) as arm:
        arm.enable()
        seen = []
        orig = arm.step

        def spy(goals=None):
            r = orig(goals)
            if goals:
                seen.append(goals["shoulder_pan"])
            return r

        arm.step = spy
        r = approach(arm, {"shoulder_pan": 2200}, {"shoulder_pan": direction}, overshoot=40, dwell_s=0.1)
        assert r.settled and r.positions["shoulder_pan"] == 2200
        tail = [g for g in seen[-20:]]
        assert all((b2 - a2) * direction >= 0 for a2, b2 in zip(tail, tail[1:]))  # final motion in direction
        if direction == -1:
            assert max(seen) >= 2200 + 39  # went past on the high side first
