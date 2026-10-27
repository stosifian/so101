import _thread
import dataclasses
import json
import threading
import time
from pathlib import Path

import pytest
from rich.console import Console

from so101.config import load_arm_config
from so101.hw.bus import MockBus, read
from so101.hw.sim import SimBus
from so101.motion.loop import Segment, control_loop, sequence
from so101.runlog.run import Run
from so101.safety import SafeArm, SafetyConfig, SafetyError, SafetyTrip, State
from so101.telemetry.live import LiveSession

FACTORY = json.loads((Path(__file__).parent / "fixtures" / "follower_factory_dump.json").read_text())
CFG = load_arm_config("follower")
IDS = CFG.joints
FAST = dataclasses.replace(SafetyConfig.from_arm_config(CFG), debounce_s=0.05)


def test_freeze_sets_goal_to_present_keeps_torque_and_refuses_goals():
    b = MockBus.from_dump(FACTORY)
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        arm.step({"shoulder_pan": 2250})  # goal moved away from present (mock doesn't move)
        b.memory[IDS["shoulder_pan"]][56] = 2190  # joint is now somewhere else
        r = arm.freeze()
        assert r.commanded["shoulder_pan"] == 2190 == read(b, IDS["shoulder_pan"], "Goal_Position")
        assert r.commanded["elbow_flex"] == 2927  # outside its limit: held, not clamped
        assert arm.frozen and arm.state is State.HOLDING
        assert all(read(b, sid, "Torque_Enable") == 1 for sid in IDS.values())
        with pytest.raises(SafetyError, match="frozen"):
            arm.step({"shoulder_pan": 2300})
        arm.step()  # monitoring still works
        assert any(e.kind == "freeze" for e in arm.events)


def test_loop_freeze_stops_motion_mid_ramp_and_holds_past_duration():
    b = SimBus.from_dump(FACTORY)
    fn, total = sequence([Segment({"shoulder_pan": 2176.0}, {"shoulder_pan": 2476.0}, 150.0)])  # 2 s ramp
    freeze, stop = threading.Event(), threading.Event()
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        threading.Timer(0.8, freeze.set).start()
        threading.Timer(total + 1.0, stop.set).start()  # well past the planned duration
        t0 = time.monotonic()
        stats = control_loop(arm, 100, fn, total, stop, freeze=freeze)
        held_for = time.monotonic() - t0
        pos = read(b, IDS["shoulder_pan"], "Present_Position")
        assert stats.frozen and held_for > total + 0.8  # ignored duration while frozen
        assert 2176 + 60 < pos < 2176 + 180  # stopped about where it was at 0.8 s (~120 ticks), not 2476
        assert read(b, IDS["shoulder_pan"], "Torque_Enable") == 1
    assert read(b, IDS["shoulder_pan"], "Torque_Enable") == 0  # exit still turns torque off


def test_trips_stay_active_while_frozen():
    b = SimBus.from_dump(FACTORY)
    freeze = threading.Event()
    freeze.set()
    with SafeArm(b, CFG, FAST) as arm:
        arm.enable()
        threading.Timer(0.2, lambda: b.inject(5, "Present_Temperature", 60)).start()
        with pytest.raises(SafetyTrip, match="temperature"):
            control_loop(arm, 100, lambda t: None, 0.1, freeze=freeze)
        assert arm.state is State.TRIPPED


def test_live_session_first_ctrl_c_freezes_second_stops(tmp_path):
    run = Run("test", "follower", root=tmp_path)
    live = LiveSession(run, {"follower": (11.0, 13.0)}, tui=False, viz=False)
    seen = {}

    def worker():
        live.freeze.wait(5)
        seen["frozen_before_stop"] = live.freeze.is_set() and not live.stop.is_set()
        live.stop.wait(5)

    th = threading.Thread(target=worker)
    th.start()
    threading.Timer(0.3, _thread.interrupt_main).start()
    threading.Timer(0.8, _thread.interrupt_main).start()
    assert live.run_until_done(th, Console(file=open("/dev/null", "w"))) is True
    live.close()
    assert seen["frozen_before_stop"] and live.stop.is_set()


def test_after_a_contact_freeze_first_ctrl_c_turns_torque_off(tmp_path):
    run = Run("test", "follower", root=tmp_path)
    live = LiveSession(run, {"follower": (11.0, 13.0)}, tui=False, viz=False)
    live.status_fn = lambda: "FROZEN by contact detection (test)"
    live.ctrl_c_stops_fn = lambda: True  # the arm is already holding

    def worker():
        live.stop.wait(5)

    th = threading.Thread(target=worker)
    th.start()
    threading.Timer(0.4, _thread.interrupt_main).start()  # a single Ctrl-C
    assert live.run_until_done(th, Console(file=open("/dev/null", "w"))) is True
    live.close()
    assert live.stop.is_set()


def test_informational_status_does_not_freeze(tmp_path):
    """Regression (step 10b): a status line like 'PLACE the object now' must not set the freeze flag."""
    run = Run("test", "follower", root=tmp_path)
    live = LiveSession(run, {"follower": (11.0, 13.0)}, tui=False, viz=False)
    live.status_fn = lambda: "PLACE a soft object between the jaws now"
    th = threading.Thread(target=lambda: time.sleep(0.5))
    th.start()
    live.run_until_done(th, Console(file=open("/dev/null", "w")))
    live.close()
    assert not live.freeze.is_set() and not live.stop.is_set()
