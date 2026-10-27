import json
import math
import threading
from pathlib import Path

import numpy as np
import pytest

from so101.config import load_arm_config
from so101.hw.sim import SimBus
from so101.kinematics.collision import CollisionModel
from so101.kinematics.fk import CHAIN_JOINTS, chain_from_mujoco, fk
from so101.kinematics.ik import limits_rad
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel
from so101.motion.cartesian import line_point_distance, plan_line
from so101.motion.loop import control_loop
from so101.safety import SafeArm, SafetyConfig, State

CFG = load_arm_config("follower")
SAFETY = SafetyConfig.from_arm_config(CFG)
MAP = load_mapping()
CHAIN = chain_from_mujoco(ArmModel(MAP.model_path).m)
LIM = limits_rad(MAP, SAFETY.limits, CHAIN_JOINTS)
CM = CollisionModel.from_safety(MAP, SAFETY)
FACTORY = json.loads((Path(__file__).parent / "fixtures" / "follower_factory_dump.json").read_text())
# a clear working pose: lift raised, elbow unfolded (like the step 6b lift runs)
WORK_TICKS = dict(shoulder_pan=2206, shoulder_lift=1673, elbow_flex=1600, wrist_flex=2400, wrist_roll=2156)
Q_WORK = np.array([MAP.joints[j].to_rad(WORK_TICKS[j]) for j in CHAIN_JOINTS])


def test_straight_line_plan():
    p0 = fk(CHAIN, Q_WORK, tcp="fixed_jaw_tip")[:3, 3]
    plan = plan_line(CHAIN, MAP, Q_WORK, p0 + [0.08, 0.0, 0.0], LIM, speed=0.03, collision=CM)
    assert plan.ok, plan.reason
    assert plan.max_line_dev_m < 5e-4  # every planned TCP point within 0.5 mm of the line
    assert plan.duration_s == pytest.approx(0.08 / 0.03 * 1.875)
    assert plan.max_dq < math.radians(2.0)
    np.testing.assert_allclose(plan.tcp_planned[-1], p0 + [0.08, 0, 0], atol=5e-4)
    assert np.ptp(plan.qs[:, 4]) == 0  # wrist_roll held


def test_line_into_the_table_is_refused():
    p0 = fk(CHAIN, Q_WORK, tcp="fixed_jaw_tip")[:3, 3]
    plan = plan_line(CHAIN, MAP, Q_WORK, [p0[0], p0[1], -0.02], LIM, collision=CM)
    assert not plan.ok and ("table collision" in plan.reason or "IK failed" in plan.reason), plan.reason


def test_unreachable_line_is_refused():
    plan = plan_line(CHAIN, MAP, Q_WORK, [1.0, 0.0, 0.2], LIM)
    assert not plan.ok and ("IK failed" in plan.reason or "joint jump" in plan.reason), plan.reason


def test_line_point_distance():
    d = line_point_distance(np.zeros(3), np.array([1.0, 0, 0]), np.array([[0.5, 0.1, 0], [2.0, 0, 0]]))
    np.testing.assert_allclose(d, [0.1, 1.0])


def test_safearm_allows_moving_away_from_the_table():
    b = SimBus.from_dump(FACTORY)  # factory rest pose: gripper ~2 mm above the table, inside the 3 mm margin
    with SafeArm(b, CFG, collision=CM, mapping=MAP) as arm:
        arm.enable()
        r = arm.step({"elbow_flex": 2927 - 30})  # unfolding the elbow lifts the gripper away: allowed
        assert not arm.frozen and r.commanded["elbow_flex"] == 2897


def test_safearm_freezes_on_predicted_table_contact():
    b = SimBus.from_dump(FACTORY)
    with SafeArm(b, CFG, collision=CM, mapping=MAP) as arm:
        arm.enable()
        before = len(b.write_log)
        arm.step({"shoulder_lift": 1121 + 60})  # raising lift swings the folded gripper into the table
        assert arm.frozen and arm.collision_hit and "table" in arm.collision_hit
        assert any(e.kind == "collision_predicted" for e in arm.events)
        assert arm.state is State.HOLDING  # torque stays on
        lift_writes = [w.value for w in b.write_log[before:] if w.servo_id == CFG.joints["shoulder_lift"]]
        assert 1181 not in lift_writes  # the colliding goal was never written


def test_executing_a_planned_line_on_the_simulator_stays_straight():
    b = SimBus.from_dump(FACTORY)
    with SafeArm(b, CFG, collision=CM, mapping=MAP) as arm:
        arm.enable()
        # move to the clear working pose first (joint-space, slow), then plan the line from there
        from so101.motion.loop import Segment, sequence
        # unfold the elbow FIRST, then the rest: raising lift with the elbow folded drives the gripper into the
        # table (the real step 6 contacts), and SafeArm would freeze
        start = {j: float(arm.last_goals[j]) for j in CHAIN_JOINTS}
        mid = {**start, "elbow_flex": float(WORK_TICKS["elbow_flex"])}
        fn, total = sequence([Segment(start, mid, 200.0, "minjerk"),
                              Segment(mid, {j: float(v) for j, v in WORK_TICKS.items()}, 200.0, "minjerk")], settle_s=0.5)  # fmt: skip
        stop = threading.Event()
        threading.Timer(total + 20, stop.set).start()  # a frozen loop holds until stop: never hang the suite
        control_loop(arm, 100, fn, total, stop)
        assert not arm.frozen, arm.collision_hit
        snap = arm.step().snapshot
        q_now = np.array([MAP.joints[j].to_rad(snap.joints[j].position) for j in CHAIN_JOINTS])
        p0 = fk(CHAIN, q_now, tcp="fixed_jaw_tip")[:3, 3]
        plan = plan_line(CHAIN, MAP, q_now, p0 + [0.06, 0.02, 0.0], LIM, speed=0.03, collision=CM)
        assert plan.ok, plan.reason
        tcp = []
        def record(t):
            s = arm.arm.snapshot()
            q = np.array([MAP.joints[j].to_rad(s.joints[j].position) for j in CHAIN_JOINTS])
            tcp.append(fk(CHAIN, q, tcp="fixed_jaw_tip")[:3, 3])
        control_loop(arm, 100, plan.goals_fn(), plan.duration_s + 0.5, stop, on_tick=record)
        assert not arm.frozen, arm.collision_hit
        dev = line_point_distance(plan.p0, plan.p1, np.array(tcp))
        assert dev.max() < 0.003, dev.max()  # sim tracks within a few mm of the line
        np.testing.assert_allclose(tcp[-1], plan.p1, atol=0.003)
