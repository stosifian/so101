import math
import time

import mujoco
import numpy as np
import pytest

from so101.config import load_arm_config
from so101.kinematics.fk import CHAIN_JOINTS, chain_from_mujoco, fk
from so101.kinematics.ik import limits_rad, solve_ik
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel
from so101.safety import SafetyConfig

MAP = load_mapping()
MODEL = ArmModel(MAP.model_path)
CHAIN = chain_from_mujoco(MODEL.m)
LIM = limits_rad(MAP, SafetyConfig.from_arm_config(load_arm_config("follower")).limits, CHAIN_JOINTS)
DOWN = np.array([0.0, 0.0, -1.0])


def test_fixed_jaw_tip_matches_mujoco_geom():
    gid = mujoco.mj_name2id(MODEL.m, mujoco.mjtObj.mjOBJ_GEOM, "fixed_jaw_sph_tip1")
    q = np.radians([20, -30, 40, 50, 10])
    MODEL.set_q(dict(zip(CHAIN_JOINTS, q)))
    np.testing.assert_allclose(fk(CHAIN, q, tcp="fixed_jaw_tip")[:3, 3], MODEL.d.geom_xpos[gid], atol=1e-9)


def test_tip_frame_z_points_along_the_fingers():
    q = np.radians([10, -20, 30, 40, 25])
    jf, _ = __import__("so101.kinematics.fk", fromlist=["frames"]).frames(CHAIN, q)
    T_tip = fk(CHAIN, q, tcp="fixed_jaw_tip")
    roll_axis_origin = jf[4][:3, 3]  # wrist_roll joint frame origin
    finger = T_tip[:3, 3] - roll_axis_origin
    assert np.dot(finger / np.linalg.norm(finger), T_tip[:3, 2]) > 0.99


def test_round_trip_random_reachable_targets():
    rng = np.random.default_rng(0)
    fails, worst = 0, 0.0
    for _ in range(200):
        q_true = rng.uniform(LIM[:, 0], LIM[:, 1])
        target = fk(CHAIN, q_true, tcp="fixed_jaw_tip")[:3, 3]
        seed = np.clip(q_true + rng.normal(0, 0.25, 5), LIM[:, 0], LIM[:, 1])  # nearby start, like a real move
        r = solve_ik(CHAIN, target, seed, LIM)
        fails += not r.success
        worst = max(worst, r.pos_err_m)
        assert np.all(r.q >= LIM[:, 0] - 1e-9) and np.all(r.q <= LIM[:, 1] + 1e-9)
    assert fails <= 4, fails  # a few seeds can land on the wrong branch; the planner reports those
    assert worst < 0.05


def test_position_plus_approach_round_trip():
    """Targets built from valid poses (position AND finger direction), so they are reachable by construction."""
    rng = np.random.default_rng(3)
    fails = 0
    for _ in range(100):
        q_true = rng.uniform(LIM[:, 0], LIM[:, 1])
        T = fk(CHAIN, q_true, tcp="fixed_jaw_tip")
        seed = np.clip(q_true + rng.normal(0, 0.2, 5), LIM[:, 0], LIM[:, 1])
        r = solve_ik(CHAIN, T[:3, 3], seed, LIM, approach=T[:3, 2])
        fails += not r.success
        if r.success:
            assert math.degrees(r.ang_err_rad) < 1.0 and r.pos_err_m < 5e-4
    assert fails <= 3, fails


def test_fingers_straight_down_near_the_table_exceeds_the_wrist_limit():
    """Known limitation (step 9a): fingers vertical at 8 cm above the table needs wrist_flex beyond its software
    limit (+72.7 deg = swept max 77 deg - 50 ticks). IK must report it, not force it."""
    r = solve_ik(CHAIN, [0.25, 0.0, 0.08], np.radians([0, -20, 40, 60, 0]), LIM, approach=DOWN)
    assert not r.success
    assert r.q[3] == pytest.approx(LIM[3, 1], abs=1e-6)  # pinned at the wrist_flex limit
    tilted = np.array([math.sin(math.radians(25)), 0.0, -math.cos(math.radians(25))])  # 25 deg off vertical, outward
    r2 = solve_ik(CHAIN, [0.25, 0.0, 0.08], np.radians([0, -20, 40, 60, 0]), LIM, approach=tilted)
    assert r2.success, r2


def test_unreachable_target_is_reported_not_forced():
    r = solve_ik(CHAIN, [1.0, 0.0, 0.2], np.zeros(5), LIM)
    assert not r.success and r.reason in ("stalled", "max_iter") and r.pos_err_m > 0.3


def test_fixed_wrist_roll_is_held():
    r = solve_ik(CHAIN, [0.22, 0.05, 0.10], np.radians([0, -20, 40, 60, 0]), LIM, fixed={4: 0.3})
    assert r.success and r.q[4] == pytest.approx(0.3)


def test_outside_start_never_moves_further_out():
    q0 = np.radians([0, -20, 40, 60, 0])
    q0[2] = LIM[2, 1] + 0.05  # elbow 3 deg past its upper limit, like the rest pose
    r = solve_ik(CHAIN, fk(CHAIN, np.radians([0, -10, 30, 50, 0]), tcp="fixed_jaw_tip")[:3, 3], q0, LIM)
    assert r.q[2] <= q0[2] + 1e-12


def test_fast_enough_for_planning():
    """Cartesian moves solve each step warm-started from the previous one."""
    q = np.radians([0, -20, 40, 60, 0])
    p0 = fk(CHAIN, q, tcp="fixed_jaw_tip")[:3, 3]
    t0 = time.perf_counter()
    for k in range(100):  # 10 cm line in 1 mm steps
        r = solve_ik(CHAIN, p0 + [0.001 * k, 0, 0], q, LIM)
        assert r.success
        q = r.q
    assert (time.perf_counter() - t0) / 100 < 0.005  # < 5 ms per warm-started step
