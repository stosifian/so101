import math

import mujoco
import numpy as np
import pytest

from so101.kinematics.fk import (CHAIN_JOINTS, FKParams, chain_from_mujoco, fk, fk_space, geometric_jacobian,
                                 se3_exp, space_screws)  # fmt: skip
from so101.kinematics.model import JOINTS, ArmModel

MODEL = ArmModel("models/so101_menagerie/scene.xml")
CHAIN = chain_from_mujoco(MODEL.m)
SITE = MODEL.tool_site
# validation poses from run 20261027-103553_model_check_follower (degrees)
CHECKS = {
    "check1": [92.37, -106.88, 1.32, -0.97, 1.93],
    "check2": [-2.02, -46.67, 89.65, -0.7, -88.95],
    "check3": [-38.32, -106.88, 25.05, -105.91, -88.77],
}


def mujoco_tool(q5):
    MODEL.set_q({**dict(zip(CHAIN_JOINTS, q5)), "gripper": 0.0})
    return MODEL.d.site_xpos[SITE].copy(), MODEL.d.site_xmat[SITE].reshape(3, 3).copy()


def angle_between(R1, R2):
    return math.degrees(math.acos(np.clip((np.trace(R1.T @ R2) - 1) / 2, -1, 1)))


def assert_matches_mujoco(q5):
    T = fk(CHAIN, q5)
    p, R = mujoco_tool(q5)
    assert np.linalg.norm(T[:3, 3] - p) < 1e-4  # 0.1 mm
    assert angle_between(T[:3, :3], R) < 0.01  # degrees


@pytest.mark.parametrize("name", list(CHECKS))
def test_matches_mujoco_on_validation_poses(name):
    assert_matches_mujoco(np.radians(CHECKS[name]))


def test_matches_mujoco_on_1000_random_poses():
    rng = np.random.default_rng(0)
    lo = np.array([MODEL.ranges[j][0] for j in CHAIN_JOINTS])
    hi = np.array([MODEL.ranges[j][1] for j in CHAIN_JOINTS])
    worst_p = worst_r = 0.0
    for q in rng.uniform(lo, hi, size=(1000, 5)):
        T = fk(CHAIN, q)
        p, R = mujoco_tool(q)
        worst_p, worst_r = max(worst_p, np.linalg.norm(T[:3, 3] - p)), max(worst_r, angle_between(T[:3, :3], R))
    assert worst_p < 1e-4 and worst_r < 0.01, (worst_p, worst_r)


def test_zero_offset_equals_shifting_the_joint():
    q = np.radians(CHECKS["check2"])
    d = np.array([0.01, -0.02, 0.03, 0.0, 0.05])
    np.testing.assert_allclose(fk(CHAIN, q, FKParams(delta=d)), fk(CHAIN, q + d), atol=1e-12)


def test_link_correction_moves_the_tool():
    q = np.radians(CHECKS["check2"])
    xi = np.zeros((5, 6))
    xi[2, 3:] = [0.001, 0, 0]  # 1 mm translation of the lower arm placement
    dp = np.linalg.norm(fk(CHAIN, q, FKParams(xi=xi))[:3, 3] - fk(CHAIN, q)[:3, 3])
    assert dp == pytest.approx(0.001, rel=1e-6)  # a pure translation moves the tool by exactly 1 mm


def test_geometric_jacobian_matches_finite_differences():
    q = np.radians(CHECKS["check2"])
    J = geometric_jacobian(CHAIN, q)
    h = 1e-6
    for i in range(5):
        dq = np.zeros(5)
        dq[i] = h
        Tp, Tm = fk(CHAIN, q + dq), fk(CHAIN, q - dq)
        np.testing.assert_allclose(J[:3, i], (Tp[:3, 3] - Tm[:3, 3]) / (2 * h), atol=1e-6)
        dR = Tp[:3, :3] @ Tm[:3, :3].T  # small rotation ~ I + [w]x * 2h
        w = np.array([dR[2, 1] - dR[1, 2], dR[0, 2] - dR[2, 0], dR[1, 0] - dR[0, 1]]) / 2 / (2 * h)
        np.testing.assert_allclose(J[3:, i], w, atol=1e-5)


def test_space_frame_poe_gives_identical_poses():
    S, M = space_screws(CHAIN)
    assert np.allclose(np.linalg.norm(S[:, :3], axis=1), 1) and np.allclose(np.sum(S[:, :3] * S[:, 3:], axis=1), 0)
    rng = np.random.default_rng(1)
    for q in rng.uniform(-1.5, 1.5, size=(200, 5)):
        np.testing.assert_allclose(fk_space(S, M, q), fk(CHAIN, q), atol=1e-12)
    # with parameters: zero offsets fold into the home pose
    p = FKParams(delta=np.array([0.02, -0.01, 0.0, 0.03, 0.0]))
    p.xi[1] = [0.001, 0, 0.002, 0.0005, 0, 0]
    S2, M2 = space_screws(CHAIN, p)
    q = np.radians(CHECKS["check3"])
    np.testing.assert_allclose(fk_space(S2, M2, q + p.delta), fk(CHAIN, q, p), atol=1e-12)


def test_se3_exp_small_and_large_angles():
    np.testing.assert_allclose(se3_exp(np.zeros(6)), np.eye(4))
    T = se3_exp(np.array([0, 0, math.pi / 2, 1.0, 0, 0]))  # quarter turn about z with v = x
    np.testing.assert_allclose(T[:3, :3] @ [1, 0, 0], [0, 1, 0], atol=1e-12)
    a = se3_exp(np.array([1e-9, 0, 0, 0, 0, 0]))
    assert np.isfinite(a).all()
