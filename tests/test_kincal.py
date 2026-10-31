"""Kinematic calibration on synthetic data: known camera/iPad/tag transforms and joint offsets, per-shot camera motion."""

import cv2
import numpy as np

from so101.analysis import kincal
from so101.analysis.handeye import T_of
from so101.analysis.pose import Camera, solve, tag_object_points
from so101.kinematics.fk import FKParams, chain_from_mujoco, fk
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel

CAM = Camera(np.array([[7007.0, 0, 3527.5], [0, 7006.0, 2319.3], [0, 0, 1]]), np.zeros(5))
MAP = load_mapping("follower")
CHAIN = chain_from_mujoco(ArmModel(MAP.model_path).m)
T_CAM_BASE = T_of(np.array([1.9, -1.2, 0.6]), np.array([80.0, 150.0, 760.0]))  # roughly the real camera
T_CAM_IPAD = T_of(np.array([-2.5, 0.1, 0.05]), np.array([-60.0, 30.0, 700.0]))
Y_TRUE = T_of(np.array([0.3, -0.2, 0.1]), np.array([15.0, 43.0, -3.0]))


def shots(true: FKParams, n: int = 40, cam_motion_mm: float = 0.0, seed: int = 0) -> list[kincal.Shot]:
    rng = np.random.default_rng(seed)
    out, obj = [], tag_object_points(20.0)
    X_ipad_base = np.linalg.inv(T_CAM_IPAD) @ T_CAM_BASE
    for i in range(n):
        ticks = {"shoulder_pan": 2237 + rng.integers(-250, 250), "shoulder_lift": 1700 + rng.integers(-400, 600),
                 "elbow_flex": 2300 + rng.integers(-500, 400), "wrist_flex": 2400 + rng.integers(-400, 400), "wrist_roll": 1138}  # fmt: skip
        q = kincal.angles_from_ticks(MAP, ticks)
        move = T_of(rng.normal(0, 1e-4, 3), rng.normal(0, cam_motion_mm, 3))  # camera bumped: iPad and arm move together
        T_ref = move @ T_CAM_IPAD
        F = fk(CHAIN, q, true, tcp=np.eye(4))
        F[:3, 3] *= 1000
        T = T_ref @ X_ipad_base @ F @ Y_TRUE
        c = cv2.projectPoints(obj, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], CAM.K, CAM.dist)[0].reshape(-1, 2)
        out.append(kincal.Shot(f"g{i}", q, c, solve(obj, c, CAM, 0.05, square=True).matrix(), f"s{i}", "run", T_ref))
    return out


def test_offsets_recovered_with_camera_motion():
    true = FKParams()
    true.delta[1], true.delta[2] = np.radians(1.2), np.radians(-0.7)
    S = shots(true, cam_motion_mm=2.0)  # 2 mm camera jitter: cancels because the iPad is measured in each photo
    m, rms = kincal.fit(S, CHAIN, CAM, 20.0, "offsets")
    assert rms < 0.05
    assert np.allclose(np.degrees(m.params.delta[[1, 2]]), [1.2, -0.7], atol=0.02)
    assert np.abs(kincal.errors_mm(m, S, CHAIN)).max() < 0.02


def test_heldout_errors_small_for_exact_model():
    S = shots(FKParams(), n=30, cam_motion_mm=1.0, seed=1)
    e = kincal.kfold(S, CHAIN, CAM, 20.0, "nominal", k=3)
    assert np.nanmax(np.linalg.norm(e, axis=1)) < 0.05


def test_calibrated_roundtrip(tmp_path):
    """A saved variant loads back into the same FK as the fitted model, and a file can't masquerade as another variant."""
    import json

    import pytest

    from so101.kinematics import calibrated

    true = FKParams()
    true.delta[1] = np.radians(0.8)
    S = shots(true, n=20)
    m, _ = kincal.fit(S, CHAIN, CAM, 20.0, "offsets")
    doc = {"variant": "offsets", "reference": "ipad", "X_ref_base": {k: v.tolist() for k, v in m.X.items()}, "Y_body_tag": m.Y.tolist(),
           "params": {"delta_rad": m.params.delta.tolist(), "xi": m.params.xi.tolist(), "compliance_rad_per_Nm": [0, 0, 0],
                      "compliance_mode": None, "tau_knee_Nm": None}}  # fmt: skip
    (tmp_path / "follower_offsets.json").write_text(json.dumps(doc))
    c = calibrated.load("offsets", tmp_path)
    q = S[0].q
    assert np.allclose(fk(CHAIN, c.effective_q(q, np.zeros(3)), c.params), fk(CHAIN, q, m.params))
    (tmp_path / "follower_links.json").write_text(json.dumps(doc))  # wrong content under another name
    with pytest.raises(ValueError):
        calibrated.load("links", tmp_path)
