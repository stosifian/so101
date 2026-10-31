"""Joint goals for a tag target with a calibrated kinematic variant (Phase 2 step 10b).

A target is the tag centre's position RELATIVE TO THE iPad (mm). For a variant (so101.kinematics.calibrated): base
target = X⁻¹ · target (X = the variant's fitted iPad←base), and position-only IK solves for the tag point
(tcp = the variant's fitted body←tag transform Y) with the variant's FKParams, wrist_roll held, seeded from a
pose that saw the tag (so the wrist faces the camera similarly). Then, for "calibrated" commands:
  1. gravity bend: find encoder angles q_enc whose modelled deflection lands on the IK solution,
     q_enc + c·τ(q_enc) = q_ik (fixed-point iteration; τ = model gravity torques);
  2. servo droop: the goal is set so the encoder SETTLES at q_enc: goal = reading − (a + b·τ) per pitch joint
     (config/kinematics/servo_droop.json).
"nominal" commands use the nominal variant's IK solution as the goal, uncompensated (today's behaviour).
"""

import json

import numpy as np

from so101.analysis import kincal
from so101.config import CONFIG_DIR
from so101.kinematics.calibrated import Calibrated
from so101.kinematics.fk import CHAIN_JOINTS
from so101.kinematics.ik import solve_ik

DROOP = CONFIG_DIR / "kinematics" / "servo_droop.json"
PITCH = ["shoulder_lift", "elbow_flex", "wrist_flex"]


def ipad_to_base_m(cal: Calibrated, p_ipad_mm: np.ndarray) -> np.ndarray:
    X = cal.X_ref_base["all"]
    return (np.linalg.inv(X) @ np.r_[p_ipad_mm, 1.0])[:3] / 1000


def solve_tag(cal: Calibrated, chain, lim: np.ndarray, p_ipad_mm: np.ndarray, q_seed: np.ndarray):
    """Geometric joint angles (5, rad) placing the tag centre at the target, or None."""
    tcp = cal.Y_body_tag.copy()
    tcp[:3, 3] /= 1000
    r = solve_ik(chain, ipad_to_base_m(cal, p_ipad_mm), q_seed, lim, tcp=tcp, params=cal.params, fixed={4: q_seed[4]})
    return r.q if r.success else None


def goals(cal: Calibrated, mapping, arm_model, q_ik: np.ndarray, base_ticks: dict, compensate: bool) -> tuple[dict, dict]:
    """(goal ticks, desired encoder reading ticks) for IK angles q_ik."""
    to_t = lambda q: {**base_ticks, **{j: round(mapping.joints[j].to_ticks(v)) for j, v in zip(CHAIN_JOINTS, q)}}  # noqa: E731
    if not compensate:
        t = to_t(q_ik)
        return t, t
    q_enc = q_ik.copy()
    for _ in range(5):  # bend: q_enc + c·τ(q_enc) = q_ik
        tau = kincal.gravity_torques(arm_model, mapping, to_t(q_enc))
        q_enc = q_ik - (cal.effective_q(q_enc, tau) - q_enc)
    reading = to_t(q_enc)
    tau = kincal.gravity_torques(arm_model, mapping, reading)
    droop = json.loads(DROOP.read_text())["joints"]
    goal = dict(reading)
    for k, j in enumerate(PITCH):
        goal[j] = round(reading[j] - (droop[j]["a"] + droop[j]["b_per_Nm"] * tau[k]))
    return goal, reading
