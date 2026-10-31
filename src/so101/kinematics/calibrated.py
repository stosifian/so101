"""Calibrated kinematics (Phase 2 step 10): load one fitted variant from config/kinematics/follower_<variant>.json.

Each variant lives in its own file (written by scripts/camera/kinematic_fit.py --save) and is never merged with
another, so a planner or analysis always names exactly which model it uses. A variant = FKParams (joint zero offsets
`delta`, link placement twists `xi`) + an optional gravity compliance (rad per N·m on shoulder_lift, elbow_flex,
wrist_flex; applied to the encoder angles before FK) + the transforms it was fitted with (iPad<-base, body<-tag).
"""

import json
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from so101.config import CONFIG_DIR
from so101.kinematics.fk import FKParams

KIN_DIR = CONFIG_DIR / "kinematics"


@dataclass(frozen=True)
class Calibrated:
    variant: str
    params: FKParams
    compliance: np.ndarray  # (3,) rad per N·m on shoulder_lift, elbow_flex, wrist_flex
    compliance_mode: str | None  # None | "lin" | "thr"
    tau_knee: float | None
    X_ref_base: dict  # run or "all" -> 4x4 (mm)
    Y_body_tag: np.ndarray
    reference: str
    path: Path
    raw: dict

    def effective_q(self, q: np.ndarray, tau: np.ndarray) -> np.ndarray:
        """Encoder angles (5, rad) + modelled gravity deflection for gravity torques tau (3, N·m)."""
        if self.compliance_mode is None:
            return q
        q = np.array(q, float).copy()
        if self.compliance_mode == "lin":
            q[[1, 2, 3]] += self.compliance * tau
        else:
            q[1] += self.compliance[0] * max(0.0, abs(tau[0]) - self.tau_knee) * np.sign(tau[0])
        return q


def available() -> list[str]:
    return sorted(p.stem.removeprefix("follower_") for p in KIN_DIR.glob("follower_*.json"))


def load(variant: str, kin_dir: Path = KIN_DIR) -> Calibrated:
    path = kin_dir / f"follower_{variant}.json"
    if not path.exists():
        raise FileNotFoundError(f"{path} (available: {available()})")
    d = json.loads(path.read_text())
    if d["variant"] != variant:
        raise ValueError(f"{path} holds variant {d['variant']!r}, not {variant!r}")
    p = d["params"]
    fp = FKParams(delta=np.array(p["delta_rad"]), xi=np.array(p["xi"]))
    return Calibrated(variant, fp, np.array(p["compliance_rad_per_Nm"]), p["compliance_mode"], p.get("tau_knee_Nm"),
                      {k: np.array(v) for k, v in d["X_ref_base"].items()}, np.array(d["Y_body_tag"]), d["reference"], path, d)  # fmt: skip
