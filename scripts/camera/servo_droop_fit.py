"""Step 10b: servo droop model: how far each pitch joint's encoder settles from its goal under gravity load.
Read-only: fits the capture runs' manifests (settle.error_ticks = reading − goal at the first snapshot).

    uv run scripts/camera/servo_droop_fit.py data/raw/<capture_run> [...]

Per joint (shoulder_lift, elbow_flex, wrist_flex): error ≈ a + b · τ, τ = static gravity torque at the commanded pose
(MuJoCo model masses, N·m). This is the encoder-VISIBLE droop (servo P-control needs an error to hold a load),
distinct from the hidden compliance in the kinematic fit. Approach direction isn't recorded per arrival here, so
the deadband (±1–2 ticks) stays in the residual. Writes config/kinematics/servo_droop.json.
"""

import argparse
import json
from pathlib import Path

import numpy as np

from so101.analysis import kincal
from so101.config import REPO_ROOT
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel
from so101.runlog.manifest import read_manifest

JOINTS = ["shoulder_lift", "elbow_flex", "wrist_flex"]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", type=Path, nargs="+")
    args = ap.parse_args()
    mapping = load_mapping("follower")
    arm = ArmModel(mapping.model_path)
    T, E = [], []
    for run in args.runs:
        for m in read_manifest(run / "manifest.jsonl")[1]:
            st = m.get("settle") or {}
            if not m.get("commanded_ticks") or "error_ticks" not in st or st.get("moved"):
                continue
            T.append(kincal.gravity_torques(arm, mapping, m["commanded_ticks"]))
            E.append([st["error_ticks"][j] for j in JOINTS])
    T, E = np.array(T), np.array(E)
    out = {"model": "error_ticks = a + b * tau_Nm (reading - goal)", "runs": [r.name for r in args.runs], "n": len(T), "joints": {}}
    for k, j in enumerate(JOINTS):
        A = np.c_[np.ones(len(T)), T[:, k]]
        coef, *_ = np.linalg.lstsq(A, E[:, k], rcond=None)
        res = E[:, k] - A @ coef
        out["joints"][j] = {"a": float(coef[0]), "b_per_Nm": float(coef[1]), "resid_sd": float(res.std(ddof=2)),
                            "tau_range": [float(T[:, k].min()), float(T[:, k].max())]}  # fmt: skip
        print(f"{j:14s} error = {coef[0]:+6.1f} {coef[1]:+7.1f}·τ ticks   resid sd {res.std(ddof=2):.1f}   τ {T[:, k].min():+.2f}…{T[:, k].max():+.2f} N·m")
    path = REPO_ROOT / "config" / "kinematics" / "servo_droop.json"
    path.write_text(json.dumps(out, indent=1))
    print(f"{len(T)} arrivals; wrote {path.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
