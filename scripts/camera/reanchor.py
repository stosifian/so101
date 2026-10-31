"""Re-anchor the calibrated variants after the iPad (or the arm base) moved: fit ONLY the iPad<-base transform on a
fresh run, every variant's arm parameters held fixed (so101.analysis.kincal.refit_transform). Read-only on raw data.

    uv run scripts/camera/reanchor.py data/raw/<anchor_capture_run> [--variants nominal,links,stiff_lin]

Writes config/kinematics/anchors/<run_id>.json: X (iPad<-base) per variant + fit RMS + how far it moved vs the
variant's own fit. Use it with so101.kinematics.calibrated.load(variant, anchor=<file>); the variant files are not
modified. The anchor run must not be part of any later accuracy test.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from rich.console import Console
from scipy.spatial.transform import Rotation

from so101.analysis import kincal
from so101.analysis.pose import Camera, solve, tag_object_points
from so101.config import REPO_ROOT
from so101.kinematics import calibrated
from so101.kinematics.fk import chain_from_mujoco
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel

sys.path.insert(0, str(Path(__file__).parent))
from kinematic_fit import detections, group_of  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--variants", default="nominal,offsets,links,full,stiff_lin,stiff_thr")
    args = ap.parse_args()
    console = Console()
    cam, obj = Camera.load(), tag_object_points(20.0)
    mapping = load_mapping("follower")
    arm = ArmModel(mapping.model_path)
    chain = chain_from_mujoco(arm.m)
    shots = []
    for r in detections(args.run, 2, console):
        if r.get("T_cam_ipad") is None:
            continue
        c = np.array(r["corners"])
        shots.append(kincal.Shot(group_of(args.run.name, r["label"]), kincal.angles_from_ticks(mapping, r["ticks"]), c,
                                 solve(obj, c, cam, 0.05, square=True).matrix(), r["label"], "all", np.array(r["T_cam_ipad"]),
                                 kincal.gravity_torques(arm, mapping, r["ticks"])))  # fmt: skip
    out = {"anchor_run": args.run.name, "n_shots": len(shots), "X_ref_base": {}, "fit_rms_px": {}, "moved_vs_fit": {}}
    for v in args.variants.split(","):
        c = calibrated.load(v)
        m = kincal.Model(v, {"all": c.X_ref_base["all"]}, c.Y_body_tag, c.params, c.raw["tag_scale"], c.compliance)
        X, rms = kincal.refit_transform(shots, chain, cam, 20.0, m)
        old = c.X_ref_base["all"]
        dt = X[:3, 3] - old[:3, 3]
        dr = np.degrees(np.linalg.norm(Rotation.from_matrix(old[:3, :3].T @ X[:3, :3]).as_rotvec()))
        out["X_ref_base"][v], out["fit_rms_px"][v] = X.tolist(), rms
        out["moved_vs_fit"][v] = {"translation_mm": dt.tolist(), "rotation_deg": dr}
        console.print(f"{v:10s} corner RMS {rms:5.1f} px; iPad<-base moved {np.round(dt, 1)} mm, {dr:.2f}° vs the variant's fit", highlight=False)
    path = REPO_ROOT / "config" / "kinematics" / "anchors" / f"{args.run.name}.json"
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(out, indent=1))
    console.print(f"{len(shots)} shots; wrote {path.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
