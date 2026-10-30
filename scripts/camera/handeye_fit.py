"""Fit the camera's pose in the arm base frame and the gripper tag's mounting on the gripper body (so101.analysis.
handeye) from runs with joint readings + photos: capture runs (scripts/motion/capture_tour.py) and teach runs
(scripts/readonly/teach_poses.py). Read-only: uses raw manifests + images, writes data/derived/handeye/.

    uv run scripts/camera/handeye_fit.py data/raw/<capture_run> [data/raw/<teach_run> ...]

Per pose: joint ticks (first snapshot; poses where the arm moved during the capture are skipped) -> forward
kinematics to the gripper body; tag corners refined on the linear RAW. Fit = pixel reprojection LM. Prints the
camera position/axis in the base frame, the tag offset on the gripper body, and per-pose residuals (an outlier
pose = a bad joint reading or detection; the residual floor = kinematic model error, step 10 territory).
Writes data/derived/handeye/handeye.json (used by scripts/sim/plan_sector_tour.py when present).
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.table import Table

from so101.analysis import handeye
from so101.analysis.apriltag import detect_tags
from so101.analysis.charuco import load_linear, to_display
from so101.analysis.pose import Camera, solve, tag_object_points
from so101.config import REPO_ROOT
from so101.kinematics.fk import CHAIN_JOINTS, chain_from_mujoco, fk
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel
from so101.runlog.manifest import read_manifest

OUT = REPO_ROOT / "data" / "derived" / "handeye"
MOVED_TICKS = 3


def gripper_body(chain, mapping, ticks: dict) -> np.ndarray:
    q = np.array([mapping.joints[j].to_rad(ticks[j]) for j in CHAIN_JOINTS])
    F = fk(chain, q, tcp=np.eye(4))
    F[:3, 3] *= 1000
    return F


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", type=Path, nargs="+")
    ap.add_argument("--tag-id", type=int, default=2)
    ap.add_argument("--tag-mm", type=float, default=20.0)
    args = ap.parse_args()
    console = Console()
    cam = Camera.load()
    mapping = load_mapping("follower")
    chain = chain_from_mujoco(ArmModel(mapping.model_path).m)

    rows = []
    for run in args.runs:
        for m in read_manifest(run / "manifest.jsonl")[1]:
            arws = [Path(f) for f in (m.get("camera") or {}).get("files", []) if f.upper().endswith(".ARW")]
            b, a = m.get("snapshot_before"), m.get("snapshot_after")
            if not arws or not b or not a:
                continue
            tb = {j: s["position"] for j, s in b["joints"].items() if s}
            ta = {j: s["position"] for j, s in a["joints"].items() if s}
            if any(abs(ta[j] - tb[j]) > MOVED_TICKS for j in tb):
                continue
            lin = load_linear(arws[0])
            t = detect_tags(to_display(lin), ids=[args.tag_id], linear=lin).get(args.tag_id)
            if t is None:
                continue
            p = solve(tag_object_points(args.tag_mm), t.corners, cam, 0.05, square=True)
            rows.append({"run": run.name, "label": m["label"], "F": gripper_body(chain, mapping, tb), "corners": t.corners,
                         "T": p.matrix(), "edge": t.edge_rms})  # fmt: skip
            console.print(f"  {run.name[:15]} {m['label']}: edge {t.edge_rms:.3f}", highlight=False)
    if len(rows) < 6:
        sys.exit(f"need >= 6 usable poses, have {len(rows)}")
    he = handeye.fit([r["F"] for r in rows], [r["corners"] for r in rows], [r["T"] for r in rows], cam, args.tag_mm)

    t = Table(title=f"hand-eye fit: {len(rows)} poses, corner RMS {he.rms_px:.2f} px", title_justify="left")
    for c in ["run", "pose", "edge", "RMS px", "pred dist mm", "pred tilt °"]:
        t.add_column(c, justify="left" if c in ("run", "pose") else "right")
    for r, e in zip(rows, he.per_pose_px):
        pr = he.predict(r["F"], cam)
        t.add_row(r["run"][:15], r["label"], f"{r['edge']:.3f}", f"[{'red' if e > 3 * np.median(he.per_pose_px) else 'default'}]{e:.2f}",
                  f"{pr['distance_mm']:.0f}", f"{pr['tilt_deg']:.0f}")  # fmt: skip
    console.print(t)
    d = he.as_dict()
    console.print(f"camera in base frame: position {np.round(d['camera_position_in_base_mm'], 0)} mm, "
                  f"optical axis {np.round(d['camera_optical_axis_in_base'], 2)}", highlight=False)  # fmt: skip
    console.print(f"tag on the gripper body: offset {np.round(d['tag_offset_on_link_mm'], 1)} mm; "
                  f"tag normal in the body frame {np.round(he.Y[:3, 2], 2)}", highlight=False)  # fmt: skip
    mm_per_px = np.median([np.linalg.norm(r["T"][:3, 3]) for r in rows]) / cam.K[0, 0]
    console.print(f"≈ {he.rms_px * mm_per_px:.2f} mm at the tag (RMS), {np.max(he.per_pose_px) * mm_per_px:.2f} mm worst pose")
    OUT.mkdir(parents=True, exist_ok=True)
    d.update(runs=[r.name for r in args.runs], poses=[{"run": r["run"], "label": r["label"], "rms_px": float(e)} for r, e in zip(rows, he.per_pose_px)])
    (OUT / "handeye.json").write_text(json.dumps(d, indent=1))
    console.print(f"wrote {(OUT / 'handeye.json').relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
