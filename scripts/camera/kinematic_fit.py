"""Step 10: kinematic calibration + accuracy from runs with joint readings and photos (so101.analysis.kincal).
Read-only on raw data; detections are cached in data/derived/kincal/cache/<run>.json (decoding RAWs is slow).

    uv run scripts/camera/kinematic_fit.py data/raw/<run> [data/raw/<run> ...]
    uv run scripts/camera/kinematic_fit.py data/raw/*_capture_follower --variants nominal,offsets,links --folds 5

Shots where the arm moved during the capture or the tag wasn't found are skipped. Pose groups (for held-out
validation): one per commanded pose; step 8 repeats of a target share a group. Variants:
  nominal  camera + tag transforms only (the hand-eye fit): today's model accuracy
  offsets  + zero offsets of shoulder_lift, elbow_flex (wrist_flex/roll offsets need roll variation)
  links    + small placement corrections of links 1-4 (6 each, weak priors)
  full     links + wrist_flex and wrist_roll offsets (need roll variation) + the tag's scale (0.5 % prior)
  stiff_lin  links + linear gravity compliance on lift/elbow/wrist (Δθ = c·τ, τ from the MuJoCo model's masses)
  stiff_thr  links + shoulder_lift compliance above 0.75 N·m only (the threshold seen in step 10: post hoc)
--save writes each variant to its own config/kinematics/follower_<variant>.json; --test scores never-fitted runs;
a comparison table of all variants goes to data/derived/kincal/comparison.md.
Reports, per variant: corner RMS on all shots, and HELD-OUT tag-centre error (k-fold by pose group) in mm:
RMS per camera axis (x, y lateral; z depth, which includes ~0.1 mm PnP noise) and lateral/3D RMS and max.
Plus the identifiability spectrum (singular values in px per prior-σ). Writes data/derived/kincal/kincal.json
(full-data fit of each variant) and kincal.png (held-out errors per pose).
"""

import argparse
import json
import re
import sys
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.table import Table

from so101.analysis import kincal
from so101.analysis.apriltag import detect_tags
from so101.analysis.charuco import CharucoSpec, detect, load_linear, to_display
from so101.analysis.pose import Camera, solve, tag_object_points
from so101.config import REPO_ROOT
from so101.kinematics.fk import CHAIN_JOINTS, chain_from_mujoco
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel
from so101.runlog.manifest import read_manifest

OUT = REPO_ROOT / "data" / "derived" / "kincal"
SPEC, CAM = CharucoSpec.load(), Camera.load()
MOVED_TICKS = 3


def detections(run: Path, tag_id: int, console: Console) -> list[dict]:
    """Per shot: label, joint ticks (first snapshot), tag corners; cached."""
    cache = OUT / "cache" / f"{run.name}_tag{tag_id}_ipad.json"
    if cache.exists():
        return json.loads(cache.read_text())
    rows = []
    for m in read_manifest(run / "manifest.jsonl")[1]:
        arws = [Path(f) for f in (m.get("camera") or {}).get("files", []) if f.upper().endswith(".ARW")]
        b, a = m.get("snapshot_before"), m.get("snapshot_after")
        if not arws or not b or not a:
            continue
        tb = {j: s["position"] for j, s in b["joints"].items() if s}
        ta = {j: s["position"] for j, s in a["joints"].items() if s}
        if any(abs(ta.get(j, tb[j]) - tb[j]) > MOVED_TICKS for j in tb):
            continue
        lin = load_linear(arws[0])
        t = detect_tags(to_display(lin), ids=[tag_id], linear=lin).get(tag_id)
        if t is None:
            continue
        row = {"label": m["label"], "ticks": tb, "corners": t.corners.tolist(), "edge": t.edge_rms, "T_cam_ipad": None}
        board = SPEC.board()
        d = detect(to_display(lin), board)
        if d is not None and len(d.ids) >= 12:
            o, i = board.matchImagePoints(d.corners.reshape(-1, 1, 2).astype(np.float32), d.ids.reshape(-1, 1))
            row["T_cam_ipad"] = solve(o.reshape(-1, 3), i.reshape(-1, 2), CAM, 0.05).matrix().tolist()
            row["ipad_n"] = len(d.ids)
        rows.append(row)
        console.print(f"  {run.name[:15]} {m['label']}", highlight=False)
    cache.parent.mkdir(parents=True, exist_ok=True)
    cache.write_text(json.dumps(rows))
    return rows


def group_of(run: str, label: str) -> str:
    base = label.split("#")[0]
    base = re.sub(r"_r\d+$", "", base)  # step 8 repeats share a group
    base = re.sub(r"^B_.*$", "B_backlash_pose", base)  # step 8 backlash: all at one commanded pose
    return f"{run}:{base}"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", type=Path, nargs="+")
    ap.add_argument("--variants", default="nominal,offsets,links")
    ap.add_argument("--folds", type=int, default=5)
    ap.add_argument("--reference", choices=["ipad", "camera"], default="ipad",
                    help="ipad (default): tag measured relative to the iPad in the same photo; camera: raw camera frame")
    ap.add_argument("--per-run", action=argparse.BooleanOptionalAction, default=False,
                    help="one iPad<-base transform per run (only if the iPad or base moved between runs: on 2026-10-30 they moved "
                         "<= 50 µm / 40\" over 3.5 h, and per-run transforms overfit the runs with few distinct poses)")  # fmt: skip
    ap.add_argument("--test", type=Path, action="append", default=[],
                    help="independent test run(s): never fitted; scored with the model fitted on `runs` (all of them)")
    ap.add_argument("--save", action="store_true", help="write each variant to config/kinematics/follower_<variant>.json")
    ap.add_argument("--tag-id", type=int, default=2)
    ap.add_argument("--tag-mm", type=float, default=20.0)
    args = ap.parse_args()
    console = Console()
    cam = Camera.load()
    mapping = load_mapping("follower")
    arm = ArmModel(mapping.model_path)
    chain = chain_from_mujoco(arm.m)
    obj = tag_object_points(args.tag_mm)

    shots = []
    for run in args.runs:
        for r in detections(run, args.tag_id, console):
            c = np.array(r["corners"])
            if args.reference == "ipad" and r.get("T_cam_ipad") is None:
                continue  # no iPad in this photo: can't be measured relative to it
            T_ref = np.array(r["T_cam_ipad"]) if args.reference == "ipad" else np.eye(4)
            shots.append(kincal.Shot(group_of(run.name, r["label"]), kincal.angles_from_ticks(mapping, r["ticks"]), c,
                                     solve(obj, c, cam, 0.05, square=True).matrix(), f"{run.name[:15]}:{r['label']}",
                                     run.name if args.per_run else "all", T_ref, kincal.gravity_torques(arm, mapping, r["ticks"])))  # fmt: skip
    def load(runs):
        out = []
        for run in runs:
            for r in detections(run, args.tag_id, console):
                if args.reference == "ipad" and r.get("T_cam_ipad") is None:
                    continue
                c = np.array(r["corners"])
                T_ref = np.array(r["T_cam_ipad"]) if args.reference == "ipad" else np.eye(4)
                out.append(kincal.Shot(group_of(run.name, r["label"]), kincal.angles_from_ticks(mapping, r["ticks"]), c,
                                       solve(obj, c, cam, 0.05, square=True).matrix(), f"{run.name[:15]}:{r['label']}",
                                       run.name if args.per_run else "all", T_ref, kincal.gravity_torques(arm, mapping, r["ticks"])))  # fmt: skip
        return out

    test = [s for s in load(args.test) if not s.label.split(":")[1].startswith(("ready", "ref"))]  # the test POINTS only
    groups = sorted({s.group for s in shots})
    console.print(f"{len(shots)} shots, {len(groups)} pose groups from {len(args.runs)} runs; reference: {args.reference}"
                  f"{', one transform per run' if args.per_run else ''}")  # fmt: skip
    if len(groups) < 2 * args.folds:
        sys.exit("too few pose groups for k-fold")

    t = Table(title=f"held-out accuracy at the tag relative to the {args.reference} ({args.folds}-fold by pose group; mm, camera axes; nominal scale)", title_justify="left")
    for c in ["variant", "n θ", "fit RMS px", "RMS x", "RMS y", "RMS z", "lateral RMS", "3D RMS", "3D max", "identifiability (min sv)"]:
        t.add_column(c, justify="right")
    results, held, test_rows = {}, {}, []
    for v in args.variants.split(","):
        model, rms = kincal.fit(shots, chain, cam, args.tag_mm, v)
        e = kincal.kfold(shots, chain, cam, args.tag_mm, v, args.folds)
        held[v] = e
        idf = kincal.identifiability(shots, chain, cam, args.tag_mm, model)
        sv = idf.get("singular_px_per_sigma", [])
        lat, d3 = np.linalg.norm(e[:, :2], axis=1), np.linalg.norm(e, axis=1)
        far = np.array([abs(s_.tau[0]) > kincal.TAU_KNEE for s_ in shots])
        split = {"inner": (~far, float(np.sqrt(np.nanmean(d3[~far] ** 2))), float(np.sqrt(np.nanmean(lat[~far] ** 2)))),
                 "far": (far, float(np.sqrt(np.nanmean(d3[far] ** 2))) if far.any() else float("nan"),
                         float(np.sqrt(np.nanmean(lat[far] ** 2))) if far.any() else float("nan"))}  # fmt: skip
        t.add_row(v, str(kincal._n_theta(v)), f"{rms:.1f}", *[f"{x:.2f}" for x in np.sqrt(np.nanmean(e**2, axis=0))],
                  f"{np.sqrt(np.nanmean(lat**2)):.2f}", f"{np.sqrt(np.nanmean(d3**2)):.2f}", f"{np.nanmax(d3):.2f}",
                  f"{min(sv):.2g} / {max(sv):.2g}" if sv else "")  # fmt: skip
        if test:
            et = kincal.errors_mm(model, test, chain)
            by = {}
            for s_, e_ in zip(test, et):
                by.setdefault(s_.group, []).append(e_)
            em = np.array([np.mean(x, axis=0) for x in by.values()])  # per test pose (mean of its shots)
            lt, d3t = np.linalg.norm(em[:, :2], axis=1), np.linalg.norm(em, axis=1)
            test_rows.append((v, em, lt, d3t))
        results[v] = {"fit_rms_px": rms, "X": {k: x.tolist() for k, x in model.X.items()}, "Y": model.Y.tolist(), "delta_deg": np.degrees(model.params.delta).tolist(),
                      "xi": model.params.xi.tolist(), "heldout_rms_mm": np.sqrt(np.nanmean(e**2, axis=0)).tolist(),
                      "heldout_3d_rms_mm": float(np.sqrt(np.nanmean(d3**2))), "heldout_3d_max_mm": float(np.nanmax(d3)),
                      "identifiability": idf, "n_theta": kincal._n_theta(v),
                      "heldout_lateral_rms_mm": float(np.sqrt(np.nanmean(lat**2))),
                      "heldout_sub_mm_3d_shots": int((d3 < 1).sum()), "n_shots": len(shots),
                      "heldout_inner": {"n_shots": int(split["inner"][0].sum()), "rms_3d_mm": split["inner"][1], "lateral_rms_mm": split["inner"][2]},
                      "heldout_far": {"n_shots": int(split["far"][0].sum()), "rms_3d_mm": split["far"][1], "lateral_rms_mm": split["far"][2]},
                      "compliance_deg_per_Nm": np.degrees(model.compliance).tolist()}  # fmt: skip
        if args.save:
            save_variant(model, v, rms, results[v], args, shots)
        if v != "nominal":
            oj, _, sc, cm = kincal._spec(v)
            console.print(f"  {v}: zero offsets (deg) {dict(zip([CHAIN_JOINTS[i] for i in oj], np.round(np.degrees(model.params.delta[oj]), 2).tolist()))}"
                          + (f", tag scale {model.tag_scale:.4f}" if sc else "")
                          + (f", compliance (deg per N·m) {np.round(np.degrees(model.compliance[: kincal._n_comp(cm)]), 2).tolist()}" if cm else ""),
                          highlight=False)  # fmt: skip
        results[v]["tag_scale"] = model.tag_scale
    console.print(t)
    if test_rows:
        tt = Table(title=f"INDEPENDENT TEST ({', '.join(r.name for r in args.test)}; {len(test_rows[0][1])} poses never used in any fit; "
                         f"mean of each pose's shots; mm, camera axes, relative to the {args.reference})", title_justify="left")  # fmt: skip
        for c in ["variant", "RMS x", "RMS y", "RMS z", "lateral RMS", "3D RMS", "3D median", "3D max", "sub-mm (3D)", "sub-mm (lateral)"]:
            tt.add_column(c, justify="right")
        for v, em, lt, d3t in test_rows:
            tt.add_row(v, *[f"{x:.2f}" for x in np.sqrt(np.mean(em**2, axis=0))], f"{np.sqrt(np.mean(lt**2)):.2f}", f"{np.sqrt(np.mean(d3t**2)):.2f}",
                       f"{np.median(d3t):.2f}", f"{d3t.max():.2f}", f"{(d3t < 1).sum()}/{len(d3t)}", f"{(lt < 1).sum()}/{len(lt)}")  # fmt: skip
            results[v]["test"] = {"runs": [r.name for r in args.test], "n_poses": len(d3t), "rms_mm": np.sqrt(np.mean(em**2, axis=0)).tolist(),
                                  "lateral_rms_mm": float(np.sqrt(np.mean(lt**2))), "rms_3d_mm": float(np.sqrt(np.mean(d3t**2))),
                                  "median_3d_mm": float(np.median(d3t)), "max_3d_mm": float(d3t.max()), "sub_mm_3d": int((d3t < 1).sum()),
                                  "sub_mm_lateral": int((lt < 1).sum())}  # fmt: skip
        console.print(tt)
    comparison(results, args, console)
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "kincal.json").write_text(json.dumps({"runs": [r.name for r in args.runs], "reference": args.reference, "per_run": args.per_run,
                                                 "n_shots": len(shots), "n_groups": len(groups),
                                                 "tag_mm": args.tag_mm, "variants": results}, indent=1))  # fmt: skip
    plot(shots, held, OUT / "kincal.png")
    console.print(f"wrote {(OUT / 'kincal.json').relative_to(REPO_ROOT)} + kincal.png")


DESCRIBE = {
    "nominal": "CAD geometry; only the iPad<-base and gripper<-tag transforms fitted",
    "offsets": "+ zero offsets of shoulder_lift, elbow_flex",
    "links": "+ zero offsets (lift, elbow) + placement corrections of links 1-4 (6 each; priors 2°, 2 mm)",
    "full": "links + wrist_flex and wrist_roll offsets + tag scale",
    "stiff_lin": "links + linear gravity compliance on lift, elbow, wrist (Δθ = c·τ, model torques)",
    "stiff_thr": f"links + shoulder_lift compliance above {kincal.TAU_KNEE} N·m only (threshold chosen from these results)",
}


def save_variant(model, v: str, rms: float, res: dict, args, shots) -> None:
    """config/kinematics/follower_<variant>.json: one file per variant (never shared), everything needed to use or
    reproduce it."""
    import subprocess

    out = REPO_ROOT / "config" / "kinematics" / f"follower_{v}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    oj, tw, sc, cm = kincal._spec(v)
    sha = subprocess.run(["git", "rev-parse", "--short", "HEAD"], capture_output=True, text=True, cwd=REPO_ROOT).stdout.strip()
    doc = {"variant": v, "description": DESCRIBE.get(v, ""), "arm": "follower",
           "reference": args.reference, "frame_note": "X = T_ref_base (ref = the iPad ChArUco when reference=ipad); Y = T_gripperbody_tag; mm",
           "tag_mm_nominal": args.tag_mm, "tag_scale": model.tag_scale, "intrinsics": "config/intrinsics_a7iv.json",
           "fitted_on_runs": [r.name for r in args.runs], "tested_on_runs": [r.name for r in args.test], "n_shots": len(shots),
           "git_sha_at_fit": sha,
           "params": {"delta_rad": model.params.delta.tolist(), "delta_deg": np.degrees(model.params.delta).tolist(),
                      "fitted_offset_joints": [CHAIN_JOINTS[i] for i in oj], "xi": model.params.xi.tolist(), "link_twists_fitted": tw,
                      "compliance_rad_per_Nm": model.compliance.tolist(), "compliance_mode": cm, "compliance_joints": [CHAIN_JOINTS[i] for i in kincal.PITCH_JOINTS],
                      "tau_knee_Nm": kincal.TAU_KNEE if cm == "thr" else None},
           "X_ref_base": {k: x.tolist() for k, x in model.X.items()}, "Y_body_tag": model.Y.tolist(),
           "metrics": {k: res[k] for k in res if k not in ("X", "Y", "xi", "delta_deg", "identifiability")}}  # fmt: skip
    out.write_text(json.dumps(doc, indent=1))


def comparison(results: dict, args, console) -> None:
    """Every variant side by side: complexity, fit, cross-validated (inner workspace vs far band) and independent test."""
    t = Table(title=f"variant comparison (relative to the {args.reference}; held-out = {args.folds}-fold by pose group; mm at the tag)", title_justify="left")
    cols = ["variant", "n θ", "fit px", "CV lat", "CV 3D", "CV 3D max", f"CV inner 3D (|τ|≤{kincal.TAU_KNEE})", "CV far 3D", "test lat", "test 3D",
            "test max", "test sub-mm lat"]  # fmt: skip
    for c in cols:
        t.add_column(c, justify="right")
    rows = []
    for v, r in results.items():
        te = r.get("test", {})
        row = [v, str(r["n_theta"]), f"{r['fit_rms_px']:.1f}", f"{r['heldout_lateral_rms_mm']:.2f}", f"{r['heldout_3d_rms_mm']:.2f}",
               f"{r['heldout_3d_max_mm']:.2f}", f"{r['heldout_inner']['rms_3d_mm']:.2f}", f"{r['heldout_far']['rms_3d_mm']:.2f}",
               f"{te['lateral_rms_mm']:.2f}" if te else "", f"{te['rms_3d_mm']:.2f}" if te else "", f"{te['max_3d_mm']:.2f}" if te else "",
               f"{te['sub_mm_lateral']}/{te['n_poses']}" if te else ""]  # fmt: skip
        t.add_row(*row)
        rows.append(row)
    console.print(t)
    md = ["| " + " | ".join(cols) + " |", "|" + "---|" * len(cols)] + ["| " + " | ".join(r) + " |" for r in rows]
    (OUT / "comparison.md").write_text(f"Variants (relative to the {args.reference}; fitted on {', '.join(r.name for r in args.runs)}; "
                                        f"test {', '.join(r.name for r in args.test) or '-'}):\n\n" + "\n".join(md) + "\n\n"
                                        + "\n".join(f"- **{v}**: {DESCRIBE.get(v, '')}" for v in results) + "\n")  # fmt: skip


def plot(shots, held: dict, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cols = {"nominal": "#9a9890", "offsets": "#eb6834", "links": "#2a78d6", "full": "#1f9e6e", "stiff_lin": "#8a5cc2", "stiff_thr": "#d43f3a"}
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.5))
    for v, e in held.items():
        axes[0].plot(np.linalg.norm(e[:, :2], axis=1), "o", ms=3, color=cols.get(v, "#1f9e6e"), label=v)
        axes[1].plot(np.linalg.norm(e, axis=1), "o", ms=3, color=cols.get(v, "#1f9e6e"), label=v)
    axes[0].set_title("held-out lateral error per shot (mm)", loc="left", fontsize=10)
    axes[1].set_title("held-out 3D error per shot (mm; depth includes ~0.1 mm PnP noise)", loc="left", fontsize=10)
    for a in axes:
        a.axhline(1.0, color="#d43f3a", lw=0.8, ls="--")
        a.set_xlabel("shot"), a.legend(fontsize=8), a.grid(color="#e4e3dd", lw=0.6)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
