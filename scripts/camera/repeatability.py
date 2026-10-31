"""Step 8 analysis: repeatability, settle check and per-joint backlash from a capture run of
config/tours/step8_repeatability.yaml (scripts/sim/plan_step8_tour.py). Read-only.

    uv run scripts/camera/repeatability.py data/raw/<capture_run>

Every shot: tag (edge-refined, linear RAW) + iPad PnP -> tag centre and orientation relative to the iPad
(camera motion cancels), reported along the camera axes of the first shot (x right, y down, z depth).
- Settle check (part A, 2 shots per arrival, first at the settle time, second ~10 s later): mean and sd of
  (late - first). A systematic difference above the noise = the arm was still moving at the first shot.
- Repeatability per target (part A, first shots): per-axis sd, and ISO 9283 RP = mean + 3 sd of the distance
  from the barycentre (3D; depth dominates, so also a lateral-only RP). Also after removing a linear time
  trend (holding/thermal drift, 5b(b)).
- Backlash per joint (part B): mean(plus approach) - mean(minus approach) in µm, and converted to joint ticks
  by projecting onto that joint's predicted image-plane motion (hand-eye model: data/derived/handeye/handeye.json),
  using the lateral (x, y) components only, where the camera is precise. Encoder side: the joint's mean
  reading after each approach (plus - minus), i.e. what the servo itself reports.
Writes data/derived/<run>/repeatability.json + repeatability.png.
"""

import argparse
import json
import re
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.table import Table

from so101.analysis.apriltag import detect_tags
from so101.analysis.charuco import CharucoSpec, detect, load_linear, to_display
from so101.analysis.handeye import HandEye
from so101.analysis.pose import Camera, relative, solve, tag_object_points
from so101.config import REPO_ROOT
from so101.kinematics.fk import CHAIN_JOINTS, chain_from_mujoco, fk
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel
from so101.runlog.manifest import read_manifest

HANDEYE = REPO_ROOT / "data" / "derived" / "handeye" / "handeye.json"
LABEL = re.compile(r"^(?P<part>[AB])_(?P<rest>.+)_r(?P<rep>\d+)(#(?P<shot>\d+))?$")


def measure(path: Path, cam: Camera, spec: CharucoSpec, tag_id: int, tag_mm: float) -> dict | None:
    lin = load_linear(path)
    gray = to_display(lin)
    t = detect_tags(gray, ids=[tag_id], linear=lin).get(tag_id)
    board = spec.board()
    d = detect(gray, board)
    if t is None or d is None or len(d.ids) < 12:
        return None
    obj, img = board.matchImagePoints(d.corners.reshape(-1, 1, 2).astype(np.float32), d.ids.reshape(-1, 1))
    tag = solve(tag_object_points(tag_mm), t.corners, cam, 0.05, square=True)
    ipad = solve(obj.reshape(-1, 3), img.reshape(-1, 2), cam, 0.05)
    return {"rel": relative(tag, ipad), "ipad": ipad, "edge": t.edge_rms}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--tag-id", type=int, default=2)
    ap.add_argument("--tag-mm", type=float, default=20.0)
    args = ap.parse_args()
    console = Console()
    cam, spec = Camera.load(), CharucoSpec.load()
    t0 = None
    shots = []
    for m in read_manifest(args.run_dir / "manifest.jsonl")[1]:
        g = LABEL.match(m["label"])
        arws = [Path(f) for f in (m.get("camera") or {}).get("files", []) if f.upper().endswith(".ARW")]
        if not g or not arws:
            continue
        r = measure(arws[0], cam, spec, args.tag_id, args.tag_mm)
        ts = m["camera"]["t_request_ns"] / 1e9
        t0 = ts if t0 is None else t0
        enc = {j: s["position"] for j, s in m["snapshot_before"]["joints"].items() if s}
        shots.append({"label": m["label"], "part": g["part"], "rest": g["rest"], "rep": int(g["rep"]), "shot": int(g["shot"] or 0),
                      "t_s": ts - t0, "m": r, "enc": enc})  # fmt: skip
        console.print(f"  {m['label']}: {'ok' if r else '[red]no tag/iPad'}", highlight=False)
    ok = [s for s in shots if s["m"]]
    if not ok:
        sys.exit("no measurable shots")
    R_axes = ok[0]["m"]["ipad"].R.T  # camera axes (first shot) in the iPad frame
    for s in ok:
        s["p_um"] = R_axes.T @ s["m"]["rel"].t * 1000  # tag centre, camera axes, µm

    out: dict = {"run": args.run_dir.name, "n_shots": len(shots), "n_measured": len(ok)}
    # --- settle check
    pairs = defaultdict(dict)
    for s in ok:
        if s["part"] == "A":
            pairs[(s["rest"], s["rep"])][s["shot"]] = s["p_um"]
    diffs = np.array([v[1] - v[0] for v in pairs.values() if 0 in v and 1 in v])
    if len(diffs):
        st = Table(title=f"settle check: late shot − first shot ({len(diffs)} arrivals; µm, camera axes)", title_justify="left")
        for c in ["", "x", "y", "z (depth)"]:
            st.add_column(c, justify="right")
        st.add_row("mean", *[f"{v:+.1f}" for v in diffs.mean(0)])
        st.add_row("sd", *[f"{v:.1f}" for v in diffs.std(0, ddof=1)])
        st.add_row("mean / s.e.", *[f"{v:+.1f}" for v in diffs.mean(0) / (diffs.std(0, ddof=1) / np.sqrt(len(diffs)))])
        console.print(st)
        out["settle"] = {"n": len(diffs), "mean_um": diffs.mean(0).tolist(), "sd_um": diffs.std(0, ddof=1).tolist()}
    # --- repeatability (first shots)
    rt = Table(title="repeatability per target (first shots; µm, camera axes)", title_justify="left")
    for c in ["target", "n", "sd x", "sd y", "sd z", "RP 3D", "RP lateral", "detrended sd x/y/z", "drift x/y/z per h"]:
        rt.add_column(c, justify="right")
    out["repeatability"] = {}
    for tgt in sorted({s["rest"] for s in ok if s["part"] == "A"}):
        S = [s for s in ok if s["part"] == "A" and s["rest"] == tgt and s["shot"] == 0]
        P, T = np.array([s["p_um"] for s in S]), np.array([s["t_s"] for s in S])
        if len(P) < 3:
            continue
        d3, dl = np.linalg.norm(P - P.mean(0), axis=1), np.linalg.norm((P - P.mean(0))[:, :2], axis=1)
        rp3, rpl = d3.mean() + 3 * d3.std(ddof=1), dl.mean() + 3 * dl.std(ddof=1)
        A = np.c_[np.ones(len(T)), T]
        coef = np.linalg.lstsq(A, P, rcond=None)[0]
        res = P - A @ coef
        sd, sdd = P.std(0, ddof=1), res.std(0, ddof=2)
        rt.add_row(tgt, str(len(P)), *[f"{v:.1f}" for v in sd], f"{rp3:.0f}", f"{rpl:.0f}", " / ".join(f"{v:.1f}" for v in sdd),
                   " / ".join(f"{v * 3600:+.0f}" for v in coef[1]))  # fmt: skip
        out["repeatability"][tgt] = {"n": len(P), "sd_um": sd.tolist(), "rp3d_um": rp3, "rp_lateral_um": rpl,
                                     "detrended_sd_um": sdd.tolist(), "drift_um_per_h": (coef[1] * 3600).tolist()}  # fmt: skip
    console.print(rt)
    # --- backlash
    he = None
    if HANDEYE.exists():
        d = json.loads(HANDEYE.read_text())
        he = HandEye(np.array(d["T_cam_base"]), np.array(d["T_link_tag"]), d["rms_px"], np.array(d["per_pose_px"]), d["tag_mm"])
    mapping = load_mapping("follower")
    chain = chain_from_mujoco(ArmModel(mapping.model_path).m)
    bt = Table(title="backlash per joint at the backlash pose: mean(plus) − mean(minus)", title_justify="left")
    for c in ["joint", "n ±", "Δ x / y / z µm", "± s.e. x / y", "camera → ticks", "encoder Δ ticks"]:
        bt.add_column(c, justify="right")
    out["backlash"] = {}
    for j in ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]:
        plus = [s for s in ok if s["part"] == "B" and s["rest"] == f"{j}_plus"]
        minus = [s for s in ok if s["part"] == "B" and s["rest"] == f"{j}_minus"]
        if len(plus) < 2 or len(minus) < 2:
            continue
        Pp, Pm = np.array([s["p_um"] for s in plus]), np.array([s["p_um"] for s in minus])
        dlt = Pp.mean(0) - Pm.mean(0)
        se = np.sqrt(Pp.var(0, ddof=1) / len(Pp) + Pm.var(0, ddof=1) / len(Pm))
        enc = np.mean([s["enc"][j] for s in plus]) - np.mean([s["enc"][j] for s in minus])
        ticks = None
        if he is not None:  # image-plane motion of the tag per tick of joint j at this pose (camera axes, µm)
            base = {k: np.mean([s["enc"][k] for s in plus + minus]) for k in CHAIN_JOINTS}

            def tag_cam(tk):
                q = np.array([mapping.joints[k].to_rad(tk[k]) for k in CHAIN_JOINTS])
                F = fk(chain, q, tcp=np.eye(4))
                F[:3, 3] *= 1000
                return he.T_cam_tag(F)[:3, 3] * 1000  # µm, camera frame

            # reporting axes = the camera frame (R_axes.T maps iPad-frame vectors to it), so no rotation needed
            g = (tag_cam({**base, j: base[j] + 5}) - tag_cam({**base, j: base[j] - 5})) / 10  # µm per tick
            ticks = float(dlt[:2] @ g[:2] / (g[:2] @ g[:2]))
        bt.add_row(j, f"{len(plus)}/{len(minus)}", " / ".join(f"{v:+.1f}" for v in dlt), " / ".join(f"{v:.1f}" for v in se[:2]),
                   "" if ticks is None else f"{ticks:+.2f}", f"{enc:+.1f}")  # fmt: skip
        out["backlash"][j] = {"delta_um": dlt.tolist(), "se_um": se.tolist(), "camera_ticks": ticks, "encoder_delta_ticks": enc}
    console.print(bt)
    od = REPO_ROOT / "data" / "derived" / args.run_dir.name
    od.mkdir(parents=True, exist_ok=True)
    out["shots"] = [{"label": s["label"], "t_s": s["t_s"], "p_um": s["p_um"].tolist() if s["m"] else None,
                     "edge": s["m"]["edge"] if s["m"] else None, "enc": s["enc"]} for s in shots]  # fmt: skip
    (od / "repeatability.json").write_text(json.dumps(out, indent=1))
    plot(ok, od / "repeatability.png")
    console.print(f"wrote {(od / 'repeatability.json').relative_to(REPO_ROOT)} + repeatability.png")


def plot(ok: list[dict], path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cols = ["#2a78d6", "#eb6834", "#1f9e6e", "#8a5cc2"]
    tg = sorted({s["rest"] for s in ok if s["part"] == "A"})
    js = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]
    fig, axes = plt.subplots(2, 2, figsize=(12, 9))
    ax = axes[0, 0]
    for c, t in zip(cols, tg):
        P = np.array([s["p_um"] for s in ok if s["part"] == "A" and s["rest"] == t and s["shot"] == 0])
        if len(P):
            ax.scatter(P[:, 0] - P[:, 0].mean(), P[:, 1] - P[:, 1].mean(), s=25, color=c, label=t)
    ax.set_title("repeatability, lateral (µm about each target's mean)", loc="left", fontsize=10)
    ax.set_xlabel("x (µm)"), ax.set_ylabel("y (µm, down)"), ax.set_aspect("equal", adjustable="datalim"), ax.legend(fontsize=8)
    ax = axes[0, 1]
    for c, t in zip(cols, tg):
        S = [s for s in ok if s["part"] == "A" and s["rest"] == t and s["shot"] == 0]
        if S:
            P = np.array([s["p_um"] for s in S])
            ax.plot([s["t_s"] / 60 for s in S], P[:, 1] - P[:, 1].mean(), "-o", ms=3, color=c, label=t)
    ax.set_title("repeatability vs time, y (µm): drift shows here", loc="left", fontsize=10)
    ax.set_xlabel("time (min)"), ax.legend(fontsize=8)
    ax = axes[1, 0]
    pairs = defaultdict(dict)
    for s in ok:
        if s["part"] == "A":
            pairs[(s["rest"], s["rep"])][s["shot"]] = s["p_um"]
    D = np.array([v[1] - v[0] for v in pairs.values() if 0 in v and 1 in v]).reshape(-1, 3)
    for k, (n, c) in enumerate(zip(["x", "y", "z (depth)"], cols)):
        ax.scatter(np.full(len(D), k) + np.random.default_rng(k).uniform(-0.15, 0.15, len(D)), D[:, k], s=18, color=c)
    ax.axhline(0, color="#6b6a63", lw=0.6)
    ax.set_xticks(range(3), ["x", "y", "z (depth)"]), ax.set_title("settle check: late − first shot (µm)", loc="left", fontsize=10)
    ax = axes[1, 1]
    for k, (j, c) in enumerate(zip(js, cols)):
        for sign, mk in (("plus", "o"), ("minus", "s")):
            P = np.array([s["p_um"] for s in ok if s["part"] == "B" and s["rest"] == f"{j}_{sign}"]).reshape(-1, 3)
            if len(P):
                ax.scatter(np.full(len(P), k) + (0.12 if sign == "plus" else -0.12), P[:, 1] - np.mean([s["p_um"][1] for s in ok if s["part"] == "B"]),
                           marker=mk, s=22, color=c, label=sign if k == 0 else None)  # fmt: skip
    ax.set_xticks(range(4), [j.replace("_", "\n") for j in js]), ax.legend(fontsize=8, title="approach from")
    ax.set_title("backlash: y at the backlash pose by approach (µm)", loc="left", fontsize=10)
    for a in axes.ravel():
        a.grid(color="#e4e3dd", lw=0.6)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
