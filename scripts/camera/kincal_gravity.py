"""Step 10: does the held-out kinematic error track gravity load? Read-only analysis of the error map.

    uv run scripts/camera/kincal_gravity.py data/raw/<run> [...]        # same runs as kincal_error_map.py

Per pose group (data/derived/kincal/error_map.json, variant `links`; relative to the iPad): |error| and its
vertical / camera-depth components, against the joint angles (encoder readings, mean over the group's shots),
the horizontal reach of the tag, and the static gravity torque on each pitch joint from the MuJoCo model's masses
(qfrc_bias at zero velocity, N·m). Spearman rank correlations (robust to the few large points) and scatter plots.
Writes data/derived/kincal/gravity.png + gravity.json.
"""

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import mujoco
import numpy as np
from rich.console import Console
from rich.table import Table
from scipy.stats import spearmanr

from so101.config import REPO_ROOT
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import JOINTS, ArmModel

sys.path.insert(0, str(Path(__file__).parent))
from kinematic_fit import OUT, detections, group_of  # noqa: E402

PITCH = ["shoulder_lift", "elbow_flex", "wrist_flex"]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", type=Path, nargs="+")
    ap.add_argument("--variant", default="links")
    args = ap.parse_args()
    console = Console()
    mapping = load_mapping("follower")
    am = ArmModel(mapping.model_path)
    dof = {j: am.m.jnt_dofadr[mujoco.mj_name2id(am.m, mujoco.mjtObj.mjOBJ_JOINT, j)] for j in JOINTS}

    ticks = defaultdict(list)
    for run in args.runs:
        for r in detections(run, 2, console):
            ticks[group_of(run.name, r["label"])].append(r["ticks"])
    pts = json.loads((OUT / "error_map.json").read_text())[args.variant]
    rows = []
    for p in pts:
        tk = {j: float(np.mean([t[j] for t in ticks[p["group"]]])) for j in JOINTS}
        q = mapping.to_q(tk)
        am.d.qpos[:] = 0
        for j, v in q.items():
            am.d.qpos[am.qadr[j]] = v
        am.d.qvel[:] = 0
        mujoco.mj_forward(am.m, am.d)
        tau = {j: float(am.d.qfrc_bias[dof[j]]) for j in PITCH}  # gravity torque each joint must hold (N·m)
        e = np.array(p["e"])
        rows.append({"label": p["label"], "err": float(np.linalg.norm(e)), "err_z_up": float(e[2]),
                     "deg": {j: float(np.degrees(q[j])) for j in PITCH}, "tau": tau,
                     "reach_mm": float(np.hypot(p["p"][0], p["p"][1])), "height_mm": float(p["p"][2])})  # fmt: skip
    err = np.array([r["err"] for r in rows])
    feats = {f"{j} angle": [r["deg"][j] for r in rows] for j in PITCH}
    feats |= {f"{j} gravity torque": [abs(r["tau"][j]) for r in rows] for j in PITCH}
    feats |= {"tag reach (horizontal)": [r["reach_mm"] for r in rows], "tag height": [r["height_mm"] for r in rows]}
    t = Table(title=f"Spearman correlation with |held-out error| ({args.variant}, {len(rows)} poses, relative to the iPad)", title_justify="left")
    for c in ["quantity", "ρ", "p-value", "range"]:
        t.add_column(c, justify="right")
    out = {}
    for name, x in feats.items():
        rho, pv = spearmanr(x, err)
        out[name] = {"rho": float(rho), "p": float(pv)}
        t.add_row(name, f"{rho:+.2f}", f"{pv:.1g}", f"{min(x):.2f} … {max(x):.2f}")
    console.print(t)
    hi = err >= np.percentile(err, 80)
    console.print(f"worst 20 % (|error| ≥ {np.percentile(err, 80):.2f} mm) vs the rest: |shoulder_lift torque| "
                  f"{np.mean(np.abs([r['tau']['shoulder_lift'] for r, h in zip(rows, hi) if h])):.2f} vs "
                  f"{np.mean(np.abs([r['tau']['shoulder_lift'] for r, h in zip(rows, hi) if not h])):.2f} N·m; reach "
                  f"{np.mean([r['reach_mm'] for r, h in zip(rows, hi) if h]):.0f} vs {np.mean([r['reach_mm'] for r, h in zip(rows, hi) if not h]):.0f} mm")  # fmt: skip
    (OUT / "gravity.json").write_text(json.dumps({"variant": args.variant, "correlations": out, "poses": rows}, indent=1))
    plot(rows, feats, err, OUT / "gravity.png")
    console.print(f"wrote {(OUT / 'gravity.png').relative_to(REPO_ROOT)} + gravity.json")


def plot(rows, feats: dict, err: np.ndarray, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    names = ["shoulder_lift angle", "elbow_flex angle", "wrist_flex angle", "shoulder_lift gravity torque",
             "elbow_flex gravity torque", "tag reach (horizontal)"]  # fmt: skip
    units = ["deg", "deg", "deg", "N·m", "N·m", "mm"]
    fig, axes = plt.subplots(2, 3, figsize=(14, 8), sharey=True)
    for a, n, u in zip(axes.ravel(), names, units):
        x = np.array(feats[n])
        a.scatter(x, err, s=22, color="#2a78d6", edgecolors="white", linewidths=0.5)
        rho, _ = spearmanr(x, err)
        a.set_title(f"{n}  (ρ = {rho:+.2f})", loc="left", fontsize=10)
        a.set_xlabel(u)
        a.axhline(1.0, color="#d43f3a", lw=0.8, ls="--")
        a.grid(color="#e4e3dd", lw=0.6)
    for a in axes[:, 0]:
        a.set_ylabel("|held-out error| at the tag (mm)")
    fig.suptitle("Kinematic error vs joint angle and gravity load (links model, relative to the iPad)", x=0.01, ha="left", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
