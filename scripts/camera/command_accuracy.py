"""Step 10b analysis: where did the tag land vs where it was told to go, per command method. Read-only.

    uv run scripts/camera/command_accuracy.py data/raw/<step10b_capture_run>

Every shot with step 10b metadata (manifest extra.meta: target id, method, target_ipad_mm): tag (edge-refined) and
iPad PnP in the same photo -> tag centre RELATIVE TO THE iPad; error = measured − target, rotated into the camera
axes of that photo (x right, y down, z depth) so lateral and depth separate. Mean of each arrival's shots. Per method:
RMS per axis, lateral and 3D RMS, median, max, sub-mm counts; paired per-target differences vs nominal; encoder
correction offsets (cal_corr). Writes data/derived/<run>/command_accuracy.json + .png.
"""

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.table import Table

from so101.analysis.apriltag import detect_tags
from so101.analysis.charuco import CharucoSpec, detect, load_linear, to_display
from so101.analysis.pose import Camera, solve, tag_object_points
from so101.config import REPO_ROOT
from so101.runlog.manifest import read_manifest

METHODS = ["nominal", "cal", "cal_corr"]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    args = ap.parse_args()
    console = Console()
    cam, spec, obj = Camera.load(), CharucoSpec.load(), tag_object_points(20.0)
    board = spec.board()
    arr = defaultdict(list)  # (target, method) -> [error camera axes]
    corr = []
    for m in read_manifest(args.run_dir / "manifest.jsonl")[1]:
        meta = (m.get("extra") or {}).get("meta") or {}
        arws = [Path(f) for f in (m.get("camera") or {}).get("files", []) if f.upper().endswith(".ARW")]
        if "method" not in meta or not arws:
            continue
        lin = load_linear(arws[0])
        gray = to_display(lin)
        t = detect_tags(gray, ids=[2], linear=lin).get(2)
        d = detect(gray, board)
        if t is None or d is None or len(d.ids) < 12:
            console.print(f"[yellow]{m['label']}: tag or iPad not found")
            continue
        o, i = board.matchImagePoints(d.corners.reshape(-1, 1, 2).astype(np.float32), d.ids.reshape(-1, 1))
        Ti = solve(o.reshape(-1, 3), i.reshape(-1, 2), cam, 0.05).matrix()
        Tt = solve(obj, t.corners, cam, 0.05, square=True).matrix()
        p = (np.linalg.inv(Ti) @ Tt)[:3, 3]
        e_cam = Ti[:3, :3] @ (p - np.array(meta["target_ipad_mm"]))
        arr[(meta["target"], meta["method"])].append(e_cam)
        c = (m.get("settle") or {}).get("correction")
        if c and m["label"].endswith("#0"):
            corr.append(c)
        console.print(f"  {m['label']}: |e| {np.linalg.norm(e_cam):.2f} mm", highlight=False)
    if not arr:
        sys.exit("no step 10b shots measured")
    E = {mth: {tg: np.mean(v, axis=0) for (tg, mm), v in arr.items() if mm == mth} for mth in METHODS}
    t = Table(title=f"command accuracy: measured − target tag position, relative to the iPad (mm, camera axes; mean of each arrival's shots)", title_justify="left")
    for c in ["method", "n", "RMS x", "RMS y", "RMS z", "lateral RMS", "3D RMS", "3D median", "3D max", "sub-mm lat", "sub-mm 3D"]:
        t.add_column(c, justify="right")
    out = {"run": args.run_dir.name, "methods": {}}
    for mth in METHODS:
        if not E[mth]:
            continue
        X = np.array(list(E[mth].values()))
        lat, d3 = np.linalg.norm(X[:, :2], axis=1), np.linalg.norm(X, axis=1)
        t.add_row(mth, str(len(X)), *[f"{v:.2f}" for v in np.sqrt(np.mean(X**2, axis=0))], f"{np.sqrt(np.mean(lat**2)):.2f}",
                  f"{np.sqrt(np.mean(d3**2)):.2f}", f"{np.median(d3):.2f}", f"{d3.max():.2f}", f"{(lat < 1).sum()}/{len(X)}", f"{(d3 < 1).sum()}/{len(X)}")  # fmt: skip
        out["methods"][mth] = {"n": len(X), "rms_mm": np.sqrt(np.mean(X**2, axis=0)).tolist(), "lateral_rms_mm": float(np.sqrt(np.mean(lat**2))),
                               "rms_3d_mm": float(np.sqrt(np.mean(d3**2))), "median_3d_mm": float(np.median(d3)), "max_3d_mm": float(d3.max()),
                               "errors_mm": {int(k): v.tolist() for k, v in E[mth].items()}}  # fmt: skip
    console.print(t)
    common = sorted(set.intersection(*[set(E[m]) for m in METHODS if E[m]]))
    if common:
        pt = Table(title=f"paired, {len(common)} targets reached by every method: 3D error (mm)", title_justify="left")
        for c in ["method", "mean", "vs nominal: mean Δ", "better on"]:
            pt.add_column(c, justify="right")
        base = np.array([np.linalg.norm(E["nominal"][k]) for k in common])
        for mth in METHODS:
            x = np.array([np.linalg.norm(E[mth][k]) for k in common])
            pt.add_row(mth, f"{x.mean():.2f}", f"{(x - base).mean():+.2f}", f"{(x < base).sum()}/{len(common)}" if mth != "nominal" else "")
        console.print(pt)
    if corr:
        applied = [c for c in corr if c["applied"]]
        offs = np.array([[abs(v) for v in c["offset_ticks"].values()] for c in applied]) if applied else np.zeros((0, 1))
        console.print(f"encoder correction: applied {len(applied)}/{len(corr)}; |offset| per joint mean {offs.mean(0).round(1).tolist() if len(offs) else '-'} ticks, "
                      f"max {offs.max() if len(offs) else '-'}")  # fmt: skip
        out["corrections"] = corr
    od = REPO_ROOT / "data" / "derived" / args.run_dir.name
    od.mkdir(parents=True, exist_ok=True)
    (od / "command_accuracy.json").write_text(json.dumps(out, indent=1))
    plot(E, od / "command_accuracy.png")
    console.print(f"wrote {(od / 'command_accuracy.json').relative_to(REPO_ROOT)} + .png")


def plot(E: dict, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cols = {"nominal": "#9a9890", "cal": "#2a78d6", "cal_corr": "#1f9e6e"}
    fig, axes = plt.subplots(1, 2, figsize=(13, 4.8))
    for k, mth in enumerate(METHODS):
        if not E[mth]:
            continue
        X = np.array(list(E[mth].values()))
        axes[0].scatter(X[:, 0], X[:, 1], s=30, color=cols[mth], label=mth, edgecolors="white", linewidths=0.5)
        d3 = np.linalg.norm(X, axis=1)
        axes[1].scatter(np.full(len(d3), k) + np.random.default_rng(k).uniform(-0.12, 0.12, len(d3)), d3, s=30, color=cols[mth])
    axes[0].add_patch(plt.Circle((0, 0), 1.0, fill=False, color="#d43f3a", ls="--", lw=0.8))
    axes[0].set_aspect("equal"), axes[0].axhline(0, color="#6b6a63", lw=0.5), axes[0].axvline(0, color="#6b6a63", lw=0.5)
    axes[0].set_xlabel("x error (mm)"), axes[0].set_ylabel("y error (mm, down)"), axes[0].legend(fontsize=8)
    axes[0].set_title("lateral landing error per target (circle: 1 mm)", loc="left", fontsize=10)
    axes[1].set_xticks(range(3), METHODS), axes[1].axhline(1.0, color="#d43f3a", lw=0.8, ls="--")
    axes[1].set_title("3D landing error per target (mm)", loc="left", fontsize=10)
    for a in axes:
        a.grid(color="#e4e3dd", lw=0.6)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
