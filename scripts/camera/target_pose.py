"""Step 5b: the gripper tag's 6-DoF pose relative to the iPad over time, from an existing run's RAW images.

    uv run scripts/camera/stability_check.py --board apriltag --tag-ids 2 -n 21 --interval 90   # capture (30 min)
    uv run scripts/camera/target_pose.py data/raw/<run_id>                                       # this script
    uv run scripts/camera/target_pose.py data/raw/<run_id> --tag-id 2 --tag-mm 20 --sigma-px 0.05

Read-only: never touches the arm or the camera. Per image: AprilTag corners (edge-line refined on the linear
image), iPad ChArUco corners, PnP for each with the step 4 intrinsics (config/intrinsics_a7iv.json), and the
relative pose T_ipad_tag (camera motion cancels). Changes vs image 0 are reported along the CAMERA axes
(x right, y down, z = depth along the line of sight, using image 0's iPad orientation), where precision is
anisotropic: lateral ~µm, depth ~f/size worse. "pred" = predicted 1σ from corner noise --sigma-px (first-order
covariance, intrinsics fixed); compare it with the observed scatter: observed ≫ pred means real motion or
noise the model misses. Scale: tag and iPad sizes are nominal until step 3 (fine for stability: a common scale
error scales the changes by the same fraction). Without an iPad in view, the tag's pose in the camera frame
is reported instead (camera motion then included). Corners below image row 3700 (uncalibrated) are flagged.
Writes data/derived/<run_id>/target_pose.json + target_pose.png.
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
from rich.console import Console
from rich.table import Table

from so101.analysis.apriltag import detect_tags
from so101.analysis.charuco import CharucoSpec, detect, load_linear, to_display
from so101.analysis.pose import CALIBRATED_MAX_Y, INTRINSICS, Camera, Pose, relative, rotation_delta, solve, tag_object_points
from so101.config import REPO_ROOT
from so101.runlog.manifest import capture_times

AS = np.degrees(1) * 3600  # rad -> arcsec


def measure(path: Path, cam: Camera, tag_id: int, tag_mm: float, spec: CharucoSpec, sigma: float) -> dict:
    lin = load_linear(path)
    gray = to_display(lin)
    out: dict = {"image": path.name}
    tags = detect_tags(gray, ids=[tag_id], linear=lin)
    if tag_id in tags:
        t = tags[tag_id]
        out["tag"] = solve(tag_object_points(tag_mm), t.corners, cam, sigma, square=True)
        out["tag_edge_rms"] = t.edge_rms
        out["tag_uncal"] = bool((t.corners[:, 1] > CALIBRATED_MAX_Y).any())
    board = spec.board()
    d = detect(gray, board)
    if d is not None and len(d.ids) >= 12:
        obj, img = board.matchImagePoints(d.corners.reshape(-1, 1, 2).astype(np.float32), d.ids.reshape(-1, 1))
        out["ipad"] = solve(obj.reshape(-1, 3), img.reshape(-1, 2), cam, sigma)
        out["ipad_n"] = len(d.ids)
        out["ipad_uncal"] = bool((d.corners[:, 1] > CALIBRATED_MAX_Y).any())
    if "tag" in out and "ipad" in out:
        out["rel"] = relative(out["tag"], out["ipad"])
    return out


def series(rows: list[dict], key: str, R_axes: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """Per image: Δposition (µm) and Δrotation (arcsec) vs the first image, and predicted 1σ of both (6,),
    all along the axes R_axes (columns = camera x, y, z expressed in the pose's reference frame)."""
    first: Pose = next(r[key] for r in rows if key in r)
    dp, dr, sd = [], [], []
    for r in rows:
        if key not in r:
            dp.append([np.nan] * 3), dr.append([np.nan] * 3), sd.append([np.nan] * 6)
            continue
        p: Pose = r[key]
        dp.append(R_axes.T @ (p.t - first.t) * 1000)
        dr.append(R_axes.T @ (first.R @ rotation_delta(first.R, p.R)) * AS)  # rotation vector in the reference frame
        A = np.zeros((6, 6))
        A[:3, :3] = R_axes.T @ (p.R if key == "rel" else _rvec_jac(p))  # σ of the rotation, same axes
        A[3:, 3:] = R_axes.T
        C = A @ p.cov @ A.T
        sd.append(np.sqrt(np.diag(C)) * np.r_[[AS] * 3, [1000] * 3])
    return np.array(dp), np.array(dr), np.array(sd)


def _rvec_jac(p: Pose, eps: float = 1e-7) -> np.ndarray:
    """d(rotation vector in the reference frame)/d(rvec) at p: maps solvePnP's rvec covariance to angles."""
    J = np.zeros((3, 3))
    for k in range(3):
        d = np.zeros(3)
        d[k] = eps
        Rp, Rm = cv2.Rodrigues(p.rvec + d)[0], cv2.Rodrigues(p.rvec - d)[0]
        J[:, k] = p.R @ (rotation_delta(p.R, Rp) - rotation_delta(p.R, Rm)) / (2 * eps)
    return J


def trend(t: np.ndarray, y: np.ndarray, min_span_s: float = 600) -> tuple[float, float]:
    """(slope per hour, scatter about the line) over finite samples; NaN for runs shorter than min_span_s
    (a per-hour slope from a 2-min run is noise scaled up 30x)."""
    ok = np.isfinite(y) & np.isfinite(t)
    if ok.sum() < 3 or np.ptp(t[ok]) < min_span_s:
        return np.nan, np.nan
    c = np.polyfit(t[ok], y[ok], 1)
    return float(c[0] * 3600), float(np.std(y[ok] - np.polyval(c, t[ok]), ddof=2))


def plot(t_min: np.ndarray, dp: np.ndarray, dr: np.ndarray, sd: np.ndarray, rows: list[dict], key: str, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    blue, orange, green, ink, muted = "#2a78d6", "#eb6834", "#1f9e6e", "#1f1f1d", "#6b6a63"
    what = "tag relative to iPad" if key == "rel" else "tag in the camera frame (includes camera motion)"
    tag_rms = np.array([r["tag"].rms_px if "tag" in r else np.nan for r in rows])
    ipad_rms = np.array([r["ipad"].rms_px if "ipad" in r else np.nan for r in rows])
    panels = [(f"{what}: Δ position, lateral (µm; band = predicted ±1σ)", [("x", dp[:, 0], sd[:, 3], blue), ("y", dp[:, 1], sd[:, 4], orange)]),
              (f"{what}: Δ position, depth = camera z (µm)", [("z", dp[:, 2], sd[:, 5], green)]),
              (f"{what}: Δ rotation (arcsec, about camera axes)", [("x", dr[:, 0], sd[:, 0], blue), ("y", dr[:, 1], sd[:, 1], orange), ("z", dr[:, 2], sd[:, 2], green)]),
              ("reprojection RMS (px)", [("tag", tag_rms, None, blue), ("iPad", ipad_rms, None, orange)])]  # fmt: skip
    fig, axes = plt.subplots(len(panels), 1, figsize=(11, 2.6 * len(panels)), sharex=True)
    for a, (title, ser) in zip(axes, panels):
        for name, y, s, c in ser:
            if not np.isfinite(y).any():
                continue
            if s is not None:
                a.fill_between(t_min, -s, s, color=c, alpha=0.12, lw=0)
            a.plot(t_min, y, "-o", color=c, lw=1.5, ms=3)
            ok = np.isfinite(y)
            a.annotate(name, (t_min[ok][-1], y[ok][-1]), xytext=(5, 0), textcoords="offset points", color=ink, va="center")
        a.set_title(title, loc="left", color=ink, fontsize=10)
        a.grid(color="#e4e3dd", lw=0.6)
        a.axhline(0, color=muted, lw=0.6)
        a.tick_params(colors=muted)
        for k in ("top", "right"):
            a.spines[k].set_visible(False)
    axes[-1].set_xlabel("time (min)", color=muted)
    fig.tight_layout()
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(path, dpi=110)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run_dir", type=Path)
    ap.add_argument("--tag-id", type=int, default=2)
    ap.add_argument("--tag-mm", type=float, default=20.0, help="outer edge of the black border (nominal until step 3)")
    ap.add_argument("--target", default="ipad_charuco", help="iPad ChArUco spec in config/targets/")
    ap.add_argument("--intrinsics", type=Path, default=INTRINSICS)
    ap.add_argument("--sigma-px", type=float, default=0.05, help="corner noise for the predicted 1σ (steps 5/5b: 0.04-0.06)")
    args = ap.parse_args()
    console = Console()
    files = sorted((args.run_dir / "images").glob("*.ARW"))
    if not files:
        sys.exit(f"no .ARW files in {args.run_dir}/images")
    cam, spec = Camera.load(args.intrinsics), CharucoSpec.load(args.target)
    rows = []
    for k, f in enumerate(files):
        rows.append(measure(f, cam, args.tag_id, args.tag_mm, spec, args.sigma_px))
        console.print(f"{k + 1}/{len(files)} {f.name}: tag {'ok' if 'tag' in rows[-1] else '[red]none[/]'}, "
                      f"iPad {rows[-1].get('ipad_n', '[yellow]none[/]')}", highlight=False)  # fmt: skip
    if not any("tag" in r for r in rows):
        sys.exit(f"tag {args.tag_id} not found in any image")
    key = "rel" if any("rel" in r for r in rows) else "tag"
    if key == "tag":
        console.print("[yellow]no iPad in view: reporting the tag in the camera frame (camera motion included)")
    if any(r.get("tag_uncal") or r.get("ipad_uncal") for r in rows):
        console.print(f"[yellow]corners below image row {CALIBRATED_MAX_Y} (outside the calibrated region) in some images")
    # camera axes in the reference frame: the iPad's (rel) or the camera's own (tag)
    ref0 = next(r for r in rows if key in r)
    R_axes = ref0["ipad"].R.T if key == "rel" else np.eye(3)
    dp, dr, sd = series(rows, key, R_axes)

    times = capture_times(args.run_dir)
    t_s = np.array([times.get(r["image"], np.nan) for r in rows])
    tbl = Table(title=f"{'tag rel. iPad' if key == 'rel' else 'tag in camera'}, Δ vs image 0 along camera axes "
                      f"(x right, y down, z depth); µm and arcsec", title_justify="left")  # fmt: skip
    for c in ["#", "t min", "dx", "dy", "dz", "rx", "ry", "rz", "tag px", "amb", "edge", "iPad n", "iPad px"]:
        tbl.add_column(c, justify="right")
    for k, r in enumerate(rows):
        if key not in r:
            tbl.add_row(str(k), f"{t_s[k] / 60:.1f}", "[red]missing", *[""] * 10)
            continue
        tg, ip = r.get("tag"), r.get("ipad")
        tbl.add_row(str(k), f"{t_s[k] / 60:.1f}", *[f"{v:+.1f}" for v in dp[k]], *[f"{v:+.1f}" for v in dr[k]],
                    f"{tg.rms_px:.3f}", f"{tg.ambiguity:.2f}", f"{r['tag_edge_rms']:.3f}",
                    str(r.get("ipad_n", "")), f"{ip.rms_px:.3f}" if ip else "")  # fmt: skip
    console.print(tbl)

    names = ["rx", "ry", "rz", "dx", "dy", "dz"]
    vals = np.hstack([dr, dp])
    summ = Table(title="summary: observed vs predicted (sigma_px " f"{args.sigma_px})", title_justify="left")
    for c in ["axis", "unit", "observed sd", "pred 1σ (median)", "trend /h", "scatter about trend"]:
        summ.add_column(c, justify="right")
    summary = {}
    for j, n in enumerate(names):
        y = vals[:, j]
        slope, scat = trend(t_s, y)
        pred = float(np.nanmedian(sd[:, j]))
        unit = "″" if n.startswith("r") else "µm"
        summary[n] = {"observed_sd": float(np.nanstd(y, ddof=1)), "pred_sigma": pred, "trend_per_h": slope, "scatter": scat}
        summ.add_row(n, unit, f"{summary[n]['observed_sd']:.1f}", f"{pred:.1f}", *(("—", "—") if np.isnan(slope) else (f"{slope:+.1f}", f"{scat:.1f}")))
    console.print(summ)

    out_dir = REPO_ROOT / "data" / "derived" / args.run_dir.name
    out_dir.mkdir(parents=True, exist_ok=True)
    pose = lambda p: None if p is None else {"rvec": p.rvec.tolist(), "t_mm": p.t.tolist(), "cov": p.cov.tolist(), "rms_px": p.rms_px,  # noqa: E731
                                            "ambiguity": p.ambiguity}  # fmt: skip
    result = {"run": args.run_dir.name, "mode": key, "tag_id": args.tag_id, "tag_mm": args.tag_mm, "target": args.target,
              "intrinsics": str(args.intrinsics), "sigma_px": args.sigma_px, "axes": "camera (x right, y down, z depth)",
              "summary": summary,
              "images": [{"image": r["image"], "t_s": float(t_s[k]), "tag": pose(r.get("tag")), "ipad": pose(r.get("ipad")),
                          "rel": pose(r.get("rel")), "ipad_n": r.get("ipad_n"), "tag_edge_rms": r.get("tag_edge_rms"),
                          "uncalibrated": bool(r.get("tag_uncal") or r.get("ipad_uncal")),
                          "d_um": dp[k].tolist(), "d_arcsec": dr[k].tolist(), "pred_sigma": sd[k].tolist()}
                         for k, r in enumerate(rows)]}  # fmt: skip
    (out_dir / "target_pose.json").write_text(json.dumps(result, indent=1, default=float))
    plot(t_s / 60, dp, dr, sd, rows, key, out_dir / "target_pose.png")
    console.print(f"wrote {(out_dir / 'target_pose.json').relative_to(REPO_ROOT)} + target_pose.png")


if __name__ == "__main__":
    main()
