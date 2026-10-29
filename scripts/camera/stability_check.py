"""Image stability sanity check: N captures of a static board, corner drift in pixels vs the first image.

    uv run scripts/camera/stability_check.py                    # 12 captures, 5 s apart (~6 s: capture takes 5.9 s), iPad ChArUco
    uv run scripts/camera/stability_check.py --board checker    # plain 9x6 checkerboard (--cols/--rows)
    uv run scripts/camera/stability_check.py -n 41 --interval 90        # 1 h (step 5 rig stability)
    uv run scripts/camera/stability_check.py --analyze data/raw/<run_id>   # re-analyze an existing run's images

Does not touch the arm. Nothing may move between captures (camera, tripod, iPad, table).
ChArUco corners are matched by ID, so a partly hidden board still works (only corners seen in both images count).
Per image vs image 0: mean shift (dx, dy) = whole-scene motion (tripod/iPad/table); a similarity fit splits off
in-plane rotation (arcsec; camera roll on its head shows here) and scale (ppm; distance change); the residual
after the similarity fit = corner-detection noise. "bg" = median shift of static-scene tiles outside the board
(moves with the camera); board−bg = the board moving within the scene.
Also writes data/derived/<run_id>/stability.png (everything vs time). Long runs: a failed capture is logged and
skipped; 3 failures in a row stop the run (camera asleep or disconnected). Writes stability.json into the run directory.
"""

import argparse
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np
import rawpy
from rich.console import Console
from rich.table import Table

from so101.analysis.charuco import CharucoSpec, detect, load_gray
from so101.config import REPO_ROOT
from so101.runlog.manifest import Manifest
from so101.runlog.run import Run
from so101.sensors import Gphoto2Camera, MockCamera


def corners(path: Path, board, pattern: tuple[int, int]) -> dict[int, np.ndarray] | None:
    """Corner id -> (x, y) full-resolution pixels from a RAW file, no lens corrections.
    board = a ChArUco board (ids from the markers) or None for a plain checkerboard (ids = corner index)."""
    if board is not None:
        d = detect(load_gray(path), board)
        return None if d is None else dict(zip(d.ids.tolist(), d.corners))
    with rawpy.imread(str(path)) as r:
        rgb = r.postprocess(no_auto_bright=True, use_camera_wb=True, output_bps=8, gamma=(2.222, 4.5))
    gray = cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)
    ok, c = cv2.findChessboardCornersSB(gray, pattern, flags=cv2.CALIB_CB_EXHAUSTIVE | cv2.CALIB_CB_ACCURACY)
    return dict(enumerate(c.reshape(-1, 2).astype(float))) if ok else None


def gray(path: Path) -> np.ndarray:
    if path.suffix.upper() == ".ARW":
        return load_gray(path)
    return cv2.imread(str(path), cv2.IMREAD_GRAYSCALE)


def background_shifts(files: list[Path], board_xy: np.ndarray, tile: int = 512, min_peak: float = 0.3) -> np.ndarray | None:
    """Median (dx, dy) of static-scene tiles vs image 0 (phase correlation), board area excluded.
    The background moves with the camera; board minus background = the target moving in the scene."""
    g0 = gray(files[0]).astype(np.float32)
    H, W = g0.shape
    (x0, y0), (x1, y1) = board_xy.min(0) - 200, board_xy.max(0) + 200
    y1 += y1 - y0  # also skip the board's mirror image on a glossy table below it (it moves with the board)
    tiles = [(x, y) for y in range(0, H - tile, tile) for x in range(0, W - tile, tile)
             if x + tile < x0 or x > x1 or y + tile < y0 or y > y1]  # fmt: skip
    win = cv2.createHanningWindow((tile, tile), cv2.CV_32F)
    imgs = [g0] + [gray(f).astype(np.float32) for f in files[1:]]
    per_tile = []
    for x, y in tiles:
        a = g0[y : y + tile, x : x + tile]
        if a.std() < 5:  # featureless
            continue
        v = [cv2.phaseCorrelate(a, g[y : y + tile, x : x + tile], win) for g in imgs[1:]]
        if np.mean([r for _, r in v]) >= min_peak:
            per_tile.append([(0.0, 0.0)] + [(-dx, -dy) for (dx, dy), _ in v])
    return np.median(np.array(per_tile), axis=0) if per_tile else None


def analyze(files: list[Path], board, pattern: tuple[int, int], console: Console) -> dict:
    pts = [corners(f, board, pattern) for f in files]
    if pts[0] is None:
        sys.exit(f"no board found in {files[0].name}")
    rows = []
    t = Table(title=f"drift vs image 0 (px; 1 px ≈ 0.14 mm × distance in m). resid = after shift+rot+scale; bg = static scene (camera); rel = board − bg", title_justify="left")
    bg = background_shifts(files, np.array(list(pts[0].values())))
    if bg is None:
        console.print("[yellow]no trackable static background (scene too dark or featureless): camera vs board motion "
                      "can't be separated in this run. Light the static scene (not the board) next time.")  # fmt: skip
    for col in ["#", "n", "dx", "dy", "rot ″", "ppm", "resid", "bg dx", "bg dy", "rel dx", "rel dy"]:
        t.add_column(col, justify="right")
    for k, (f, p) in enumerate(zip(files, pts)):
        if p is None:
            rows.append({"image": f.name, "found": False})
            t.add_row(str(k), "[red]none", *[""] * 9)
            continue
        ids = sorted(set(p) & set(pts[0]))
        d = np.array([p[i] - pts[0][i] for i in ids])
        shift = d.mean(axis=0)
        a = np.array([pts[0][i] for i in ids])
        M, _ = cv2.estimateAffinePartial2D(a, a + d, method=cv2.LMEDS)  # rotation + scale + shift
        rot_as = float(np.degrees(np.arctan2(M[1, 0], M[0, 0])) * 3600)
        scale_ppm = float((np.hypot(M[0, 0], M[1, 0]) - 1) * 1e6)
        res = np.linalg.norm(a + d - (a @ M[:, :2].T + M[:, 2]), axis=1)
        r = {"image": f.name, "found": True, "n_common": len(ids), "dx": float(shift[0]), "dy": float(shift[1]),
             "shift": float(np.hypot(*shift)), "rot_arcsec": rot_as, "scale_ppm": scale_ppm, "residual_rms": float(np.sqrt((res**2).mean())), "residual_max": float(res.max())}  # fmt: skip
        b = bg[k] if bg is not None else (np.nan, np.nan)
        r.update(bg_dx=float(b[0]), bg_dy=float(b[1]), rel_dx=float(shift[0] - b[0]), rel_dy=float(shift[1] - b[1]))
        rows.append(r)
        t.add_row(str(k), str(len(ids)), f"{r['dx']:+.3f}", f"{r['dy']:+.3f}", f"{rot_as:+.1f}", f"{scale_ppm:+.1f}", f"{r['residual_rms']:.3f}",
                  f"{b[0]:+.3f}", f"{b[1]:+.3f}", f"{r['rel_dx']:+.3f}", f"{r['rel_dy']:+.3f}")
    console.print(t)
    return {"board": "charuco" if board is not None else f"checker {pattern[0]}x{pattern[1]}", "n_corners": len(pts[0]), "images": rows}


def capture_times(run_dir: Path) -> dict[str, float]:
    """image name -> seconds since the first capture request (from the manifest)."""
    t = {}
    for line in open(run_dir / "manifest.jsonl"):
        m = json.loads(line)
        if m.get("type") == "pose" and m["camera"]["ok"]:
            for f in m["camera"]["files"]:
                t[Path(f).name] = m["camera"]["t_request_ns"]
    t0 = min(t.values())
    return {k: (v - t0) / 1e9 for k, v in t.items()}


def plot(result: dict, times: dict[str, float], path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    rows = [r for r in result["images"] if r.get("found") and r["image"] in times]
    t = np.array([times[r["image"]] for r in rows]) / 60
    g = lambda k: np.array([r.get(k, np.nan) for r in rows])  # noqa: E731
    blue, orange, ink, muted = "#2a78d6", "#eb6834", "#1f1f1d", "#6b6a63"
    panels = [("camera: static background shift (px)", [("dx", g("bg_dx"), blue), ("dy", g("bg_dy"), orange)]),
              ("board relative to background (px)", [("dx", g("rel_dx"), blue), ("dy", g("rel_dy"), orange)]),
              ("board in-plane rotation (arcsec)", [(None, g("rot_arcsec"), blue)]),
              ("board scale (ppm; + = closer)", [(None, g("scale_ppm"), blue)]),
              ("detection residual RMS (px; image 0 is the reference)", [(None, np.where(t == 0, np.nan, g("residual_rms")), blue)])]  # fmt: skip
    fig, axes = plt.subplots(len(panels), 1, figsize=(11, 2.3 * len(panels)), sharex=True)
    for a, (title, series) in zip(axes, panels):
        for name, y, c in series:
            a.plot(t, y, "-o", color=c, lw=1.5, ms=3)
            if name:
                a.annotate(name, (t[-1], y[-1]), xytext=(5, 0), textcoords="offset points", color=ink, va="center")
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
    ap.add_argument("-n", type=int, default=12)
    ap.add_argument("--interval", type=float, default=5.0, help="s between capture starts")
    ap.add_argument("--board", choices=["charuco", "checker"], default="charuco")
    ap.add_argument("--target", default="ipad_charuco", help="ChArUco spec in config/targets/")
    ap.add_argument("--cols", type=int, default=9, help="inner corners across")
    ap.add_argument("--rows", type=int, default=6, help="inner corners down")
    ap.add_argument("--analyze", type=Path, help="existing run dir: analyze its images, capture nothing")
    ap.add_argument("--mock", action="store_true", help="exercise the capture path only (no analysis)")
    ap.add_argument("--notes", default="")
    args = ap.parse_args()
    console = Console()
    pattern = (args.cols, args.rows)
    board = CharucoSpec.load(args.target).board() if args.board == "charuco" else None

    if args.analyze:
        files = sorted((args.analyze / "images").glob("*.ARW"))
        if not files:
            sys.exit(f"no .ARW files in {args.analyze}/images")
        result = analyze(files, board, pattern, console)  # read-only: raw run directories are immutable
        out = REPO_ROOT / "data" / "derived" / args.analyze.name / "stability.png"
        plot(result, capture_times(args.analyze), out)
        console.print(f"wrote {out.relative_to(REPO_ROOT)}")
        return

    cam = MockCamera() if args.mock else Gphoto2Camera()
    info = cam.info()
    run = Run("stability_mock" if args.mock else "stability", "camera",
              notes=f"static {args.board} stability n={args.n} interval={args.interval}; {args.notes}")  # fmt: skip
    run.write_json("camera.json", info)
    manifest = Manifest(run, "stability")
    files, streak = [], 0
    try:
        for i in range(args.n):
            t0 = time.monotonic()
            r = cam.capture(run.dir / "images", f"img_{i:04d}")
            manifest.add(f"capture_{i:04d}", commanded_ticks=None, before=None, after=None, camera={**r.as_dict(), "backend": cam.name})
            console.print(f"{i + 1}/{args.n}  {r.duration_s:.2f} s  {'ok' if r.ok else '[red]' + str(r.error)}")
            streak = 0 if r.ok else streak + 1
            if streak >= 3:
                console.print("[red]3 failed captures in a row (camera asleep or disconnected?): stopping, analyzing what we have")
                break
            files += [f for f in r.files if f.suffix.upper() == ".ARW"]
            if i < args.n - 1:
                time.sleep(max(0.0, args.interval - (time.monotonic() - t0)))
    finally:
        manifest.close()
    if args.mock:
        return
    if not files:
        sys.exit("no images captured")
    result = analyze(files, board, pattern, console)
    run.write_json("stability.json", result)
    out = REPO_ROOT / "data" / "derived" / run.run_id / "stability.png"
    plot(result, capture_times(run.dir), out)
    console.print(f"[bold]{run.run_id}[/]  plot: {out.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
