"""Image stability sanity check: N captures of a static board, corner drift in pixels vs the first image.

    uv run scripts/camera/stability_check.py                    # 12 captures, 5 s apart (~6 s: capture takes 5.9 s), iPad ChArUco
    uv run scripts/camera/stability_check.py --board checker    # plain 9x6 checkerboard (--cols/--rows)
    uv run scripts/camera/stability_check.py -n 10 --interval 30
    uv run scripts/camera/stability_check.py --analyze data/raw/<run_id>   # re-analyze an existing run's images

Does not touch the arm. Nothing may move between captures (camera, tripod, iPad, table).
ChArUco corners are matched by ID, so a partly hidden board still works (only corners seen in both images count).
Per image vs image 0: mean shift (dx, dy) = whole-scene motion (tripod/iPad/table), and the residual RMS after
removing that shift = corner-detection noise + any rotation/flex. Writes stability.json into the run directory.
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


def analyze(files: list[Path], board, pattern: tuple[int, int], console: Console) -> dict:
    pts = [corners(f, board, pattern) for f in files]
    if pts[0] is None:
        sys.exit(f"no board found in {files[0].name}")
    rows = []
    t = Table(title=f"corner drift vs {files[0].name} (px; 1 px ≈ 73 µm at 0.5 m, 56 µm at 0.38 m)", title_justify="left")
    for col in ["image", "corners", "dx", "dy", "|shift|", "residual RMS", "residual max"]:
        t.add_column(col, justify="right")
    for f, p in zip(files, pts):
        if p is None:
            rows.append({"image": f.name, "found": False})
            t.add_row(f.name, "[red]not found", "", "", "", "", "")
            continue
        ids = sorted(set(p) & set(pts[0]))
        d = np.array([p[i] - pts[0][i] for i in ids])
        shift = d.mean(axis=0)
        res = np.linalg.norm(d - shift, axis=1)
        r = {"image": f.name, "found": True, "n_common": len(ids), "dx": float(shift[0]), "dy": float(shift[1]),
             "shift": float(np.hypot(*shift)), "residual_rms": float(np.sqrt((res**2).mean())), "residual_max": float(res.max())}  # fmt: skip
        rows.append(r)
        t.add_row(f.name, str(len(ids)), f"{r['dx']:+.3f}", f"{r['dy']:+.3f}", f"{r['shift']:.3f}", f"{r['residual_rms']:.3f}", f"{r['residual_max']:.3f}")
    console.print(t)
    return {"board": "charuco" if board is not None else f"checker {pattern[0]}x{pattern[1]}", "n_corners": len(pts[0]), "images": rows}


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
        analyze(files, board, pattern, console)  # read-only: raw run directories are immutable
        return

    cam = MockCamera() if args.mock else Gphoto2Camera()
    info = cam.info()
    run = Run("stability_mock" if args.mock else "stability", "camera",
              notes=f"static {args.board} stability n={args.n} interval={args.interval}; {args.notes}")  # fmt: skip
    run.write_json("camera.json", info)
    manifest = Manifest(run, "stability")
    files = []
    try:
        for i in range(args.n):
            t0 = time.monotonic()
            r = cam.capture(run.dir / "images", f"img_{i:04d}")
            manifest.add(f"capture_{i:04d}", commanded_ticks=None, before=None, after=None, camera={**r.as_dict(), "backend": cam.name})
            console.print(f"{i + 1}/{args.n}  {r.duration_s:.2f} s  {'ok' if r.ok else '[red]' + str(r.error)}")
            if not r.ok:
                sys.exit("capture failed; stopping")
            files += [f for f in r.files if f.suffix.upper() == ".ARW"]
            if i < args.n - 1:
                time.sleep(max(0.0, args.interval - (time.monotonic() - t0)))
    finally:
        manifest.close()
    if args.mock:
        return
    result = analyze(files, board, pattern, console)
    run.write_json("stability.json", result)
    console.print(f"[bold]{run.run_id}[/]")


if __name__ == "__main__":
    main()
