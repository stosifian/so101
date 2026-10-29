"""Phase 2 step 4: camera intrinsics from the iPad ChArUco.

    uv run scripts/camera/calibrate_intrinsics.py capture                 # interactive: Enter = shoot, q = done
    uv run scripts/camera/calibrate_intrinsics.py capture --every 8 --count 30   # hands-free, spoken cues
    uv run scripts/camera/calibrate_intrinsics.py analyze data/raw/<run_id>
    uv run scripts/camera/calibrate_intrinsics.py analyze data/raw/<run_id> --drop img_0007,img_0012

Does not touch the arm. Setup: iPad showing config/targets/ipad_charuco_2360x1640.png full screen (1:1),
camera at the working distance (frame the extended arm first), focus taped, settings per config/camera_a7iv.yaml.
capture: after each shot it reports corners found (of 96) and a 6 x 4 coverage map of the frame so you can
  aim the next view at empty cells. Aim for ~30 views: board in every part of the frame (corners and edges
  matter most for distortion), tilted up to ~40 deg in different directions, some views closer/farther.
  --every S: hands-free (you hold the iPad). Spoken cues: "hold" ~1 s before the shot, "move" --hold-s after
  the trigger (the exposure is over by then; the download continues). One shot every S s (≥ ~7.5: capture
  5.9 s + detection ~1 s). Ctrl-C stops early.
analyze: calibrates and writes data/derived/<run_id>/intrinsics.json + residuals.png.
  Pass: overall RMS ~0.1-0.2 px; no view far above the rest; residual arrows random (no swirl or pattern);
  no hot corners on the board map (moiré or glare); fx ≈ fy, between ~6833 px (infinity) and the
  thin-lens value at --focus-m (~7500 px at 0.38 m); principal point near the centre.
Nothing else depends on the distance; but intrinsics are only valid at the focus they were shot at.
"""

import argparse
import json
import shutil
import subprocess
import sys
import threading
import time
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.table import Table

from so101.analysis.charuco import CharucoSpec, detect, load_gray
from so101.analysis.intrinsics import calibrate, coverage_grid, residual_figure
from so101.config import REPO_ROOT
from so101.runlog.manifest import Manifest
from so101.runlog.run import Run, sha256_file
from so101.sensors import Gphoto2Camera

PX_MM = 35.9 / 7008  # sensor width over 7008 px


def f_expected_px(focus_m: float, f_mm: float = 35.0) -> float:
    """Rough sanity value: thin-lens image distance f·u/(u−f) in pixels (≈ 6833 px at infinity, ≈ 7500 at 0.38 m).
    The real lens focuses internally and breathes differently, so this is a ±10 % plausibility check only."""
    u = focus_m * 1000
    return f_mm * u / (u - f_mm) / PX_MM


def show_coverage(console: Console, grid: np.ndarray) -> None:
    for row in grid:
        console.print("  " + " ".join(f"[{'green' if v else 'red'}]{v:4d}[/]" for v in row))


def cue(word: str) -> None:
    """Spoken cue (macOS `say`, non-blocking), terminal bell otherwise."""
    if shutil.which("say"):
        subprocess.Popen(["say", "-r", "230", word], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    else:
        print("\a", end="", flush=True)


def cmd_capture(args, console: Console) -> None:
    spec = CharucoSpec.load(args.target)
    board = spec.board()
    cam = Gphoto2Camera()
    info = cam.info()
    console.print(f"{info.get('model')} fw {info.get('firmware')}  {info.get('config')}")
    run = Run("intrinsics", "camera", config_paths=[REPO_ROOT / f"config/targets/{args.target}.yaml", REPO_ROOT / "config/camera_a7iv.yaml"],
              notes=f"step 4 iPad intrinsics; {args.notes}")  # fmt: skip
    run.write_json("camera.json", info)
    manifest = Manifest(run, "intrinsics")
    pts, size, i = [], None, 0
    timed = args.every is not None
    if timed:
        console.print(f"hands-free: {args.count} shots, one every {args.every:g} s. Cues: 'hold' = keep still, 'move' = next pose.")
        cue("first pose")
        time.sleep(max(args.every - 1.0, 2.0))
    next_shot = time.monotonic()
    try:
        while True:
            if timed:
                if i >= args.count:
                    break
                time.sleep(max(0.0, next_shot - 1.0 - time.monotonic()))
                cue("hold")
                time.sleep(1.0)
                next_shot = time.monotonic() + args.every
                box: dict = {}
                th = threading.Thread(target=lambda: box.setdefault("r", cam.capture(run.dir / "images", f"img_{i:04d}")))
                th.start()
                th.join(args.hold_s)
                cue("move")
                th.join()
                r = box["r"]
            else:
                try:
                    key = input(f"[view {i}] Enter = capture, q = finish: ").strip().lower()
                except EOFError:
                    break
                if key == "q":
                    break
                r = cam.capture(run.dir / "images", f"img_{i:04d}")
            raw = [f for f in r.files if f.suffix.upper() == ".ARW"]
            det = None
            if r.ok and raw:
                gray = load_gray(raw[0])
                size = (gray.shape[1], gray.shape[0])
                det = detect(gray, board)
            n = 0 if det is None else len(det.ids)
            manifest.add(f"view_{i:04d}", commanded_ticks=None, before=None, after=None,
                         camera={**r.as_dict(), "backend": cam.name}, extra={"corners": n})  # fmt: skip
            if not r.ok:
                console.print(f"[red]capture failed: {r.error}")
                i += 1
                continue
            colour = "green" if n >= 0.5 * spec.n_corners else ("yellow" if n >= 12 else "red")
            console.print(f"  [{colour}]{n}/{spec.n_corners} corners[/] ({r.duration_s:.1f} s)")
            if det is not None and n >= 12:
                pts.append(det.corners)
            if size:
                console.print(f"  coverage after {len(pts)} usable views (corners per cell; red = empty):")
                show_coverage(console, coverage_grid(np.vstack(pts) if pts else np.zeros((0, 2)), size))
            i += 1
    finally:
        manifest.close()
    console.print(f"[bold]{run.run_id}[/]: {len(pts)} usable views. Next: analyze data/raw/{run.run_id}")


def cmd_analyze(args, console: Console) -> None:
    run_dir = Path(args.run_dir)
    spec = CharucoSpec.load(args.target)
    board = spec.board()
    drop = set(filter(None, args.drop.split(",")))
    files = [f for f in sorted((run_dir / "images").glob("*.ARW")) if f.stem not in drop]
    if not files:
        sys.exit(f"no .ARW files in {run_dir}/images")
    dets, size = {}, None
    for f in files:
        gray = load_gray(f)
        size = (gray.shape[1], gray.shape[0])
        dets[f.stem] = detect(gray, board)
        console.print(f"  {f.stem}: {0 if dets[f.stem] is None else len(dets[f.stem].ids)} corners")
    cal = calibrate(dets, board, size)
    out = REPO_ROOT / "data" / "derived" / run_dir.name
    out.mkdir(parents=True, exist_ok=True)
    result = {"run_id": run_dir.name, "target": spec.name, "target_spec_sha256": sha256_file(REPO_ROOT / f"config/targets/{spec.name}.yaml"),
              "units": "mm (nominal display pitch; scale does not affect intrinsics)", "dropped": sorted(drop),
              "frame": "rawpy full visible frame, no lens corrections (so101.analysis.charuco.load_gray)",
              **cal.as_dict()}  # fmt: skip
    (out / "intrinsics.json").write_text(json.dumps(result, indent=2))
    residual_figure(cal, spec.square_mm, out / "residuals.png")

    t = Table(title=f"intrinsics: {len(cal.views)} views, {sum(len(p) for p in cal.image_points)} corners", title_justify="left")
    for c in ["parameter", "value", "± 1σ", "check"]:
        t.add_column(c, justify="right")
    W, H = size
    K = cal.K
    t.add_row("RMS (px)", f"{cal.rms_px:.3f}", "", "ok" if cal.rms_px <= 0.2 else "[yellow]> 0.2")
    f_lo, f_hi = f_expected_px(1e6), f_expected_px(args.focus_m)
    for name in ("fx", "fy"):
        val = K[0, 0] if name == "fx" else K[1, 1]
        ok = 0.97 * f_lo <= val <= 1.05 * f_hi
        t.add_row(name, f"{val:.1f}", f"{cal.std[name]:.1f}", "ok" if ok else f"[yellow]expected ~{f_lo:.0f}–{f_hi:.0f}")
    t.add_row("cx", f"{K[0, 2]:.1f}", f"{cal.std['cx']:.1f}", f"centre {W / 2:.0f}")
    t.add_row("cy", f"{K[1, 2]:.1f}", f"{cal.std['cy']:.1f}", f"centre {H / 2:.0f}")
    for k in ["k1", "k2", "p1", "p2", "k3"]:
        t.add_row(k, f"{cal.dist[['k1', 'k2', 'p1', 'p2', 'k3'].index(k)]:+.5f}", f"{cal.std[k]:.5f}", "")
    console.print(t)
    med = float(np.median(cal.per_view_rms))
    worst = sorted(zip(cal.per_view_rms, cal.views), reverse=True)[:5]
    console.print("worst views: " + ", ".join(f"{v} {e:.3f}" + (" [yellow](>2× median)[/]" if e > 2 * med else "") for e, v in worst))
    console.print(f"wrote {out.relative_to(REPO_ROOT)}/intrinsics.json + residuals.png")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    sub = ap.add_subparsers(dest="cmd", required=True)
    c = sub.add_parser("capture")
    c.add_argument("--notes", default="")
    c.add_argument("--every", type=float, help="hands-free: seconds between shots (spoken cues)")
    c.add_argument("--count", type=int, default=30, help="hands-free: number of shots")
    c.add_argument("--hold-s", type=float, default=2.0, help="hands-free: keep still this long after the trigger")
    a = sub.add_parser("analyze")
    a.add_argument("run_dir")
    a.add_argument("--drop", default="", help="comma-separated image stems to leave out")
    a.add_argument("--focus-m", type=float, default=0.5, help="focus distance, only for the fx/fy plausibility check")
    for p in (c, a):
        p.add_argument("--target", default="ipad_charuco")
    args = ap.parse_args()
    console = Console()
    cmd_capture(args, console) if args.cmd == "capture" else cmd_analyze(args, console)


if __name__ == "__main__":
    main()
