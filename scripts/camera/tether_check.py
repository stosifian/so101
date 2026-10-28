"""Phase 2 step 1: tethering reliability. N consecutive captures over gphoto2 (spec §6: accept at ~50/50).

    uv run scripts/camera/tether_check.py --mock -n 5          # no camera: checks the script itself
    uv run scripts/camera/tether_check.py --detect             # just list cameras (gphoto2 --auto-detect)
    uv run scripts/camera/tether_check.py -n 50                # the real test
    uv run scripts/camera/tether_check.py -n 50 --interval 2   # with a pause between captures

Does not touch the arm. Camera prep: USB Connection = PC Remote, Still Img. Save Dest = PC Only (or PC+Camera),
then on macOS `killall ptpcamerad` (it grabs the camera when it is plugged in).
Writes data/raw/<run_id>/ (kind tether): header.json + camera info (model, firmware, settings), images/,
manifest.jsonl (one row per capture: ok, files, request/done times, error) and tether.json (summary).
Stops early after --max-consecutive-failures failures in a row (a disconnect, not a glitch).
"""

import argparse
import json
import sys
import time
from statistics import median

from rich.console import Console

from so101.runlog.manifest import Manifest
from so101.runlog.run import Run
from so101.sensors import Gphoto2Camera, MockCamera


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("-n", type=int, default=50, help="captures")
    ap.add_argument("--interval", type=float, default=0.0, help="pause between captures, s")
    ap.add_argument("--timeout", type=float, default=30.0, help="per-capture timeout (hang), s")
    ap.add_argument("--max-consecutive-failures", type=int, default=3)
    ap.add_argument("--detect", action="store_true", help="only run gphoto2 --auto-detect and --summary")
    ap.add_argument("--mock", action="store_true")
    ap.add_argument("--notes", default="")
    args = ap.parse_args()
    console = Console()

    cam = MockCamera(delay_s=0.05) if args.mock else Gphoto2Camera(timeout_s=args.timeout)
    if args.detect:
        if not args.mock:
            console.print(cam.detect())
        console.print_json(json.dumps(cam.info()))
        return
    try:
        info = cam.info()
    except Exception as e:
        sys.exit(f"camera not ready: {e}")
    console.print(f"[bold]{info.get('model')}[/] firmware {info.get('firmware')}  settings {info.get('config')}")

    run = Run("tether_mock" if args.mock else "tether", "camera",
              notes=f"step 1 tether check n={args.n} interval={args.interval}; {args.notes}")  # fmt: skip
    run.write_json("camera.json", info)
    manifest = Manifest(run, "tether")
    images = run.dir / "images"
    results, streak = [], 0
    try:
        for i in range(args.n):
            r = cam.capture(images, f"img_{i:04d}")
            results.append(r)
            manifest.add(f"capture_{i:04d}", commanded_ticks=None, before=None, after=None,
                         camera={**r.as_dict(), "backend": cam.name})  # fmt: skip
            size = sum(f.stat().st_size for f in r.files) / 1e6
            mark = "[green]ok[/]" if r.ok else f"[red]FAIL[/] {r.error}"
            console.print(f"{i + 1:3d}/{args.n}  {r.duration_s:6.2f} s  {size:6.1f} MB  {[f.suffix for f in r.files]}  {mark}")
            streak = 0 if r.ok else streak + 1
            if streak >= args.max_consecutive_failures:
                console.print(f"[red]{streak} failures in a row: stopping (disconnect?)")
                break
            if args.interval:
                time.sleep(args.interval)
    except KeyboardInterrupt:
        console.print("[yellow]interrupted")
    finally:
        manifest.close()
        cam.close()

    ok = [r for r in results if r.ok]
    durs = sorted(r.duration_s for r in ok)
    longest = cur = 0
    for r in results:
        cur = cur + 1 if r.ok else 0
        longest = max(longest, cur)
    summary = {"requested": args.n, "attempted": len(results), "ok": len(ok), "longest_ok_streak": longest,
               "failures": [{"index": i, "error": r.error} for i, r in enumerate(results) if not r.ok],
               "duration_s": {"median": median(durs), "p95": durs[int(0.95 * (len(durs) - 1))], "max": durs[-1]} if durs else None,
               "passed": len(ok) == args.n == len(results)}  # fmt: skip
    run.write_json("tether.json", summary)
    verdict = "[green]PASS[/]" if summary["passed"] else "[red]NOT PASSED[/]"
    console.print(f"[bold]{run.run_id}[/]  {verdict}: {len(ok)}/{args.n} ok, longest streak {longest}, "
                  f"capture time {summary['duration_s']}")  # fmt: skip


if __name__ == "__main__":
    main()
