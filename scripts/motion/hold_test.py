"""Step 5b: first torque-on. Enable every joint via SafeArm, hold still, torque off. No goals are sent.

    uv run scripts/motion/hold_test.py --arm-moves --tui
    uv run scripts/motion/hold_test.py --arm-moves --sim --seconds 5     # same script on SimBus, no hardware

Sequence: preflight -> SafeArm.enable() (per joint: goal := present, speed 200, accel 10, read back,
torque on, read back) -> monitor for --seconds with every trip active -> torque off -> keep reading
for --after-s so the log shows what the joints do once released (elbow settles ~4 deg onto its stop).

Setup: follower clamped, rest pose, hand NEAR the arm (not holding it), 12 V switch within reach.
Stop early with Ctrl-C (torque off) or by cutting 12 V. Live Rerun plots are on by default (--no-viz to skip).
Writes data/raw/<run_id>/ (kind hold) + safety_events.json + hold_test.json.
"""

import argparse
import json
import sys
import threading
import time
from statistics import median

import pyarrow.parquet as pq
from rich.console import Console
from rich.table import Table

from so101.config import REPO_ROOT, load_arm_config
from so101.hw.bus import FeetechBus
from so101.hw.sim import SimBus
from so101.motion.loop import control_loop
from so101.runlog.run import Run
from so101.safety import SafeArm, SafetyTrip
from so101.telemetry.live import LiveSession


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm-moves", action="store_true", help="required for every scripts/motion/ script")
    ap.add_argument("--tui", action="store_true")
    ap.add_argument("--viz", action=argparse.BooleanOptionalAction, default=True,
                    help="live Rerun plots, saved as telemetry.rrd (default on; the viewer settles before torque-on)")
    ap.add_argument("--seconds", type=float, default=20.0, help="hold duration with torque on")
    ap.add_argument("--after-s", type=float, default=1.5, help="keep logging this long after torque off")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--notes", default="")
    ap.add_argument("--sim", action="store_true", help="use SimBus instead of the arm")
    args = ap.parse_args()
    if not args.arm_moves:
        sys.exit("refusing to run without --arm-moves (scripts/motion/ convention)")

    console = Console()
    cfg = load_arm_config("follower")
    if args.sim:
        dump = REPO_ROOT / "data/raw/20261026-112606_regdump_follower/registers.json"
        bus = SimBus.from_dump(json.loads(dump.read_text()), glitch_rate=0.01)
    else:
        bus = FeetechBus(cfg.port, cfg.baudrate, cfg.protocol_end)
    kind = "hold_sim" if args.sim else "hold"
    run = Run(kind, "follower", config_paths=[cfg.path], notes=f"step 5b hold test; sim={args.sim}; {args.notes}")
    live = LiveSession(run, {"follower": cfg.expected_voltage_v}, args.tui, args.viz,
                       title=f"[TORQUE ON: HOLD] {run.run_id} — Ctrl-C = torque off")  # fmt: skip
    result: dict = {"marks": {}}
    marks = result["marks"]

    def worker() -> None:
        try:
            with SafeArm(bus, cfg, hub=live.hub) as arm:
                result["arm"] = arm
                marks["enable_start_ns"] = time.monotonic_ns()
                result["enable"] = arm.enable()
                marks["hold_start_ns"] = time.monotonic_ns()
                result["loop"] = control_loop(arm, args.rate, lambda t: None, args.seconds, live.stop)
                marks["torque_off_ns"] = time.monotonic_ns()
                result["disable_failed"] = arm.disable()
                t_end = time.monotonic() + args.after_s
                while time.monotonic() < t_end:  # monitoring only (allowed while disabled)
                    arm.step()
                    time.sleep(1 / args.rate)
        except SafetyTrip as e:
            marks.setdefault("torque_off_ns", time.monotonic_ns())
            result["trip"] = e
        except BaseException as e:
            marks.setdefault("torque_off_ns", time.monotonic_ns())
            result["error"] = e

    th = threading.Thread(target=worker, name="safearm-hold")
    th.start()
    live.footer = ["TORQUE ON (hold, no goals). Ctrl-C = torque off. Hand near the arm."]
    interrupted = live.run_until_done(th, console)
    summary = live.close()
    bus.close()

    arm, enable = result.get("arm"), result.get("enable")
    if arm:
        run.write_json("safety_events.json", arm.report())

    per_joint = {}
    if summary["rows"] and "hold_start_ns" in marks:
        rows = pq.read_table(run.dir / "telemetry").to_pylist()
        h0, off = marks["hold_start_ns"], marks["torque_off_ns"]
        for j in cfg.joints:
            jr = [r for r in rows if r["joint"] == j and r["position"] is not None]
            hold = [r for r in jr if h0 <= r["t_start_ns"] < off]
            after = [r for r in jr if r["t_start_ns"] >= off]
            held = enable.present[j] if enable else None
            if not hold:
                continue
            per_joint[j] = {
                "held_at": held,
                "max_dev_hold": max(abs(r["position"] - held) for r in hold),
                "load_med_pct": median(r["load"] for r in hold) / 10,
                "load_max_abs_pct": max(abs(r["load"]) for r in hold) / 10,
                "current_med_ma": median(r["current"] for r in hold) * 6.5,
                "temp_start_c": median(r["temperature_c"] for r in hold[:50]),
                "temp_end_c": median(r["temperature_c"] for r in hold[-50:]),
                "pos_after_off": after[-1]["position"] if after else None,
            }

    loop = result.get("loop")
    run.write_json("hold_test.json", {
        "interrupted": interrupted,
        "trip": str(result["trip"]) if "trip" in result else None,
        "error": repr(result["error"]) if "error" in result else None,
        "outside_limits_at_enable": enable.outside_limits if enable else None,
        "disable_failed": result.get("disable_failed"),
        "loop": vars(loop) if loop else None,
        "marks_ns": marks,
        "per_joint": per_joint,
        "bus_failures": {f"{k[0]}:{k[1]}": v for k, v in bus.stats.failures.items()},
        "logger": summary,
    })  # fmt: skip

    console.print(f"[bold]{run.run_id}[/] ({'SIMULATION' if args.sim else 'real follower'})")
    if "trip" in result:
        console.print(f"[red]{result['trip']}[/] → torque off on all joints")
    elif "error" in result:
        console.print(f"[red]error (torque turned off on exit): {result['error']!r}")
    else:
        console.print("[green]enable → hold → torque off completed, no trips" + (" (stopped early)" if interrupted else ""))
    if loop:
        console.print(f"hold loop: {loop.cycles} cycles in {loop.duration_s:.1f} s, overruns {loop.overruns}, "
                      f"skipped {loop.skipped_slots}")  # fmt: skip
    if result.get("disable_failed"):
        console.print(f"[red]could not confirm torque off on {result['disable_failed']} — cut 12 V")
    if per_joint:
        table = Table(title="hold summary (position in ticks)", title_justify="left")
        for col in ["joint", "held at", "max dev", "load med %", "|load| max %", "mA med", "°C start→end", "after off"]:
            table.add_column(col, justify="left" if col == "joint" else "right")
        for j, d in per_joint.items():
            table.add_row(j, str(d["held_at"]), str(d["max_dev_hold"]), f"{d['load_med_pct']:.1f}",
                          f"{d['load_max_abs_pct']:.1f}", f"{d['current_med_ma']:.0f}",
                          f"{d['temp_start_c']:g}→{d['temp_end_c']:g}", str(d["pos_after_off"]))  # fmt: skip
        console.print(table)
    console.print(f"bus failures: {dict(bus.stats.failures) or 0}   logger: {summary}")


if __name__ == "__main__":
    main()
