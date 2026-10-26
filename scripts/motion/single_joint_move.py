"""Step 5c: first commanded move. One joint, small and slow, out and back; every other joint holds.

    uv run scripts/motion/single_joint_move.py --arm-moves --tui
    uv run scripts/motion/single_joint_move.py --arm-moves --joint shoulder_pan --amplitude 100 --speed 50 --cycles 2
    uv run scripts/motion/single_joint_move.py --arm-moves --sim                # same script on SimBus, no hardware

Sequence: read the start position with torque off -> check the target is inside the software
limits (else refuse before enabling) -> SafeArm.enable() -> per cycle: ramp out at --speed, hold
--hold-s, ramp back, hold -> torque off -> log --after-s more. Only --joint gets goals.

First-move limits enforced here, tighter than config: |amplitude| <= 300 ticks, speed <= 150 ticks/s.
Setup: follower clamped, rest pose, hand near the arm, 12 V switch within reach. Ctrl-C = torque off.
Writes data/raw/<run_id>/ (kind move) + safety_events.json + single_move.json.
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
from so101.motion.loop import Segment, control_loop, sequence
from so101.runlog.run import Run
from so101.safety import SafeArm, SafetyError, SafetyTrip
from so101.telemetry.live import LiveSession

MAX_AMPLITUDE, MAX_SPEED = 300, 150  # first-move stage limits (config caps are 200 ticks/s, 100/step)
SETTLE_TOL = 2  # ticks


def analyse(rows: list[dict], joint: str, min_hold_samples: int = 30) -> dict:
    """Group the joint's samples into runs of identical commanded goal. A run of >= min_hold_samples
    (0.3 s at 100 Hz) is a hold; anything shorter is part of a ramp (at 50 ticks/s the rounded goal
    repeats every other cycle, so goal-unchanged alone does not mean stopped). Measures tracking
    during ramps, and overshoot + settle time at each hold that follows a ramp."""
    js = sorted((r for r in rows if r["joint"] == joint and r["goal"] is not None and r["position"] is not None),
                key=lambda r: r["seq"])  # fmt: skip
    runs: list[list[dict]] = []
    for r in js:
        if runs and runs[-1][0]["goal"] == r["goal"]:
            runs[-1].append(r)
        else:
            runs.append([r])
    ramps, holds, stops = [], [], []
    for k, run_ in enumerate(runs):
        if len(run_) < min_hold_samples:
            ramps += run_
            continue
        holds += run_
        if k == 0 or len(runs[k - 1]) >= min_hold_samples:
            continue  # hold not preceded by a ramp (e.g. the very start)
        target = run_[0]["goal"]
        direction = 1 if target > runs[k - 1][0]["goal"] else -1
        t_stop = run_[0]["t_start_ns"]
        over = max(direction * (r["position"] - target) for r in run_)
        settled_at = next(((run_[i]["t_start_ns"] - t_stop) / 1e6 for i in range(len(run_))
                           if all(abs(r["position"] - target) <= SETTLE_TOL for r in run_[i:])), None)  # fmt: skip
        moving_off = next(((r["t_start_ns"] - t_stop) / 1e6 for r in run_ if r["moving"] == 0), None)
        stops.append({"target": target, "direction": direction, "overshoot_ticks": max(over, 0),
                      "settle_ms": settled_at, "moving_zero_after_ms": moving_off,
                      "final_error": run_[-1]["position"] - target})  # fmt: skip
    err = [abs(r["goal"] - r["position"]) for r in ramps]
    return {
        "ramp_samples": len(ramps),
        "hold_samples": len(holds),
        "ramp_err_median": median(err) if err else None,
        "ramp_err_max": max(err) if err else None,
        "moving_frac_ramp": sum(r["moving"] for r in ramps) / len(ramps) if ramps else None,
        "moving_frac_hold": sum(r["moving"] for r in holds) / len(holds) if holds else None,
        "load_med_ramp_pct": median(abs(r["load"]) for r in ramps) / 10 if ramps else None,
        "load_med_hold_pct": median(abs(r["load"]) for r in holds) / 10 if holds else None,
        "current_max_units": max((r["current"] for r in js), default=None),
        "stops": stops,
    }


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm-moves", action="store_true", help="required for every scripts/motion/ script")
    ap.add_argument("--joint", default="shoulder_pan")
    ap.add_argument("--amplitude", type=int, default=100, help=f"ticks (|value| <= {MAX_AMPLITUDE}); sign = direction")
    ap.add_argument("--speed", type=float, default=50.0, help=f"ramp speed, ticks/s (<= {MAX_SPEED})")
    ap.add_argument("--cycles", type=int, default=2)
    ap.add_argument("--hold-s", type=float, default=2.0)
    ap.add_argument("--after-s", type=float, default=1.5)
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--tui", action="store_true")
    ap.add_argument("--viz", action="store_true", help="off by default: viewer startup disturbs timing")
    ap.add_argument("--notes", default="")
    ap.add_argument("--sim", action="store_true", help="use SimBus instead of the arm")
    args = ap.parse_args()
    if not args.arm_moves:
        sys.exit("refusing to run without --arm-moves (scripts/motion/ convention)")
    if not 0 < abs(args.amplitude) <= MAX_AMPLITUDE or not 0 < args.speed <= MAX_SPEED or args.cycles < 1:
        sys.exit(f"step 5c limits: 0 < |amplitude| <= {MAX_AMPLITUDE}, 0 < speed <= {MAX_SPEED}, cycles >= 1")

    console = Console()
    cfg = load_arm_config("follower")
    if args.joint not in cfg.joints:
        sys.exit(f"unknown joint {args.joint}; one of {list(cfg.joints)}")
    if args.sim:
        dump = REPO_ROOT / "data/raw/20261026-112606_regdump_follower/registers.json"
        bus = SimBus.from_dump(json.loads(dump.read_text()), glitch_rate=0.01)
    else:
        bus = FeetechBus(cfg.port, cfg.baudrate, cfg.protocol_end)
    kind = "move_sim" if args.sim else "move"
    run = Run(kind, "follower", config_paths=[cfg.path],
              notes=f"step 5c {args.joint} {args.amplitude:+d} ticks @ {args.speed:g}/s x{args.cycles}; sim={args.sim}; "
                    f"{args.notes}")  # fmt: skip
    live = LiveSession(run, {"follower": cfg.expected_voltage_v}, args.tui, args.viz,
                       title=f"[TORQUE ON: MOVING {args.joint}] {run.run_id} — Ctrl-C = torque off")  # fmt: skip
    result: dict = {"marks": {}}
    marks = result["marks"]

    def worker() -> None:
        try:
            with SafeArm(bus, cfg, hub=live.hub) as arm:
                result["arm"] = arm
                start = arm.step().snapshot.joints[args.joint]  # torque off, monitoring only
                if start is None:
                    raise SafetyError(f"{args.joint}: no reading before enable")
                p0 = float(start.position)
                target = p0 + args.amplitude
                lo, hi = arm.safety.limits[args.joint]
                if not (lo <= p0 <= hi and lo <= target <= hi):
                    raise SafetyError(f"{args.joint}: start {p0:.0f} or target {target:.0f} outside limits [{lo}, {hi}]; "
                                      "not enabling")  # fmt: skip
                a, b = {args.joint: p0}, {args.joint: target}
                segs = [s for _ in range(args.cycles) for s in (Segment(a, b, args.speed), Segment(b, a, args.speed))]
                goal_fn, total = sequence(segs, settle_s=args.hold_s)
                result["plan"] = {"start": p0, "target": target, "total_s": total}
                result["enable"] = arm.enable()
                marks["move_start_ns"] = time.monotonic_ns()
                result["loop"] = control_loop(arm, args.rate, goal_fn, total, live.stop)
                marks["torque_off_ns"] = time.monotonic_ns()
                result["disable_failed"] = arm.disable()
                t_end = time.monotonic() + args.after_s
                while time.monotonic() < t_end:
                    arm.step()
                    time.sleep(1 / args.rate)
        except SafetyTrip as e:
            marks.setdefault("torque_off_ns", time.monotonic_ns())
            result["trip"] = e
        except BaseException as e:
            marks.setdefault("torque_off_ns", time.monotonic_ns())
            result["error"] = e

    th = threading.Thread(target=worker, name="safearm-move")
    th.start()
    live.footer = [f"TORQUE ON: moving {args.joint} {args.amplitude:+d} ticks @ {args.speed:g} ticks/s. Ctrl-C = torque off."]
    interrupted = live.run_until_done(th, console)
    summary = live.close()
    bus.close()

    arm = result.get("arm")
    if arm:
        run.write_json("safety_events.json", arm.report())
    stats = analyse(pq.read_table(run.dir / "telemetry").to_pylist(), args.joint) if summary["rows"] else {}
    loop = result.get("loop")
    run.write_json("single_move.json", {
        "joint": args.joint, "amplitude": args.amplitude, "speed": args.speed, "cycles": args.cycles,
        "interrupted": interrupted,
        "trip": str(result["trip"]) if "trip" in result else None,
        "error": repr(result["error"]) if "error" in result else None,
        "plan": result.get("plan"), "loop": vars(loop) if loop else None,
        "disable_failed": result.get("disable_failed"), "marks_ns": marks, "analysis": stats,
        "clamps": {f"{j}:{k}": n for (j, k), n in arm.clamps.items()} if arm else None,
        "bus_failures": {f"{k[0]}:{k[1]}": v for k, v in bus.stats.failures.items()}, "logger": summary,
    })  # fmt: skip

    console.print(f"[bold]{run.run_id}[/] ({'SIMULATION' if args.sim else 'real follower'})")
    if "trip" in result:
        console.print(f"[red]{result['trip']}[/] → torque off on all joints")
    elif "error" in result:
        console.print(f"[red]stopped (torque off): {result['error']}")
    else:
        console.print("[green]move completed, torque off, no trips" + (" (stopped early)" if interrupted else ""))
    if loop:
        console.print(f"loop: {loop.cycles} cycles in {loop.duration_s:.1f} s, overruns {loop.overruns}, "
                      f"skipped {loop.skipped_slots}")  # fmt: skip
    if arm:
        console.print(f"clamps: {dict((f'{j}:{k}', n) for (j, k), n in arm.clamps.items()) or 'none'}")
    if result.get("disable_failed"):
        console.print(f"[red]could not confirm torque off on {result['disable_failed']} — cut 12 V")
    if stats.get("ramp_samples"):
        console.print(
            f"{args.joint}: ramp tracking |goal−pos| median {stats['ramp_err_median']:.0f} / max {stats['ramp_err_max']} ticks; "
            f"Moving=1 in {stats['moving_frac_ramp']:.0%} of ramp vs {stats['moving_frac_hold']:.0%} of hold samples; "
            f"|load| median ramp {stats['load_med_ramp_pct']:.1f} % / hold {stats['load_med_hold_pct']:.1f} %; "
            f"current max {stats['current_max_units']} units"
        )
        table = Table(title="stops", title_justify="left")
        for col in ["target", "dir", "overshoot", "settle ms (±2)", "Moving→0 ms", "final err"]:
            table.add_column(col, justify="right")
        for s in stats["stops"]:
            table.add_row(str(s["target"]), f"{s['direction']:+d}", str(s["overshoot_ticks"]),
                          "—" if s["settle_ms"] is None else f"{s['settle_ms']:.0f}",
                          "—" if s["moving_zero_after_ms"] is None else f"{s['moving_zero_after_ms']:.0f}",
                          str(s["final_error"]))  # fmt: skip
        console.print(table)
    console.print(f"bus failures: {dict(bus.stats.failures) or 0}   logger: {summary}")


if __name__ == "__main__":
    main()
