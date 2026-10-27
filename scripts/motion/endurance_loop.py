"""Step 10c: endurance / thermal loop (Phase 2 drift baseline). Default 15 minutes.

    uv run scripts/motion/endurance_loop.py --arm-moves                     # 15 min, no TUI/Rerun (timing)
    uv run scripts/motion/endurance_loop.py --arm-moves --minutes 5 --tui
    uv run scripts/motion/endurance_loop.py --arm-moves --sim --yes --minutes 0.5    # SimBus

Place the arm BY HAND in a pose clear of the table (as for the Cartesian demo) and hold it: that pose is the
REFERENCE. Each cycle (~60 s):
  1. approach the reference from below on every joint (40-tick overshoot), wait_until_settled, record a
     manifest entry (snapshots before/after, settle result): repeatability vs temperature;
  2. exercise: min-jerk out-and-back moves of pan (+-250), lift (+-150) and elbow (+-150) around the reference.
Every planned move is checked before torque-on (limits + table, inside-margin rule); at runtime SafeArm checks
every goal against the table, contact detection and all trips (55 degC) stay active. Ends holding at the
reference: support the arm, Ctrl-C = torque off. Ctrl-C during the run = freeze, again = torque off.
Writes data/raw/<run_id>/ (kind endurance) + manifest.jsonl + safety_events.json + endurance.json.
"""

import argparse
import json
import signal
import sys
import threading
import time
from statistics import median

import numpy as np
import pyarrow.parquet as pq
from rich.console import Console
from rich.table import Table

from so101.config import REPO_ROOT, load_arm_config
from so101.hw.arm import Arm
from so101.hw.bus import FeetechBus
from so101.hw.sim import SimBus
from so101.kinematics.collision import CollisionModel
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel
from so101.motion.loop import Segment, control_loop, sequence
from so101.motion.settle import approach
from so101.runlog.manifest import Manifest
from so101.runlog.run import Run
from so101.safety import SafeArm, SafetyConfig, SafetyTrip
from so101.telemetry.live import LiveSession

EXERCISE = {"shoulder_pan": 250, "shoulder_lift": 150, "elbow_flex": 150}  # +- ticks around the reference
REF_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]
SHORT = {"shoulder_pan": "pan", "shoulder_lift": "lift", "elbow_flex": "elbow", "wrist_flex": "wrist_flex"}


def exercise_plan(ref: dict[str, float], speed: float, reps: int):
    segs, pose = [], dict(ref)
    for _ in range(reps):
        for j, amp in EXERCISE.items():
            for target in (ref[j] + amp, ref[j] - amp, ref[j]):
                nxt = {**pose, j: target}
                segs.append(Segment(pose, nxt, speed, "minjerk"))
                pose = nxt
    return sequence(segs, settle_s=0.2)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm-moves", action="store_true", help="required for every scripts/motion/ script")
    ap.add_argument("--minutes", type=float, default=15.0)
    ap.add_argument("--speed", type=float, default=150.0)
    ap.add_argument("--reps", type=int, default=1, help="exercise repetitions per cycle (1 = ~35 s cycles, ~25 reference visits in 15 min)")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--tui", action="store_true")
    ap.add_argument("--viz", action=argparse.BooleanOptionalAction, default=False, help="off by default (timing)")
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--notes", default="")
    ap.add_argument("--sim", action="store_true")
    args = ap.parse_args()
    if not args.arm_moves:
        sys.exit("refusing to run without --arm-moves (scripts/motion/ convention)")
    if not 0 < args.minutes <= 30 or not 0 < args.speed <= 150:
        sys.exit("stage limits: minutes 0..30, speed <= 150 ticks/s")

    console = Console()
    cfg = load_arm_config("follower")
    safety = SafetyConfig.from_arm_config(cfg)
    mapping = load_mapping("follower")
    collision = CollisionModel.from_safety(mapping, safety)
    if args.sim:
        bus = SimBus.from_dump(json.loads((REPO_ROOT / "data/raw/20261026-112606_regdump_follower/registers.json").read_text()))
        for j, t in dict(shoulder_lift=1673, elbow_flex=1600, wrist_flex=2400, wrist_roll=2156).items():
            bus.memory[cfg.joints[j]][56] = t  # simulated hand-placed clear pose
    else:
        bus = FeetechBus(cfg.port, cfg.baudrate, cfg.protocol_end)
    snap = Arm(bus, cfg.joints).snapshot()
    if not snap.ok:
        bus.close()
        sys.exit(f"incomplete reading: {snap.errors}")
    ticks0 = {j: s.position for j, s in snap.joints.items()}
    if bad := mapping.check(ticks0, ArmModel(mapping.model_path).ranges):
        bus.close()
        sys.exit(f"refusing: pose implausible for the mapping: {bad}")
    ref = {j: float(ticks0[j]) for j in REF_JOINTS}

    # --- validate the exercise path and the reference approach before torque-on ----------------------
    problems = []
    for j, amp in EXERCISE.items():
        lo, hi = safety.limits[j]
        if not (lo <= ref[j] - amp - 40 and ref[j] + amp <= hi):
            problems.append(f"{j}: {ref[j]:.0f} ± {amp} (+40 overshoot) leaves its limits [{lo}, {hi}]")
    fn, ex_total = exercise_plan(ref, args.speed, 1)
    start_rep = collision.check_ticks(ticks0)
    worst = float("inf")
    for t in np.arange(0, ex_total, 0.02):
        goals = fn(t) or {}
        rep = collision.check_ticks({**ticks0, **{k: round(v) for k, v in goals.items()}})
        worst = min(worst, rep.min_table)
        if v := collision.table_violations(rep, start_rep):
            problems.append(f"exercise at t={t:.1f} s: {v[0]}")
            break
    for j in REF_JOINTS:  # approach from below: 40 ticks under the reference
        rep = collision.check_ticks({**ticks0, j: ticks0[j] - 40})
        if v := collision.table_violations(rep, start_rep):
            problems.append(f"approach overshoot on {j}: {v[0]}")
    cycle_s = ex_total * args.reps + 6
    console.print(f"reference pose {dict((k, int(v)) for k, v in ref.items())}; exercise ±{EXERCISE} ticks at "
                  f"{args.speed:g} t/s, {args.reps} reps/cycle (≈ {cycle_s:.0f} s per cycle), {args.minutes:g} min; "
                  f"closest to table along the exercise {worst * 1000:+.0f} mm")  # fmt: skip
    if problems:
        bus.close()
        sys.exit("refused before torque-on:\n  " + "\n  ".join(problems))
    if not args.yes:
        try:
            input("Hold the arm in this pose. Enter = enable torque and start the loop (Ctrl-C aborts)… ")
        except KeyboardInterrupt:
            bus.close()
            sys.exit("\naborted before enabling")

    run = Run("endurance_sim" if args.sim else "endurance", "follower", config_paths=[cfg.path, mapping.path],
              notes=f"step 10c {args.minutes} min ref={ref}; sim={args.sim}; {args.notes}")  # fmt: skip
    manifest = Manifest(run, "endurance")
    live = LiveSession(run, {"follower": cfg.expected_voltage_v}, args.tui, args.viz,
                       title=f"[TORQUE ON: ENDURANCE {args.minutes:g} min] {run.run_id} — Ctrl-C = freeze, again = torque off")  # fmt: skip
    result: dict = {"cycles": 0, "marks": {}}
    ex_fn, ex_total = exercise_plan(ref, args.speed, args.reps)

    def worker() -> None:
        try:
            with SafeArm(bus, cfg, hub=live.hub, collision=collision, mapping=mapping) as arm:
                result["arm"] = arm
                arm.enable()
                t_end = time.monotonic() + args.minutes * 60
                result["marks"]["start_ns"] = time.monotonic_ns()
                while time.monotonic() < t_end and not live.stop.is_set():
                    before = arm.step().snapshot
                    r = approach(arm, ref, {j: 1 for j in REF_JOINTS}, rate_hz=args.rate, dwell_s=0.3,
                                 stop=live.stop, freeze=live.freeze)  # fmt: skip
                    if r is None:
                        break  # frozen (contact / predicted collision / Ctrl-C) or stopped
                    after = arm.step().snapshot
                    manifest.add("ref", commanded_ticks={j: int(v) for j, v in ref.items()}, before=before, after=after,
                                 settle={"settled": r.settled, "t_encoder_s": r.t_encoder_s, "errors": r.errors,
                                         "approach_dir": r.approach_dir},
                                 extra={"cycle": result["cycles"], "t_s": (time.monotonic_ns() - result["marks"]["start_ns"]) / 1e9})  # fmt: skip
                    control_loop(arm, args.rate, ex_fn, ex_total, live.stop, freeze=live.freeze)
                    if arm.frozen:
                        break
                    result["cycles"] += 1
                result["marks"]["end_ns"] = time.monotonic_ns()
                # hold until released: frozen (Ctrl-C/contact) or normal end at the reference
                result["holding_at_end"] = True
                control_loop(arm, args.rate, lambda t: None, float("inf"), live.stop)
                result["disable_failed"] = arm.disable()
        except SafetyTrip as e:
            result["trip"] = e
        except BaseException as e:
            result["error"] = e

    def status():
        a = result.get("arm")
        if a is not None and (a.contact_hit or a.collision_hit):
            return f"FROZEN ({'contact ' + a.contact_hit.joint if a.contact_hit else a.collision_hit}): support the arm, Ctrl-C = torque off"
        if result.get("holding_at_end"):
            return "HOLDING (loop done): support the arm, then Ctrl-C = torque off"
        return f"cycle {result['cycles']}"

    live.status_fn = status
    live.ctrl_c_stops_fn = lambda: bool(((a := result.get("arm")) is not None and a.frozen) or result.get("holding_at_end"))
    th = threading.Thread(target=worker, name="safearm-endurance")
    th.start()
    if not args.tui:
        console.print("[bold]running[/] (no TUI); status lines print on change. Ctrl-C = freeze, again = torque off")
    live.run_until_done(th, console)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    manifest.close()
    summary = live.close()
    bus.close()

    arm = result.get("arm")
    if arm:
        run.write_json("safety_events.json", arm.report())
    from so101.runlog.manifest import read_manifest

    _, poses = read_manifest(run.dir / "manifest.jsonl")
    visits = [{"t_s": p["extra"]["t_s"], "settled": p["settle"]["settled"],
               "error": p["settle"]["errors"],
               "temp": {j: v["temperature_c"] for j, v in p["snapshot_after"]["joints"].items() if v}} for p in poses]  # fmt: skip
    temps = {}
    if summary["rows"] and "start_ns" in result["marks"]:
        rows = pq.read_table(run.dir / "telemetry").to_pylist()
        m0, m1 = result["marks"]["start_ns"], result["marks"].get("end_ns", 1 << 62)
        for j in cfg.joints:
            xs = [r for r in rows if r["joint"] == j and r["temperature_c"] is not None and m0 <= r["t_start_ns"] <= m1]
            if len(xs) > 200:
                first = median(r["temperature_c"] for r in xs[:500])
                last = median(r["temperature_c"] for r in xs[-500:])
                temps[j] = {"start_c": first, "end_c": last, "rise_c": last - first,
                            "load_med_pct": median(abs(r["load"]) for r in xs) / 10}  # fmt: skip
    run.write_json("endurance.json", {"minutes": args.minutes, "cycles": result["cycles"], "reference": ref,
                                      "visits": visits, "temperature": temps,
                                      "trip": str(result.get("trip")) if "trip" in result else None,
                                      "error": repr(result["error"]) if "error" in result else None, "logger": summary})  # fmt: skip

    console.print(f"[bold]{run.run_id}[/] ({'SIMULATION' if args.sim else 'real follower'}): {result['cycles']} cycles, "
                  f"{len(visits)} reference visits")  # fmt: skip
    for k in ("trip", "error"):
        if k in result:
            console.print(f"[red]{k}: {result[k]}")
    if temps:
        t = Table(title="temperature (median of first/last 5 s of the loop)", title_justify="left")
        for col in ["joint", "start °C", "end °C", "rise", "median |load| %"]:
            t.add_column(col, justify="left" if col == "joint" else "right")
        for j, v in temps.items():
            t.add_row(j, f"{v['start_c']:g}", f"{v['end_c']:g}", f"{v['rise_c']:+g}", f"{v['load_med_pct']:.1f}")
        console.print(t)
    if visits:
        errs = {j: [v["error"].get(j) for v in visits if v["error"].get(j) is not None] for j in REF_JOINTS}
        console.print("reference-pose repeatability (error at settle, ticks; same approach direction): "
                      + ", ".join(f"{SHORT[j]} {min(e)}..{max(e)}" for j, e in errs.items() if e))  # fmt: skip
    console.print(f"bus failures: {dict(bus.stats.failures) or 0}   logger: {summary}")


if __name__ == "__main__":
    main()
