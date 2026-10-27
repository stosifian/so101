"""Step 10b: gripper squeeze. Load and current vs squeeze depth; doubles as the Present_Current scale check.

    uv run scripts/motion/gripper_squeeze.py --arm-moves --tui
    uv run scripts/motion/gripper_squeeze.py --arm-moves --torque-limit 30 --step 30
    uv run scripts/motion/gripper_squeeze.py --arm-moves --sim --yes             # SimBus, no hardware

The arm stays RESTING on its stops (other joints hold, idle). Only the gripper moves:
  open to --open-ticks -> pause --place-s (place a soft object between the jaws now, or leave it empty for a
  jaw-on-jaw baseline) -> close in --step tick steps down to --close-ticks (past closed), holding --hold-s
  each -> open -> pause --remove-s (take the object out) -> back to the start reading -> torque off.
The gripper's torque is capped at --torque-limit % (SRAM Torque_Limit, restored to 100 % on exit) and it is
exempt from contact detection (squeezing IS a contact). Its load trip is 80 % (config override); all other
trips and contact detection on the other joints stay active. Ctrl-C once = freeze, again = torque off.
Writes data/raw/<run_id>/ (kind squeeze) + safety_events.json + squeeze.json.
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
from so101.motion.loop import Segment, control_loop
from so101.runlog.run import Run
from so101.safety import SafeArm, SafetyConfig, SafetyTrip
from so101.telemetry.live import LiveSession

J = "gripper"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm-moves", action="store_true", help="required for every scripts/motion/ script")
    ap.add_argument("--open-ticks", type=int, default=2700, help="open position (~46 deg open)")
    ap.add_argument("--close-ticks", type=int, default=2100, help="final squeeze goal (closed ~2172; below = past closed)")
    ap.add_argument("--step", type=int, default=40, help="ticks per closing step")
    ap.add_argument("--hold-s", type=float, default=1.5)
    ap.add_argument("--place-s", type=float, default=6.0)
    ap.add_argument("--remove-s", type=float, default=4.0)
    ap.add_argument("--torque-limit", type=float, default=50.0, help="gripper torque cap, %% (LeRobot uses 50)")
    ap.add_argument("--speed", type=float, default=150.0)
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--tui", action="store_true")
    ap.add_argument("--viz", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--notes", default="")
    ap.add_argument("--sim", action="store_true")
    args = ap.parse_args()
    if not args.arm_moves:
        sys.exit("refusing to run without --arm-moves (scripts/motion/ convention)")
    if not 0 < args.torque_limit <= 60 or not 10 <= args.step <= 100:
        sys.exit("stage limits: torque limit 0..60 %, step 10..100 ticks")

    console = Console()
    cfg = load_arm_config("follower")
    lo, hi = SafetyConfig.from_arm_config(cfg).limits[J]
    if args.sim:
        bus = SimBus.from_dump(json.loads((REPO_ROOT / "data/raw/20261026-112606_regdump_follower/registers.json").read_text()))
    else:
        bus = FeetechBus(cfg.port, cfg.baudrate, cfg.protocol_end)
    start = Arm(bus, cfg.joints).snapshot().joints[J]
    if start is None:
        bus.close()
        sys.exit("no gripper reading")
    g0 = float(start.position)
    steps = list(range(args.open_ticks - args.step, args.close_ticks - 1, -args.step))
    if steps[-1] != args.close_ticks:
        steps.append(args.close_ticks)
    console.print(f"gripper at {g0:.0f} (limits [{lo}, {hi}]; goals below {lo} only move back toward the range "
                  f"if it starts outside, else they are clamped to {lo})")  # fmt: skip
    console.print(f"plan: open → {args.open_ticks}, pause {args.place_s:g} s (place object), close in {len(steps)} steps "
                  f"{steps[0]} → {steps[-1]} ({args.hold_s:g} s each), open, pause {args.remove_s:g} s, back to {g0:.0f}; "
                  f"torque cap {args.torque_limit:g} %")  # fmt: skip
    if args.close_ticks < lo:
        console.print(f"[yellow]note: --close-ticks {args.close_ticks} is below the gripper's software limit {lo}; "
                      f"SafeArm will clamp the deepest steps to {lo} (≈ {(lo - 2172) * 360 / 4096:+.1f}° from closed)")  # fmt: skip
    if not args.yes:
        try:
            input("Arm resting on its stops? Enter = enable torque and start (Ctrl-C aborts)… ")
        except KeyboardInterrupt:
            bus.close()
            sys.exit("\naborted before enabling")

    # gripper goal schedule (other joints get no goals and hold)
    step_starts = []  # (goal, hold window start, end) per squeeze step
    t = 0.0
    timeline: list[tuple[float, Segment, float, str]] = []  # (t_start, segment, hold after it, label)

    def add(seg: Segment, hold: float, label: str):
        nonlocal t
        timeline.append((t, seg, hold, label))
        t += seg.duration_s + hold

    add(Segment({J: g0}, {J: float(args.open_ticks)}, args.speed, "minjerk"), args.place_s, "open + place object")
    prev = float(args.open_ticks)
    for g in steps:
        add(Segment({J: prev}, {J: float(g)}, args.speed, "minjerk"), args.hold_s, f"squeeze step {g}")
        step_starts.append((g, t - args.hold_s, t))  # the hold window of this step
        prev = float(g)
    add(Segment({J: prev}, {J: float(args.open_ticks)}, args.speed, "minjerk"), args.remove_s, "open + remove object")
    add(Segment({J: float(args.open_ticks)}, {J: g0}, args.speed, "minjerk"), 0.5, "back to start")
    total = t

    def goal_fn(tt: float):
        for t0, seg, hold, _ in reversed(timeline):
            if tt >= t0:
                return seg.at(tt - t0)
        return None

    def phase(tt: float) -> str:
        for t0, seg, hold, label in reversed(timeline):
            if tt >= t0:
                return label
        return ""

    run = Run("squeeze_sim" if args.sim else "squeeze", "follower", config_paths=[cfg.path],
              notes=f"step 10b gripper squeeze cap {args.torque_limit}% steps {steps[0]}..{steps[-1]}; sim={args.sim}; "
                    f"{args.notes}")  # fmt: skip
    live = LiveSession(run, {"follower": cfg.expected_voltage_v}, args.tui, args.viz,
                       title=f"[TORQUE ON: GRIPPER SQUEEZE] {run.run_id} — Ctrl-C = freeze, again = torque off")  # fmt: skip
    result: dict = {"marks": {}}
    marks = result["marks"]

    def worker() -> None:
        try:
            with SafeArm(bus, cfg, hub=live.hub) as arm:
                result["arm"] = arm
                arm.set_contact_exempt(J)
                arm.enable()
                arm.set_torque_limit(J, args.torque_limit)
                marks["start_ns"] = time.monotonic_ns()
                result["loop"] = control_loop(arm, args.rate, goal_fn, total, live.stop, freeze=live.freeze,
                                              on_tick=lambda tt: result.__setitem__("phase", phase(tt)))  # fmt: skip
                result["disable_failed"] = arm.disable()
        except SafetyTrip as e:
            result["trip"] = e
        except BaseException as e:
            result["error"] = e

    def status():
        a = result.get("arm")
        if a is not None and a.contact_hit:
            return f"FROZEN by contact detection ({a.contact_hit.joint}): support the arm, then Ctrl-C = torque off"
        ph = result.get("phase", "")
        if ph.startswith("open + place"):
            return "PLACE a soft object between the jaws now (or leave empty)"
        if ph.startswith("open + remove"):
            return "REMOVE the object now"
        return f"phase: {ph}" if ph else None

    live.status_fn = status
    live.ctrl_c_stops_fn = lambda: bool(((a := result.get("arm")) is not None and a.frozen) or result.get("holding_at_end"))
    th = threading.Thread(target=worker, name="safearm-squeeze")
    th.start()
    live.footer = [f"TORQUE ON: gripper squeeze (cap {args.torque_limit:g} %). Ctrl-C = freeze, again = torque off."]
    live.run_until_done(th, console)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    summary = live.close()
    bus.close()

    arm, loop = result.get("arm"), result.get("loop")
    if arm:
        run.write_json("safety_events.json", arm.report())
    table_rows = []
    if summary["rows"] and "start_ns" in marks:
        rows = [r for r in pq.read_table(run.dir / "telemetry").to_pylist() if r["joint"] == J and r["position"] is not None]
        for g, a, b in step_starts:
            lo_t, hi_t = marks["start_ns"] + int((b - 0.5) * 1e9), marks["start_ns"] + int(b * 1e9)  # last 0.5 s of hold
            w = [r for r in rows if lo_t <= r["t_start_ns"] <= hi_t]
            if w:
                table_rows.append({"goal": g, "pos": median(r["position"] for r in w),
                                   "err": median(r["position"] - g for r in w), "load_pct": median(r["load"] for r in w) / 10,
                                   "current_units": median(r["current"] for r in w),
                                   "current_max": max(r["current"] for r in w)})  # fmt: skip
    fit = None
    pts = [(r["load_pct"], r["current_units"]) for r in table_rows if abs(r["load_pct"]) > 3]
    if len(pts) >= 3:
        L, C = np.array(pts).T
        k, c0 = np.polyfit(np.abs(L), C, 1)
        fit = {"units_per_pct": float(k), "offset_units": float(c0), "mA_per_pct_assuming_6.5mA": float(k * 6.5)}
    run.write_json("squeeze.json", {"torque_limit_pct": args.torque_limit, "steps": table_rows, "current_vs_load": fit,
                                    "trip": str(result.get("trip")) if "trip" in result else None,
                                    "error": repr(result["error"]) if "error" in result else None,
                                    "loop": vars(loop) if loop else None, "logger": summary})  # fmt: skip

    console.print(f"[bold]{run.run_id}[/] ({'SIMULATION' if args.sim else 'real follower'})")
    for k in ("trip", "error"):
        if k in result:
            console.print(f"[red]{k}: {result[k]}")
    if table_rows:
        t = Table(title=f"gripper squeeze (cap {args.torque_limit:g} %), last 0.5 s of each step", title_justify="left")
        for col in ["goal", "position", "error (ticks)", "load %", "current (units)", "≈ mA @6.5"]:
            t.add_column(col, justify="right")
        for r in table_rows:
            t.add_row(str(r["goal"]), f"{r['pos']:.0f}", f"{r['err']:+.0f}", f"{r['load_pct']:+.1f}",
                      f"{r['current_units']:.0f}", f"{r['current_units'] * 6.5:.0f}")  # fmt: skip
        console.print(t)
    if fit:
        console.print(f"current vs |load|: {fit['units_per_pct']:.2f} units per % (≈ {fit['mA_per_pct_assuming_6.5mA']:.1f} mA/% "
                      f"if 6.5 mA/unit is right); a stalled STS3215 at 12 V draws ~2.7 A at 100 %, i.e. ~27 mA/%")  # fmt: skip
    console.print(f"bus failures: {dict(bus.stats.failures) or 0}   logger: {summary}")


if __name__ == "__main__":
    main()
