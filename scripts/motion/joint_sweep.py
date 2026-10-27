"""Step 6: per-joint sweep with holds at waypoints. One joint moves; every other joint holds.

    uv run scripts/motion/joint_sweep.py --arm-moves --joint shoulder_pan --tui
    uv run scripts/motion/joint_sweep.py --arm-moves --joint shoulder_lift --tui          # anchored (auto)
    uv run scripts/motion/joint_sweep.py --arm-moves --joint wrist_flex --profile generic --fraction 0.5
    uv run scripts/motion/joint_sweep.py --arm-moves --joint elbow_flex --sim --yes       # SimBus, no hardware

Profiles (so101.motion.sweep): `generic` = 7 waypoints across the middle --fraction of the limits, out
and back; `anchored` = --steps waypoints of --step ticks from rest toward mid-range, out and back.
`auto` (default) = anchored for wrist_flex / elbow_flex / shoulder_lift, generic for the others.
Joints that start outside their limits enter the range first (50 ticks/s) and end there.

Before torque goes on, the plan (waypoints, extent, duration) is printed and you press Enter (--yes to
skip). Stage limits: speed <= 150 ticks/s, fraction <= 0.8, anchored extent <= 900 ticks.
Ctrl-C once = freeze (hold, torque on); again = torque off; cutting 12 V always works. Keep the
terminal focused. For gravity joints keep a hand under the arm: a trip or the 2nd Ctrl-C drops it.
Writes data/raw/<run_id>/ (kind sweep) + safety_events.json + sweep.json.
"""

import argparse
import json
import sys
import threading
import time

import pyarrow.parquet as pq
from rich.console import Console
from rich.table import Table

from so101.analysis.moves import STICK_TOL, analyse, hysteresis
from so101.config import REPO_ROOT, load_arm_config
from so101.hw.arm import Arm
from so101.hw.bus import FeetechBus
from so101.hw.sim import SimBus
from so101.motion.loop import control_loop
from so101.motion.sweep import GRAVITY_JOINTS, plan_sweep, sweep_goals
from so101.runlog.run import Run
from so101.safety import SafeArm, SafetyConfig, SafetyError, SafetyTrip
from so101.telemetry.live import LiveSession

MAX_SPEED, MAX_FRACTION, MAX_ANCHORED = 150, 0.8, 900
START_TOL = 20  # ticks the joint may drift between planning and enabling


def fmt(v, spec="{:+.1f}"):
    return "—" if v is None else spec.format(v)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm-moves", action="store_true", help="required for every scripts/motion/ script")
    ap.add_argument("--joint", required=True)
    ap.add_argument("--profile", choices=["auto", "generic", "anchored"], default="auto")
    ap.add_argument("--fraction", type=float, default=0.5)
    ap.add_argument("--waypoints", type=int, default=7)
    ap.add_argument("--step", type=int, default=150)
    ap.add_argument("--steps", type=int, default=4)
    ap.add_argument("--speed", type=float, default=150.0)
    ap.add_argument("--hold-s", type=float, default=1.0)
    ap.add_argument("--after-s", type=float, default=1.5)
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--tui", action="store_true")
    ap.add_argument("--viz", action=argparse.BooleanOptionalAction, default=True,
                    help="live Rerun plots, saved as telemetry.rrd (default on; the viewer settles before torque-on)")
    ap.add_argument("--yes", action="store_true", help="don't wait for Enter after printing the plan")
    ap.add_argument("--notes", default="")
    ap.add_argument("--sim", action="store_true", help="use SimBus instead of the arm")
    args = ap.parse_args()
    if not args.arm_moves:
        sys.exit("refusing to run without --arm-moves (scripts/motion/ convention)")
    if not 0 < args.speed <= MAX_SPEED or not 0 < args.fraction <= MAX_FRACTION:
        sys.exit(f"step 6 limits: 0 < speed <= {MAX_SPEED}, 0 < fraction <= {MAX_FRACTION}")
    if args.step * args.steps > MAX_ANCHORED:
        sys.exit(f"step 6 limit: anchored extent step*steps <= {MAX_ANCHORED} ticks")

    console = Console()
    cfg = load_arm_config("follower")
    if args.joint not in cfg.joints:
        sys.exit(f"unknown joint {args.joint}; one of {list(cfg.joints)}")
    profile = args.profile if args.profile != "auto" else ("anchored" if args.joint in GRAVITY_JOINTS else "generic")
    lo, hi = SafetyConfig.from_arm_config(cfg).limits[args.joint]

    if args.sim:
        dump = REPO_ROOT / "data/raw/20261026-112606_regdump_follower/registers.json"
        bus = SimBus.from_dump(json.loads(dump.read_text()), glitch_rate=0.01)
    else:
        bus = FeetechBus(cfg.port, cfg.baudrate, cfg.protocol_end)

    # --- plan from the current position (read-only, torque off) and confirm ----------------
    start_state = Arm(bus, cfg.joints).snapshot().joints[args.joint]
    if start_state is None:
        bus.close()
        sys.exit(f"{args.joint}: no reading")
    start = start_state.position
    try:
        plan = plan_sweep(args.joint, start, lo, hi, profile, fraction=args.fraction, n=args.waypoints,
                          step=args.step, steps=args.steps)  # fmt: skip
    except ValueError as e:
        bus.close()
        sys.exit(f"plan refused: {e}")
    goal_fn, total = sweep_goals(plan, args.speed, args.hold_s)
    far = max(plan.path, key=lambda w: abs(w - plan.home))
    console.print(f"[bold]{args.joint}[/] {profile} sweep, limits [{lo}, {hi}], start {start}"
                  + (f" (outside: enters range at {plan.entry:.0f} first, ends there)" if plan.entry is not None else ""))  # fmt: skip
    console.print(f"  waypoints: {[int(w) for w in plan.path]}  → home {plan.home:.0f}")
    console.print(f"  furthest point {far:.0f}: {abs(far - plan.home):.0f} ticks ≈ {abs(far - plan.home) * 360 / 4096:.0f}° "
                  f"from home; {args.speed:g} ticks/s, hold {args.hold_s:g} s; ≈ {total:.0f} s total")  # fmt: skip
    if args.joint in GRAVITY_JOINTS:
        console.print("  [bold yellow]gravity joint: keep a hand under the arm (a trip or 2nd Ctrl-C drops it)")
    if not args.yes:
        try:
            input("Press Enter to enable torque and start (Ctrl-C to abort, nothing has been written)… ")
        except KeyboardInterrupt:
            bus.close()
            sys.exit("\naborted before enabling")

    run = Run("sweep_sim" if args.sim else "sweep", "follower", config_paths=[cfg.path],
              notes=f"step 6 {args.joint} {profile} path={[int(w) for w in plan.path]} speed={args.speed:g}; "
                    f"sim={args.sim}; {args.notes}")  # fmt: skip
    live = LiveSession(run, {"follower": cfg.expected_voltage_v}, args.tui, args.viz,
                       title=f"[TORQUE ON: SWEEP {args.joint}] {run.run_id} — Ctrl-C = freeze, again = torque off")  # fmt: skip
    result: dict = {"marks": {}}
    marks = result["marks"]

    def worker() -> None:
        try:
            with SafeArm(bus, cfg, hub=live.hub) as arm:
                result["arm"] = arm
                now = arm.step().snapshot.joints[args.joint]
                if now is None or abs(now.position - start) > START_TOL:
                    raise SafetyError(f"{args.joint} moved since planning ({start} → {now and now.position}); not enabling")
                result["enable"] = arm.enable()
                marks["move_start_ns"] = time.monotonic_ns()
                result["loop"] = control_loop(arm, args.rate, goal_fn, total, live.stop, freeze=live.freeze)
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

    live.status_fn = lambda: (f"FROZEN by contact detection ({a.contact_hit.joint}, {a.contact_hit.role}, "
                              f"{abs(a.contact_hit.error)} ticks): support the arm, then Ctrl-C = torque off"
                              if (a := result.get("arm")) is not None and a.contact_hit else None)  # fmt: skip
    th = threading.Thread(target=worker, name="safearm-sweep")
    th.start()
    live.footer = [f"TORQUE ON: {profile} sweep of {args.joint}. Ctrl-C = freeze, again = torque off."]
    interrupted = live.run_until_done(th, console)
    summary = live.close()
    bus.close()

    arm, loop = result.get("arm"), result.get("loop")
    if arm:
        run.write_json("safety_events.json", arm.report())
    stats = analyse(pq.read_table(run.dir / "telemetry").to_pylist(), args.joint) if summary["rows"] else {}
    hyst = hysteresis(stats.get("stops", []))
    run.write_json("sweep.json", {
        "joint": args.joint, "profile": profile, "limits": [lo, hi], "start": start,
        "plan": {"entry": plan.entry, "path": plan.path, "home": plan.home, "total_s": total},
        "speed": args.speed, "hold_s": args.hold_s, "interrupted": interrupted,
        "frozen": bool(loop and loop.frozen),
        "trip": str(result["trip"]) if "trip" in result else None,
        "error": repr(result["error"]) if "error" in result else None,
        "loop": vars(loop) if loop else None, "disable_failed": result.get("disable_failed"),
        "marks_ns": marks, "analysis": stats, "hysteresis": hyst,
        "clamps": {f"{j}:{k}": n for (j, k), n in arm.clamps.items()} if arm else None,
        "bus_failures": {f"{k[0]}:{k[1]}": v for k, v in bus.stats.failures.items()}, "logger": summary,
    })  # fmt: skip

    console.print(f"[bold]{run.run_id}[/] ({'SIMULATION' if args.sim else 'real follower'})")
    if "trip" in result:
        console.print(f"[red]{result['trip']}[/] → torque off on all joints")
    elif "error" in result:
        console.print(f"[red]stopped (torque off): {result['error']}")
    elif loop and loop.frozen:
        console.print(f"[yellow]FROZEN by contact detection: {arm.contact_hit.detail}; held, then torque off"
                      if arm and arm.contact_hit else "[yellow]FROZEN by Ctrl-C, held, then torque off; no trips")
    else:
        console.print("[green]sweep completed, torque off, no trips")
    if loop:
        console.print(f"loop: {loop.cycles} cycles in {loop.duration_s:.1f} s, overruns {loop.overruns}, skipped {loop.skipped_slots}")
    if arm:
        console.print(f"clamps: {dict((f'{j}:{k}', n) for (j, k), n in arm.clamps.items()) or 'none'}")
    if result.get("disable_failed"):
        console.print(f"[red]could not confirm torque off on {result['disable_failed']} — cut 12 V")
    if stats.get("stuck_frac_while_off_goal") is not None:
        console.print(f"stick-slip: stuck in {stats['stuck_frac_while_off_goal']:.0%} of samples while > {STICK_TOL} "
                      f"ticks off goal; longest stall {stats['longest_stall_ms']:.0f} ms; ramp lag median "
                      f"{fmt(stats['ramp_err_median'], '{:.0f}')} ticks")  # fmt: skip
    if hyst:
        t = Table(title=f"{args.joint}: steady state per waypoint (last 0.5 s of each hold)", title_justify="left")
        for col in ["target", "err ↑", "err ↓", "hysteresis", "load ↑ %", "load ↓ %", "jitter"]:
            t.add_column(col, justify="right")
        for h in hyst:
            t.add_row(str(h["target"]), fmt(h["err_up"]), fmt(h["err_down"]), fmt(h["hysteresis"]),
                      fmt(h["load_up_pct"]), fmt(h["load_down_pct"]), str(h["jitter_max"]))  # fmt: skip
        console.print(t)
    console.print(f"bus failures: {dict(bus.stats.failures) or 0}   logger: {summary}")


if __name__ == "__main__":
    main()
