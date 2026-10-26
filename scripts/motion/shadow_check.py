"""Step 5a: SafeArm in SHADOW mode on the real follower. Torque stays off; nothing is written.

    uv run scripts/motion/shadow_check.py --arm-moves --tui --viz
    uv run scripts/motion/shadow_check.py --arm-moves --seconds 120
    uv run scripts/motion/shadow_check.py --arm-moves --sim --seconds 10   # same script on SimBus, no hardware

The first script that runs SafeArm against the real bus, so it lives in scripts/motion/ and needs
--arm-moves, although nothing here can move the arm. Two independent guards:
  1. SafeArm(shadow=True): enable() is refused and every write raises inside SafeArm.
  2. This script replaces bus.write with a function that raises and counts attempts (must stay 0).

What it checks on hardware:
- preflight: all six servos answer, torque is off
- the control loop at --rate on real reads (timing, overruns)
- trip checks on live data for --seconds: no false trips from the real ADC glitches
- goals from the same out-and-back plan as the sim demo, validated against real positions and
  logged (goal column). With torque off nothing moves, so goals are clamped to present +/- max_step;
  move joints by hand to see the clamps follow.
Writes data/raw/<run_id>/ (kind shadow) + safety_events.json + shadow_check.json.
"""

import argparse
import json
import sys
import threading

from rich.console import Console

from so101.config import REPO_ROOT, load_arm_config
from so101.hw.bus import BusError, FeetechBus
from so101.hw.sim import SimBus
from so101.motion.loop import control_loop, out_and_back
from so101.runlog.run import Run
from so101.safety import SafeArm, SafetyTrip
from so101.telemetry.live import LiveSession


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm-moves", action="store_true", help="required for every scripts/motion/ script")
    ap.add_argument("--tui", action="store_true")
    ap.add_argument("--viz", action=argparse.BooleanOptionalAction, default=True,
                    help="live Rerun plots, saved as telemetry.rrd (default on; the viewer settles before torque-on)")
    ap.add_argument("--seconds", type=float, default=60.0)
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--notes", default="")
    ap.add_argument("--sim", action="store_true", help="use SimBus (factory dump + glitches) instead of the arm")
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
    attempts: list[tuple] = []

    def refuse_write(*a, **k):
        attempts.append(a)
        raise BusError("shadow_check: bus.write is disabled in this script")

    bus.write = refuse_write  # guard 2, independent of SafeArm's shadow flag

    kind = "shadow_sim" if args.sim else "shadow"
    run = Run(kind, "follower", config_paths=[cfg.path], notes=f"step 5a shadow check; sim={args.sim}; {args.notes}")
    live = LiveSession(run, {"follower": cfg.expected_voltage_v}, args.tui, args.viz,
                       title=f"[SHADOW: torque off, no writes] {run.run_id} — Ctrl-C = freeze, again = stop")  # fmt: skip
    result: dict = {}

    def worker() -> None:
        try:
            with SafeArm(bus, cfg, hub=live.hub, shadow=True) as arm:
                result["arm"] = arm
                start = {j: s.position for j, s in arm.step().snapshot.joints.items() if s is not None}
                plan, period = out_and_back(start, arm.safety.limits, amplitude=200, speed=150)
                result["loop"] = control_loop(arm, args.rate, lambda t: plan(t % period), args.seconds, live.stop, freeze=live.freeze)
        except SafetyTrip as e:
            result["trip"] = e
        except BaseException as e:
            result["error"] = e

    th = threading.Thread(target=worker, name="safearm-shadow")
    th.start()
    live.footer = ["SHADOW MODE: torque off, all writes blocked (2 guards)"]
    interrupted = live.run_until_done(th, console)
    summary = live.close()
    bus.close()

    arm = result.get("arm")
    if arm:
        run.write_json("safety_events.json", arm.report())
    loop = result.get("loop")
    run.write_json("shadow_check.json", {
        "write_attempts": len(attempts),
        "interrupted": interrupted,
        "trip": str(result["trip"]) if "trip" in result else None,
        "error": repr(result["error"]) if "error" in result else None,
        "loop": vars(loop) if loop else None,
        "bus_failures": {f"{k[0]}:{k[1]}": v for k, v in bus.stats.failures.items()},
        "bus_servo_errors": {f"{k[0]}:{k[1]}": v for k, v in bus.stats.servo_errors.items()},
        "logger": summary,
    })  # fmt: skip

    console.print(f"[bold]{run.run_id}[/] (shadow: torque off, no writes)")
    console.print(f"write attempts reaching the bus: {len(attempts)}" + ("" if not attempts else "  [red]← BUG"))
    if loop:
        console.print(f"loop: {loop.cycles} cycles in {loop.duration_s:.1f} s, overruns {loop.overruns}, "
                      f"skipped {loop.skipped_slots}")  # fmt: skip
    if "trip" in result:
        console.print(f"[red]{result['trip']}[/]  ← with torque off this is a FALSE trip or a real fault; investigate")
    elif "error" in result:
        console.print(f"[red]error: {result['error']!r}")
    else:
        console.print("[green]no trips")
    if arm:
        console.print(f"clamps: {dict((f'{j}:{k}', n) for (j, k), n in arm.clamps.items()) or 'none'}")
        console.print("events: " + (", ".join(f"{e.kind}({e.joint or 'arm'})" for e in arm.events) or "none"))
    console.print(f"bus: failures {dict(bus.stats.failures) or 0}, servo error flags {dict(bus.stats.servo_errors) or 0}")
    console.print(f"logger: {summary}")


if __name__ == "__main__":
    main()
