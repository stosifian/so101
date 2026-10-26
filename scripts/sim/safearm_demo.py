"""SafeArm end to end on a SIMULATED follower. No hardware: this script never opens a serial port.

    uv run scripts/sim/safearm_demo.py --tui --viz                       # sweep every joint, park, torque off
    uv run scripts/sim/safearm_demo.py --tui --inject temperature:wrist_roll:6   # watch a trip happen

The simulated arm starts in the real follower's factory register state (goal/speed/accel = 0,
elbow and gripper outside their soft limits), with the ~1 % V/degC glitches on IDs 4-6. Each joint
moves `--amplitude` ticks toward the middle of its range and back, then everything holds.
Runs write data/raw/<run_id>/ like real runs (kind sim_safearm), plus safety_events.json.
"""

import argparse
import json
import threading
from statistics import median

import pyarrow.parquet as pq
from rich.console import Console
from rich.table import Table

from so101.config import REPO_ROOT, load_arm_config
from so101.hw.sim import SimBus
from so101.motion.loop import control_loop, out_and_back
from so101.runlog.run import Run
from so101.safety import SafeArm, SafetyTrip
from so101.telemetry.live import LiveSession

FACTORY_DUMP = REPO_ROOT / "data/raw/20261026-112606_regdump_follower/registers.json"
FAULTS = {
    "temperature": lambda b, sid: b.inject(sid, "Present_Temperature", 60),
    "voltage": lambda b, sid: b.inject(sid, "Present_Voltage", 100),
    "load": lambda b, sid: b.inject(sid, "Present_Load", -700),
    "status": lambda b, sid: b.inject(sid, "Status", 0x20),
    "comm": lambda b, sid: b.dead.add(sid),
    "torque_off": lambda b, sid: b.inject(sid, "Torque_Enable", 0),
}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tui", action="store_true")
    ap.add_argument("--viz", action="store_true")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--amplitude", type=int, default=200, help="ticks per joint move (default 200, ~18 deg)")
    ap.add_argument("--speed", type=float, default=150.0, help="ramp speed, ticks/s (cap is 200)")
    ap.add_argument("--glitch-rate", type=float, default=0.01)
    ap.add_argument("--inject", default=None, help=f"REASON:JOINT:SECONDS, reason in {sorted(FAULTS)}")
    args = ap.parse_args()

    console = Console()
    cfg = load_arm_config("follower")
    bus = SimBus.from_dump(json.loads(FACTORY_DUMP.read_text()), glitch_rate=args.glitch_rate)
    run = Run("sim_safearm", "follower", config_paths=[cfg.path], notes=f"SIMULATION; inject={args.inject}")
    live = LiveSession(run, {"follower": cfg.expected_voltage_v}, args.tui, args.viz,
                       title=f"[SIM] {run.run_id} — Ctrl-C to stop")  # fmt: skip

    fault = None
    if args.inject:
        reason, joint, at = args.inject.split(":")
        fault = (FAULTS[reason], cfg.joints[joint], float(at))

    result = {}

    def worker() -> None:  # SafeArm and every bus call live in this one thread
        try:
            with SafeArm(bus, cfg, hub=live.hub) as arm:
                result["arm"] = arm
                report = arm.enable()
                goal_fn, total = out_and_back(report.present, arm.safety.limits, args.amplitude, args.speed)
                result["planned_s"] = total
                done = threading.Event()

                def on_tick(t: float) -> None:
                    if fault and t >= fault[2] and not done.is_set():
                        done.set()
                        fault[0](bus, fault[1])

                result["loop"] = control_loop(arm, args.rate, goal_fn, total + 0.5, live.stop, on_tick)
        except SafetyTrip as e:
            result["trip"] = e
        except BaseException as e:
            result["error"] = e

    th = threading.Thread(target=worker, name="safearm-sim")
    th.start()
    live.footer = ["SafeArm running (simulation)"]
    live.run_until_done(th, console)
    summary = live.close()
    arm = result.get("arm")
    if arm:
        run.write_json("safety_events.json", arm.report())

    console.print(f"[bold]{run.run_id}[/] (simulation)")
    if "loop" in result:
        s = result["loop"]
        console.print(f"loop: {s.cycles} cycles in {s.duration_s:.1f} s, overruns {s.overruns}, skipped {s.skipped_slots}")
    if "trip" in result:
        console.print(f"[red]{result['trip']}[/] → torque off on all joints, state {arm.state.value}")
    if "error" in result:
        console.print(f"[red]error: {result['error']!r}")
    if arm:
        console.print(f"clamps: {dict((f'{j}:{k}', n) for (j, k), n in arm.clamps.items()) or 'none'}")
        console.print("events: " + ", ".join(f"{e.kind}({e.joint or 'arm'})" for e in arm.events))
    console.print(f"logger: {summary}")
    if summary["rows"]:
        rows = pq.read_table(run.dir / "telemetry").to_pylist()
        table = Table(title="tracking (sim): |goal − position| where a goal was commanded", title_justify="left")
        for col in ["joint", "goals", "median err", "max err"]:
            table.add_column(col, justify="left" if col == "joint" else "right")
        for j in cfg.joints:
            errs = [abs(r["goal"] - r["position"]) for r in rows if r["joint"] == j and r["goal"] is not None
                    and r["position"] is not None]  # fmt: skip
            table.add_row(j, str(len(errs)), f"{median(errs):.0f}" if errs else "—", str(max(errs)) if errs else "—")
        console.print(table)


if __name__ == "__main__":
    main()
