"""Step 6c: approach targets from both directions, wait_until_settled, report settle time and error.

    uv run scripts/motion/settle_check.py --arm-moves --tui
    uv run scripts/motion/settle_check.py --arm-moves --joint shoulder_pan --offsets -150 0 150 --repeats 3
    uv run scripts/motion/settle_check.py --arm-moves --sim --yes                 # SimBus, no hardware

For each target (start + offset) and each approach direction (+1 from below, -1 from above), `approach()`
overshoots 40 ticks to the far side if needed, arrives moving in that direction (min-jerk, 150 ticks/s
peak), then `wait_until_settled()` (Moving == 0 and steady 0.2 s, then --dwell-s). Repeated --repeats
times: a first look at repeatability per direction (Phase 2 measures it properly, with the camera).
Watch the end effector at each stop: the encoders don't see structural wobble; --dwell-s is the guess.
Ctrl-C once = freeze (hold), again = torque off. Writes data/raw/<run_id>/ (kind settle) + settle.json.
"""

import argparse
import json
import signal
import sys
import threading
from statistics import mean, median, pstdev

from rich.console import Console
from rich.table import Table

from so101.config import REPO_ROOT, load_arm_config
from so101.hw.arm import Arm
from so101.hw.bus import FeetechBus
from so101.hw.sim import SimBus
from so101.motion.settle import DEFAULT_DWELL_S, approach
from so101.motion.sweep import GRAVITY_JOINTS
from so101.runlog.run import Run
from so101.safety import SafeArm, SafetyConfig, SafetyTrip
from so101.telemetry.live import LiveSession


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm-moves", action="store_true", help="required for every scripts/motion/ script")
    ap.add_argument("--joint", default="shoulder_pan")
    ap.add_argument("--offsets", type=int, nargs="+", default=[-150, 0, 150], help="targets relative to start (ticks)")
    ap.add_argument("--repeats", type=int, default=2)
    ap.add_argument("--dwell-s", type=float, default=DEFAULT_DWELL_S)
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--tui", action="store_true")
    ap.add_argument("--viz", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--notes", default="")
    ap.add_argument("--sim", action="store_true")
    args = ap.parse_args()
    if not args.arm_moves:
        sys.exit("refusing to run without --arm-moves (scripts/motion/ convention)")

    console = Console()
    cfg = load_arm_config("follower")
    lo, hi = SafetyConfig.from_arm_config(cfg).limits[args.joint]
    if args.sim:
        bus = SimBus.from_dump(json.loads((REPO_ROOT / "data/raw/20261026-112606_regdump_follower/registers.json").read_text()))
    else:
        bus = FeetechBus(cfg.port, cfg.baudrate, cfg.protocol_end)
    st = Arm(bus, cfg.joints).snapshot().joints[args.joint]
    if st is None:
        bus.close()
        sys.exit(f"{args.joint}: no reading")
    start = st.position
    targets = [start + o for o in args.offsets]
    span = (min(targets) - 40, max(targets) + 40)  # includes the overshoot
    if not (lo <= span[0] and span[1] <= hi):
        bus.close()
        sys.exit(f"targets ± 40 overshoot {span} leave the limits [{lo}, {hi}] (start {start}); not enabling")
    console.print(f"[bold]{args.joint}[/]: targets {targets} (start {start}), each from below (+1) and above (−1), "
                  f"×{args.repeats}; overshoot 40, min-jerk 150 t/s peak, dwell {args.dwell_s:g} s")  # fmt: skip
    if args.joint in GRAVITY_JOINTS:
        console.print("  [bold yellow]gravity joint: keep a hand near the arm")
    if not args.yes:
        try:
            input("Press Enter to enable torque and start (Ctrl-C aborts, nothing written)… ")
        except KeyboardInterrupt:
            bus.close()
            sys.exit("\naborted before enabling")

    run = Run("settle_sim" if args.sim else "settle", "follower", config_paths=[cfg.path],
              notes=f"step 6c {args.joint} targets={targets} repeats={args.repeats}; sim={args.sim}; {args.notes}")  # fmt: skip
    live = LiveSession(run, {"follower": cfg.expected_voltage_v}, args.tui, args.viz,
                       title=f"[TORQUE ON: SETTLE CHECK {args.joint}] {run.run_id} — Ctrl-C = freeze, again = torque off")  # fmt: skip
    result: dict = {"trials": []}

    def worker() -> None:
        try:
            with SafeArm(bus, cfg, hub=live.hub) as arm:
                result["arm"] = arm
                arm.enable()
                for rep in range(args.repeats):
                    for tgt in targets:
                        for d in (1, -1):
                            if live.stop.is_set() or live.freeze.is_set() or arm.frozen:
                                raise KeyboardInterrupt
                            r = approach(arm, {args.joint: tgt}, {args.joint: d}, rate_hz=args.rate, dwell_s=args.dwell_s)
                            if r is None:  # frozen by contact detection
                                raise KeyboardInterrupt
                            result["trials"].append({"rep": rep, "target": tgt, "dir": d, "settled": r.settled,
                                                     "t_encoder_s": r.t_encoder_s, "error": r.errors.get(args.joint),
                                                     "position": r.positions.get(args.joint)})  # fmt: skip
                # hold until released if the user froze; otherwise done
                while (live.freeze.is_set() or arm.frozen) and not live.stop.is_set():
                    arm.step()
                    live.stop.wait(1 / args.rate)
        except KeyboardInterrupt:
            arm_ = result.get("arm")
            while arm_ is not None and not live.stop.is_set():  # hold (frozen) until the next Ctrl-C
                if not arm_.frozen:
                    arm_.freeze()
                arm_.step()
                live.stop.wait(1 / args.rate)
        except SafetyTrip as e:
            result["trip"] = e
        except BaseException as e:
            result["error"] = e

    def status():
        a = result.get("arm")
        if a is not None and a.contact_hit:
            return f"FROZEN by contact detection ({a.contact_hit.joint}): support the arm, then Ctrl-C = torque off"
        return None

    live.status_fn = status
    live.ctrl_c_stops_fn = lambda: bool(((a := result.get("arm")) is not None and a.frozen) or result.get("holding_at_end"))
    th = threading.Thread(target=worker, name="safearm-settle")
    th.start()
    live.footer = [f"TORQUE ON: settle check of {args.joint}. Ctrl-C = freeze, again = torque off."]
    live.run_until_done(th, console)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    summary = live.close()
    bus.close()

    arm, trials = result.get("arm"), result["trials"]
    if arm:
        run.write_json("safety_events.json", arm.report())
    run.write_json("settle.json", {"joint": args.joint, "start": start, "targets": targets, "dwell_s": args.dwell_s,
                                   "trials": trials, "trip": str(result.get("trip")) if "trip" in result else None,
                                   "error": repr(result["error"]) if "error" in result else None, "logger": summary})  # fmt: skip
    console.print(f"[bold]{run.run_id}[/] ({'SIMULATION' if args.sim else 'real follower'})")
    for k in ("trip", "error"):
        if k in result:
            console.print(f"[red]{k}: {result[k]}")
    if trials:
        t = Table(title=f"{args.joint}: settle per target and approach direction", title_justify="left")
        for col in ["target", "dir", "n", "settled", "encoder settle ms (median/max)", "error mean", "error spread (max−min)"]:
            t.add_column(col, justify="right")
        for tgt in targets:
            for d in (1, -1):
                xs = [x for x in trials if x["target"] == tgt and x["dir"] == d]
                if not xs:
                    continue
                ts = [x["t_encoder_s"] * 1000 for x in xs if x["t_encoder_s"] is not None]
                es = [x["error"] for x in xs if x["error"] is not None]
                t.add_row(str(tgt), f"{d:+d}", str(len(xs)), f"{sum(x['settled'] for x in xs)}/{len(xs)}",
                          f"{median(ts):.0f}/{max(ts):.0f}" if ts else "—", f"{mean(es):+.1f}" if es else "—",
                          str(max(es) - min(es)) if es else "—")  # fmt: skip
        console.print(t)
    console.print(f"bus failures: {dict(bus.stats.failures) or 0}   logger: {summary}")


if __name__ == "__main__":
    main()
