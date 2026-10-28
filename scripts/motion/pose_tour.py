"""All-joint motion test: a pose tour (pick-and-place emulation + workspace spread), every joint moving together.

    uv run scripts/motion/pose_tour.py --arm-moves --sim --yes                 # SimBus first, no hardware
    uv run scripts/motion/pose_tour.py --arm-moves                             # real follower, 3 s dwell per pose
    uv run scripts/motion/pose_tour.py --arm-moves --dwell-s 10                # time to check focus at each pose
    uv run scripts/motion/pose_tour.py --arm-moves --tour config/tours/other.yaml

Start with the arm resting on its stops. The tour (config/tours/pick_and_place.yaml) goes: ready → pick
(approach, descend, close the gripper on nothing, lift) → place on the other side (descend, open, retreat) →
far reach → high → wrist twist → wide left/right → ready → park (the starting pose, clamped into the
software limits; elbow stops ~4° short of its rest stop and settles onto it at torque-off).
Every move is planned from the current pose and checked BEFORE torque-on: IK, joint limits, and the table
at 20 ms samples along each path (3 mm margin, inside-margin rule). Any failure refuses before anything is
written. At runtime SafeArm checks every goal against the table, contact detection and all trips stay
active. Ctrl-C once = freeze, again = torque off. At the end it holds at park until Ctrl-C (support the
arm; it settles onto its rest stops at torque-off).
Writes data/raw/<run_id>/ (kind tour) + safety_events.json + tour.json (plan + measured pose per waypoint).
"""

import argparse
import json
import signal
import sys
import threading
import time

import numpy as np
import pyarrow.parquet as pq
from rich.console import Console
from rich.table import Table

from so101.config import REPO_ROOT, load_arm_config
from so101.hw.arm import Arm
from so101.hw.bus import FeetechBus
from so101.hw.sim import SimBus
from so101.kinematics.collision import CollisionModel
from so101.kinematics.fk import CHAIN_JOINTS, chain_from_mujoco
from so101.kinematics.ik import limits_rad
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel
from so101.motion.loop import control_loop
from so101.motion.tour import ALL_JOINTS, load_tour, plan_tour
from so101.runlog.run import Run, sha256_file
from so101.safety import SafeArm, SafetyConfig, SafetyTrip
from so101.telemetry.live import LiveSession

MAX_SPEED = 300.0  # stage limit, peak ticks/s


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm-moves", action="store_true", help="required for every scripts/motion/ script")
    ap.add_argument("--tour", default="config/tours/pick_and_place.yaml")
    ap.add_argument("--speed", type=float, help="override the tour's peak ticks/s")
    ap.add_argument("--dwell-s", type=float, help="override the default hold at each waypoint")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--hold-at-end", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--tui", action="store_true")
    ap.add_argument("--viz", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--notes", default="")
    ap.add_argument("--sim", action="store_true")
    args = ap.parse_args()
    if not args.arm_moves:
        sys.exit("refusing to run without --arm-moves (scripts/motion/ convention)")

    console = Console()
    tour_path = REPO_ROOT / args.tour
    waypoints, opts = load_tour(tour_path)
    speed = args.speed or opts.get("speed", 150.0)
    dwell = args.dwell_s if args.dwell_s is not None else opts.get("dwell_s", 3.0)
    if not 0 < speed <= MAX_SPEED or any((w.speed or 0) > MAX_SPEED for w in waypoints):
        sys.exit(f"stage limit: peak speed <= {MAX_SPEED:g} ticks/s")

    cfg = load_arm_config("follower")
    safety = SafetyConfig.from_arm_config(cfg)
    mapping = load_mapping("follower")
    chain = chain_from_mujoco(ArmModel(mapping.model_path).m)
    lim = limits_rad(mapping, safety.limits, CHAIN_JOINTS)
    collision = CollisionModel.from_safety(mapping, safety)
    if args.sim:
        bus = SimBus.from_dump(json.loads((REPO_ROOT / "data/raw/20261026-112606_regdump_follower/registers.json").read_text()))
    else:
        bus = FeetechBus(cfg.port, cfg.baudrate, cfg.protocol_end)

    # --- plan from the current pose (torque off, read-only) -------------------------------------
    snap = Arm(bus, cfg.joints).snapshot()
    if not snap.ok:
        bus.close()
        sys.exit(f"incomplete reading: {snap.errors}")
    ticks0 = {j: s.position for j, s in snap.joints.items()}
    bad = mapping.check(ticks0, ArmModel(mapping.model_path).ranges)
    if bad:
        bus.close()
        sys.exit(f"refusing: pose implausible for the mapping (check calibration): {bad}")
    plan = plan_tour(waypoints, ticks0, chain=chain, mapping=mapping, limits_ticks=safety.limits, limits_rad=lim,
                     collision=collision, speed=speed, dwell_s=dwell)  # fmt: skip
    t = Table(title=f"tour {tour_path.name}: {len(plan.steps)}/{len(waypoints)} waypoints planned", title_justify="left")
    for c in ["#", "waypoint", "move s", "dwell s", "tool point mm (x, y, z)", "closest to table mm", "warnings"]:
        t.add_column(c, justify="right" if c not in ("waypoint", "warnings") else "left")
    for i, s in enumerate(plan.steps):
        t.add_row(str(i), s.waypoint.name, f"{s.segment.duration_s:.1f}", f"{s.dwell_s:g}", ", ".join(f"{v:.0f}" for v in s.tool_mm),
                  f"{s.min_table_mm:+.0f}", "; ".join(s.warnings)[:60])  # fmt: skip
    console.print(t)
    if not plan.ok:
        bus.close()
        sys.exit(f"plan refused: {plan.reason}. Nothing was written.")
    console.print(f"total {plan.duration_s / 60:.1f} min at {speed:g} ticks/s peak; all joints move together")
    if not args.yes:
        try:
            input("Arm resting on its stops, workspace clear (nothing within ~35 cm)? Enter = enable torque and run (Ctrl-C aborts)… ")
        except KeyboardInterrupt:
            bus.close()
            sys.exit("\naborted before enabling")

    run = Run("tour_sim" if args.sim else "tour", "follower", config_paths=[cfg.path, mapping.path, tour_path],
              notes=f"pose tour {tour_path.name} speed {speed} dwell {dwell}; sim={args.sim}; {args.notes}")  # fmt: skip
    live = LiveSession(run, {"follower": cfg.expected_voltage_v}, args.tui, args.viz,
                       title=f"[TORQUE ON: POSE TOUR] {run.run_id} — Ctrl-C = freeze, again = torque off")  # fmt: skip
    result: dict = {"marks": {}}
    marks = result["marks"]
    goal_fn, phase_fn = plan.goal_fn(), plan.phase_fn()
    total = plan.duration_s

    def worker() -> None:
        try:
            with SafeArm(bus, cfg, hub=live.hub, collision=collision, mapping=mapping) as arm:
                result["arm"] = arm
                now = arm.step().snapshot
                drift = max(abs(now.joints[j].position - ticks0[j]) for j in ALL_JOINTS)
                if drift > 20:
                    raise RuntimeError(f"arm moved {drift} ticks since planning; not enabling")
                arm.enable()
                marks["start_ns"] = time.monotonic_ns()
                result["loop"] = control_loop(arm, args.rate, goal_fn, total, live.stop, freeze=live.freeze,
                                              on_tick=lambda tt: result.__setitem__("phase", phase_fn(tt)))  # fmt: skip
                marks["end_ns"] = time.monotonic_ns()
                if args.hold_at_end and not live.stop.is_set() and not arm.frozen:
                    result["holding_at_end"] = True
                    control_loop(arm, args.rate, lambda tt: None, float("inf"), live.stop)
                result["disable_failed"] = arm.disable()
        except SafetyTrip as e:
            result["trip"] = e
        except BaseException as e:
            result["error"] = e

    def status():
        a = result.get("arm")
        if a is not None and (a.contact_hit or a.collision_hit):
            why = f"contact ({a.contact_hit.joint})" if a.contact_hit else f"predicted table contact ({a.collision_hit})"
            return f"FROZEN by {why}: support the arm, then Ctrl-C = torque off"
        if result.get("holding_at_end"):
            return "PARKED: support the arm, then Ctrl-C = torque off (it settles onto its rest stops)"
        if "phase" in result:
            i, name, what = result["phase"]
            return f"waypoint {i + 1}/{len(plan.steps)} {name}: {what}" + (" (check focus now)" if what == "dwell" else "")
        return None

    live.status_fn = status
    live.ctrl_c_stops_fn = lambda: bool(((a := result.get("arm")) is not None and a.frozen) or result.get("holding_at_end"))
    th = threading.Thread(target=worker, name="safearm-tour")
    th.start()
    live.footer = ["TORQUE ON: pose tour. Ctrl-C = freeze, again = torque off."]
    live.run_until_done(th, console)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    summary = live.close()
    bus.close()

    arm, loop = result.get("arm"), result.get("loop")
    if arm:
        run.write_json("safety_events.json", arm.report())
    # measured pose at the end of each dwell (last 0.3 s) vs commanded
    reached = []
    if summary["rows"] and "start_ns" in marks:
        rows = [r for r in pq.read_table(run.dir / "telemetry").to_pylist() if r["position"] is not None]
        t0 = 0.0
        for s in plan.steps:
            t_end = t0 + s.segment.duration_s + s.dwell_s
            lo, hi = marks["start_ns"] + int((t_end - 0.3) * 1e9), marks["start_ns"] + int(t_end * 1e9)
            err = {}
            for j in ALL_JOINTS:
                w = [r["position"] for r in rows if r["joint"] == j and lo <= r["t_start_ns"] <= hi]
                if w:
                    err[j] = float(np.median(w) - s.segment.target[j])
            reached.append({"waypoint": s.waypoint.name, "commanded": s.segment.target, "error_ticks": err})
            t0 = t_end
    run.write_json("tour.json", {
        "tour": str(tour_path.relative_to(REPO_ROOT)), "tour_sha256": sha256_file(tour_path), "speed": speed, "dwell_s": dwell,
        "plan": [{"waypoint": s.waypoint.name, "target_ticks": s.segment.target, "move_s": s.segment.duration_s,
                  "dwell_s": s.dwell_s, "tool_mm": s.tool_mm, "min_table_mm": s.min_table_mm, "warnings": s.warnings} for s in plan.steps],
        "reached": reached, "frozen": bool(arm and arm.frozen), "collision_hit": arm.collision_hit if arm else None,
        "trip": str(result.get("trip")) if "trip" in result else None,
        "error": repr(result["error"]) if "error" in result else None,
        "loop": vars(loop) if loop else None, "logger": summary,
    })  # fmt: skip

    console.print(f"[bold]{run.run_id}[/] ({'SIMULATION' if args.sim else 'real follower'})")
    for k in ("trip", "error"):
        if k in result:
            console.print(f"[red]{k}: {result[k]}")
    if arm and (arm.contact_hit or arm.collision_hit):
        console.print(f"[yellow]FROZEN: {arm.collision_hit or arm.contact_hit.detail}")
    elif "trip" not in result and "error" not in result:
        console.print("[green]tour done, parked, torque off")
    if reached:
        t = Table(title="end of each dwell: position − commanded (ticks)", title_justify="left")
        t.add_column("waypoint")
        for j in ALL_JOINTS:
            t.add_column(j, justify="right")
        for r in reached:
            t.add_row(r["waypoint"], *[f"{r['error_ticks'][j]:+.0f}" if j in r["error_ticks"] else "" for j in ALL_JOINTS])
        console.print(t)
    if loop:
        console.print(f"loop: {loop.cycles} cycles, overruns {loop.overruns}, skipped {loop.skipped_slots}")
    console.print(f"bus failures: {dict(bus.stats.failures) or 0}   logger: {summary}")


if __name__ == "__main__":
    main()
