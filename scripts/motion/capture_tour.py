"""Step 6: capture tour. Visit a tour's waypoints with all joints moving together and, at each one: settle ->
arm snapshot -> one camera capture -> arm snapshot, written to the run manifest (spec §7). Then park.

    uv run scripts/motion/capture_tour.py --arm-moves --sim --yes                         # SimBus + mock camera
    uv run scripts/motion/capture_tour.py --arm-moves                                     # config/tours/step6_sector.yaml
    uv run scripts/motion/capture_tour.py --arm-moves --tour config/tours/other.yaml --settle-s 5

Same planner and safety as pose_tour.py (plan from the current pose and every path checked against limits
and the table BEFORE torque-on; SafeArm trips, contact detection and the runtime table check stay active;
Ctrl-C once = freeze, again = torque off). The arm keeps being controlled (holding the target) while the
camera captures. No capture at `park`/start-pose waypoints or at waypoints with `capture: false` (via points
that set the approach direction). A waypoint's `desired_ticks` adds one encoder correction after settling: the
goal is nudged by (desired − reading) so the reading lands there (step 10b; skipped if any joint is off by more than
--max-correct-ticks), then it settles again before the shots. `--shots N` (or a waypoint's `shots`) takes N photos per waypoint,
--shot-gap-s apart (step 8 settle check: first vs late shot); their manifest labels are `<name>#<k>`. The tour's dwell_s is not used (--settle-s is).
Stillness check: any joint that moved more than --still-ticks between the two snapshots is flagged.
At the end it holds at park until Ctrl-C (support the arm; it settles onto its rest stops at torque-off).
Writes data/raw/<run_id>/ (kind capture): manifest.jsonl (commanded ticks, snapshots, settle, camera),
images/, safety_events.json, capture.json (summary: missed captures, manifest↔files, stillness flags).
Tag analysis afterwards (read-only): scripts/readonly/teach_poses.py --analyze data/raw/<run_id>, then
scripts/camera/teach_map.py data/raw/<run_id>.
"""

import argparse
import json
import signal
import sys
import threading
import time
from pathlib import Path

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
from so101.motion.loop import Segment, control_loop
from so101.motion.tour import ALL_JOINTS, load_tour, plan_tour
from so101.runlog.manifest import Manifest
from so101.runlog.run import Run, sha256_file
from so101.sensors import Gphoto2Camera, MockCamera
from so101.safety import SafeArm, SafetyConfig, SafetyTrip
from so101.telemetry.live import LiveSession

MAX_SPEED = 300.0  # stage limit, peak ticks/s


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm-moves", action="store_true", help="required for every scripts/motion/ script")
    ap.add_argument("--tour", default="config/tours/step6_sector.yaml")
    ap.add_argument("--settle-s", type=float, default=3.0, help="hold before the first snapshot (ring-down, step 7, not measured yet)")
    ap.add_argument("--shots", type=int, default=1, help="photos per capture waypoint (a waypoint's `shots` overrides)")
    ap.add_argument("--shot-gap-s", type=float, default=10.0, help="start-to-start time between photos at one waypoint (settle check: first vs late shot)")
    ap.add_argument("--max-correct-ticks", type=int, default=30, help="encoder correction (waypoint desired_ticks): skip if any joint is off by more")
    ap.add_argument("--still-ticks", type=int, default=2, help="stillness check: max joint change between the two snapshots")
    ap.add_argument("--speed", type=float, help="override the tour's peak ticks/s")
    ap.add_argument("--rate", type=float, default=100.0)
    ap.add_argument("--hold-at-end", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--tui", action="store_true")
    ap.add_argument("--viz", action=argparse.BooleanOptionalAction, default=True)
    ap.add_argument("--yes", action="store_true")
    ap.add_argument("--notes", default="")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--sim-freeze-at", type=int, help="test hook (--sim only): freeze during capture N (0-based), "
                    "as a Ctrl-C would, then end the frozen hold 2 s later")  # fmt: skip
    args = ap.parse_args()
    if args.sim_freeze_at is not None and not args.sim:
        sys.exit("--sim-freeze-at needs --sim")
    if not args.arm_moves:
        sys.exit("refusing to run without --arm-moves (scripts/motion/ convention)")

    console = Console()
    tour_path = REPO_ROOT / args.tour
    waypoints, opts = load_tour(tour_path)
    speed = args.speed or opts.get("speed", 150.0)
    dwell = 0.0  # holds are event-driven here (settle + capture), not planned
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
    camera = MockCamera(delay_s=0.5) if args.sim else Gphoto2Camera()
    cam_info = camera.info()  # fails here, before torque-on, if the camera isn't reachable
    def n_shots(w) -> int:
        return 0 if (w.start_pose or not w.capture) else (w.shots or args.shots)

    n_cap = sum(n_shots(s.waypoint) for s in plan.steps)
    n_wp = sum(n_shots(s.waypoint) > 0 for s in plan.steps)
    console.print(f"{n_cap} captures at {n_wp} waypoints; ~{(plan.duration_s + n_wp * args.settle_s + n_cap * max(7, args.shot_gap_s)) / 60:.1f} min")
    if not args.yes:
        try:
            input("Arm resting on its stops, workspace clear (nothing within ~35 cm)? Enter = enable torque and run (Ctrl-C aborts)… ")
        except KeyboardInterrupt:
            bus.close()
            sys.exit("\naborted before enabling")

    run = Run("capture_sim" if args.sim else "capture", "follower", config_paths=[cfg.path, mapping.path, tour_path],
              notes=f"capture tour {tour_path.name} speed {speed} settle {args.settle_s}; sim={args.sim}; {args.notes}")  # fmt: skip
    run.write_json("camera.json", cam_info)
    manifest = Manifest(run, "capture")
    live = LiveSession(run, {"follower": cfg.expected_voltage_v}, args.tui, args.viz,
                       title=f"[TORQUE ON: CAPTURE TOUR] {run.run_id} — Ctrl-C = freeze, again = torque off")  # fmt: skip
    result: dict = {"marks": {}, "captures": []}
    marks = result["marks"]
    goal_fn, phase_fn = plan.goal_fn(), plan.phase_fn()
    total = plan.duration_s

    def hold_until(arm, target: dict, done: threading.Event, max_s: float):
        """Keep commanding `target` until `done` (or Ctrl-C / max_s); freeze and trips work as in control_loop."""
        return control_loop(arm, args.rate, lambda t: target, max_s, done, freeze=live.freeze,
                            on_tick=lambda t: done.set() if live.stop.is_set() else None)  # fmt: skip

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
                for i, st in enumerate(plan.steps):
                    seg, name = st.segment, st.waypoint.name
                    result["phase"] = (i, name, "moving")
                    control_loop(arm, args.rate, seg.at, seg.duration_s, live.stop, freeze=live.freeze)
                    if live.stop.is_set() or arm.frozen:
                        break
                    if n_shots(st.waypoint) == 0:
                        continue
                    target = seg.target
                    result["phase"] = (i, name, "settling")
                    hold_until(arm, target, threading.Event(), args.settle_s)
                    if live.stop.is_set() or arm.frozen:
                        break
                    correction = None
                    if st.waypoint.desired_ticks:  # one encoder-reading correction (no external sensor)
                        rd = arm.step(target).snapshot
                        off = {j: st.waypoint.desired_ticks[j] - rd.joints[j].position for j in st.waypoint.desired_ticks if rd.joints.get(j)}
                        if off and max(abs(v) for v in off.values()) <= args.max_correct_ticks:
                            new = {**target, **{j: target[j] + v for j, v in off.items()}}
                            result["phase"] = (i, name, "correcting")
                            nudge = Segment(dict(target), new, 50.0, "minjerk")
                            control_loop(arm, args.rate, nudge.at, nudge.duration_s, live.stop, freeze=live.freeze)
                            if live.stop.is_set() or arm.frozen:
                                break
                            target = new
                            hold_until(arm, target, threading.Event(), args.settle_s)
                            if live.stop.is_set() or arm.frozen:
                                break
                            correction = {"offset_ticks": off, "applied": True}
                        else:
                            correction = {"offset_ticks": off, "applied": False}
                    for k in range(n_shots(st.waypoint)):
                        t_shot = time.monotonic()
                        before = arm.step(None if arm.frozen else target).snapshot  # a frozen arm refuses goals
                        result["phase"] = (i, name, f"capturing {k + 1}/{n_shots(st.waypoint)}")
                        done, box = threading.Event(), {}

                        def shoot(stem=f"img_{len(result['captures']):04d}_{name}_s{k}") -> None:
                            if args.sim_freeze_at == len(result["captures"]):  # test hook: a Ctrl-C mid-capture
                                live.freeze.set()
                                threading.Timer(2.0, live.stop.set).start()
                            try:
                                box["r"] = camera.capture(run.dir / "images", stem)
                            finally:
                                done.set()

                        threading.Thread(target=shoot, name="capture").start()
                        hold_until(arm, target, done, 60.0)
                        done.wait(60.0)
                        # a freeze (Ctrl-C, contact) can land during the capture: then only read, never command, so the
                        # worker falls through to the frozen hold below instead of raising (which would drop torque)
                        after = arm.step(None if arm.frozen else target).snapshot
                        r = box.get("r")
                        moved = {j: after.joints[j].position - before.joints[j].position for j in ALL_JOINTS
                                 if before.joints.get(j) and after.joints.get(j)
                                 and abs(after.joints[j].position - before.joints[j].position) > args.still_ticks}  # fmt: skip
                        err = {j: before.joints[j].position - target[j] for j in ALL_JOINTS if before.joints.get(j)}
                        label = name if n_shots(st.waypoint) == 1 else f"{name}#{k}"
                        manifest.add(label, commanded_ticks={j: int(v) for j, v in target.items()}, before=before, after=after,
                                     settle={"settle_s": args.settle_s, "shot": k, "since_arrival_s": args.settle_s + k * args.shot_gap_s,
                                             "error_ticks": err, "still": not moved, "moved": moved, "correction": correction},
                                     extra={"meta": st.waypoint.meta or {}, "desired_ticks": st.waypoint.desired_ticks},
                                     camera=None if r is None else {**r.as_dict(), "backend": camera.name})  # fmt: skip
                        result["captures"].append({"waypoint": label, "ok": bool(r and r.ok), "files": [] if r is None else [str(f) for f in r.files],
                                                   "error": None if r is None else r.error, "moved": moved, "error_ticks": err})  # fmt: skip
                        if live.stop.is_set() or arm.frozen:
                            break
                        if k + 1 < n_shots(st.waypoint):  # hold until the next shot's start time
                            hold_until(arm, target, threading.Event(), max(0.0, args.shot_gap_s - (time.monotonic() - t_shot)))
                    if live.stop.is_set() or arm.frozen:
                        break
                marks["end_ns"] = time.monotonic_ns()
                if args.hold_at_end and not live.stop.is_set() and not arm.frozen:
                    result["holding_at_end"] = True
                    control_loop(arm, args.rate, lambda tt: None, float("inf"), live.stop)
                elif arm.frozen and not live.stop.is_set():
                    control_loop(arm, args.rate, lambda tt: None, float("inf"), live.stop)  # frozen: hold until Ctrl-C
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
            return f"waypoint {i + 1}/{len(plan.steps)} {name}: {what} ({len(result['captures'])}/{n_cap} captured)"
        return None

    live.status_fn = status
    live.ctrl_c_stops_fn = lambda: bool(((a := result.get("arm")) is not None and a.frozen) or result.get("holding_at_end"))
    th = threading.Thread(target=worker, name="safearm-capture")
    th.start()
    live.footer = ["TORQUE ON: capture tour. Ctrl-C = freeze, again = torque off."]
    live.run_until_done(th, console)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    summary = live.close()
    bus.close()
    manifest.close()
    camera.close() if hasattr(camera, "close") else None

    arm, loop = result.get("arm"), result.get("loop")
    if arm:
        run.write_json("safety_events.json", arm.report())
    caps = result["captures"]
    files = sorted((run.dir / "images").glob("*")) if (run.dir / "images").exists() else []
    listed = {Path(f).name for c in caps for f in c["files"]}
    summary_cap = {"planned": n_cap, "attempted": len(caps), "missed": sum(not c["ok"] for c in caps),
                   "manifest_files_match": listed == {f.name for f in files}, "stillness_flags": [c["waypoint"] for c in caps if c["moved"]]}  # fmt: skip
    run.write_json("capture.json", {
        "tour": str(tour_path.relative_to(REPO_ROOT)), "tour_sha256": sha256_file(tour_path), "speed": speed, "settle_s": args.settle_s,
        "still_ticks": args.still_ticks, "summary": summary_cap, "captures": caps,
        "plan": [{"waypoint": s.waypoint.name, "target_ticks": s.segment.target, "move_s": s.segment.duration_s,
                  "tool_mm": s.tool_mm, "min_table_mm": s.min_table_mm, "warnings": s.warnings} for s in plan.steps],
        "frozen": bool(arm and arm.frozen), "collision_hit": arm.collision_hit if arm else None,
        "trip": str(result.get("trip")) if "trip" in result else None,
        "error": repr(result["error"]) if "error" in result else None, "logger": summary,
    })  # fmt: skip

    console.print(f"[bold]{run.run_id}[/] ({'SIMULATION' if args.sim else 'real follower'})")
    for k in ("trip", "error"):
        if k in result:
            console.print(f"[red]{k}: {result[k]}")
    if arm and (arm.contact_hit or arm.collision_hit):
        console.print(f"[yellow]FROZEN: {arm.collision_hit or arm.contact_hit.detail}")
    elif "trip" not in result and "error" not in result:
        console.print("[green]capture tour done, parked, torque off")
    if caps:
        t = Table(title="per capture: position − commanded at the first snapshot (ticks); moved = change between snapshots", title_justify="left")
        for c in ["waypoint", "capture", *ALL_JOINTS, "moved"]:
            t.add_column(c, justify="left" if c in ("waypoint", "capture", "moved") else "right")
        for c in caps:
            t.add_row(c["waypoint"], "ok" if c["ok"] else f"[red]{c['error']}", *[f"{c['error_ticks'].get(j, 0):+.0f}" for j in ALL_JOINTS],
                      "[yellow]" + str(c["moved"]) if c["moved"] else "")  # fmt: skip
        console.print(t)
    ok = summary_cap["missed"] == 0 and summary_cap["manifest_files_match"] and not summary_cap["stillness_flags"] and summary_cap["attempted"] == n_cap
    console.print(f"[{'green' if ok else 'red'}]dry-run checks: {summary_cap}")
    if loop:
        console.print(f"loop: {loop.cycles} cycles, overruns {loop.overruns}, skipped {loop.skipped_slots}")
    console.print(f"bus failures: {dict(bus.stats.failures) or 0}   logger: {summary}")


if __name__ == "__main__":
    main()
