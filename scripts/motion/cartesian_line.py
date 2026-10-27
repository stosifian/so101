"""Step 9d: Cartesian demo. The tool point (fixed-jaw tip) moves along a straight line and back.

    uv run scripts/motion/cartesian_line.py --arm-moves --tui                        # 8 cm forward and back
    uv run scripts/motion/cartesian_line.py --arm-moves --dx 0 --dy 0.08 --speed 0.02
    uv run scripts/motion/cartesian_line.py --arm-moves --sim --yes                  # SimBus, no hardware

Before torque-on: place the arm BY HAND in a pose clear of the table (e.g. elbow unfolded, gripper up), the
line starts at the current tool point. The whole out-and-back path is planned (IK per 10 ms sample, warm-
started, wrist_roll held) and validated: IK success, joint limits, joint continuity, table collision
(3 mm margin with the inside-margin rule). Any failure refuses before anything is written.
At runtime SafeArm also checks every goal against the table and freezes on a predicted contact; contact
detection and all trips stay active. Ctrl-C once = freeze, again = torque off.
Straightness is measured from the encoders through FK (the camera measures the real line in Phase 2).
Writes data/raw/<run_id>/ (kind cartesian) + safety_events.json + cartesian.json.
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

from so101.config import REPO_ROOT, load_arm_config
from so101.hw.arm import Arm
from so101.hw.bus import FeetechBus
from so101.hw.sim import SimBus
from so101.kinematics.collision import CollisionModel
from so101.kinematics.fk import CHAIN_JOINTS, DEFAULT_TCP, chain_from_mujoco, fk
from so101.kinematics.ik import limits_rad
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel
from so101.motion.cartesian import line_point_distance, plan_line
from so101.motion.loop import control_loop
from so101.runlog.run import Run
from so101.safety import SafeArm, SafetyConfig, SafetyTrip
from so101.telemetry.live import LiveSession

MAX_SPEED, MAX_LEN = 0.05, 0.15  # stage limits: 5 cm/s, 15 cm line


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm-moves", action="store_true", help="required for every scripts/motion/ script")
    ap.add_argument("--dx", type=float, default=0.08, help="line vector, metres, base frame (x = forward)")
    ap.add_argument("--dy", type=float, default=0.0)
    ap.add_argument("--dz", type=float, default=0.0)
    ap.add_argument("--speed", type=float, default=0.03, help="peak TCP speed, m/s (min-jerk)")
    ap.add_argument("--pause-s", type=float, default=1.0, help="hold at the far end")
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
    vec = np.array([args.dx, args.dy, args.dz])
    if not 0 < args.speed <= MAX_SPEED or not 0 < np.linalg.norm(vec) <= MAX_LEN:
        sys.exit(f"stage limits: 0 < speed <= {MAX_SPEED} m/s, 0 < line length <= {MAX_LEN} m")

    console = Console()
    cfg = load_arm_config("follower")
    safety = SafetyConfig.from_arm_config(cfg)
    mapping = load_mapping("follower")
    chain = chain_from_mujoco(ArmModel(mapping.model_path).m)
    lim = limits_rad(mapping, safety.limits, CHAIN_JOINTS)
    collision = CollisionModel.from_safety(mapping, safety)

    if args.sim:
        bus = SimBus.from_dump(json.loads((REPO_ROOT / "data/raw/20261026-112606_regdump_follower/registers.json").read_text()))
        # simulated "placed by hand": a clear working pose (elbow unfolded), like the step 6b runs
        for j, t in dict(shoulder_lift=1673, elbow_flex=1600, wrist_flex=2400, wrist_roll=2156).items():
            bus.memory[cfg.joints[j]][56] = t
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
    q0 = np.array([mapping.joints[j].to_rad(ticks0[j]) for j in CHAIN_JOINTS])
    grip_rad = mapping.joints["gripper"].to_rad(ticks0["gripper"])
    p0 = fk(chain, q0, tcp=DEFAULT_TCP)[:3, 3]
    out = plan_line(chain, mapping, q0, p0 + vec, lim, speed=args.speed, rate_hz=args.rate, collision=collision,
                    gripper_rad=grip_rad)  # fmt: skip
    back = plan_line(chain, mapping, out.qs[-1], p0, lim, speed=args.speed, rate_hz=args.rate, collision=collision,
                     gripper_rad=grip_rad) if out.ok else out  # fmt: skip
    console.print(f"tool point now at {np.round(p0 * 1000, 1)} mm (base frame); line {np.round(vec * 1000, 1)} mm "
                  f"at {args.speed * 1000:.0f} mm/s peak, out, hold {args.pause_s:g} s, back")  # fmt: skip
    for name, pl in (("out", out), ("back", back)):
        console.print(f"  {name:4s}: {'[green]ok' if pl.ok else '[red]REFUSED'}[/] {pl.reason if not pl.ok else ''} "
                      f"duration {pl.duration_s:.1f} s, max joint step {np.degrees(pl.max_dq):.2f}°/cycle, "
                      f"IK line residual {pl.max_line_dev_m * 1000:.2f} mm, closest to table {pl.min_table_m * 1000:+.0f} mm")  # fmt: skip
    if not (out.ok and back.ok):
        bus.close()
        sys.exit("plan refused: nothing was written. Move the arm to a clearer pose or shorten the line.")
    if not args.yes:
        try:
            input("Hold the arm in this pose if it isn't resting. Enter = enable torque and run (Ctrl-C aborts)… ")
        except KeyboardInterrupt:
            bus.close()
            sys.exit("\naborted before enabling")

    run = Run("cartesian_sim" if args.sim else "cartesian", "follower", config_paths=[cfg.path, mapping.path],
              notes=f"step 9d line {np.round(vec * 1000, 1).tolist()} mm @ {args.speed} m/s; sim={args.sim}; {args.notes}")  # fmt: skip
    live = LiveSession(run, {"follower": cfg.expected_voltage_v}, args.tui, args.viz,
                       title=f"[TORQUE ON: CARTESIAN LINE] {run.run_id} — Ctrl-C = freeze, again = torque off")  # fmt: skip
    result: dict = {"marks": {}}
    marks = result["marks"]
    t_back = out.duration_s + args.pause_s
    total = t_back + back.duration_s + 0.5
    out_fn, back_fn = out.goals_fn(), back.goals_fn()

    def goal_fn(t: float):
        return out_fn(t) if t < t_back else back_fn(t - t_back)

    def worker() -> None:
        try:
            with SafeArm(bus, cfg, hub=live.hub, collision=collision, mapping=mapping) as arm:
                result["arm"] = arm
                now = arm.step().snapshot
                drift = max(abs(now.joints[j].position - ticks0[j]) for j in CHAIN_JOINTS)
                if drift > 20:
                    raise RuntimeError(f"arm moved {drift} ticks since planning; not enabling")
                arm.enable()
                marks["move_start_ns"] = time.monotonic_ns()
                result["loop"] = control_loop(arm, args.rate, goal_fn, total, live.stop, freeze=live.freeze)
                marks["move_end_ns"] = time.monotonic_ns()
                if args.hold_at_end and not live.stop.is_set() and not arm.frozen:
                    result["holding_at_end"] = True
                    control_loop(arm, args.rate, lambda t: None, float("inf"), live.stop)
                marks["torque_off_ns"] = time.monotonic_ns()
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
            return "HOLDING AT END: support the arm, then Ctrl-C = torque off"
        return None

    live.status_fn = status
    live.ctrl_c_stops_fn = lambda: bool(((a := result.get("arm")) is not None and a.frozen) or result.get("holding_at_end"))
    th = threading.Thread(target=worker, name="safearm-cartesian")
    th.start()
    live.footer = ["TORQUE ON: Cartesian line. Ctrl-C = freeze, again = torque off."]
    live.run_until_done(th, console)
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    summary = live.close()
    bus.close()

    arm, loop = result.get("arm"), result.get("loop")
    if arm:
        run.write_json("safety_events.json", arm.report())
    # measured TCP path (encoders -> FK) during the move window
    stats = {}
    if summary["rows"] and "move_start_ns" in marks:
        rows = pq.read_table(run.dir / "telemetry").to_pylist()
        by: dict[int, dict] = {}
        for r in rows:
            if marks["move_start_ns"] <= r["t_start_ns"] <= marks.get("move_end_ns", 1 << 62) and r["position"] is not None:
                by.setdefault(r["seq"], {})[r["joint"]] = r
        pts, ts = [], []
        for seq in sorted(by):
            js = by[seq]
            if all(j in js for j in CHAIN_JOINTS):
                q = np.array([mapping.joints[j].to_rad(js[j]["position"]) for j in CHAIN_JOINTS])
                pts.append(fk(chain, q, tcp=DEFAULT_TCP)[:3, 3])
                ts.append((js[CHAIN_JOINTS[0]]["t_start_ns"] - marks["move_start_ns"]) / 1e9)
        if pts:
            P, T = np.array(pts), np.array(ts)
            moving = (T < out.duration_s) | ((T > t_back) & (T < t_back + back.duration_s))
            dev = line_point_distance(p0, p0 + vec, P[moving]) if moving.any() else np.zeros(1)
            far = P[(T > out.duration_s) & (T < t_back)]
            end = P[T > t_back + back.duration_s]
            stats = {"line_dev_max_mm": float(dev.max() * 1000), "line_dev_rms_mm": float(np.sqrt(np.mean(dev**2)) * 1000),
                     "far_end_err_mm": float(np.linalg.norm(far.mean(0) - (p0 + vec)) * 1000) if len(far) else None,
                     "return_err_mm": float(np.linalg.norm(end.mean(0) - p0) * 1000) if len(end) else None}  # fmt: skip
    run.write_json("cartesian.json", {
        "p0_m": p0.tolist(), "vector_m": vec.tolist(), "speed_mps": args.speed,
        "plan": {"out": {"duration_s": out.duration_s, "max_dq_rad": out.max_dq, "min_table_m": out.min_table_m},
                 "back": {"duration_s": back.duration_s, "max_dq_rad": back.max_dq, "min_table_m": back.min_table_m}},
        "measured": stats, "frozen": bool(arm and arm.frozen), "collision_hit": arm.collision_hit if arm else None,
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
        console.print("[green]line done" + (", held at end" if result.get("holding_at_end") else "") + ", torque off")
    if loop:
        console.print(f"loop: {loop.cycles} cycles, overruns {loop.overruns}, skipped {loop.skipped_slots}")
    if stats:
        console.print(f"straightness (encoders→FK): max {stats['line_dev_max_mm']:.1f} mm, RMS {stats['line_dev_rms_mm']:.1f} mm "
                      f"from the ideal line; far end {stats['far_end_err_mm']:.1f} mm, return {stats['return_err_mm']:.1f} mm "
                      "(lag + stops-short, see hardware log)")  # fmt: skip
    console.print(f"bus failures: {dict(bus.stats.failures) or 0}   logger: {summary}")


if __name__ == "__main__":
    main()
