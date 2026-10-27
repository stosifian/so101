"""Step 7: show the MuJoCo SO-101 model posed from the real arm's ticks, live in Rerun. Read-only.

    uv run scripts/readonly/model_check.py                 # follower, mapping from config/calibration
    uv run scripts/readonly/model_check.py --sim           # SimBus (factory rest pose), no hardware

Rerun shows two models: "real" (posed from the arm through the tick->radian mapping) and a transparent
"ghost" at the reference pose (default: all joints 0 = LeRobot's middle-of-range calibration pose).
Move the arm by hand (torque is never enabled): if the mapping is right, "real" follows the arm exactly.
To verify zero and sign: match the arm to the ghost, record it; then move single joints and record.

In the terminal, type a label and press Enter to record the current pose (ticks + mapped angles) to
data/raw/<run_id>/poses.json; `q` + Enter quits. Every recorded pose is checked against the model ranges.

Interactive zero adjustment (live): `<joint> <+/-deg>` shifts that joint's zero, e.g. `elbow +2`, `wroll -1.5`
(joints: pan, lift, elbow, wflex, wroll, grip). Positive moves the model in the joint's + direction.
`adj` shows the adjustments, `reset <joint>` undoes one, `save` writes data/raw/<run_id>/mapping_proposal.yaml
(old vs new zero_ticks). The mapping config is only changed after review.
"""

import argparse
import dataclasses
import json
import math
import re
import threading
import time

import numpy as np
import rerun as rr
import rerun.blueprint as rrb
from rich.console import Console
from rich.table import Table

from so101.config import REPO_ROOT, load_arm_config
from so101.hw.arm import Arm
from so101.hw.bus import FeetechBus
from so101.hw.sim import SimBus
from so101.kinematics.mapping import RAD_PER_TICK, load_mapping
from so101.kinematics.model import JOINTS, ArmModel
from so101.runlog.run import Run


def log_model(rec, root: str, model: ArmModel, meshes, alpha: float) -> None:
    for vm in meshes:
        rgba = vm.rgba.copy()
        rgba[3] = alpha
        rr.log(f"{root}/geom_{vm.geom_id}", rr.Mesh3D(vertex_positions=vm.vertices, triangle_indices=vm.faces,
                                                      albedo_factor=rgba), static=True, recording=rec)  # fmt: skip


def log_pose(rec, root: str, model: ArmModel, meshes, static: bool = False) -> None:
    for vm in meshes:
        pos, mat = model.geom_pose(vm.geom_id)
        rr.log(f"{root}/geom_{vm.geom_id}", rr.Transform3D(translation=pos, mat3x3=mat), static=static, recording=rec)


SHORT = {"pan": "shoulder_pan", "lift": "shoulder_lift", "elbow": "elbow_flex", "wflex": "wrist_flex",
         "wroll": "wrist_roll", "grip": "gripper"}  # fmt: skip


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--ghost", default="0,0,0,0,0,0", help="reference pose in degrees, 6 comma-separated values")
    ap.add_argument("--sim", action="store_true")
    ap.add_argument("--notes", default="")
    args = ap.parse_args()

    console = Console()
    cfg = load_arm_config("follower")
    mapping = load_mapping("follower")
    real, ghost = ArmModel(mapping.model_path), ArmModel(mapping.model_path)
    meshes = real.visual_meshes()
    ghost_q = dict(zip(JOINTS, (math.radians(float(x)) for x in args.ghost.split(","))))

    if args.sim:
        bus = SimBus.from_dump(json.loads((REPO_ROOT / "data/raw/20261026-112606_regdump_follower/registers.json").read_text()))
    else:
        bus = FeetechBus(cfg.port, cfg.baudrate, cfg.protocol_end)
    bus.claim("model_check")  # read-only, but nothing else should use the bus meanwhile
    arm = Arm(bus, cfg.joints)

    run = Run("model_check_sim" if args.sim else "model_check", "follower",
              config_paths=[cfg.path, mapping.path], notes=f"step 7 mapping check ({mapping.status}); {args.notes}")  # fmt: skip
    rec = rr.RecordingStream("so101", recording_id=run.run_id)
    rr.spawn(connect=False, recording=rec)
    rr.set_sinks(rr.GrpcSink(), rr.FileSink(str(run.path("model.rrd"))), recording=rec)
    rr.send_blueprint(rrb.Blueprint(rrb.Spatial3DView(origin="/model", name="SO-101: real (from ticks) vs ghost")),
                      make_active=True, recording=rec)  # fmt: skip
    time.sleep(2.0)  # viewer settle (step 3e)
    rr.log("/model", rr.ViewCoordinates.RIGHT_HAND_Z_UP, static=True, recording=rec)
    log_model(rec, "/model/real", real, meshes, alpha=1.0)
    log_model(rec, "/model/ghost", ghost, meshes, alpha=0.25)
    ghost.set_q(ghost_q)
    log_pose(rec, "/model/ghost", ghost, meshes, static=True)

    original = mapping
    state: dict = {"snap": None, "stop": False, "t0": time.monotonic(), "mapping": mapping, "adj_deg": {}}

    def reader() -> None:
        while not state["stop"]:
            snap = arm.snapshot()
            state["snap"] = snap
            ticks = {j: s.position for j, s in snap.joints.items() if s is not None}
            if len(ticks) == len(JOINTS):
                real.set_q(state["mapping"].to_q(ticks))
                rr.set_time("monotonic", duration=np.timedelta64(snap.t_start_ns - run.header["started_monotonic_ns"], "ns"),
                            recording=rec)  # fmt: skip
                log_pose(rec, "/model/real", real, meshes)
                rr.log("/model/tool", rr.Points3D([real.tool_pos()], radii=0.006, colors=[255, 60, 60]), recording=rec)
            time.sleep(0.1)

    th = threading.Thread(target=reader, daemon=True)
    th.start()
    console.print(f"[bold]{run.run_id}[/]: mapping status [yellow]{mapping.status}[/] ({mapping.path.name}); "
                  f"ghost = {args.ghost} deg. Torque is never enabled: move the arm by hand.")  # fmt: skip
    poses = []
    try:
        while True:
            label = input("label = record pose | <joint> +/-deg = nudge zero | adj | reset <joint> | save | q > ").strip()
            if label.lower() == "q":
                break
            m = re.fullmatch(r"(pan|lift|elbow|wflex|wroll|grip)\s+([+-]?\d+(?:\.\d+)?)", label)
            if m:
                j, deg = SHORT[m.group(1)], float(m.group(2))
                state["adj_deg"][j] = state["adj_deg"].get(j, 0.0) + deg
                jm = state["mapping"].joints[j]
                # model angle q = sign*(t - zero): to move the model by +deg, lower zero by sign*deg
                new = dataclasses.replace(jm, zero_ticks=jm.zero_ticks - jm.sign * math.radians(deg) / RAD_PER_TICK)
                state["mapping"] = dataclasses.replace(state["mapping"], joints={**state["mapping"].joints, j: new})
                console.print(f"{j}: zero {original.joints[j].zero_ticks:.0f} → {new.zero_ticks:.1f} "
                              f"(total {state['adj_deg'][j]:+.1f}°)")  # fmt: skip
                continue
            if label.lower() == "adj":
                console.print({j: f"{a:+.1f}° → zero {state['mapping'].joints[j].zero_ticks:.1f}" for j, a in state["adj_deg"].items()} or "no adjustments")
                continue
            m = re.fullmatch(r"reset\s+(pan|lift|elbow|wflex|wroll|grip)", label.lower())
            if m:
                j = SHORT[m.group(1)]
                state["adj_deg"].pop(j, None)
                state["mapping"] = dataclasses.replace(state["mapping"], joints={**state["mapping"].joints, j: original.joints[j]})
                console.print(f"{j}: back to zero {original.joints[j].zero_ticks:.0f}")
                continue
            if label.lower() == "save":
                prop = {"source_mapping": str(mapping.path.relative_to(REPO_ROOT)), "run_id": run.run_id,
                        "joints": {j: {"old_zero_ticks": original.joints[j].zero_ticks,
                                       "new_zero_ticks": round(state["mapping"].joints[j].zero_ticks, 1),
                                       "adjust_deg": round(state["adj_deg"].get(j, 0.0), 2)} for j in JOINTS}}  # fmt: skip
                path = run.dir / f"mapping_proposal_{time.strftime('%H%M%S')}.json"
                path.write_text(json.dumps(prop, indent=2))
                console.print(f"saved proposal → {path}")
                continue
            snap = state["snap"]
            if snap is None or not snap.ok:
                console.print("[red]no complete reading yet")
                continue
            ticks = {j: snap.joints[j].position for j in JOINTS}
            q = state["mapping"].to_q(ticks)
            bad = state["mapping"].check(ticks, real.ranges)
            real.set_q(q)
            poses.append({"label": label or f"pose{len(poses) + 1}", "t_ns": snap.t_start_ns, "ticks": ticks,
                          "adjust_deg": dict(state["adj_deg"]),
                          "q_deg": {j: round(math.degrees(v), 2) for j, v in q.items()},
                          "tool_m": [round(float(x), 4) for x in real.tool_pos()], "outside_model_range": list(bad)})  # fmt: skip
            t = Table(title=f"recorded '{poses[-1]['label']}'", title_justify="left")
            for col in ["joint", "ticks", "q (deg)", "model range (deg)", ""]:
                t.add_column(col, justify="left" if col == "joint" else "right")
            for j in JOINTS:
                lo, hi = real.ranges[j]
                t.add_row(j, str(ticks[j]), f"{math.degrees(q[j]):+.1f}", f"{math.degrees(lo):+.0f}…{math.degrees(hi):+.0f}",
                          "[red]OUTSIDE" if j in bad else "")  # fmt: skip
            console.print(t)
            console.print(f"tool at {poses[-1]['tool_m']} m (model frame)")
    except (KeyboardInterrupt, EOFError):
        pass
    finally:
        state["stop"] = True
        th.join(1.0)
        bus.release("model_check")
        bus.close()
        run.write_json("poses.json", {"mapping": str(mapping.path.relative_to(REPO_ROOT)), "status": mapping.status,
                                      "ghost_deg": args.ghost, "poses": poses})  # fmt: skip
        rec.flush()
        console.print(f"saved {len(poses)} poses → {run.dir / 'poses.json'}; replay: uv run rerun {run.dir / 'model.rrd'}")


if __name__ == "__main__":
    main()
