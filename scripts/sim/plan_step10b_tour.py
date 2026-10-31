"""Step 10b planning: live command-accuracy tour (planning only; nothing is sent to the arm).

    uv run scripts/sim/plan_step10b_tour.py                       # 15 targets × 3 methods, 2 shots each
    uv run scripts/sim/plan_step10b_tour.py --n 12 --jitter-mm 15 --seed 0

Targets: tag-centre positions RELATIVE TO THE iPad (mm), each a step 10 test pose's measured tag position moved by
a random offset (±--jitter-mm per axis), so none was ever visited exactly. Each target is commanded three ways,
in a rotating order, every arrival preceded by a transit through `ready` (same approach for all methods):
  nominal   IK with config/kinematics/follower_nominal.json, goal = IK angles (today's behaviour)
  cal       IK with follower_stiff_lin.json + gravity-bend and servo-droop feedforward (so101.motion.calibrated_ik)
  cal_corr  as cal + one encoder-reading correction after settling (capture_tour.py desired_ticks)
--anchor (after the iPad moved): today's iPad<-base per variant; targets are the same physical spots expressed in
today's iPad frame, and the visibility check uses the anchor run's iPad pose. Targets the calibrated model predicts out of frame / tilted > 70° are dropped. Every move is table-checked by the
planner. Writes config/tours/step10b_command.yaml and data/derived/step10b/targets.json.
"""

import argparse
import json
import sys
from pathlib import Path

import cv2
import numpy as np
import yaml

from so101.analysis.pose import Camera, solve, tag_object_points
from so101.config import REPO_ROOT, load_arm_config
from so101.kinematics import calibrated
from so101.kinematics.collision import CollisionModel
from so101.kinematics.fk import CHAIN_JOINTS, chain_from_mujoco, fk
from so101.kinematics.ik import limits_rad
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel
from so101.motion.calibrated_ik import goals, solve_tag
from so101.motion.tour import Waypoint, plan_tour
from so101.safety import SafetyConfig

PARK = {"shoulder_pan": 2237, "shoulder_lift": 1056, "elbow_flex": 2878, "wrist_flex": 2972, "wrist_roll": 1138, "gripper": 2082}
READY = {"shoulder_pan": 2237, "shoulder_lift": 1677, "elbow_flex": 2257, "wrist_flex": 2833, "wrist_roll": 1138}
METHODS = ["nominal", "cal", "cal_corr"]
OUT = REPO_ROOT / "data" / "derived" / "step10b"


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="20261030-202915_capture_follower", help="test run whose tag poses seed the targets")
    ap.add_argument("--n", type=int, default=15)
    ap.add_argument("--jitter-mm", type=float, default=15.0)
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--variant", default="stiff_lin")
    ap.add_argument("--anchor", type=Path, help="config/kinematics/anchors/<run>.json: today's iPad<-base per variant (the iPad moved)")
    args = ap.parse_args()
    rng = np.random.default_rng(args.seed)
    cam, obj = Camera.load(), tag_object_points(20.0)
    mapping = load_mapping("follower")
    arm = ArmModel(mapping.model_path)
    chain = chain_from_mujoco(arm.m)
    cfg = load_arm_config("follower")
    safety = SafetyConfig.from_arm_config(cfg)
    lim = limits_rad(mapping, safety.limits, CHAIN_JOINTS)
    col = CollisionModel.from_safety(mapping, safety)
    nom, cal = calibrated.load("nominal", anchor=args.anchor), calibrated.load(args.variant, anchor=args.anchor)
    cal_fit = calibrated.load(args.variant)  # its own X, from the fit (the source run's setup)

    rows = json.loads((REPO_ROOT / "data/derived/kincal/cache" / f"{args.source}_tag2_ipad.json").read_text())
    src = {}
    for r in rows:
        name = r["label"].split("#")[0]
        if not name.startswith("t_") or r.get("T_cam_ipad") is None:
            continue
        Ti = np.array(r["T_cam_ipad"])
        Tt = solve(obj, np.array(r["corners"]), cam, 0.05, square=True).matrix()
        src.setdefault(name, []).append(((np.linalg.inv(Ti) @ Tt)[:3, 3], r["ticks"], Ti))
    T_cam_ipad = np.mean([x[2] for v in src.values() for x in v], axis=0)  # for the visibility check only
    move = np.eye(4)  # source-run iPad frame -> today's iPad frame, through the arm base (best model's transforms)
    if args.anchor:
        move = cal.X_ref_base["all"] @ np.linalg.inv(cal_fit.X_ref_base["all"])
        a = json.loads(args.anchor.read_text())
        rows_a = json.loads((REPO_ROOT / "data/derived/kincal/cache" / f"{a['anchor_run']}_tag2_ipad.json").read_text())
        T_cam_ipad = np.mean([np.array(r["T_cam_ipad"]) for r in rows_a if r.get("T_cam_ipad")], axis=0)

    def visible(c: calibrated.Calibrated, q: np.ndarray) -> str:
        F = fk(chain, q, c.params, tcp=np.eye(4))
        F[:3, 3] *= 1000
        T = T_cam_ipad @ c.X_ref_base["all"] @ F @ c.Y_body_tag
        px = cv2.projectPoints(obj, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], cam.K, cam.dist)[0].reshape(-1, 2)
        tilt = np.degrees(np.arccos(np.clip(-T[:3, 2] @ T[:3, 3] / np.linalg.norm(T[:3, 3]), -1, 1)))
        if not ((px[:, 0] > 150).all() and (px[:, 0] < 7028 - 150).all() and (px[:, 1] > 150).all() and (px[:, 1] < 3600).all()):
            return "out of frame"
        return f"tilt {tilt:.0f}°" if tilt > 70 else ""

    targets, wps = [], [Waypoint("start_ready", ticks=READY, capture=False)]
    for k, (name, v) in enumerate(sorted(src.items())):
        if len(targets) >= args.n:
            break
        p_src = (move @ np.r_[np.mean([x[0] for x in v], axis=0), 1.0])[:3]  # same physical spot, today's iPad frame
        ticks0 = v[0][1]
        target = p_src + rng.uniform(-args.jitter_mm, args.jitter_mm, 3)
        q_seed = np.array([mapping.joints[j].to_rad(ticks0[j]) for j in CHAIN_JOINTS])
        base = {**ticks0, "gripper": PARK["gripper"]}
        qn, qc = solve_tag(nom, chain, lim, target, q_seed), solve_tag(cal, chain, lim, target, q_seed)
        if qn is None or qc is None:
            print(f"skip {name}: IK failed")
            continue
        if (why := visible(cal, qc)) or (why := visible(nom, qn)):
            print(f"skip {name}: {why}")
            continue
        g_nom, _ = goals(nom, mapping, arm, qn, base, compensate=False)
        g_cal, rd_cal = goals(cal, mapping, arm, qc, base, compensate=True)
        ti = len(targets)
        cmds = {"nominal": (g_nom, None), "cal": (g_cal, None), "cal_corr": (g_cal, rd_cal)}
        order = METHODS[ti % 3 :] + METHODS[: ti % 3]
        new = []
        for mth in order:
            g, desired = cmds[mth]
            meta = {"target": ti, "method": mth, "target_ipad_mm": [round(float(x), 3) for x in target], "source": name}
            new.append(Waypoint(f"T{ti:02d}_{mth}_transit", ticks=READY, capture=False))
            new.append(Waypoint(f"T{ti:02d}_{mth}", ticks={j: int(x) for j, x in g.items()}, shots=2, meta=meta,
                                desired_ticks={j: int(rd_cal[j]) for j in CHAIN_JOINTS} if desired else None))  # fmt: skip
        plan = plan_tour(wps + new, PARK, chain=chain, mapping=mapping, limits_ticks=safety.limits, limits_rad=lim, collision=col, dwell_s=0)
        if not plan.ok:
            print(f"skip {name}: {plan.reason}")
            continue
        wps += new
        targets.append({"target": ti, "source": name, "target_ipad_mm": target.tolist(), "goals": {m: cmds[m][0] for m in METHODS},
                        "desired_reading_cal": rd_cal})  # fmt: skip
    wps += [Waypoint("ref_end", ticks=READY, shots=2), Waypoint("park", start_pose=True, dwell_s=0)]
    plan = plan_tour(wps, PARK, chain=chain, mapping=mapping, limits_ticks=safety.limits, limits_rad=lim, collision=col, dwell_s=0)
    if not plan.ok:
        sys.exit(f"tour refused: {plan.reason}")

    def w2d(w: Waypoint) -> dict:
        d = {"name": w.name}
        if w.start_pose:
            return {**d, "start_pose": True, "dwell_s": 0}
        d["ticks"] = {j: int(x) for j, x in w.ticks.items()}
        for k_ in ("capture", "shots", "desired_ticks", "meta"):
            v_ = getattr(w, k_)
            if v_ is not None and not (k_ == "capture" and v_):
                d[k_] = v_
        return d

    out = REPO_ROOT / "config" / "tours" / "step10b_command.yaml"
    out.write_text(f"# Step 10b: command accuracy. {len(targets)} tag targets (rel. iPad, mm) × {METHODS}, 2 shots each, transit via ready.\n"
                   f"# Generated by scripts/sim/plan_step10b_tour.py (variant {args.variant}, source {args.source}, seed {args.seed}, anchor {args.anchor.name if args.anchor else '-'}).\n"
                   + yaml.safe_dump({"speed": 150, "dwell_s": 0, "waypoints": [w2d(w) for w in wps]}, sort_keys=False, default_flow_style=None))  # fmt: skip
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "targets.json").write_text(json.dumps({"variant": args.variant, "source": args.source, "seed": args.seed,
                                                  "anchor": str(args.anchor) if args.anchor else None, "targets": targets}, indent=1, default=float))
    n_arr = 3 * len(targets)
    print(f"{len(targets)} targets, {n_arr} arrivals, {2 * n_arr + 2} shots; moves {plan.duration_s / 60:.1f} min; "
          f"closest to table {min(s.min_table_mm for s in plan.steps):+.0f} mm; ~{(plan.duration_s + n_arr * 23 + len(targets) * 4) / 60:.0f} min total")
    print(f"wrote {out.relative_to(REPO_ROOT)}, {(OUT / 'targets.json').relative_to(REPO_ROOT)}")
    for t in targets:
        d = {j: t["goals"]["cal"][j] - t["goals"]["nominal"][j] for j in CHAIN_JOINTS}
        print(f"  T{t['target']:02d} ({t['source']}): cal − nominal goal ticks {d}")


if __name__ == "__main__":
    main()
