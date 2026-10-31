"""Step 8 planning: repeatability + per-joint backlash tour for scripts/motion/capture_tour.py. Planning only.

    uv run scripts/sim/plan_step8_tour.py                      # targets from the step 6 run, defaults below
    uv run scripts/sim/plan_step8_tour.py --reps 10 --backlash-reps 4 --delta 100

Joint targets are the commanded ticks of named poses in a step 6 capture run (all seen sharp or usable there).
Part A (repeatability): each target approached `--reps` times from the same side: a via point with every
arm joint (pan, lift, elbow, wrist_flex) `--delta` ticks above the target, then the target. Targets are
interleaved (A B C A B C ...) so slow drift spreads evenly, hopping through `ready` between them (a direct
p22 -> p01 joint-space path came within 3 mm of the table). 2 shots per arrival (settle check: first vs late).
Part B (backlash): at the backlash pose, one joint at a time approaches from +delta and from -delta (others
unchanged), alternating, `--backlash-reps` times each; 1 shot per arrival.
Labels: A_<target>_r<k> / B_<joint>_<plus|minus>_r<k>; via points are `..._via` with capture: false.
Every move is planned and table-checked by the tour planner (as in pose_tour.py) from the park pose.
Writes config/tours/step8_repeatability.yaml.
"""

import argparse
import json
import sys

import yaml

from so101.config import REPO_ROOT, load_arm_config
from so101.kinematics.collision import CollisionModel
from so101.kinematics.fk import CHAIN_JOINTS, chain_from_mujoco
from so101.kinematics.ik import limits_rad
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel
from so101.motion.tour import Waypoint, plan_tour
from so101.safety import SafetyConfig

PARK = {"shoulder_pan": 2237, "shoulder_lift": 1056, "elbow_flex": 2878, "wrist_flex": 2972, "wrist_roll": 1138, "gripper": 2082}
ARM_JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex"]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--from-run", default="20261030-162649_capture_follower", help="step 6 capture run with the named poses")
    ap.add_argument("--targets", default="p01,p04,p22", help="part A targets (names in that run)")
    ap.add_argument("--backlash-pose", default="ready")
    ap.add_argument("--reps", type=int, default=10)
    ap.add_argument("--backlash-reps", type=int, default=4)
    ap.add_argument("--delta", type=int, default=100, help="ticks (~8.8°) the approach starts from")
    args = ap.parse_args()

    named = {}
    for line in open(REPO_ROOT / "data" / "raw" / args.from_run / "manifest.jsonl"):
        m = json.loads(line)
        if m.get("type") == "pose":
            named[m["label"].split("#")[0]] = {j: int(v) for j, v in m["commanded_ticks"].items() if j != "gripper"}  # gripper: keep current
    targets = args.targets.split(",")
    missing = [n for n in [*targets, args.backlash_pose] if n not in named]
    if missing:
        sys.exit(f"not in {args.from_run}: {missing}")

    def via(t: dict, joints: list[str], sign: int) -> dict:
        return {**t, **{j: t[j] + sign * args.delta for j in joints}}

    wps = [Waypoint("start_ready", ticks=named["ready"], capture=False)]
    for k in range(args.reps):
        for n in targets:
            wps.append(Waypoint(f"A_{n}_r{k:02d}_transit", ticks=named["ready"], capture=False))  # hop via ready: direct target-to-target paths dip to the table
            wps.append(Waypoint(f"A_{n}_r{k:02d}_via", ticks=via(named[n], ARM_JOINTS, +1), capture=False))
            wps.append(Waypoint(f"A_{n}_r{k:02d}", ticks=named[n], shots=2))
    b = named[args.backlash_pose]
    for k in range(args.backlash_reps):
        for j in ARM_JOINTS:
            for sign, tag in ((+1, "plus"), (-1, "minus")):
                wps.append(Waypoint(f"B_{j}_{tag}_r{k:02d}_via", ticks=via(b, [j], sign), capture=False))
                wps.append(Waypoint(f"B_{j}_{tag}_r{k:02d}", ticks=b, shots=1))
    wps.append(Waypoint("park", start_pose=True, dwell_s=0))

    cfg = load_arm_config("follower")
    safety = SafetyConfig.from_arm_config(cfg)
    mapping = load_mapping("follower")
    chain = chain_from_mujoco(ArmModel(mapping.model_path).m)
    plan = plan_tour(wps, PARK, chain=chain, mapping=mapping, limits_ticks=safety.limits,
                     limits_rad=limits_rad(mapping, safety.limits, CHAIN_JOINTS),
                     collision=CollisionModel.from_safety(mapping, safety), dwell_s=0.0)  # fmt: skip
    if not plan.ok:
        sys.exit(f"plan refused: {plan.reason}")
    out = REPO_ROOT / "config" / "tours" / "step8_repeatability.yaml"
    doc = {"speed": 150, "dwell_s": 0,
           "waypoints": [{"name": w.name, **({"ticks": w.ticks} if w.ticks else {}), **({"capture": False} if not w.capture else {}),
                          **({"shots": w.shots} if w.shots else {}), **({"start_pose": True, "dwell_s": 0} if w.start_pose else {})}
                         for w in wps]}  # fmt: skip
    out.write_text(f"# Step 8: repeatability ({len(targets)} targets × {args.reps}, same side, 2 shots) + backlash at "
                   f"{args.backlash_pose} ({len(ARM_JOINTS)} joints × ±{args.delta} ticks × {args.backlash_reps}).\n"
                   f"# Generated by scripts/sim/plan_step8_tour.py from {args.from_run}. Run: scripts/motion/capture_tour.py --tour <this>.\n"
                   + yaml.safe_dump(doc, sort_keys=False, default_flow_style=None))  # fmt: skip
    n_arr = sum(1 for w in wps if w.capture and not w.start_pose and w.name != "start_ready")
    n_shots = sum((w.shots or 1) for w in wps if w.capture and not w.start_pose and w.name != "start_ready")
    worst = min(s.min_table_mm for s in plan.steps)
    print(f"{len(wps)} waypoints, {n_arr} arrivals, {n_shots} shots; moves {plan.duration_s / 60:.1f} min; closest to table {worst:+.0f} mm")
    print(f"wrote {out.relative_to(REPO_ROOT)}")


if __name__ == "__main__":
    main()
