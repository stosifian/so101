"""Step 6b planning: a spherical-sector working volume from the hand-taught poses (step 6a), filled with an even
grid of tool points, each solved and checked by the tour planner. Planning only: nothing is sent to the arm.

    uv run scripts/sim/plan_sector_tour.py data/raw/<teach_run> [...]          # ~20 points
    uv run scripts/sim/plan_sector_tour.py data/raw/*_teach_follower --n 20 --z-min 30

Sector: centred on the pan axis at shoulder-lift height; r = reach from there, theta = azimuth (≈ pan),
phi = elevation. Bounds = min/max over the CLEAN taught poses (tag found, usable sharpness, arm still).
Grid: an even 3D Cartesian grid (spacing chosen to give ~--n points) clipped to the sector, then filtered:
  - z >= --z-min (mm): two taught poses read slightly below the table in the model;
  - predicted tag-camera distance within the taught clean range (linear fit of distance vs tool point over the
    clean poses; the sector's corners combine extremes never taught, e.g. full reach on the camera side);
  - predicted tag position in the image (linear fit, same way) inside the frame and above the uncalibrated band;
  - position-only IK seeded from the nearest clean taught pose (keeps the wrist, so the tag, oriented as it was
    there; a fixed 45°-down orientation was unreachable for most of the sector) + joint limits + table check;
  - every move between consecutive points (and back to park) passes the planner's table check.
Writes config/tours/step6_sector.yaml (the tour: points in a nearest-neighbour order, then park) and
data/derived/sector_plan/sector_plan.html + .png (planned points over the taught ones, sector wireframe,
rejected candidates with the reason) + plan.json (joint targets in ticks and degrees).
"""

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
import yaml

from so101.config import REPO_ROOT, load_arm_config
from so101.kinematics.collision import CollisionModel
from so101.analysis.handeye import HandEye
from so101.analysis.pose import Camera
from so101.kinematics.fk import CHAIN_JOINTS, chain_from_mujoco, fk, frames
from so101.kinematics.ik import limits_rad, solve_ik
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel
from so101.motion.tour import ALL_JOINTS, Waypoint, plan_tour
from so101.safety import SafetyConfig

PARK = {"shoulder_pan": 2237, "shoulder_lift": 1056, "elbow_flex": 2878, "wrist_flex": 2972, "wrist_roll": 1138, "gripper": 2082}  # tour 20261030-124817
SHARP, USABLE = 0.13, 0.30  # as scripts/camera/teach_map.py
OUT = REPO_ROOT / "data" / "derived" / "sector_plan"
HANDEYE = REPO_ROOT / "data" / "derived" / "handeye" / "handeye.json"  # scripts/camera/handeye_fit.py


def clean_poses(runs: list[Path]) -> tuple[list[dict], list[dict]]:
    """(clean, all) taught poses from data/derived/<run>/teach.json."""
    allp = []
    for r in runs:
        f = REPO_ROOT / "data" / "derived" / r.name / "teach.json"
        if f.exists():
            allp += [{**p, "run": r.name} for p in json.loads(f.read_text())["poses"]]
    clean = [p for p in allp if p["tag"]["found"] and not p["moved"] and not p["tag"]["uncalibrated"] and p["tag"]["edge_rms"] <= USABLE]
    return clean, allp


def to_sph(p: np.ndarray, c: np.ndarray) -> np.ndarray:
    d = p - c
    r = np.linalg.norm(d, axis=-1)
    return np.stack([r, np.arctan2(d[..., 1], d[..., 0]), np.arcsin(d[..., 2] / r)], axis=-1)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", type=Path, nargs="+")
    ap.add_argument("--n", type=int, default=20, help="target number of planned points")
    ap.add_argument("--pan-min", type=int, help="ticks; stricter iPad-side pan bound than the taught minimum")
    ap.add_argument("--max-tilt", type=float, default=70.0, help="deg, tag normal vs line of sight (hand-eye; found up to 72°, lost at 78-88°)")
    ap.add_argument("--no-handeye", action="store_true", help="ignore data/derived/handeye/handeye.json (linear fits only)")
    ap.add_argument("--z-min", type=float, default=30.0, help="mm, tool point")
    ap.add_argument("--rolls", default="0", help="wrist_roll offsets (deg) to try at every position, e.g. 0,-25,25 (step 10: makes roll/wrist offsets observable)")
    ap.add_argument("--roll-cycle", action="store_true", help="one roll per position, cycling through --rolls (first visible one in that order): more distinct positions for the same count")
    ap.add_argument("--wrist-alts", type=int, default=1, help="wrist-angle solutions per position and roll (2 = also the next-best visible one, >= 15° apart)")
    ap.add_argument("--shots", type=int, help="photos per waypoint in the tour file (capture_tour.py)")
    ap.add_argument("--name", default="step6_sector", help="tour file config/tours/<name>.yaml; outputs data/derived/<name>/ (step6_sector: data/derived/sector_plan/)")
    ap.add_argument("--overlay", type=Path, help="a previous plan.json whose points are drawn for comparison")
    ap.add_argument("--dwell-s", type=float, default=10.0, help="hold per point in the tour file (capture comes with the step 6 script)")
    args = ap.parse_args()
    global OUT
    OUT = OUT if args.name == "step6_sector" else REPO_ROOT / "data" / "derived" / args.name
    rolls = [float(x) for x in args.rolls.split(",")]

    clean, allp = clean_poses(args.runs)
    if len(clean) < 4:
        sys.exit(f"need >= 4 clean taught poses, have {len(clean)}")
    cfg = load_arm_config("follower")
    safety = SafetyConfig.from_arm_config(cfg)
    mapping = load_mapping("follower")
    chain = chain_from_mujoco(ArmModel(mapping.model_path).m)
    lim = limits_rad(mapping, safety.limits, CHAIN_JOINTS)
    collision = CollisionModel.from_safety(mapping, safety)

    cam = Camera.load()
    he = None
    if HANDEYE.exists() and not args.no_handeye:
        d = json.loads(HANDEYE.read_text())
        he = HandEye(np.array(d["T_cam_base"]), np.array(d["T_link_tag"]), d["rms_px"], np.array(d["per_pose_px"]), d["tag_mm"])
    jf, _ = frames(chain, np.zeros(5))
    centre = np.array([0.0, 0.0, jf[1][2, 3] * 1000])  # pan axis at shoulder-lift height (mm)
    P = np.array([p["tool_mm"] for p in clean])
    S = to_sph(P, centre)
    lo, hi = S.min(0), S.max(0)
    # tag-camera distance ≈ a + g·p (fit over clean poses)
    A = np.c_[np.ones(len(P)), P]
    dist = np.array([p["tag"]["distance_mm"] for p in clean])
    coef = np.linalg.lstsq(A, dist, rcond=None)[0]
    d_lo, d_hi = dist.min(), dist.max()
    fit_sd = float(np.std(dist - A @ coef))
    # where the tag lands in the image ≈ linear in the tool point too (fit over clean poses): keeps the grid's
    # high corners from putting the tag above the frame (the taught high poses lost it there)
    uv = np.array([p["tag"]["centre_px"] for p in clean])
    cuv = np.linalg.lstsq(A, uv, rcond=None)[0]
    uv_sd = np.std(uv - A @ cuv, axis=0)
    v_lo, v_hi = 250.0, 3700.0 - 250  # rows: half a tag + fit error clear of the top edge / the uncalibrated band
    u_lo, u_hi = 250.0, 7028.0 - 250

    def candidates(step: float) -> np.ndarray:
        bb_lo, bb_hi = P.min(0) - 150, P.max(0) + 150
        bb_lo[2] = args.z_min  # anchor the grid's lowest layer at the height floor (most taught poses are low)
        g = np.stack(np.meshgrid(*[np.arange(bb_lo[k], bb_hi[k] + 1e-9, step) for k in range(3)], indexing="ij"), -1).reshape(-1, 3)
        s = to_sph(g, centre)
        inside = np.all((s >= lo - 1e-9) & (s <= hi + 1e-9), axis=1)
        return g[inside]

    def prefilter(g: np.ndarray) -> tuple[np.ndarray, list[str]]:
        reasons = []
        for p in g:
            dp = coef[0] + coef[1:] @ p
            u, v = cuv[0] + p @ cuv[1:]
            if he is not None:  # the hand-eye prediction replaces the linear fits (checked per IK solution)
                reasons.append(f"z {p[2]:.0f} < {args.z_min:.0f} mm" if p[2] < args.z_min else "")
                continue
            reasons.append(f"z {p[2]:.0f} < {args.z_min:.0f} mm" if p[2] < args.z_min else
                           f"predicted camera distance {dp:.0f} mm outside taught {d_lo:.0f}-{d_hi:.0f}" if not d_lo <= dp <= d_hi else
                           f"tag predicted at image ({u:.0f}, {v:.0f}): outside the frame / calibrated region" if not (u_lo <= u <= u_hi and v_lo <= v <= v_hi) else "")  # fmt: skip
        return g, reasons

    Q = [{j: min(max(p["ticks"][j], safety.limits[j][0]), safety.limits[j][1]) for j in ALL_JOINTS} for p in clean]
    pans = [q["shoulder_pan"] for q in Q]
    pan_lo, pan_hi = max(min(pans), args.pan_min or 0), max(pans)

    def to_ticks(q: np.ndarray, base: dict) -> dict:
        return {**base, **{j: round(mapping.joints[j].to_ticks(v)) for j, v in zip(CHAIN_JOINTS, q)}}

    def view(t: dict) -> dict | None:
        """Hand-eye prediction of the tag in the image for joint ticks t (None without a hand-eye fit)."""
        if he is None:
            return None
        q = np.array([mapping.joints[j].to_rad(t[j]) for j in CHAIN_JOINTS])
        F = fk(chain, q, tcp=np.eye(4))
        F[:3, 3] *= 1000
        return he.predict(F, cam)

    def visible(v: dict) -> str:
        c = v["corners_px"]
        if not ((c[:, 0] > 150).all() and (c[:, 0] < 7028 - 150).all() and (c[:, 1] > 150).all() and (c[:, 1] < 3700 - 100).all()):
            return f"tag predicted outside the frame / calibrated region (centre {c.mean(0).round(0).tolist()})"
        if v["tilt_deg"] > args.max_tilt:
            return f"tag tilt {v['tilt_deg']:.0f}° > {args.max_tilt:.0f}°"
        if not d_lo <= v["distance_mm"] <= d_hi:
            return f"tag {v['distance_mm']:.0f} mm from the camera, outside taught {d_lo:.0f}-{d_hi:.0f}"
        return ""

    def solve_point(p: np.ndarray, idx: int = 0) -> tuple[list[tuple[dict, str]], str]:
        """Position-only IK from several seeds: the nearest clean taught pose with its wrist_flex varied, then the
        other taught poses; for each --rolls offset (wrist_roll held at the seed's roll + offset). Solutions differ
        mostly in wrist angle, i.e. in which way the tag faces. With a hand-eye fit: per roll, keep the solution that
        faces the tag most squarely at the camera (in frame, tilt <= --max-tilt, distance in the taught range), and
        with --wrist-alts 2 also the best one whose wrist_flex differs by >= 15°; without a fit: the first solution.
        Each kept solution must pass the planner's limit + table check from park. Returns ([(ticks, suffix)], why)."""
        order = list(np.argsort(np.linalg.norm(P - p, axis=1)))
        base = []
        for k in order[:1]:
            q0 = np.array([mapping.joints[j].to_rad(Q[k][j]) for j in CHAIN_JOINTS])
            for dw in (0, -20, 20, -40, 40, -60, 60):
                q = q0.copy()
                q[3] += np.radians(dw)
                base.append((k, q))
        base += [(k, np.array([mapping.joints[j].to_rad(Q[k][j]) for j in CHAIN_JOINTS])) for k in order[1:]]
        out, why = [], "IK failed"
        order_r = rolls[idx % len(rolls):] + rolls[: idx % len(rolls)] if args.roll_cycle else rolls
        for droll in order_r:
            if args.roll_cycle and out:
                break
            sols = []
            for k, q0 in base:
                q0 = q0.copy()
                q0[4] += np.radians(droll)
                r = solve_ik(chain, p / 1000, q0, lim, fixed={4: q0[4]})
                if not r.success:
                    continue
                t = to_ticks(r.q, Q[k])
                if not pan_lo <= t["shoulder_pan"] <= pan_hi:  # the iPad side: never past the taught pan extremes
                    why = f"shoulder_pan {t['shoulder_pan']} outside the taught range {pan_lo}-{pan_hi}"
                    continue
                v = view(t)
                if v is not None and (w := visible(v)):
                    why = w
                    continue
                sols.append((v["tilt_deg"] if v else 0.0, t, v))
                if he is None:
                    break
            kept = []
            for _, t, v in sorted(sols, key=lambda x: x[0]):
                if len(kept) >= args.wrist_alts:
                    break
                if any(abs(t["wrist_flex"] - k2["wrist_flex"]) < 15 / 0.088 for k2 in kept):  # ~15° apart (0.088°/tick)
                    continue
                pl = plan_tour([Waypoint("probe", ticks=t)], PARK, chain=chain, mapping=mapping, limits_ticks=safety.limits,
                               limits_rad=lim, collision=collision)  # fmt: skip
                if pl.ok:
                    kept.append(t)
                else:
                    why = pl.reason.replace("probe: ", "")
            for a_i, t in enumerate(kept):
                sfx = ("" if droll == 0 else f"_r{droll:+.0f}") + ("" if a_i == 0 else f"_w{a_i + 1}")
                out.append((t, sfx))
        return out, why

    # spacing: the largest grid step that still yields >= n feasible waypoints (positions × variants)
    for step in np.arange(110.0, 24.0, -5.0):
        g, why = prefilter(candidates(step))
        if sum(not w for w in why) * (1 if args.roll_cycle else len(rolls)) * args.wrist_alts < args.n:
            continue
        rejected, feas = [], []
        for i_p, (p, w) in enumerate(zip(g, why)):
            vs, w2 = ([], w) if w else solve_point(p, i_p)
            (feas.append((p, vs)) if vs else rejected.append((p, w or w2)))
        if sum(len(vs) for _, vs in feas) >= args.n:
            break
    # thin POSITIONS by farthest-point sampling (keeps the spread) until their variants reach n
    pos = np.array([p for p, _ in feas])
    keep = [int(np.argmin(np.linalg.norm(pos - pos.mean(0), axis=1)))]
    while sum(len(feas[k][1]) for k in keep) < args.n and len(keep) < len(feas):
        d = np.min(np.linalg.norm(pos[:, None] - pos[keep][None], axis=2), axis=1)
        d[keep] = -1
        keep.append(int(np.argmax(d)))
    for k in sorted(set(range(len(feas))) - set(keep)):
        rejected.append((pos[k], "thinned (spacing)"))
    pts = np.array([feas[k][0] for k in keep for _ in feas[k][1]])
    tk = [t for k in keep for t, _ in feas[k][1]]
    sfx = [s_ for k in keep for _, s_ in feas[k][1]]
    # order: nearest-neighbour from ready, but each move must pass the planner (joint-space paths can dip toward
    # the table between two fine points); a point no move can reach is dropped
    def wp(i: int) -> Waypoint:
        return Waypoint(f"p{i:02d}{sfx[i]}", ticks=tk[i], shots=args.shots)

    ready = Waypoint("ready", pos_m=[0.25, 0.0, 0.14], approach=[0.707, 0.0, -0.707])  # known-good first move off the rest stops (5b(b))

    def plan_of(seq: list[int], close: bool = False):
        ws = [ready] + [wp(i) for i in seq] + ([Waypoint("park", start_pose=True, dwell_s=0)] if close else [])
        return plan_tour(ws, PARK, chain=chain, mapping=mapping, limits_ticks=safety.limits, limits_rad=lim,
                         collision=collision, dwell_s=args.dwell_s)  # fmt: skip

    seq, cur, left = [], np.array([250.0, 0, 140]), list(range(len(pts)))
    while left:
        for k in sorted(left, key=lambda i: np.linalg.norm(pts[i] - cur)):
            if plan_of(seq + [k]).ok:
                seq.append(k), left.remove(k)
                cur = pts[k]
                break
        else:
            for k in left:
                rejected.append((pts[k], "no table-safe move from the previous point"))
            break
    while seq and not plan_of(seq, close=True).ok:  # the last move back to park must pass too
        rejected.append((pts[seq.pop()], "no table-safe move back to park"))
    pts, tk, sfx = pts[seq], [tk[i] for i in seq], [sfx[i] for i in seq]
    wps = [ready] + [Waypoint(f"p{i:02d}{sfx[i]}", ticks=t, shots=args.shots) for i, t in enumerate(tk)]
    wps.append(Waypoint("park", start_pose=True, dwell_s=0))
    plan = plan_tour(wps, PARK, chain=chain, mapping=mapping, limits_ticks=safety.limits, limits_rad=lim, collision=collision, dwell_s=args.dwell_s)
    if not plan.ok:
        sys.exit(f"full tour refused: {plan.reason}")

    # outputs
    OUT.mkdir(parents=True, exist_ok=True)
    tour = {"speed": 150, "dwell_s": args.dwell_s,
            "waypoints": [{"name": "ready", "pos_m": ready.pos_m, "approach": ready.approach}]
                         + [{"name": w.name, "ticks": {j: int(v) for j, v in w.ticks.items()}, **({"shots": w.shots} if w.shots else {})}
                            for w in wps[1:-1]]
                         + [{"name": "park", "start_pose": True, "dwell_s": 0}]}  # fmt: skip
    tour_path = REPO_ROOT / "config" / "tours" / f"{args.name}.yaml"
    header = (f"# {args.name}: {len(pts)} points (rolls {args.rolls}°, wrist alternatives {args.wrist_alts}, shots {args.shots or 1}) on an even grid ({step:.0f} mm) inside the spherical sector of the clean hand-taught poses\n"
              f"# ({', '.join(sorted({p['run'] for p in clean}))}), generated by scripts/sim/plan_sector_tour.py. Joint targets: position-only IK\n# seeded from the nearest clean taught pose (wrist/tag orientation like that pose). Starts at `ready` (the\n# 5b(b) pose: a known table-safe first move off the rest stops, and a repeated sharp reference capture).\n"
              f"# Sector (centre = pan axis at shoulder-lift height, z {centre[2]:.0f} mm): r {lo[0]:.0f}-{hi[0]:.0f} mm, "
              f"azimuth {math.degrees(lo[1]):.1f} to {math.degrees(hi[1]):.1f}°, elevation {math.degrees(lo[2]):.1f} to {math.degrees(hi[2]):.1f}°.\n")  # fmt: skip
    tour_path.write_text(header + yaml.safe_dump(tour, sort_keys=False, default_flow_style=None))
    rows = []
    for s in plan.steps:
        q = {j: s.segment.target[j] for j in ALL_JOINTS}
        deg = {j: round(math.degrees(mapping.joints[j].to_rad(q[j])), 1) for j in CHAIN_JOINTS}
        v = view(q)
        dp = v["distance_mm"] if v else coef[0] + coef[1:] @ np.array(s.tool_mm)
        rows.append({"name": s.waypoint.name, "ticks": q, "deg": deg, "tool_mm": s.tool_mm, "pred_cam_mm": float(dp),
                     "pred_tilt_deg": None if v is None else v["tilt_deg"],
                     "pred_centre_px": None if v is None else v["corners_px"].mean(0).round(0).tolist(),
                     "min_table_mm": s.min_table_mm, "warnings": s.warnings})  # fmt: skip
    (OUT / "plan.json").write_text(json.dumps({"sector": {"centre_mm": centre.tolist(), "r_mm": [lo[0], hi[0]],
        "azimuth_deg": np.degrees([lo[1], hi[1]]).tolist(), "elevation_deg": np.degrees([lo[2], hi[2]]).tolist()},
        "grid_mm": step, "distance_fit": {"coef": coef.tolist(), "sd_mm": fit_sd, "taught_range_mm": [d_lo, d_hi]},
        "points": rows, "rejected": [{"tool_mm": p.round(1).tolist(), "reason": w} for p, w in rejected]}, indent=1))  # fmt: skip
    overlay = json.loads(args.overlay.read_text())["points"] if args.overlay else None
    plot(allp, clean, rows, rejected, centre, lo, hi, step, overlay)

    print(f"sector: r {lo[0]:.0f}-{hi[0]:.0f} mm, azimuth {math.degrees(lo[1]):.1f} to {math.degrees(hi[1]):.1f}°, "
          f"elevation {math.degrees(lo[2]):.1f} to {math.degrees(hi[2]):.1f}° (centre z {centre[2]:.0f} mm; {len(clean)} clean poses)")
    print(f"shoulder_pan kept within {pan_lo}-{pan_hi} ticks (iPad side = low end)")
    print(f"image-position fit sd {uv_sd[0]:.0f} / {uv_sd[1]:.0f} px (u / v)")
    print(f"distance fit sd {fit_sd:.0f} mm; grid {step:.0f} mm; {len(pts)} planned, {len(rejected)} rejected; tour {plan.duration_s / 60:.1f} min")
    print(f"view prediction: {'hand-eye fit (' + str(round(he.rms_px, 1)) + ' px RMS), max tilt ' + str(args.max_tilt) + '°' if he else 'linear fits'}")
    print(f"{'pt':5} {'tool x,y,z mm':>18} {'cam mm':>6} {'tilt':>5} " + " ".join(f"{j[:11]:>11}" for j in CHAIN_JOINTS) + "   (deg / ticks)")
    for r in rows:
        print(f"{r['name']:5} {', '.join(f'{v:.0f}' for v in r['tool_mm']):>18} {r['pred_cam_mm']:6.0f} "
              f"{'' if r['pred_tilt_deg'] is None else format(r['pred_tilt_deg'], '5.0f')} "
              + " ".join(f"{r['deg'][j]:6.1f}/{r['ticks'][j]:4.0f}" for j in CHAIN_JOINTS))
    print(f"wrote {tour_path.relative_to(REPO_ROOT)}, {(OUT / 'sector_plan.html').relative_to(REPO_ROOT)} + .png + plan.json")


def sector_edges(centre, lo, hi, n=24) -> list[np.ndarray]:
    """Wireframe: the 12 edges of the (r, azimuth, elevation) box."""
    edges = []
    def pt(r, t, f):
        return centre + r * np.array([math.cos(f) * math.cos(t), math.cos(f) * math.sin(t), math.sin(f)])
    for r in (lo[0], hi[0]):
        for f in (lo[2], hi[2]):
            edges.append(np.array([pt(r, t, f) for t in np.linspace(lo[1], hi[1], n)]))
        for t in (lo[1], hi[1]):
            edges.append(np.array([pt(r, t, f) for f in np.linspace(lo[2], hi[2], n)]))
    for t in (lo[1], hi[1]):
        for f in (lo[2], hi[2]):
            edges.append(np.array([pt(r, t, f) for r in np.linspace(lo[0], hi[0], 2)]))
    return edges


def plot(allp, clean, rows, rejected, centre, lo, hi, step, overlay=None) -> None:
    cls_col = {"sharp": "#1f9e6e", "usable": "#eb8c34", "not usable": "#d43f3a"}

    def cls(p):
        t = p["tag"]
        if not t["found"] or p["moved"] or t["uncalibrated"] or t["edge_rms"] > USABLE:
            return "not usable"
        return "sharp" if t["edge_rms"] <= SHARP else "usable"

    traces = []
    for name, col in cls_col.items():
        s = [p for p in allp if cls(p) == name]
        traces.append({"type": "scatter3d", "mode": "markers", "name": f"taught: {name} ({len(s)})",
                       "x": [p["tool_mm"][0] for p in s], "y": [p["tool_mm"][1] for p in s], "z": [p["tool_mm"][2] for p in s],
                       "hovertext": [p["label"] for p in s], "hoverinfo": "text",
                       "marker": {"size": 6, "color": col, "line": {"color": "#ffffff", "width": 1}}})  # fmt: skip
    traces.append({"type": "scatter3d", "mode": "markers+text", "name": f"planned ({len(rows)})",
                   "x": [r["tool_mm"][0] for r in rows[:-1]], "y": [r["tool_mm"][1] for r in rows[:-1]], "z": [r["tool_mm"][2] for r in rows[:-1]],
                   "text": [r["name"] for r in rows[:-1]], "textfont": {"size": 9, "color": "#2a78d6"},
                   "hovertext": [f"{r['name']}: tool {', '.join(f'{v:.0f}' for v in r['tool_mm'])} mm, camera ≈ {r['pred_cam_mm']:.0f} mm"
                                 + ("" if r["pred_tilt_deg"] is None else f", tag tilt ≈ {r['pred_tilt_deg']:.0f}°") + "<br>"
                                 + " ".join(f"{j[:5]} {v:.0f}°" for j, v in r["deg"].items()) for r in rows[:-1]],
                   "hoverinfo": "text", "marker": {"size": 5, "color": "#2a78d6", "symbol": "diamond"}})  # fmt: skip
    traces.append({"type": "scatter3d", "mode": "lines", "name": "tour order", "x": [r["tool_mm"][0] for r in rows[:-1]],
                   "y": [r["tool_mm"][1] for r in rows[:-1]], "z": [r["tool_mm"][2] for r in rows[:-1]],
                   "line": {"color": "#2a78d6", "width": 2, "dash": "dot"}, "hoverinfo": "skip", "visible": "legendonly"})  # fmt: skip
    if overlay:
        ov = [r for r in overlay if r["name"] not in ("ready", "park")]
        traces.append({"type": "scatter3d", "mode": "markers", "name": f"step 6 tour ({len(ov)})",
                       "x": [r["tool_mm"][0] for r in ov], "y": [r["tool_mm"][1] for r in ov], "z": [r["tool_mm"][2] for r in ov],
                       "hovertext": [f"step 6 {r['name']}" for r in ov], "hoverinfo": "text",
                       "marker": {"size": 5, "color": "#8a5cc2", "symbol": "square", "opacity": 0.7}})  # fmt: skip
    rj = [(p, w) for p, w in rejected]
    traces.append({"type": "scatter3d", "mode": "markers", "name": f"rejected candidates ({len(rj)})",
                   "x": [p[0] for p, _ in rj], "y": [p[1] for p, _ in rj], "z": [p[2] for p, _ in rj],
                   "hovertext": [w for _, w in rj], "hoverinfo": "text", "visible": "legendonly",
                   "marker": {"size": 3, "color": "#9a9890", "symbol": "x"}})  # fmt: skip
    for k, e in enumerate(sector_edges(centre, lo, hi)):
        traces.append({"type": "scatter3d", "mode": "lines", "name": "sector", "legendgroup": "sector", "showlegend": k == 0,
                       "x": e[:, 0].tolist(), "y": e[:, 1].tolist(), "z": e[:, 2].tolist(), "hoverinfo": "skip",
                       "line": {"color": "#6b6a63", "width": 2}})  # fmt: skip
    traces.append({"type": "scatter3d", "mode": "markers+text", "name": "base", "x": [0], "y": [0], "z": [0], "text": ["base"],
                   "marker": {"size": 4, "color": "#1f1f1d", "symbol": "diamond"}, "hoverinfo": "name"})  # fmt: skip
    title = (f"{OUT.name} plan: {len(rows) - 1} grid points ({step:.0f} mm) in the sector of the {len(clean)} clean taught poses "
             f"(r {lo[0]:.0f}-{hi[0]:.0f} mm, az {math.degrees(lo[1]):.0f}…{math.degrees(hi[1]):.0f}°, el {math.degrees(lo[2]):.0f}…{math.degrees(hi[2]):.0f}°)")
    layout = {"title": {"text": title, "font": {"size": 13}}, "margin": {"l": 0, "r": 0, "t": 40, "b": 0}, "legend": {"x": 0.01, "y": 0.95},
              "scene": {"aspectmode": "data", "xaxis": {"title": "x forward (mm)"}, "yaxis": {"title": "y left (mm)"}, "zaxis": {"title": "z up (mm)"}}}  # fmt: skip
    OUT.mkdir(parents=True, exist_ok=True)
    (OUT / "sector_plan.html").write_text(f"""<!doctype html><html><head><meta charset="utf-8"><title>Sector plan</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<script src="https://cdn.jsdelivr.net/npm/plotly.js-dist-min@2.35.2/plotly.min.js"></script>
<style>html,body{{margin:0;height:100%;background:#fff;font-family:system-ui,sans-serif}}#p{{width:100%;height:100vh}}</style>
</head><body><div id="p"></div><script>
Plotly.newPlot("p", {json.dumps(traces)}, {json.dumps(layout)}, {{responsive: true}});
</script></body></html>""")

    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(15, 5.4))
    views = [("3D", None), ("top (x–y)", (0, 1)), ("side (x–z)", (0, 2))]
    edges = sector_edges(centre, lo, hi)
    for k, (name, ax_ix) in enumerate(views):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d" if ax_ix is None else None)
        for e in edges:
            if ax_ix is None:
                ax.plot(e[:, 0], e[:, 1], e[:, 2], color="#b9b7ad", lw=0.8)
            else:
                ax.plot(e[:, ax_ix[0]], e[:, ax_ix[1]], color="#b9b7ad", lw=0.8)
        for cname, col in cls_col.items():
            s = np.array([p["tool_mm"] for p in allp if cls(p) == cname]).reshape(-1, 3)
            if len(s):
                args = (s[:, 0], s[:, 1], s[:, 2]) if ax_ix is None else (s[:, ax_ix[0]], s[:, ax_ix[1]])
                ax.scatter(*args, c=col, s=45, edgecolors="white", label=f"taught: {cname}", **({"depthshade": False} if ax_ix is None else {}))
        if overlay:
            o = np.array([r["tool_mm"] for r in overlay if r["name"] not in ("ready", "park")])
            args = (o[:, 0], o[:, 1], o[:, 2]) if ax_ix is None else (o[:, ax_ix[0]], o[:, ax_ix[1]])
            ax.scatter(*args, c="#8a5cc2", marker="s", s=22, alpha=0.6, label="step 6 tour", **({"depthshade": False} if ax_ix is None else {}))
        q = np.array([r["tool_mm"] for r in rows[:-1]])
        args = (q[:, 0], q[:, 1], q[:, 2]) if ax_ix is None else (q[:, ax_ix[0]], q[:, ax_ix[1]])
        ax.scatter(*args, c="#2a78d6", marker="D", s=22, label="planned", **({"depthshade": False} if ax_ix is None else {}))
        lab = ["x forward (mm)", "y left (mm)", "z up (mm)"]
        if ax_ix is None:
            ax.set_xlabel(lab[0]), ax.set_ylabel(lab[1]), ax.set_zlabel(lab[2])
        else:
            ax.scatter([0], [0], c="#1f1f1d", marker="D", s=25)
            for r in rows[:-1]:
                ax.annotate(r["name"], (r["tool_mm"][ax_ix[0]], r["tool_mm"][ax_ix[1]]), xytext=(3, 3), textcoords="offset points", fontsize=7, color="#2a78d6")
            ax.set_xlabel(lab[ax_ix[0]]), ax.set_ylabel(lab[ax_ix[1]])
            ax.set_aspect("equal", adjustable="datalim")
            ax.grid(color="#e4e3dd", lw=0.6)
        ax.set_title(name, loc="left", fontsize=10)
    fig.axes[0].legend(loc="upper left", fontsize=7)
    fig.suptitle(title, x=0.01, ha="left", fontsize=10)
    fig.tight_layout()
    fig.savefig(OUT / "sector_plan.png", dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    main()
