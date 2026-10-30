"""Step 6 prep: pose the UNPOWERED arm by hand at the extremes of a planned experiment; at each pose record the
joint readings and a photo, and check the gripper tag there. Read-only on the servo bus (never writes a register).

    uv run scripts/readonly/teach_poses.py                         # gripper tag id 2
    uv run scripts/readonly/teach_poses.py --tag-id 2 --notes "step 6 extremes"
    uv run scripts/readonly/teach_poses.py --analyze data/raw/<run_id>   # re-analyze a run, no hardware

Per pose: type a label (e.g. ipad_side_far_low), hold the arm still (support it, hand clear of the tag),
press Enter. It reads every joint, takes one RAW photo, reads every joint again, then reports:
joint ticks (and whether the arm moved during the capture), tool point + table clearance from the model,
and the tag: found?, size in px, edge-line RMS (focus: ~0.10-0.12 px is sharp here), outside the calibrated
region (rows > 3700)?, distance and tilt to the line of sight, and the iPad corner count.
Empty label + Enter = done; Ctrl-C also stops cleanly. A missing tag or a failed capture is reported and the
session carries on; teach.json is rewritten after every pose, so nothing recorded is lost.
Re-analyze a run (e.g. one that stopped early): --analyze data/raw/<run_id>. At the end: per-joint min/max over the poses whose tag check passed = the proposed
joint range for the experiment, compared with the software limits in config/arm_follower.yaml.
Writes data/raw/<run_id>/ (kind teach): manifest.jsonl (snapshots + camera), images/; results in
data/derived/<run_id>/teach.json (rewritten after every pose and by --analyze).
Torque stays off: raised poses need your hand. Keep 12 V off or on, either is fine (nothing is enabled).
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from rich.console import Console
from rich.table import Table

from so101.analysis.apriltag import detect_tags
from so101.analysis.charuco import CharucoSpec, detect, load_linear, to_display
from so101.analysis.pose import CALIBRATED_MAX_Y, Camera, solve, tag_object_points
from so101.config import REPO_ROOT, load_arm_config
from so101.hw.arm import Arm, ArmSnapshot
from so101.hw.bus import FeetechBus
from so101.kinematics.collision import CollisionModel
from so101.kinematics.fk import CHAIN_JOINTS, DEFAULT_TCP, chain_from_mujoco, fk
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel
from so101.runlog.manifest import Manifest, read_manifest
from so101.runlog.run import Run
from so101.safety import SafetyConfig
from so101.sensors import Gphoto2Camera

MOVED_TICKS = 3  # a joint that changed more than this during the capture = the arm moved (blur, wrong reading)
SHARP_EDGE_PX = 0.30  # usable up to this edge-line RMS: blur test 10-30 (tag 3, edge 0.23 vs 0.10) doubled corner noise only (~0.055 vs 0.028 px)


def ticks(s: ArmSnapshot) -> dict[str, int]:
    return {j: st.position for j, st in s.joints.items() if st is not None}


def tag_check(path, tag_id: int, tag_mm: float, cam: Camera, spec: CharucoSpec) -> dict:
    lin = load_linear(path)
    gray = to_display(lin)
    out: dict = {"found": False}
    t = detect_tags(gray, ids=[tag_id], linear=lin).get(tag_id)
    if t is not None:
        p = solve(tag_object_points(tag_mm), t.corners, cam, 0.05, square=True)
        ray = p.t / np.linalg.norm(p.t)
        out.update(found=True, centre_px=t.corners.mean(0).round(1).tolist(),
                   side_px=float(np.mean([np.linalg.norm(t.corners[k] - t.corners[(k + 1) % 4]) for k in range(4)])),
                   edge_rms=t.edge_rms, uncalibrated=bool((t.corners[:, 1] > CALIBRATED_MAX_Y).any()),
                   distance_mm=float(np.linalg.norm(p.t)), tilt_deg=float(np.degrees(np.arccos(abs(p.R[:, 2] @ ray)))),
                   ambiguity=p.ambiguity)  # fmt: skip
    d = detect(gray, spec.board())
    out["ipad_n"] = 0 if d is None else len(d.ids)
    out["ok"] = bool(out["found"] and out["edge_rms"] <= SHARP_EDGE_PX and not out["uncalibrated"])
    return out


def analyze_pose(label: str, t0: dict, t1: dict, arw, ctx: dict, args) -> dict:
    """Everything reported for one pose; t0/t1 = joint ticks before/after the capture."""
    mapping, safety = ctx["mapping"], ctx["safety"]
    moved = {j: t1[j] - t0[j] for j in t0 if j in t1 and abs(t1[j] - t0[j]) > MOVED_TICKS}
    q = np.array([mapping.joints[j].to_rad(t1[j]) for j in CHAIN_JOINTS])
    tool_mm = (fk(ctx["chain"], q, tcp=DEFAULT_TCP)[:3, 3] * 1000).round(1).tolist()
    table_mm = float(ctx["collision"].check_ticks(t1).min_table * 1000)
    outside = {j: v for j, v in t1.items() if not safety.limits[j][0] - MOVED_TICKS <= v <= safety.limits[j][1] + MOVED_TICKS}
    tc = tag_check(arw, args.tag_id, args.tag_mm, ctx["cam"], ctx["spec"])
    return {"label": label, "image": arw.name, "ticks": t1, "moved": moved, "tool_mm": tool_mm, "table_mm": table_mm,
            "outside_limits": outside, "tag": tc}  # fmt: skip


def report_pose(console: Console, p: dict) -> None:
    tc = p["tag"]
    if tc["found"]:
        soft = " [yellow](soft)[/]" if tc["edge_rms"] > SHARP_EDGE_PX else ""
        unc = " [red]UNCALIBRATED REGION[/]" if tc["uncalibrated"] else ""
        tag = f"tag {tc['side_px']:.0f} px, edge {tc['edge_rms']:.3f}{soft}, {tc['distance_mm']:.0f} mm, tilt {tc['tilt_deg']:.0f}°{unc}"
    else:
        tag = "[red]tag NOT found[/]"
    console.print(f"[bold]{p['label']}[/]: {'[green]OK' if tc['ok'] else '[red]CHECK'}[/]  {tag}; iPad {tc['ipad_n']} corners; "
                  f"tool {p['tool_mm']} mm, table {p['table_mm']:+.0f} mm", highlight=False)  # fmt: skip
    console.print("  ticks " + "  ".join(f"{j} {v}" for j, v in p["ticks"].items()), highlight=False)
    if p["moved"]:
        console.print(f"  [yellow]arm moved during the capture {p['moved']}: redo for a clean reading")
    if p["outside_limits"]:
        console.print(f"  [yellow]outside the software limits: {p['outside_limits']} (the experiment range gets clamped to them)")


def summarize(console: Console, poses: list[dict], ctx: dict, run_dir, args, final: bool) -> None:
    """Write data/derived/<run_id>/teach.json (after every pose, so a crash or Ctrl-C loses nothing; never into the
    immutable raw directory); print the tables at the end."""
    cfg, safety = ctx["cfg"], ctx["safety"]
    good = [p for p in poses if p["tag"]["ok"] and not p["moved"]]
    rng = {}
    for j in cfg.joints if good else []:
        lo, hi = safety.limits[j]
        v = [min(max(p["ticks"][j], lo), hi) for p in good]  # clamp each reading first (a pose beyond a limit counts as at it)
        rng[j] = [min(v), max(v)]
    out = REPO_ROOT / "data" / "derived" / run_dir.name / "teach.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"tag_id": args.tag_id, "sharp_edge_px": SHARP_EDGE_PX, "poses": poses,
                                                    "proposed_range": rng}, indent=2))  # fmt: skip
    if not final:
        return
    t = Table(title=f"{run_dir.name}: {len(poses)} poses, {len(good)} usable (tag found, sharp, calibrated region, arm still)", title_justify="left")
    for c in ["label", "ok", *cfg.joints, "edge", "tag mm", "tool mm"]:
        t.add_column(c, justify="left" if c == "label" else "right")
    for p in poses:
        tc = p["tag"]
        t.add_row(p["label"], "yes" if p in good else "[red]no", *[str(p["ticks"].get(j, "")) for j in cfg.joints],
                  f"{tc['edge_rms']:.3f}" if tc["found"] else "-", f"{tc['distance_mm']:.0f}" if tc["found"] else "-",
                  ", ".join(f"{v:.0f}" for v in p["tool_mm"]))  # fmt: skip
    console.print(t)
    if rng:
        rt = Table(title="proposed joint range for the experiment (usable poses, clamped to the software limits)", title_justify="left")
        for c in ["joint", "min", "max", "software limits"]:
            rt.add_column(c, justify="right")
        for j, (lo, hi) in rng.items():
            rt.add_row(j, str(lo), str(hi), str(list(safety.limits[j])))
        console.print(rt)
    console.print(f"[bold]{run_dir.name}[/]: wrote {out.relative_to(REPO_ROOT)}")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--tag-id", type=int, default=2)
    ap.add_argument("--tag-mm", type=float, default=20.0)
    ap.add_argument("--analyze", type=Path, help="existing teach run dir: re-analyze its manifest + images (no arm, no camera)")
    ap.add_argument("--notes", default="")
    args = ap.parse_args()
    console = Console()

    cfg = load_arm_config("follower")
    safety = SafetyConfig.from_arm_config(cfg)
    mapping = load_mapping("follower")
    ctx = {"cfg": cfg, "safety": safety, "mapping": mapping, "chain": chain_from_mujoco(ArmModel(mapping.model_path).m),
           "collision": CollisionModel.from_safety(mapping, safety), "cam": Camera.load(), "spec": CharucoSpec.load()}  # fmt: skip

    if args.analyze:  # rebuild teach.json from a run that stopped early (raw files are only read)
        poses = []
        for m in read_manifest(args.analyze / "manifest.jsonl")[1]:
            arws = [Path(f) for f in m["camera"]["files"] if f.upper().endswith(".ARW")]
            if not (m["camera"]["ok"] and arws and m["snapshot_before"] and m["snapshot_after"]):
                continue
            tk = lambda s: {j: st["position"] for j, st in s["joints"].items() if st}  # noqa: E731
            try:
                p = analyze_pose(m["label"], tk(m["snapshot_before"]), tk(m["snapshot_after"]), arws[0], ctx, args)
            except Exception as e:  # noqa: BLE001
                console.print(f"[red]{m['label']}: analysis failed ({type(e).__name__}: {e}); skipped")
                continue
            poses.append(p)
            report_pose(console, p)
        if not poses:
            sys.exit("no analyzable poses")
        summarize(console, poses, ctx, args.analyze, args, final=True)
        return

    bus = FeetechBus(cfg.port, cfg.baudrate, cfg.protocol_end)
    arm = Arm(bus, cfg.joints)
    camera = Gphoto2Camera()
    run = Run("teach", "follower", config_paths=[cfg.path, mapping.path], notes=f"hand-taught poses, arm unpowered; {args.notes}")
    run.write_json("camera.json", camera.info())
    manifest = Manifest(run, "teach")
    poses = []
    try:
        while True:
            try:
                label = input("\nlabel for this pose (empty = done), hold the arm still, then Enter: ").strip()
            except EOFError:
                break
            if not label:
                break
            before = arm.snapshot()
            r = camera.capture(run.dir / "images", f"pose_{len(poses):02d}_{label}")
            after = arm.snapshot()
            manifest.add(label, commanded_ticks=None, before=before, after=after, camera={**r.as_dict(), "backend": camera.name})
            if not (before.ok and after.ok):
                console.print(f"[red]incomplete servo reading: {before.errors or after.errors}; redo this pose")
                continue
            arws = [f for f in r.files if f.suffix.upper() == ".ARW"]
            if not r.ok or not arws:
                console.print(f"[red]capture failed: {r.error or 'no RAW file'}; redo this pose")
                continue
            try:  # an analysis problem (no tag, odd image) must never end the session
                p = analyze_pose(label, ticks(before), ticks(after), arws[0], ctx, args)
            except Exception as e:  # noqa: BLE001
                console.print(f"[red]{label}: analysis failed ({type(e).__name__}: {e}); photo + joints are saved, carry on")
                continue
            poses.append(p)
            report_pose(console, p)
            summarize(console, poses, ctx, run.dir, args, final=False)
    except KeyboardInterrupt:
        console.print("\n[yellow]Ctrl-C: stopping; everything recorded so far is kept")
    finally:
        manifest.close()
        bus.close()
    if not poses:
        sys.exit(f"no poses recorded ({run.run_id})")
    summarize(console, poses, ctx, run.dir, args, final=True)


if __name__ == "__main__":
    main()
