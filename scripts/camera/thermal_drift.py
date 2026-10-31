"""Step 11 analysis: thermal drift across hold → cool-down → return (read-only).

    uv run scripts/camera/thermal_drift.py --cold data/raw/<1-shot run> --hold data/raw/<hold camera run> \\
        --cool data/raw/<cool-down camera run> --return data/raw/<return camera run> \\
        --telemetry data/raw/<hold tour> data/raw/<cool-down readout> data/raw/<return tour>

Every photo: tag relative to the iPad (as scripts/camera/target_pose.py), along the camera axes of the first hold
shot (x right, y down, z depth), µm. Two pose series:
  ready (hold + return): Δ vs the first hold shot      rest (cold + cool-down): Δ vs the cold reference shot
Telemetry (one clock: host monotonic): joint temperatures and encoder positions over all phases.
Reports, for the hold: drift vs time and vs elbow temperature (which fits better), the rate in the first vs the last
15 min (a plateau with flat temperature = heat; continuing = creep), and any encoder tick changes (slip). For the
cool-down: rest-pose change vs temperature drop. Return test: the return position vs the hold's start and end.
Writes data/derived/step11_<hold run>/thermal.json + thermal.png.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np
import pyarrow.parquet as pq
from rich.console import Console
from rich.table import Table

from so101.analysis.charuco import CharucoSpec
from so101.analysis.pose import Camera
from so101.config import REPO_ROOT
from so101.runlog.manifest import read_manifest

sys.path.insert(0, str(Path(__file__).parent))
from target_pose import measure  # noqa: E402

JOINTS = ["shoulder_pan", "shoulder_lift", "elbow_flex", "wrist_flex", "wrist_roll"]


def shots(run: Path, cam, spec) -> list[dict]:
    t_req = {}
    for m in read_manifest(run / "manifest.jsonl")[1]:
        for f in (m.get("camera") or {}).get("files", []):
            t_req[Path(f).name] = m["camera"]["t_request_ns"]
    out = []
    for f in sorted((run / "images").glob("*.ARW")):
        r = measure(f, cam, 2, 20.0, spec, 0.05)
        if "rel" in r:
            out.append({"t_ns": t_req[f.name], "p_ipad": r["rel"].t, "R_cam_ipad": r["ipad"].R, "image": f.name})
    return out


def telemetry(runs: list[Path]) -> dict:
    """joint -> (t_ns array, temperature, position)"""
    acc = {j: ([], [], []) for j in JOINTS}
    for run in runs:
        t = pq.read_table(run / "telemetry", columns=["t_start_ns", "joint", "temperature_c", "position"])
        J = np.array(t.column("joint").to_pylist())
        ts = np.array(t.column("t_start_ns").to_pylist(), dtype=float)
        T = np.array(t.column("temperature_c").to_pylist(), dtype=float)
        P = np.array(t.column("position").to_pylist(), dtype=float)
        for j in JOINTS:
            s = J == j
            acc[j][0].extend(ts[s]), acc[j][1].extend(T[s]), acc[j][2].extend(P[s])
    out = {}
    for j, (a, b, c) in acc.items():
        o = np.argsort(a)
        out[j] = (np.array(a)[o], np.array(b)[o], np.array(c)[o])
    return out


def at(tel, j, t_ns, col, win_s=10.0) -> float:
    ts, T, P = tel[j]
    s = np.abs(ts - t_ns) < win_s * 1e9
    return float(np.nanmedian((T if col == "temp" else P)[s])) if s.any() else float("nan")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--cold", type=Path, required=True)
    ap.add_argument("--hold", type=Path, required=True)
    ap.add_argument("--cool", type=Path, required=True)
    ap.add_argument("--return", dest="ret", type=Path, required=True)
    ap.add_argument("--telemetry", type=Path, nargs="+", required=True)
    args = ap.parse_args()
    console = Console()
    cam, spec = Camera.load(), CharucoSpec.load()
    S = {k: shots(r, cam, spec) for k, r in [("cold", args.cold), ("hold", args.hold), ("cool", args.cool), ("ret", args.ret)]}
    for k, v in S.items():
        console.print(f"{k}: {len(v)} shots measured")
    if not S["hold"]:
        sys.exit("no hold shots")
    R = S["hold"][0]["R_cam_ipad"]  # report along the camera axes of the first hold shot
    t0 = S["hold"][0]["t_ns"]
    tel = telemetry(args.telemetry)

    def series(key, ref):
        return [{"phase": key, "t_min": (s["t_ns"] - t0) / 60e9, "d_um": (R @ (s["p_ipad"] - ref)) * 1000, "t_ns": s["t_ns"]} for s in S[key]]

    ready = series("hold", S["hold"][0]["p_ipad"]) + series("ret", S["hold"][0]["p_ipad"])
    rest_ref = S["cold"][0]["p_ipad"] if S["cold"] else S["cool"][0]["p_ipad"]
    rest = series("cold", rest_ref) + series("cool", rest_ref)
    for row in ready + rest:
        row["temp"] = {j: at(tel, j, row["t_ns"], "temp") for j in JOINTS}

    out: dict = {"runs": {k: str(v) for k, v in vars(args).items() if isinstance(v, Path)}}
    # --- hold
    H = [r for r in ready if r["phase"] == "hold"]
    th = np.array([r["t_min"] for r in H])
    D = np.array([r["d_um"] for r in H])
    Te = np.array([r["temp"]["elbow_flex"] for r in H])
    t = Table(title="hold at `ready` (Δ vs the first hold shot, µm, camera axes)", title_justify="left")
    for c in ["axis", "total Δ", "rate first 15 min (µm/min)", "rate last 15 min", "fit vs time: resid", "fit vs elbow temp: resid"]:
        t.add_column(c, justify="right")
    first, last = th <= th[0] + 15, th >= th[-1] - 15
    for k, ax in enumerate(["x", "y", "z (depth)"]):
        y = D[:, k]
        r1 = np.polyfit(th[first], y[first], 1)[0] if first.sum() > 2 else np.nan
        r2 = np.polyfit(th[last], y[last], 1)[0] if last.sum() > 2 else np.nan
        ok = np.isfinite(Te)
        rt = np.std(y - np.polyval(np.polyfit(th, y, 1), th), ddof=2)
        rT = np.std(y[ok] - np.polyval(np.polyfit(Te[ok], y[ok], 1), Te[ok]), ddof=2) if ok.sum() > 3 and np.ptp(Te[ok]) > 0 else np.nan
        t.add_row(ax, f"{y[-1]:+.0f}", f"{r1:+.2f}", f"{r2:+.2f}", f"{rt:.1f}", f"{rT:.1f}")
        out.setdefault("hold", {})[ax] = {"total_um": float(y[-1]), "rate_first15": float(r1), "rate_last15": float(r2), "resid_time": float(rt), "resid_temp": float(rT)}
    console.print(t)
    temps = {j: (H[0]["temp"][j], H[-1]["temp"][j]) for j in JOINTS}
    console.print("temperatures over the hold (°C): " + ", ".join(f"{j} {a:.0f} → {b:.0f}" for j, (a, b) in temps.items()), highlight=False)
    tick = {}
    for j in JOINTS:
        ts, _, P = tel[j]
        s = (ts >= S["hold"][0]["t_ns"]) & (ts <= S["hold"][-1]["t_ns"])
        if s.any():
            tick[j] = int(np.nanmax(P[s]) - np.nanmin(P[s]))
    console.print(f"encoder range during the hold (ticks; > 0 = the reading changed: slip candidate): {tick}")
    out["hold"]["temps_c"] = temps
    out["hold"]["encoder_range_ticks"] = tick
    # --- cool-down
    C = [r for r in rest if r["phase"] == "cool"]
    if C:
        Dc = np.array([r["d_um"] for r in C])
        console.print(f"cool-down at rest: Δ vs cold reference, first → last shot (µm x/y/z): {np.round(Dc[0], 0).tolist()} → {np.round(Dc[-1], 0).tolist()}; "
                      f"elbow {C[0]['temp']['elbow_flex']:.0f} → {C[-1]['temp']['elbow_flex']:.0f} °C", highlight=False)  # fmt: skip
        out["cool"] = {"first_um": Dc[0].tolist(), "last_um": Dc[-1].tolist(), "elbow_c": [C[0]["temp"]["elbow_flex"], C[-1]["temp"]["elbow_flex"]]}
    # --- return test
    Rt = [r for r in ready if r["phase"] == "ret"]
    if Rt:
        start = np.mean([r["d_um"] for r in H[:3]], axis=0)
        end = np.mean([r["d_um"] for r in H[-3:]], axis=0)
        ret = np.mean([r["d_um"] for r in Rt], axis=0)
        rt = Table(title="return test at `ready` (µm, camera axes, Δ vs the first hold shot)", title_justify="left")
        for c in ["", "x", "y", "z"]:
            rt.add_column(c, justify="right")
        for name, v in [("hold start (3 shots)", start), ("hold end (3 shots)", end), ("return (mean)", ret)]:
            rt.add_row(name, *[f"{x:+.0f}" for x in v])
        console.print(rt)
        frac = float(np.dot(ret - start, end - start) / max(np.dot(end - start, end - start), 1e-9))
        console.print(f"return lands {frac * 100:.0f} % of the way from the hold start to the hold end (0 % = fully recovered: elastic/thermal; 100 % = none: creep/slip)")
        enc_ret = {j: at(tel, j, Rt[-1]["t_ns"], "pos") - at(tel, j, H[1]["t_ns"], "pos") for j in JOINTS}
        console.print(f"encoder at return − hold start (ticks): {({j: (round(v) if np.isfinite(v) else None) for j, v in enc_ret.items()})}")
        out["return"] = {"start_um": start.tolist(), "end_um": end.tolist(), "return_um": ret.tolist(), "fraction_of_hold_drift": frac, "encoder_delta_ticks": enc_ret}
    od = REPO_ROOT / "data" / "derived" / f"step11_{args.hold.name}"
    od.mkdir(parents=True, exist_ok=True)
    (od / "thermal.json").write_text(json.dumps(out, indent=1, default=float))
    plot(ready, rest, tel, t0, S, od / "thermal.png")
    console.print(f"wrote {(od / 'thermal.json').relative_to(REPO_ROOT)} + thermal.png")


def plot(ready, rest, tel, t0, S, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    cols = ["#2a78d6", "#eb6834", "#1f9e6e"]
    fig, axes = plt.subplots(4, 1, figsize=(12, 12), sharex=True)
    for a, series, title in [(axes[0], ready, "tag at `ready` (hold, then return): Δ vs first hold shot (µm)"),
                             (axes[1], rest, "tag at rest (cold reference, then cool-down): Δ vs cold shot (µm)")]:  # fmt: skip
        if series:
            t = np.array([r["t_min"] for r in series])
            D = np.array([r["d_um"] for r in series])
            for k, (n, c) in enumerate(zip(["x", "y", "z (depth)"], cols)):
                a.plot(t, D[:, k], "o-", ms=3, lw=1, color=c, label=n)
            a.legend(fontsize=8)
        a.set_title(title, loc="left", fontsize=10)
    for j, c in zip(["shoulder_lift", "elbow_flex", "wrist_flex"], cols):
        ts, T, P = tel[j]
        tm = (ts - t0) / 60e9
        axes[2].plot(tm, T, ".", ms=1, color=c, label=j)
        axes[3].plot(tm, P - np.nanmedian(P[(ts >= S["hold"][0]["t_ns"]) & (ts <= S["hold"][-1]["t_ns"])]) if len(P) else P, ".", ms=1, color=c, label=j)
    axes[2].set_title("servo temperature (°C)", loc="left", fontsize=10), axes[2].legend(fontsize=8, markerscale=8)
    axes[3].set_title("encoder position − its median during the hold (ticks; only meaningful while torque is on)", loc="left", fontsize=10)
    axes[3].set_ylim(-20, 20), axes[3].legend(fontsize=8, markerscale=8)
    for a in axes:
        a.grid(color="#e4e3dd", lw=0.6)
        a.axhline(0, color="#6b6a63", lw=0.5)
        for key, shade in (("cool", "#eeeeea"),):
            if S[key]:
                a.axvspan((S[key][0]["t_ns"] - t0) / 60e9, (S[key][-1]["t_ns"] - t0) / 60e9, color=shade, zorder=0)
    axes[-1].set_xlabel("time since the first hold shot (min); grey = unpowered cool-down")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
