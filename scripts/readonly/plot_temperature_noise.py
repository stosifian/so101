"""Plot servo temperature noise from a logged run (offline, reads parquet only; never touches the bus).

    uv run scripts/readonly/plot_temperature_noise.py data/raw/20261027-092438_sweep_follower --joint shoulder_lift

Three panels on one host-monotonic time axis: the driven joint's position, its raw temperature samples with a
running median and the debounced trip level, and the raw temperatures of the joints at rest. Shows the two
temperature-ADC artefacts in docs/hardware_log.md ("Telemetry quirks"): single-sample spikes on the moving joint,
and smaller at-rest glitches on IDs 4–6.
Writes data/derived/<run>/temperature_noise.png (or --out).
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import pyarrow as pa
import pyarrow.parquet as pq

from so101.config import REPO_ROOT

TRIP_C = 55  # SafeArm over-temperature trip (debounced: continuously for 0.5 s)


def running_median(x: np.ndarray, n: int) -> np.ndarray:
    pad = np.pad(x, (n // 2, n - 1 - n // 2), mode="edge")
    return np.median(np.lib.stride_tricks.sliding_window_view(pad, n), axis=1)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--joint", required=True, help="the joint being driven")
    ap.add_argument("--median", type=int, default=25, help="running-median window (samples)")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()

    files = sorted((args.run / "telemetry").glob("*.parquet"))
    t = pa.concat_tables([pq.read_table(f) for f in files]).to_pydict()
    t0 = min(t["t_start_ns"])
    series: dict[str, dict[str, list]] = {}
    for j, ts, pos, temp in zip(t["joint"], t["t_start_ns"], t["position"], t["temperature_c"]):
        s = series.setdefault(j, {"t": [], "pos": [], "temp": []})
        s["t"].append((ts - t0) / 1e9)
        s["pos"].append(pos)
        s["temp"].append(temp)
    s = {k: np.asarray(v) for k, v in series[args.joint].items()}

    fig, (ax_p, ax_t, ax_r) = plt.subplots(3, 1, figsize=(9, 7.5), sharex=True,
                                            gridspec_kw={"height_ratios": [1, 1.6, 1.3]})
    ax_p.plot(s["t"], s["pos"], color="0.25", lw=1.5)
    ax_p.set_ylabel("position (ticks)")
    ax_p.set_title(f"{args.joint} (being driven)", loc="left", fontsize=10)

    med = running_median(s["temp"].astype(float), args.median)
    ax_t.scatter(s["t"], s["temp"], s=6, color="0.6", label="raw samples", zorder=2)
    ax_t.plot(s["t"], med, color="#2a6fdb", lw=2, label=f"running median ({args.median} samples)", zorder=3)
    ax_t.axhline(TRIP_C, color="#c0392b", ls="--", lw=1.2, label=f"trip level {TRIP_C} °C (debounced 0.5 s)")
    n_spk = int(np.sum(s["temp"] > np.median(s["temp"]) + 5))
    ax_t.set_title(f"{args.joint} temperature: {n_spk} single-sample spikes, max {s['temp'].max()} °C, "
                   f"median {np.median(s['temp']):.0f} °C", loc="left", fontsize=10)
    ax_t.set_ylabel("°C")
    ax_t.legend(loc="upper right", fontsize=8, frameon=False)

    colors = ["#2a6fdb", "#e08a1e", "#3a9d5d", "#8e44ad", "#c0392b", "#555555"]
    rest = [j for j in series if j != args.joint]
    for c, j in zip(colors, rest):
        r = series[j]
        ax_r.plot(r["t"], r["temp"], color=c, lw=1, label=j)
    ax_r.set_title("joints at rest: raw temperature", loc="left", fontsize=10)
    ax_r.set_ylabel("°C")
    ax_r.set_xlabel("time (s)")
    ax_r.legend(loc="upper right", fontsize=8, ncol=len(rest), frameon=False)

    for ax in (ax_p, ax_t, ax_r):
        ax.grid(alpha=0.25)
        ax.spines[["top", "right"]].set_visible(False)
    fig.suptitle(f"Servo temperature noise, run {args.run.name}", fontsize=11)
    fig.tight_layout()

    out = args.out or REPO_ROOT / "data/derived" / args.run.name / "temperature_noise.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == "__main__":
    main()
