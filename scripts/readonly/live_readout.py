"""Torque-off live readout: poll the arm(s), log every sample to parquet. Read-only.

    uv run scripts/readonly/live_readout.py                         # both arms, 100 Hz, until Ctrl-C
    uv run scripts/readonly/live_readout.py --arm leader --seconds 30 --notes "by-hand sweep"
    uv run scripts/readonly/live_readout.py --tui                   # live table in the terminal
    uv run scripts/readonly/live_readout.py --tui --viz             # + live Rerun plots
    uv run scripts/readonly/live_readout.py --tui --arm follower --record-limits   # by-hand range sweep

Writes data/raw/<run_id>/ (header.json, telemetry/part-*.parquet, telemetry_summary.json, and
telemetry.rrd with --viz; replay it with `uv run rerun <run_dir>/telemetry.rrd`). On exit, reads
the parquet back and prints a per-joint summary. With --record-limits it also proposes software
limits from the sweep (data/derived/<run_id>/limits_proposed.json + a yaml snippet). It never
edits config; limits are applied to config/arm_<name>.yaml by hand after review.
"""

import argparse
import time
from contextlib import nullcontext as _null
from statistics import median

import pyarrow.parquet as pq
from rich.console import Console
from rich.live import Live
from rich.table import Table

from so101.analysis.limits import propose_from_run, save_proposal, yaml_snippet
from so101.config import load_arm_config
from so101.hw.arm import Arm
from so101.hw.bus import FeetechBus
from so101.runlog.run import Run
from so101.telemetry.hub import Hub
from so101.telemetry.parquet_log import ParquetLogger
from so101.telemetry.poller import Poller
from so101.telemetry.tui import Tui
from so101.telemetry.viz import RerunView


def print_readback(console: Console, run: Run) -> None:
    """Per-joint summary from disk. V/°C are medians; 'glitches' counts samples > 1 unit off the
    median (servo ADC noise on IDs 4-6, notes 2026-10-26), so one bad sample can't look like heating."""
    rows = pq.read_table(run.dir / "telemetry").to_pylist()
    table = Table(title=f"{run.run_id} — read back from disk", title_justify="left")
    for col in ["arm", "joint", "rows", "errors", "pos min", "pos max", "range", "V med", "°C med", "V/°C glitches"]:
        table.add_column(col, justify="left" if col in ("arm", "joint") else "right")
    groups: dict[tuple[str, str], list[dict]] = {}
    for r in rows:
        groups.setdefault((r["arm"], r["joint"]), []).append(r)  # insertion order = kinematic order
    for (arm, joint), jr in sorted(groups.items(), key=lambda kv: kv[0][0]):
        ok = [r for r in jr if r["error"] is None]
        if not ok:
            table.add_row(arm, joint, str(len(jr)), str(len(jr)), "—", "—", "—", "—", "—", "—")
            continue
        pos = [r["position"] for r in ok]
        v_med = median(r["voltage_dv"] for r in ok)
        t_med = median(r["temperature_c"] for r in ok)
        glitches = sum(1 for r in ok if abs(r["voltage_dv"] - v_med) > 1 or abs(r["temperature_c"] - t_med) > 1)
        table.add_row(
            arm, joint, str(len(jr)), str(len(jr) - len(ok)), str(min(pos)), str(max(pos)),
            str(max(pos) - min(pos)), f"{v_med / 10:.1f}", f"{t_med:g}", str(glitches),
        )  # fmt: skip
    console.print(table)


def print_limits(console: Console, run: Run, margin: int) -> None:
    limits = propose_from_run(run.dir, margin)
    path = save_proposal(run.run_id, margin, limits)
    for arm, joints in limits.items():
        table = Table(title=f"{arm} — proposed software limits (margin {margin} ticks)", title_justify="left")
        for col in ["joint", "status", "swept min", "swept max", "range", "limit min", "limit max", "spikes"]:
            table.add_column(col, justify="left" if col in ("joint", "status") else "right")
        for joint, l in joints.items():
            style = "green" if l.status == "ok" else "yellow"
            table.add_row(
                joint, f"[{style}]{l.status}", str(l.raw_min), str(l.raw_max),
                str(l.raw_max - l.raw_min) if l.raw_min is not None else "—",
                str(l.proposed[0]) if l.proposed else "—", str(l.proposed[1]) if l.proposed else "—",
                str(l.spikes_dropped),
            )  # fmt: skip
        console.print(table)
        console.print(yaml_snippet(run.run_id, margin, joints), markup=False, highlight=False)
    console.print(f"proposal saved: {path}  (config not changed)")


def poller_line(p: Poller) -> str:
    s = p.stats
    return (
        f"{p.name}: {s.cycles} cycles, overruns {s.overruns}, skipped {s.skipped_slots}, "
        f"failed snaps {s.failed_snapshots}, read max {s.max_read_ms:.2f} ms"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=["follower", "leader", "both"], default="both")
    ap.add_argument("--rate", type=float, default=100.0, help="Hz per arm")
    ap.add_argument("--seconds", type=float, default=None, help="stop after this long (default: Ctrl-C)")
    ap.add_argument("--notes", default="", help="free text stored in the run header")
    ap.add_argument("--tui", action="store_true", help="live joint table in the terminal")
    ap.add_argument("--viz", action="store_true", help="live Rerun plots (also saved as telemetry.rrd)")
    ap.add_argument("--record-limits", action="store_true", help="propose software limits from this sweep")
    ap.add_argument("--margin", type=int, default=50, help="ticks inward from the swept extremes (default 50)")
    args = ap.parse_args()

    console = Console()
    names = ["follower", "leader"] if args.arm == "both" else [args.arm]
    cfgs = [load_arm_config(n) for n in names]
    run = Run("telemetry", args.arm, config_paths=[c.path for c in cfgs], notes=args.notes)
    hub = Hub()
    logger = ParquetLogger(hub.subscribe("parquet", maxlen=100_000), run)
    tui = None
    if args.tui:
        tui = Tui(hub.subscribe("tui", maxlen=500), {c.name: c.expected_voltage_v for c in cfgs},
                  title=f"{run.run_id} — Ctrl-C to stop")  # fmt: skip
    viz = None
    if args.viz:
        viz = RerunView(hub.subscribe("viz", maxlen=5000), names, run.path("telemetry.rrd"), run.run_id,
                        run.header["started_monotonic_ns"])  # fmt: skip
    buses, pollers = [], []
    try:
        for cfg in cfgs:
            bus = FeetechBus(cfg.port, cfg.baudrate, cfg.protocol_end)
            buses.append(bus)
            pollers.append(Poller(Arm(bus, cfg.joints), hub, cfg.name, args.rate))
        logger.start()
        for p in pollers:
            p.start()
        console.print(f"[bold]{run.run_id}[/]: logging {', '.join(names)} at {args.rate:g} Hz — Ctrl-C to stop")
        t_end = None if args.seconds is None else time.monotonic() + args.seconds
        with Live(tui.render(), console=console, refresh_per_second=10) if tui else _null() as live:
            while t_end is None or time.monotonic() < t_end:
                time.sleep(0.1)
                if viz:
                    viz.update()
                if tui:
                    tui.footer = [poller_line(p) for p in pollers] + [f"logger: {logger.samples} samples"]
                    tui.update()
                    live.update(tui.render())
                dead = [p for p in pollers if not p.alive]
                if dead:
                    console.print(f"[red]poller died: {', '.join(f'{p.name}: {p.error!r}' for p in dead)}")
                    break
    except KeyboardInterrupt:
        pass
    finally:
        for p in pollers:
            p.stop()
        summary = logger.stop()
        for b in buses:
            b.close()
        if viz:
            viz.close()

    for p in pollers:
        console.print(poller_line(p))
    console.print(f"logger: {summary}")
    if viz:
        console.print(f"rerun: {viz.samples} samples, viz dropped {viz.sub.dropped} → {run.dir / 'telemetry.rrd'}")
        if viz.close_error:
            console.print(f"[yellow]rerun viewer was gone at shutdown ({viz.close_error}); parquet is unaffected")
    if summary["rows"]:
        print_readback(console, run)
    if args.record_limits and summary["rows"]:
        print_limits(console, run, args.margin)


if __name__ == "__main__":
    main()
