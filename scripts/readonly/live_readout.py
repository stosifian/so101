"""Torque-off live readout: poll the arm(s), log every sample to parquet. Read-only.

    uv run scripts/readonly/live_readout.py                         # both arms, 100 Hz, until Ctrl-C
    uv run scripts/readonly/live_readout.py --arm leader --seconds 30 --notes "by-hand sweep"
    uv run scripts/readonly/live_readout.py --tui                   # live table in the terminal

Writes data/raw/<run_id>/ (header.json, telemetry/part-*.parquet, telemetry_summary.json) and,
on exit, reads the parquet back and prints a per-joint summary.
Coming next: --viz (3e), limit recording (3f).
"""

import argparse
import time
from contextlib import nullcontext as _null
from statistics import median

import pyarrow.parquet as pq
from rich.console import Console
from rich.live import Live
from rich.table import Table

from so101.config import load_arm_config
from so101.hw.arm import Arm
from so101.hw.bus import FeetechBus
from so101.runlog.run import Run
from so101.telemetry.hub import Hub
from so101.telemetry.parquet_log import ParquetLogger
from so101.telemetry.poller import Poller
from so101.telemetry.tui import Tui


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

    for p in pollers:
        console.print(poller_line(p))
    console.print(f"logger: {summary}")
    if summary["rows"]:
        print_readback(console, run)


if __name__ == "__main__":
    main()
