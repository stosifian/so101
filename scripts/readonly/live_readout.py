"""Torque-off live readout: poll the arm(s), log every sample to parquet. Read-only.

    uv run scripts/readonly/live_readout.py                         # both arms, 100 Hz, until Ctrl-C
    uv run scripts/readonly/live_readout.py --arm leader --seconds 30 --notes "by-hand sweep"

Writes data/raw/<run_id>/ (header.json, telemetry/part-*.parquet, telemetry_summary.json) and,
on exit, reads the parquet back and prints a per-joint summary.
Coming next: --tui (3d), --viz (3e), limit recording (3f).
"""

import argparse
import time

import pyarrow.compute as pc
import pyarrow.parquet as pq
from rich.console import Console
from rich.table import Table

from so101.config import load_arm_config
from so101.hw.arm import Arm
from so101.hw.bus import FeetechBus
from so101.runlog.run import Run
from so101.telemetry.hub import Hub
from so101.telemetry.parquet_log import ParquetLogger
from so101.telemetry.poller import Poller


def print_readback(console: Console, run: Run) -> None:
    t = pq.read_table(run.dir / "telemetry")
    table = Table(title=f"{run.run_id} — read back from disk", title_justify="left")
    for col in ["arm", "joint", "rows", "errors", "pos min", "pos max", "range", "°C max", "V min"]:
        table.add_column(col, justify="left" if col in ("arm", "joint") else "right")
    for arm in sorted(set(t["arm"].to_pylist())):
        a = t.filter(pc.equal(t["arm"], arm))
        for joint in dict.fromkeys(a["joint"].to_pylist()):  # keeps kinematic order
            j = a.filter(pc.equal(a["joint"], joint))
            pos = pc.min_max(j["position"])
            lo, hi = pos["min"].as_py(), pos["max"].as_py()
            v_min = pc.min(j["voltage_dv"]).as_py()
            table.add_row(
                arm, joint, str(j.num_rows), str(pc.sum(pc.is_valid(j["error"])).as_py()),
                str(lo), str(hi), str(hi - lo if lo is not None else "—"),
                str(pc.max(j["temperature_c"]).as_py()), f"{v_min / 10:.1f}" if v_min is not None else "—",
            )  # fmt: skip
    console.print(table)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=["follower", "leader", "both"], default="both")
    ap.add_argument("--rate", type=float, default=100.0, help="Hz per arm")
    ap.add_argument("--seconds", type=float, default=None, help="stop after this long (default: Ctrl-C)")
    ap.add_argument("--notes", default="", help="free text stored in the run header")
    args = ap.parse_args()

    console = Console()
    names = ["follower", "leader"] if args.arm == "both" else [args.arm]
    cfgs = [load_arm_config(n) for n in names]
    run = Run("telemetry", args.arm, config_paths=[c.path for c in cfgs], notes=args.notes)
    hub = Hub()
    logger = ParquetLogger(hub.subscribe("parquet", maxlen=100_000), run)
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
        while t_end is None or time.monotonic() < t_end:
            time.sleep(0.2)
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
        s = p.stats
        console.print(
            f"{p.name}: {s.cycles} cycles, overruns {s.overruns}, skipped {s.skipped_slots}, "
            f"failed snaps {s.failed_snapshots}, read max {s.max_read_ms:.2f} ms"
        )
    console.print(f"logger: {summary}")
    if summary["rows"]:
        print_readback(console, run)


if __name__ == "__main__":
    main()
