"""Run the background poller(s) for a few seconds and report whether the rate holds. Read-only.

    uv run scripts/readonly/poll_check.py                          # both arms, 100 Hz, 10 s
    uv run scripts/readonly/poll_check.py --arm follower --rate 250 --seconds 5

Both arms poll concurrently (one thread each, separate ports), like the live readout will.
Nothing is saved; parquet logging arrives in step 3c.
"""

import argparse
import time

import numpy as np
from rich.console import Console
from rich.table import Table

from so101.config import load_arm_config
from so101.hw.arm import Arm
from so101.hw.bus import FeetechBus
from so101.telemetry.hub import Hub
from so101.telemetry.poller import Poller


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=["follower", "leader", "both"], default="both")
    ap.add_argument("--rate", type=float, default=100.0, help="Hz per arm")
    ap.add_argument("--seconds", type=float, default=10.0)
    args = ap.parse_args()

    names = ["follower", "leader"] if args.arm == "both" else [args.arm]
    hub = Hub()
    sub = hub.subscribe("poll_check", maxlen=1_000_000)  # keep everything; we only drain at the end
    buses, pollers = [], []
    console = Console()
    try:
        for name in names:
            cfg = load_arm_config(name)
            bus = FeetechBus(cfg.port, cfg.baudrate, cfg.protocol_end)
            buses.append(bus)
            pollers.append(Poller(Arm(bus, cfg.joints), hub, name, args.rate))
        console.print(f"polling {', '.join(names)} at {args.rate:g} Hz for {args.seconds:g} s …")
        for p in pollers:
            p.start()
        time.sleep(args.seconds)
    finally:
        for p in pollers:
            p.stop()
        for b in buses:
            b.close()

    samples = sub.drain()
    period_ms = 1000 / args.rate
    table = Table(title=f"poller check — target {args.rate:g} Hz ({period_ms:.2f} ms)", title_justify="left")
    for col in ["arm", "samples", "rate Hz", "period mean", "period std", "p99", "max", "read max",
                "overruns", "skipped", "failed snaps", "seq gaps", "thread error"]:  # fmt: skip
        table.add_column(col, justify="left" if col == "arm" else "right")
    for p in pollers:
        mine = [s for s in samples if s.arm == p.name]
        t = np.array([s.snapshot.t_start_ns for s in mine], dtype=np.int64)
        dt = np.diff(t) / 1e6 if len(t) > 1 else np.array([np.nan])
        span_s = (t[-1] - t[0]) / 1e9 if len(t) > 1 else np.nan
        seqs = [s.seq for s in mine]
        gaps = sum(b - a - 1 for a, b in zip(seqs, seqs[1:]))
        st = p.stats
        table.add_row(
            p.name, str(len(mine)), f"{(len(mine) - 1) / span_s:.1f}",
            f"{dt.mean():.2f}", f"{dt.std():.3f}", f"{np.percentile(dt, 99):.2f}", f"{dt.max():.2f}",
            f"{st.max_read_ms:.2f}", str(st.overruns), str(st.skipped_slots), str(st.failed_snapshots),
            str(gaps), "[red]" + repr(p.error) if p.error else "none",
        )  # fmt: skip
    console.print(table)
    console.print(f"subscriber dropped: {sub.dropped}")


if __name__ == "__main__":
    main()
