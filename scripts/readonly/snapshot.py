"""Read the telemetry block of every joint, N times back to back, and report values + read timing. Read-only.

    uv run scripts/readonly/snapshot.py                     # both arms, 100 cycles each
    uv run scripts/readonly/snapshot.py --arm follower -n 500

Nothing is saved: this is a quick health/timing check. Logged telemetry comes with the poller (step 3b-3c).
"""

import argparse

import numpy as np
from rich.console import Console
from rich.table import Table

from so101.config import load_arm_config
from so101.hw.arm import Arm, ArmSnapshot
from so101.hw.bus import FeetechBus


def show(console: Console, arm: str, snap: ArmSnapshot, positions: dict[str, list[int]], v_range) -> None:
    table = Table(title=f"{arm} — last snapshot", title_justify="left")
    for col in ["joint", "pos", "pos span", "speed", "load %", "V", "°C", "status", "moving", "mA"]:
        table.add_column(col, justify="left" if col == "joint" else "right")
    for joint, s in snap.joints.items():
        if s is None:
            table.add_row(joint, f"[red]{snap.errors[joint]}")
            continue
        ok_v = v_range[0] <= s.voltage_v <= v_range[1]
        span = max(positions[joint]) - min(positions[joint]) if positions[joint] else 0
        table.add_row(
            joint, str(s.position), str(span), str(s.speed), f"{s.load / 10:.1f}",
            f"[{'green' if ok_v else 'red'}]{s.voltage_v:.1f}", str(s.temperature_c),
            str(s.status) if s.status == 0 else f"[red]{s.status}", str(s.moving), f"{s.current_ma:.0f}",
        )  # fmt: skip
    console.print(table)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=["follower", "leader", "both"], default="both")
    ap.add_argument("-n", type=int, default=100, help="snapshots per arm (back to back, no sleep)")
    args = ap.parse_args()

    console = Console()
    for name in ["follower", "leader"] if args.arm == "both" else [args.arm]:
        cfg = load_arm_config(name)
        bus = FeetechBus(cfg.port, cfg.baudrate, cfg.protocol_end)
        try:
            arm = Arm(bus, cfg.joints)
            durations, failed = [], 0
            positions: dict[str, list[int]] = {j: [] for j in cfg.joints}
            for _ in range(args.n):
                snap = arm.snapshot()
                durations.append(snap.duration_ms)
                failed += not snap.ok
                for j, s in snap.joints.items():
                    if s is not None:
                        positions[j].append(s.position)
            show(console, name, snap, positions, cfg.expected_voltage_v)
            d = np.array(durations)
            console.print(
                f"{args.n} snapshots: cycle ms mean {d.mean():.2f} / p50 {np.median(d):.2f} / "
                f"p99 {np.percentile(d, 99):.2f} / max {d.max():.2f}  →  ~{1000 / d.mean():.0f} Hz max poll rate"
            )
            flags = {f"{k[0]}:{k[1]}": v for k, v in bus.stats.servo_errors.items()}
            console.print(
                f"bus: {bus.stats.transactions} transactions, snapshots with a failed servo: {failed}, "
                f"failures={dict(bus.stats.failures) or 0}, servo error flags={flags or 0}\n"
            )
        finally:
            bus.close()


if __name__ == "__main__":
    main()
