"""Dump every register of every servo to data/raw/<run_id>/registers.json. Read-only.

    uv run scripts/readonly/dump_registers.py                 # follower and leader
    uv run scripts/readonly/dump_registers.py --arm follower --notes "before calibration"
"""

import argparse

from rich.console import Console
from rich.table import Table

from so101.config import load_arm_config
from so101.hw.bus import FeetechBus
from so101.hw.dump import dump_arm
from so101.runlog.run import Run

SUMMARY_COLUMNS = [
    "Model", "Firmware_Major", "Firmware_Minor", "Present_Voltage", "Present_Temperature", "Present_Position",
    "Min_Angle_Limit", "Max_Angle_Limit", "Offset", "Mode", "Torque_Enable", "Lock", "Status",
]  # fmt: skip


def summarize(console: Console, arm: str, dump: dict, v_range: tuple[float, float]) -> None:
    table = Table(title=f"{arm} — register summary", title_justify="left")
    table.add_column("joint")
    table.add_column("id", justify="right")
    for c in SUMMARY_COLUMNS:
        table.add_column(c.replace("Present_", "").replace("_Limit", "").replace("Firmware_", "fw_"), justify="right")
    for joint, s in dump["servos"].items():
        if not s["responded"]:
            table.add_row(joint, str(s["id"]), "[red]no response")
            continue
        regs = s["registers"]
        cells = []
        for c in SUMMARY_COLUMNS:
            v = regs.get(c, {}).get("value")
            text = "ERR" if v is None else str(v)
            if c == "Present_Voltage" and v is not None:
                ok = v_range[0] <= v / 10 <= v_range[1]
                text = f"[{'green' if ok else 'red'}]{v / 10:.1f} V"
            cells.append(text)
        table.add_row(joint, str(s["id"]), *cells)
    console.print(table)
    stats = dump["bus_stats"]
    console.print(
        f"bus: {stats['transactions']} transactions, failures={stats['failures'] or 0}, "
        f"servo error flags={stats['servo_errors'] or 0}\n"
    )


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--arm", choices=["follower", "leader", "both"], default="both")
    ap.add_argument("--notes", default="", help="free text stored in the run header (e.g. 'before calibration')")
    args = ap.parse_args()

    console = Console()
    for name in ["follower", "leader"] if args.arm == "both" else [args.arm]:
        cfg = load_arm_config(name)
        bus = FeetechBus(cfg.port, cfg.baudrate, cfg.protocol_end)
        try:
            dump = dump_arm(bus, cfg.joints)
        finally:
            bus.close()
        run = Run("regdump", name, config_paths=[cfg.path], notes=args.notes)
        run.write_json("registers.json", dump)
        summarize(console, name, dump, cfg.expected_voltage_v)
        console.print(f"saved → {run.dir}\n")


if __name__ == "__main__":
    main()
