"""List USB serial ports, or identify which port belongs to an arm by unplugging it.

Read-only: list/identify never open a port; --probe only sends ping and read packets (no writes).

    uv run scripts/readonly/find_port.py            # list ports with USB details
    uv run scripts/readonly/find_port.py --identify # unplug-diff, like lerobot-find-port
    uv run scripts/readonly/find_port.py --probe    # ping IDs 1-6, read voltage -> follower vs leader
"""

import argparse

from serial.tools import list_ports
from serial.tools.list_ports_common import ListPortInfo


def usb_ports() -> dict[str, ListPortInfo]:
    # macOS exposes each device twice (tty.* and cu.*); cu.* is the one to open from a host program.
    return {p.device: p for p in list_ports.comports() if p.device.startswith("/dev/cu.") and p.vid is not None}


def describe(p: ListPortInfo) -> str:
    return (
        f"{p.device}  vid:pid={p.vid:04x}:{p.pid:04x}  serial={p.serial_number}  "
        f"product={p.product!r}  manufacturer={p.manufacturer!r}"
    )


def probe(port: str, ids=range(1, 7)) -> None:
    """Ping servo IDs and read supply voltage. Reads only — tells follower (~12 V) from leader (~5 V)."""
    from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

    PRESENT_VOLTAGE = 62  # STS3215 control table, 1 byte, 0.1 V units (moves to so101.hw.registers in step 2)
    ph = PortHandler(port)
    if not ph.openPort():
        print(f"{port}: could not open")
        return
    pkt = PacketHandler(0)  # protocol_end=0 for STS-series servos
    print(f"{port}:")
    try:
        for i in ids:
            model, comm, err = pkt.ping(ph, i)
            if comm != COMM_SUCCESS:
                print(f"  id {i}: no response ({pkt.getTxRxResult(comm)})")
                continue
            v, comm, err = pkt.read1ByteTxRx(ph, i, PRESENT_VOLTAGE)
            volts = f"{v / 10:.1f} V" if comm == COMM_SUCCESS else "voltage read failed"
            print(f"  id {i}: model {model}  {volts}  error_byte={err}")
    finally:
        ph.closePort()


def identify() -> None:
    before = usb_ports()
    print("Ports now:")
    for p in before.values():
        print("  " + describe(p))
    input("\nUnplug the USB cable of ONE arm's controller board, then press Enter...")
    removed = set(before) - set(usb_ports())
    if len(removed) != 1:
        print(f"Expected exactly one port to disappear, got {sorted(removed) or 'none'}. Try again.")
        return
    port = removed.pop()
    print(f"\nThat arm is on: {describe(before[port])}")
    print("Record it in config/arm_*.yaml and docs/hardware_log.md, then plug it back in.")


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--identify", action="store_true", help="find an arm's port by unplugging it")
    ap.add_argument("--probe", action="store_true", help="ping servo IDs 1-6 on every port and read voltage (read-only)")
    args = ap.parse_args()
    if args.identify:
        identify()
    elif args.probe:
        for port in usb_ports():
            probe(port)
    else:
        ports = usb_ports()
        if not ports:
            print("No USB serial devices found. Check cable/hub and `system_profiler SPUSBDataType`.")
        for p in ports.values():
            print(describe(p))


if __name__ == "__main__":
    main()
