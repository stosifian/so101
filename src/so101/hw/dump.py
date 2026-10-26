"""Read-only dump of every known register on every servo of an arm."""

import time

from so101.hw.bus import BusError, ServoBus
from so101.hw.registers import REGISTERS, decode


def dump_servo(bus: ServoBus, servo_id: int) -> dict:
    regs, errors = {}, {}
    for name, reg in REGISTERS.items():
        try:
            raw = bus.read_raw(servo_id, reg)
        except BusError as e:
            errors[name] = str(e)
            continue
        # Keep raw alongside decoded so a decode bug can never lose information.
        regs[name] = {"raw": raw, "value": decode(reg, raw), "addr": reg.addr, "area": reg.area, "unit": reg.unit}
    return {"registers": regs, "errors": errors}


def dump_arm(bus: ServoBus, joints: dict[str, int]) -> dict:
    """joints: name -> servo id. Returns a JSON-serialisable dict."""
    servos = {}
    for joint, servo_id in joints.items():
        t = time.monotonic_ns()
        model = bus.ping(servo_id)
        entry = {"joint": joint, "id": servo_id, "responded": model is not None, "t_monotonic_ns": t}
        if model is not None:
            entry.update(dump_servo(bus, servo_id))
        servos[joint] = entry
    return {
        "servos": servos,
        "bus_stats": {
            "transactions": bus.stats.transactions,
            "failures": {f"{k[0]}:{k[1]}": v for k, v in bus.stats.failures.items()},
            "servo_errors": {f"{k[0]}:{k[1]}": v for k, v in bus.stats.servo_errors.items()},
        },
    }
