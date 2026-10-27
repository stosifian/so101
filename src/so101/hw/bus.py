"""Servo bus: the only code that talks to the serial port.

`ServoBus` is the interface; `FeetechBus` is the real implementation (scservo_sdk) and
`MockBus` is an in-memory stand-in for tests and dry runs.

Writes (step 4) go through one policy, `check_write`, shared by both buses: only a short list of
SRAM registers, only sane values, and only by the bus owner. EEPROM writes are not possible
through this class at all (hard rule: no EEPROM writes without explicit approval).
"""

import threading
import time
from collections import Counter
from dataclasses import dataclass, field
from typing import Protocol

from so101.hw.registers import REGISTERS, Register, decode, encode

# The only registers a bus will write, with the values it accepts. All SRAM (volatile).
# 0 means "unlimited" for Goal_Speed and Acceleration on STS servos, so 0 is refused for both;
# Torque_Enable=128 recalibrates the position offset (an EEPROM write) per Feetech docs, so only 0/1.
WRITABLE: dict[str, range] = {
    "Torque_Enable": range(0, 2),
    "Goal_Position": range(0, 4096),  # ticks
    "Goal_Speed": range(1, 1 << 15),  # ticks/s, never 0
    "Acceleration": range(1, 255),  # 100 ticks/s^2, never 0
    # 0.1 % of max output (SRAM; power-up value = Max_Torque_Limit = 1000). Step 10a, user-approved: caps the
    # gripper during a squeeze (LeRobot caps it at 50 % too, but via the EEPROM Max_Torque_Limit). 0 refused.
    "Torque_Limit": range(1, 1001),
}


def check_write(name: str, value: int) -> tuple[Register, int]:
    """Validate a write against WRITABLE; return the register and its raw encoded value."""
    if name not in WRITABLE:
        reg = REGISTERS.get(name)
        area = reg.area if reg else "unknown"
        raise BusError(f"write to {name} ({area}) refused: only {sorted(WRITABLE)} are writable")
    if value not in WRITABLE[name]:
        allowed = WRITABLE[name]
        raise BusError(f"write {name}={value} refused: allowed {allowed.start}..{allowed.stop - 1}")
    reg = REGISTERS[name]
    return reg, encode(reg, value)

# Error bits in the status packet's error byte (scservo_sdk protocol_packet_handler).
SERVO_ERROR_BITS = {1: "voltage", 2: "angle", 4: "overheat", 8: "overcurrent", 32: "overload"}


class BusError(RuntimeError):
    pass


def error_flags(error_byte: int) -> list[str]:
    return [name for bit, name in SERVO_ERROR_BITS.items() if error_byte & bit]


@dataclass
class BusStats:
    """Debug counters, so flaky wiring shows up as numbers instead of vague behaviour."""

    transactions: int = 0
    failures: Counter = field(default_factory=Counter)  # (servo_id, reason) -> count
    servo_errors: Counter = field(default_factory=Counter)  # (servo_id, flag) -> count
    last_latency_s: float = 0.0


class BusOwnership:
    """One owner per bus per session: a poller (read-only sessions) or SafeArm's loop (motion).

    scservo_sdk is request/reply on one serial line; two threads using it at once can read each
    other's replies. claim() turns "only one user" from a convention into an error at startup.
    """

    def __init__(self):
        self._owner: str | None = None
        self._owner_lock = threading.Lock()

    @property
    def owner(self) -> str | None:
        return self._owner

    def claim(self, owner: str) -> None:
        with self._owner_lock:
            if self._owner is not None:
                raise BusError(f"bus already owned by {self._owner!r}; {owner!r} cannot claim it")
            self._owner = owner

    def release(self, owner: str) -> None:
        with self._owner_lock:
            if self._owner != owner:
                raise BusError(f"{owner!r} cannot release a bus owned by {self._owner!r}")
            self._owner = None

    def _require_owner(self, owner: str) -> None:
        if self._owner is None or self._owner != owner:
            raise BusError(f"write by {owner!r} refused: bus owner is {self._owner!r} (claim it first)")


class ServoBus(Protocol):
    stats: BusStats
    owner: str | None
    last_error: dict[int, int]  # servo id -> error byte of its last successful reply

    def ping(self, servo_id: int) -> int | None:
        """Return the model number, or None if the servo does not answer."""

    def read_raw(self, servo_id: int, reg: Register) -> int:
        """Return the unsigned raw value of one register. Raises BusError."""

    def read_block(self, servo_id: int, addr: int, size: int) -> bytes:
        """Read `size` contiguous bytes starting at `addr` in one transaction. Raises BusError."""

    def write(self, owner: str, servo_id: int, name: str, value: int) -> None:
        """Write one WRITABLE register (decoded value). Only the bus owner may write. Raises BusError."""

    def claim(self, owner: str) -> None: ...

    def release(self, owner: str) -> None: ...

    def close(self) -> None: ...


def read(bus: ServoBus, servo_id: int, name: str) -> int:
    """Read one register by name and decode sign-magnitude values."""
    reg = REGISTERS[name]
    return decode(reg, bus.read_raw(servo_id, reg))


class FeetechBus(BusOwnership):
    def __init__(self, port: str, baudrate: int = 1_000_000, protocol_end: int = 0, retries: int = 2):
        from scservo_sdk import COMM_SUCCESS, PacketHandler, PortHandler

        super().__init__()
        self._ok = COMM_SUCCESS
        self.last_error: dict[int, int] = {}
        self.port_name = port
        self.retries = retries
        self.stats = BusStats()
        self._port = PortHandler(port)
        self._port.baudrate = baudrate
        if not self._port.openPort():
            raise BusError(f"could not open {port}")
        self._pkt = PacketHandler(protocol_end)

    def ping(self, servo_id: int) -> int | None:
        model, comm, _ = self._pkt.ping(self._port, servo_id)
        self.stats.transactions += 1
        return model if comm == self._ok else None

    def read_raw(self, servo_id: int, reg: Register) -> int:
        try:
            return int.from_bytes(self.read_block(servo_id, reg.addr, reg.size), "little")
        except BusError as e:
            raise BusError(f"{reg.name}: {e}") from None

    def read_block(self, servo_id: int, addr: int, size: int) -> bytes:
        for attempt in range(self.retries + 1):
            t0 = time.monotonic()
            data, comm, err = self._pkt.readTxRx(self._port, servo_id, addr, size)
            self.stats.last_latency_s = time.monotonic() - t0
            self.stats.transactions += 1
            if comm == self._ok and len(data) != size:
                # scservo_sdk slices the reply without checking its declared length (Feetech-Servo-SDK
                # issue #5): a short reply would otherwise surface as a ValueError in decode_block.
                self.stats.failures[(servo_id, f"short reply ({len(data)}/{size} bytes)")] += 1
                continue
            if comm == self._ok:
                self.last_error[servo_id] = err
                for flag in error_flags(err):
                    self.stats.servo_errors[(servo_id, flag)] += 1
                return bytes(data)
            self.stats.failures[(servo_id, self._pkt.getTxRxResult(comm))] += 1
        raise BusError(f"id {servo_id} @{addr}+{size}: failed after {self.retries + 1} tries "
                       f"(last: {dict(self.stats.failures)})")

    def write(self, owner: str, servo_id: int, name: str, value: int) -> None:
        reg, raw = check_write(name, value)
        self._require_owner(owner)
        send = self._pkt.write1ByteTxRx if reg.size == 1 else self._pkt.write2ByteTxRx
        for attempt in range(self.retries + 1):  # retrying is safe: writing the same value twice is idempotent
            t0 = time.monotonic()
            comm, err = send(self._port, servo_id, reg.addr, raw)
            self.stats.last_latency_s = time.monotonic() - t0
            self.stats.transactions += 1
            if comm == self._ok:
                for flag in error_flags(err):
                    self.stats.servo_errors[(servo_id, flag)] += 1
                return
            self.stats.failures[(servo_id, self._pkt.getTxRxResult(comm))] += 1
        raise BusError(f"id {servo_id} write {name}={value}: {self._pkt.getTxRxResult(comm)} after {self.retries + 1} tries")

    def close(self) -> None:
        self._port.closePort()


@dataclass(frozen=True)
class WriteRecord:
    t_ns: int  # monotonic
    servo_id: int
    name: str
    value: int


class MockBus(BusOwnership):
    """In-memory servos. `memory[id][addr] = raw value` for the registers you care about.

    Writes go through the same `check_write` policy and owner check as FeetechBus, land in
    `memory`, and are appended to `write_log` so tests can assert on order and values.
    It does not simulate motion or firmware behaviour (that comes with SafeArm tests).
    """

    def __init__(self, memory: dict[int, dict[int, int]] | None = None):
        super().__init__()
        self.memory = memory if memory is not None else {}
        self.stats = BusStats()
        self.write_log: list[WriteRecord] = []
        self.last_error: dict[int, int] = {}  # tests/sim may set this to inject error bytes

    @classmethod
    def from_dump(cls, dump: dict) -> "MockBus":
        """Servos in the exact register state of a real dump (hw.dump.dump_arm / registers.json),
        e.g. the follower's factory power-up state: Goal_Position=0, Goal_Speed=0, Acceleration=0."""
        mem = {}
        for s in dump["servos"].values():
            if s["responded"]:
                mem[s["id"]] = {r["addr"]: r["raw"] for r in s["registers"].values()}
        return cls(mem)

    @classmethod
    def with_servos(cls, ids, model: int = 777, voltage_dv: int = 120, position: int = 2048) -> "MockBus":
        regs = REGISTERS
        mem = {
            i: {
                regs["Model"].addr: model,
                regs["ID"].addr: i,
                regs["Present_Voltage"].addr: voltage_dv,
                regs["Present_Temperature"].addr: 25,
                regs["Present_Position"].addr: position,
                regs["Max_Angle_Limit"].addr: 4095,
            }
            for i in ids
        }
        return cls(mem)

    def ping(self, servo_id: int) -> int | None:
        self.stats.transactions += 1
        servo = self.memory.get(servo_id)
        return None if servo is None else servo.get(REGISTERS["Model"].addr, 0)

    def read_raw(self, servo_id: int, reg: Register) -> int:
        self.stats.transactions += 1
        if servo_id not in self.memory:
            self.stats.failures[(servo_id, "no status packet")] += 1
            raise BusError(f"id {servo_id}: no status packet")
        return self.memory[servo_id].get(reg.addr, 0)

    def read_block(self, servo_id: int, addr: int, size: int) -> bytes:
        """Assemble the bytes of every known register inside [addr, addr+size); gaps read as 0."""
        self.stats.transactions += 1
        if servo_id not in self.memory:
            self.stats.failures[(servo_id, "no status packet")] += 1
            raise BusError(f"id {servo_id}: no status packet")
        buf = bytearray(size)
        for reg in REGISTERS.values():
            if addr <= reg.addr and reg.addr + reg.size <= addr + size:
                raw = self.memory[servo_id].get(reg.addr, 0)
                buf[reg.addr - addr : reg.addr - addr + reg.size] = raw.to_bytes(reg.size, "little")
        return bytes(buf)

    def write(self, owner: str, servo_id: int, name: str, value: int) -> None:
        reg, raw = check_write(name, value)
        self._require_owner(owner)
        self.stats.transactions += 1
        if servo_id not in self.memory:
            self.stats.failures[(servo_id, "no status packet")] += 1
            raise BusError(f"id {servo_id}: no status packet")
        self.memory[servo_id][reg.addr] = raw
        self.write_log.append(WriteRecord(time.monotonic_ns(), servo_id, name, value))

    def close(self) -> None:
        pass
