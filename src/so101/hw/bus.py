"""Servo bus: the only code that talks to the serial port.

`ServoBus` is the interface; `FeetechBus` is the real implementation (scservo_sdk) and
`MockBus` is an in-memory stand-in for tests and dry runs.

Step 2 is read-only on purpose: there is no write method yet. Writes arrive in step 4,
behind the safety layer.
"""

import threading
import time
from collections import Counter
from dataclasses import dataclass, field
from typing import Protocol

from so101.hw.registers import REGISTERS, Register, decode

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


class ServoBus(Protocol):
    stats: BusStats
    owner: str | None

    def ping(self, servo_id: int) -> int | None:
        """Return the model number, or None if the servo does not answer."""

    def read_raw(self, servo_id: int, reg: Register) -> int:
        """Return the unsigned raw value of one register. Raises BusError."""

    def read_block(self, servo_id: int, addr: int, size: int) -> bytes:
        """Read `size` contiguous bytes starting at `addr` in one transaction. Raises BusError."""

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
            if comm == self._ok:
                for flag in error_flags(err):
                    self.stats.servo_errors[(servo_id, flag)] += 1
                return bytes(data)
            self.stats.failures[(servo_id, self._pkt.getTxRxResult(comm))] += 1
        raise BusError(f"id {servo_id} @{addr}+{size}: {self._pkt.getTxRxResult(comm)} after {self.retries + 1} tries")

    def close(self) -> None:
        self._port.closePort()


class MockBus(BusOwnership):
    """In-memory servos. `memory[id][addr] = raw value` for the registers you care about."""

    def __init__(self, memory: dict[int, dict[int, int]] | None = None):
        super().__init__()
        self.memory = memory if memory is not None else {}
        self.stats = BusStats()

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

    def close(self) -> None:
        pass
