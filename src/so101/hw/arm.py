"""Arm: a named set of servos on one bus. Read-only for now (writes arrive in step 4, via SafeArm).

`snapshot()` reads the telemetry block of every joint, one transaction per servo, and stamps
the whole read with host monotonic time. Phase 2 calls it before and after each image.
"""

import dataclasses
import time
from dataclasses import dataclass, field

from so101.hw.bus import BusError, ServoBus
from so101.hw.registers import REGISTERS, decode

# Present_Position (56) .. Present_Current (69, 2 bytes): one contiguous read per servo.
# Addresses 64, 67 and 68 sit inside the block but aren't in our table; their bytes are ignored.
TELEMETRY_REGISTERS = [
    "Present_Position", "Present_Speed", "Present_Load", "Present_Voltage",
    "Present_Temperature", "Status", "Moving", "Present_Current",
]  # fmt: skip
BLOCK_ADDR = REGISTERS[TELEMETRY_REGISTERS[0]].addr
BLOCK_SIZE = REGISTERS[TELEMETRY_REGISTERS[-1]].addr + REGISTERS[TELEMETRY_REGISTERS[-1]].size - BLOCK_ADDR


@dataclass(frozen=True)
class JointState:
    """One servo's telemetry in raw register units (hardware layer speaks ticks)."""

    position: int  # ticks
    speed: int  # ticks/s
    load: int  # 0.1 %
    voltage_dv: int  # 0.1 V
    temperature_c: int  # degC
    status: int  # bitmask
    moving: int  # 0/1
    current: int  # 6.5 mA units
    error_byte: int = 0  # status-packet error byte of this read (hw.bus.SERVO_ERROR_BITS)

    @property
    def voltage_v(self) -> float:
        return self.voltage_dv / 10

    @property
    def current_ma(self) -> float:
        return self.current * 6.5


def decode_block(block: bytes) -> JointState:
    if len(block) != BLOCK_SIZE:
        raise ValueError(f"telemetry block is {len(block)} bytes, expected {BLOCK_SIZE}")
    v = {}
    for name in TELEMETRY_REGISTERS:
        reg = REGISTERS[name]
        off = reg.addr - BLOCK_ADDR
        v[name] = decode(reg, int.from_bytes(block[off : off + reg.size], "little"))
    return JointState(
        position=v["Present_Position"],
        speed=v["Present_Speed"],
        load=v["Present_Load"],
        voltage_dv=v["Present_Voltage"],
        temperature_c=v["Present_Temperature"],
        status=v["Status"],
        moving=v["Moving"],
        current=v["Present_Current"],
    )


@dataclass(frozen=True)
class ArmSnapshot:
    t_start_ns: int  # monotonic, before the first servo read
    t_end_ns: int  # monotonic, after the last
    joints: dict[str, JointState | None]  # None = read failed this cycle
    errors: dict[str, str] = field(default_factory=dict)  # joint -> BusError text

    @property
    def ok(self) -> bool:
        return not self.errors

    @property
    def duration_ms(self) -> float:
        return (self.t_end_ns - self.t_start_ns) / 1e6


class Arm:
    def __init__(self, bus: ServoBus, joints: dict[str, int]):
        self.bus = bus
        self.joints = dict(joints)  # name -> servo id, kinematic order

    def snapshot(self) -> ArmSnapshot:
        """Read every joint once. A failing servo becomes None + an error; it never aborts the snapshot."""
        states, errors = {}, {}
        t0 = time.monotonic_ns()
        for joint, servo_id in self.joints.items():
            try:
                block = self.bus.read_block(servo_id, BLOCK_ADDR, BLOCK_SIZE)
                states[joint] = dataclasses.replace(decode_block(block), error_byte=self.bus.last_error.get(servo_id, 0))
            except BusError as e:
                states[joint], errors[joint] = None, str(e)
        return ArmSnapshot(t0, time.monotonic_ns(), states, errors)
