"""SimBus: MockBus plus crude motion and the glitches we've seen on the real arm. No hardware.

With torque on, Present_Position moves toward Goal_Position at Goal_Speed (ticks/s), advanced on
every read from host monotonic time. Deliberately simple: no acceleration profile, gravity, load,
backlash or overshoot. It exercises SafeArm's loop; it does not predict real motion (step 5-6 will).

Modelled from hardware observations (docs/hardware_log.md):
- power-up state comes from a real register dump (MockBus.from_dump)
- V/degC ADC glitches on IDs 4-6, single-sample, ~1 % (glitch_rate)
- Moving=1 while position != goal (ASSUMPTION; real semantics under torque unverified until step 5)

Fault injection for demos and tests: `inject(servo_id, register_name, value)` overrides a register
until cleared; `dead` holds ids that stop answering.
"""

import random
import time

from so101.hw.bus import BusError, MockBus
from so101.hw.registers import REGISTERS, decode, encode

GLITCH_IDS = (4, 5, 6)
_A = {n: REGISTERS[n].addr for n in ["Torque_Enable", "Goal_Position", "Goal_Speed", "Present_Position",
                                     "Present_Speed", "Moving", "Present_Voltage", "Present_Temperature"]}  # fmt: skip


class SimBus(MockBus):
    def __init__(self, memory=None, glitch_rate: float = 0.0, seed: int = 0):
        super().__init__(memory)
        self.glitch_rate = glitch_rate
        self.dead: set[int] = set()
        self._overrides: dict[tuple[int, int], int] = {}  # (id, addr) -> raw
        self._rng = random.Random(seed)
        self._t = time.monotonic_ns()
        self._frac: dict[int, float] = {}  # sub-tick motion carried between reads

    @classmethod
    def from_dump(cls, dump: dict, glitch_rate: float = 0.0, seed: int = 0) -> "SimBus":
        base = MockBus.from_dump(dump)
        return cls(base.memory, glitch_rate, seed)

    # --- fault injection -------------------------------------------------------------------

    def inject(self, servo_id: int, name: str, value: int) -> None:
        reg = REGISTERS[name]
        self._overrides[(servo_id, reg.addr)] = encode(reg, value)

    def clear(self, servo_id: int | None = None) -> None:
        self._overrides = {k: v for k, v in self._overrides.items() if servo_id is not None and k[0] != servo_id}
        self.dead.discard(servo_id) if servo_id is not None else self.dead.clear()

    # --- motion ----------------------------------------------------------------------------

    def _advance(self) -> None:
        now = time.monotonic_ns()
        dt, self._t = (now - self._t) / 1e9, now
        for sid, m in self.memory.items():
            pos = decode(REGISTERS["Present_Position"], m.get(_A["Present_Position"], 0))
            goal = decode(REGISTERS["Goal_Position"], m.get(_A["Goal_Position"], 0))
            if m.get(_A["Torque_Enable"], 0) != 1 or pos == goal:
                m[_A["Present_Speed"]], m[_A["Moving"]] = 0, 0
                continue
            speed = m.get(_A["Goal_Speed"], 0) or 3000  # 0 = unlimited; SafeArm never writes 0
            travel = self._frac.get(sid, 0.0) + speed * dt
            step = min(int(travel), abs(goal - pos))
            self._frac[sid] = travel - int(travel)
            direction = 1 if goal > pos else -1
            m[_A["Present_Position"]] = encode(REGISTERS["Present_Position"], pos + direction * step)
            m[_A["Present_Speed"]] = encode(REGISTERS["Present_Speed"], direction * speed)
            m[_A["Moving"]] = 1

    # --- reads with overrides, dead servos and glitches ------------------------------------

    def _check_alive(self, servo_id: int) -> None:
        if servo_id in self.dead:
            self.stats.transactions += 1
            self.stats.failures[(servo_id, "no status packet")] += 1
            raise BusError(f"id {servo_id}: no status packet (sim: dead)")

    def read_raw(self, servo_id, reg):
        self._check_alive(servo_id)
        self._advance()
        if (servo_id, reg.addr) in self._overrides:
            self.stats.transactions += 1
            return self._overrides[(servo_id, reg.addr)]
        return super().read_raw(servo_id, reg)

    def read_block(self, servo_id, addr, size):
        self._check_alive(servo_id)
        self._advance()
        buf = bytearray(super().read_block(servo_id, addr, size))
        for (sid, a), raw in self._overrides.items():
            reg = next(r for r in REGISTERS.values() if r.addr == a)
            if sid == servo_id and addr <= a and a + reg.size <= addr + size:
                buf[a - addr : a - addr + reg.size] = raw.to_bytes(reg.size, "little")
        if servo_id in GLITCH_IDS and self._rng.random() < self.glitch_rate:
            if self._rng.random() < 0.5:
                buf[_A["Present_Voltage"] - addr] = max(0, buf[_A["Present_Voltage"] - addr] - self._rng.randint(3, 15))
            else:
                buf[_A["Present_Temperature"] - addr] += self._rng.randint(2, 11)
        return bytes(buf)

    def write(self, owner, servo_id, name, value):
        self._check_alive(servo_id)
        self._advance()
        super().write(owner, servo_id, name, value)
        if (servo_id, REGISTERS[name].addr) in self._overrides:  # e.g. injected torque_off sticks
            self.memory[servo_id][REGISTERS[name].addr] = self._overrides[(servo_id, REGISTERS[name].addr)]
