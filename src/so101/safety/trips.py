"""Trip checks on telemetry snapshots. Pure logic (no bus), so recorded runs can be replayed.

Timing uses each snapshot's monotonic t_start_ns, never the wall clock, so a replay of a parquet
run behaves exactly like the live loop did.

- temperature / voltage / load: must be out of range *continuously* for a window. One good sample
  resets the window, so the single-sample ADC glitches on IDs 4-6 (notes 2026-10-26) can't trip,
  while a real overheat or sag (sustained) does.
- Status register: overload / overcurrent / angle bits trip at once. The voltage and overheat bits
  are debounced like the V/degC checks: the servo derives them from the same glitchy ADC (step 4c
  replay: the leader gripper sets bit 0 on exactly the samples where its voltage glitches below its
  4.0 V Min_Voltage_Limit; 27 of ~264k samples, never on the follower).
- Communication: N consecutive snapshots with a failed servo.
Torque-cleared and packet error-byte checks need the bus and live in SafeArm.
"""

from dataclasses import dataclass

from so101.hw.arm import ArmSnapshot
from so101.safety.config import SafetyConfig

# Status / packet error-byte bits (same layout; hw.bus.SERVO_ERROR_BITS).
ADC_BITS = 0x01 | 0x04  # voltage, overheat: from the V/degC ADC, so debounced


@dataclass(frozen=True)
class Trip:
    t_ns: int
    joint: str | None
    reason: str  # temperature | voltage | load | status | comm | torque_off | servo_error
    value: float | str
    detail: str


class TripChecker:
    def __init__(self, safety: SafetyConfig):
        self.s = safety
        self._since: dict[tuple[str, str], int] = {}  # (joint, reason) -> t_ns first out of range
        self._comm_fails = 0

    def _sustained(self, joint: str, reason: str, bad: bool, t_ns: int, window_s: float) -> bool:
        key = (joint, reason)
        if not bad:
            self._since.pop(key, None)
            return False
        start = self._since.setdefault(key, t_ns)
        return (t_ns - start) / 1e9 >= window_s

    def feed(self, snap: ArmSnapshot) -> Trip | None:
        s, t = self.s, snap.t_start_ns
        if snap.ok:
            self._comm_fails = 0
        else:
            self._comm_fails += 1
            if self._comm_fails >= s.comm_fail_trip:
                return Trip(t, None, "comm", self._comm_fails,
                            f"{self._comm_fails} consecutive failed snapshots: {snap.errors}")  # fmt: skip
        lo_v, hi_v = s.voltage_v
        for joint, st in snap.joints.items():
            if st is None:
                continue
            if st.status & ~ADC_BITS:
                return Trip(t, joint, "status", st.status, f"Status register {st.status:#04x}")
            if self._sustained(joint, "status", bool(st.status & ADC_BITS), t, s.debounce_s):
                return Trip(t, joint, "status", st.status,
                            f"Status register {st.status:#04x} (voltage/overheat bit) for {s.debounce_s} s")  # fmt: skip
            if self._sustained(joint, "temperature", st.temperature_c >= s.temp_trip_c, t, s.debounce_s):
                return Trip(t, joint, "temperature", st.temperature_c,
                            f">= {s.temp_trip_c} degC for {s.debounce_s} s")  # fmt: skip
            if self._sustained(joint, "voltage", not lo_v <= st.voltage_v <= hi_v, t, s.debounce_s):
                return Trip(t, joint, "voltage", st.voltage_v,
                            f"outside [{lo_v}, {hi_v}] V for {s.debounce_s} s")  # fmt: skip
            limit = s.load_trip_overrides.get(joint, s.load_trip_pct)
            if self._sustained(joint, "load", abs(st.load) / 10 > limit, t, s.load_trip_s):
                return Trip(t, joint, "load", st.load / 10, f"|load| > {limit} % for {s.load_trip_s} s")
        return None
