"""SafeArm: the only way torque gets enabled or a goal gets written (CLAUDE.md hard rule).

Step 4b: ownership, the torque-enable sequence and torque-off. Goal commands, clamps and trips
come in 4c.

Patterns copied from LeRobot (huggingface/lerobot @ 45cca1a, 2026-10-26):
- `SerialMotorsBus.disconnect(disable_torque=True)` (src/lerobot/motors/motors_bus.py): torque off
  on every motor when the arm is released, even after an error. Same here in __exit__.
What we change and why:
- LeRobot's `enable_torque` sets Torque_Enable=1 without syncing Goal_Position in Python. Our servos
  power up with Goal_Position=0 and Goal_Speed/Acceleration=0 (= unlimited), so that would lunge.
  We write goal := present and speed/accel caps first, read them back, then enable, one joint at
  a time, and read Torque_Enable back.
- No `configure()` at connect: LeRobot rewrites PID, Return_Delay, Phase and gripper limits (EEPROM)
  on every connect. We write nothing but the SRAM registers in hw.bus.WRITABLE.
"""

import enum
import time
from dataclasses import dataclass, field

from so101.config import ArmConfig
from so101.hw.arm import Arm
from so101.hw.bus import BusError, ServoBus, read
from so101.safety.config import SafetyConfig


class State(enum.Enum):
    DISABLED = "disabled"  # torque off on every joint
    ENABLING = "enabling"  # inside the enable sequence
    HOLDING = "holding"  # torque on, goal = position at enable
    TRIPPED = "tripped"  # a safety check failed; torque was turned off; needs reset()


class SafetyError(RuntimeError):
    pass


@dataclass(frozen=True)
class SafetyEvent:
    t_ns: int  # monotonic
    kind: str  # enable_ok | outside_limits | enable_failed | disable_failed | trip | ...
    joint: str | None
    detail: str


@dataclass
class EnableReport:
    present: dict[str, int] = field(default_factory=dict)  # position held at torque-on
    outside_limits: list[str] = field(default_factory=list)


class SafeArm:
    OWNER = "safearm"

    def __init__(self, bus: ServoBus, cfg: ArmConfig, safety: SafetyConfig | None = None):
        self.bus, self.cfg = bus, cfg
        self.safety = safety or SafetyConfig.from_arm_config(cfg)
        self.arm = Arm(bus, cfg.joints)
        self.state = State.DISABLED
        self.events: list[SafetyEvent] = []
        self._entered = False

    # --- lifecycle -------------------------------------------------------------------------

    def __enter__(self) -> "SafeArm":
        self.bus.claim(self.OWNER)  # raises if a poller (or anything else) owns the bus
        self._entered = True
        try:
            self._preflight()
        except BaseException:
            # Release without disable(): if torque was already on, turning it off would drop the arm.
            self.bus.release(self.OWNER)
            self._entered = False
            raise
        return self

    def __exit__(self, *exc) -> None:
        try:
            self.disable()
        finally:
            self.bus.release(self.OWNER)
            self._entered = False

    def _preflight(self) -> None:
        """Every joint answers and torque is off. We don't adopt an arm already under torque:
        its goal/speed/accel state is unknown, and turning it off would drop the arm."""
        on = []
        for joint, sid in self.cfg.joints.items():
            try:
                if read(self.bus, sid, "Torque_Enable") != 0:
                    on.append(joint)
            except BusError as e:
                raise SafetyError(f"preflight: {joint} (id {sid}) not reachable: {e}") from None
        if on:
            raise SafetyError(f"preflight: torque already on for {on}; support the arm and power-cycle it")

    def _event(self, kind: str, joint: str | None, detail: str) -> None:
        self.events.append(SafetyEvent(time.monotonic_ns(), kind, joint, detail))

    def _write(self, sid: int, name: str, value: int) -> None:
        self.bus.write(self.OWNER, sid, name, value)

    # --- torque on -------------------------------------------------------------------------

    def enable(self) -> EnableReport:
        """Safe torque-enable, one joint at a time:
        goal := present, speed/accel caps, read all three back, check the joint hasn't moved,
        Torque_Enable=1, read it back. Any failure turns torque off everywhere and raises."""
        if not self._entered:
            raise SafetyError("use SafeArm as a context manager (with SafeArm(...) as arm:)")
        if self.state is not State.DISABLED:
            raise SafetyError(f"enable() needs state DISABLED, not {self.state.value}")
        self.state = State.ENABLING
        report = EnableReport()
        try:
            for joint, sid in self.cfg.joints.items():
                report.present[joint] = self._enable_joint(joint, sid)
                lo, hi = self.safety.limits[joint]
                if not lo <= report.present[joint] <= hi:
                    report.outside_limits.append(joint)
                    self._event("outside_limits", joint, f"holding {report.present[joint]} outside [{lo}, {hi}]")
        except BaseException as e:
            self._event("enable_failed", None, str(e))
            self.disable()
            self.state = State.TRIPPED
            raise SafetyError(f"enable failed, torque turned off: {e}") from e
        self.state = State.HOLDING
        self._event("enable_ok", None, f"holding {report.present}")
        return report

    def _enable_joint(self, joint: str, sid: int) -> int:
        s = self.safety
        present = read(self.bus, sid, "Present_Position")
        self._write(sid, "Goal_Position", present)
        self._write(sid, "Goal_Speed", s.speed_cap)
        self._write(sid, "Acceleration", s.accel_cap)
        for name, want in (("Goal_Position", present), ("Goal_Speed", s.speed_cap), ("Acceleration", s.accel_cap)):
            got = read(self.bus, sid, name)
            if got != want:
                raise SafetyError(f"{joint}: {name} read back {got}, wrote {want}")
        now = read(self.bus, sid, "Present_Position")
        if abs(now - present) > s.enable_drift_tol:
            raise SafetyError(f"{joint}: moved {now - present} ticks during enable (tol {s.enable_drift_tol})")
        self._write(sid, "Torque_Enable", 1)
        if (te := read(self.bus, sid, "Torque_Enable")) != 1:
            raise SafetyError(f"{joint}: Torque_Enable read back {te} after writing 1")
        return present

    # --- torque off ------------------------------------------------------------------------

    def disable(self) -> list[str]:
        """Torque off on every joint. Tries all joints even if some fail; never raises.
        Returns the joints that could not be confirmed off. Note: the arm falls under gravity."""
        failed = []
        for joint, sid in self.cfg.joints.items():
            try:
                self._write(sid, "Torque_Enable", 0)
                if read(self.bus, sid, "Torque_Enable") != 0:
                    failed.append(joint)
            except BusError:
                failed.append(joint)
        if failed:
            self._event("disable_failed", None, f"could not confirm torque off: {failed}")
        if self.state is not State.TRIPPED:
            self.state = State.DISABLED
        return failed

    def reset(self) -> None:
        """Acknowledge a trip (torque is already off) so enable() may be called again."""
        if self.state is State.TRIPPED:
            self._event("reset", None, "trip acknowledged")
            self.state = State.DISABLED
