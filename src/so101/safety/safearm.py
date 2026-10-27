"""SafeArm: the only way torque gets enabled or a goal gets written (CLAUDE.md hard rule).

One control cycle is `step(goals)`: snapshot -> trip checks -> clamp goals -> write -> publish.
SafeArm owns the follower bus for the whole session (no poller runs), so every read and write
happens in this one thread in a fixed order. The caller sets the loop rate.

Shadow mode (`shadow=True`, step 5a): the full loop runs on the real arm with torque off and the
write path disabled. Goals are validated and logged, trips are detected and raised, nothing is
ever written (not even torque-off).

Patterns copied from LeRobot (huggingface/lerobot @ 45cca1a, 2026-10-26):
- `SerialMotorsBus.disconnect(disable_torque=True)` (src/lerobot/motors/motors_bus.py): torque off
  on every motor when the arm is released, even after an error. Same here in __exit__.
- `ensure_safe_goal_position` (src/lerobot/robots/utils.py): clamp each goal to present +/-
  max_relative_target. Same here, but always on (LeRobot's default is None = off).
What we change and why:
- LeRobot's `enable_torque` sets Torque_Enable=1 without syncing Goal_Position in Python. Our servos
  power up with Goal_Position=0 and Goal_Speed/Acceleration=0 (= unlimited), so that would lunge.
  We write goal := present and speed/accel caps first, read them back, then enable, one joint at
  a time, and read Torque_Enable back.
- No `configure()` at connect: LeRobot rewrites PID, Return_Delay, Phase and gripper limits (EEPROM)
  on every connect. We write nothing but the SRAM registers in hw.bus.WRITABLE.
"""

import enum
import logging
import time
from collections import Counter
from dataclasses import asdict, dataclass, field

from so101.config import ArmConfig
from so101.hw.arm import Arm, ArmSnapshot
from so101.hw.bus import BusError, ServoBus, read
from so101.safety.config import SafetyConfig
from so101.safety.contact import Contact, ContactChecker
from so101.safety.trips import Trip, TripChecker
from so101.telemetry.hub import Hub, Sample


log = logging.getLogger("so101.safety")


class State(enum.Enum):
    DISABLED = "disabled"  # torque off on every joint
    ENABLING = "enabling"  # inside the enable sequence
    HOLDING = "holding"  # torque on, goal = position at enable
    TRIPPED = "tripped"  # a safety check failed; torque was turned off; needs reset()


class SafetyError(RuntimeError):
    pass


class SafetyTrip(SafetyError):
    """A safety check failed. Torque was turned off (unless shadow) before this was raised."""

    def __init__(self, trip: Trip):
        super().__init__(f"TRIP {trip.reason} on {trip.joint or 'arm'}: {trip.value} ({trip.detail})")
        self.trip = trip


@dataclass(frozen=True)
class SafetyEvent:
    t_ns: int  # monotonic
    kind: str  # enable_ok | outside_limits | enable_failed | disable_failed | trip | ...
    joint: str | None
    detail: str


@dataclass(frozen=True)
class StepResult:
    snapshot: ArmSnapshot
    commanded: dict[str, int]  # goals actually written (or, in shadow mode, that would have been)


@dataclass
class EnableReport:
    present: dict[str, int] = field(default_factory=dict)  # position held at torque-on
    outside_limits: list[str] = field(default_factory=list)


class SafeArm:
    OWNER = "safearm"

    def __init__(self, bus: ServoBus, cfg: ArmConfig, safety: SafetyConfig | None = None,
                 hub: Hub | None = None, shadow: bool = False, collision=None, mapping=None):  # fmt: skip
        """collision + mapping (optional, step 9c): before goals are written, the goal pose is checked against
        the table (kinematics.collision, with the inside-margin rule vs the measured pose); a predicted
        contact freezes the arm like contact detection. Self-collision is logged (warn-only)."""
        self.bus, self.cfg, self.hub, self.shadow = bus, cfg, hub, shadow
        if (collision is None) != (mapping is None):
            raise SafetyError("collision checking needs both a CollisionModel and a Mapping")
        self.collision, self.mapping = collision, mapping
        self.collision_hit: str | None = None
        self._self_warned: set[str] = set()
        self.safety = safety or SafetyConfig.from_arm_config(cfg)
        self.arm = Arm(bus, cfg.joints)
        self.state = State.DISABLED
        self.events: list[SafetyEvent] = []
        self.clamps: Counter = Counter()  # (joint, max_step | limit | no_present) -> count
        self.trips = TripChecker(self.safety)
        self.seq = 0
        self._entered = False
        self._last_torque_check_ns: int | None = None
        self._servo_errors_seen = 0
        self.frozen = False  # set by freeze() or contact detection; goals refused until the next enable()
        self.contact: ContactChecker | None = None  # created at enable()
        self.contact_hit: Contact | None = None

    # --- lifecycle -------------------------------------------------------------------------

    def __enter__(self) -> "SafeArm":
        self.bus.claim(self.OWNER)  # raises if a poller (or anything else) owns the bus
        self._entered = True
        self._servo_errors_seen = sum(
            n for (_, flag), n in self.bus.stats.servo_errors.items() if flag not in ("voltage", "overheat")
        )
        for name in self.safety.disabled_contact_checks():
            log.warning("contact check %s is DISABLED (null in config)", name)
            self._event("check_disabled", None, f"{name} = null: this contact check is off for the session")
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
        if self.shadow:
            raise SafetyError(f"shadow mode: refusing to write {name}={value} to id {sid}")
        self.bus.write(self.OWNER, sid, name, value)

    # --- torque on -------------------------------------------------------------------------

    def enable(self) -> EnableReport:
        """Safe torque-enable, one joint at a time:
        goal := present, speed/accel caps, read all three back, check the joint hasn't moved,
        Torque_Enable=1, read it back. Any failure turns torque off everywhere and raises."""
        if not self._entered:
            raise SafetyError("use SafeArm as a context manager (with SafeArm(...) as arm:)")
        if self.shadow:
            raise SafetyError("shadow mode: enable() is not allowed")
        if self.state is not State.DISABLED:
            raise SafetyError(f"enable() needs state DISABLED, not {self.state.value}")
        self._last_torque_check_ns = None
        self.frozen = False
        self.collision_hit = None
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
        self.contact = ContactChecker(self.safety, report.present, time.monotonic_ns())
        self.contact_hit = None
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
        if self.shadow:
            return []  # torque was verified off at preflight and nothing can turn it on
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

    @property
    def last_goals(self) -> dict[str, int]:
        """Goal_Position each joint was last commanded (enable hold position until moved)."""
        return dict(self.contact.goal) if self.contact is not None else {}

    # --- freeze ----------------------------------------------------------------------------

    def freeze(self) -> StepResult:
        """Stop all motion but keep torque on: Goal_Position := present position on every joint, then
        refuse further goals (monitoring and trips continue). The servo decelerates at the
        Acceleration cap: from 150 ticks/s at 1000 ticks/s^2 that is ~0.15 s and ~11 ticks (~1 deg).
        For an imminent collision this beats torque-off, which drops the gravity-loaded joints."""
        snap_goals = {j: s.position for j, s in self.arm.snapshot().joints.items() if s is not None}
        result = self.step(snap_goals)  # clamps keep these as-is: present is always allowed
        self.frozen = True
        missing = sorted(set(self.cfg.joints) - set(snap_goals))
        self._event("freeze", None, f"goal := present {result.commanded}" + (f"; no reading for {missing}" if missing else ""))
        return result

    # --- speed ---------------------------------------------------------------------------

    def set_speed(self, joint: str, ticks_s: int) -> None:
        """Lower one joint's Goal_Speed below the configured cap (e.g. to let the servo's own profile
        run a slow move). Never above the cap; enable() resets every joint to the cap."""
        if not self._entered:
            raise SafetyError("use SafeArm as a context manager (with SafeArm(...) as arm:)")
        if joint not in self.cfg.joints:
            raise SafetyError(f"unknown joint {joint!r}")
        if not 1 <= ticks_s <= self.safety.speed_cap:
            raise SafetyError(f"speed {ticks_s} outside 1..{self.safety.speed_cap} ticks/s (the configured cap)")
        sid = self.cfg.joints[joint]
        self._write(sid, "Goal_Speed", int(ticks_s))
        if (got := read(self.bus, sid, "Goal_Speed")) != ticks_s:
            raise SafetyError(f"{joint}: Goal_Speed read back {got}, wrote {ticks_s}")
        self._event("set_speed", joint, f"Goal_Speed {ticks_s} ticks/s")

    # --- control cycle ---------------------------------------------------------------------

    def step(self, goals: dict[str, float] | None = None) -> StepResult:
        """One cycle: snapshot, trip checks, clamp + write goals, publish. Raises SafetyTrip.

        Without goals it only monitors (allowed in any state). Goals need torque on (HOLDING),
        or shadow mode. Joints not in `goals` keep their current Goal_Position."""
        if not self._entered:
            raise SafetyError("use SafeArm as a context manager (with SafeArm(...) as arm:)")
        if goals and not (self.state is State.HOLDING or self.shadow):
            raise SafetyError(f"goals need state HOLDING (enable() first), not {self.state.value}")
        if goals and self.frozen:
            raise SafetyError("frozen: goals are refused until the next enable()")
        snap = self.arm.snapshot()
        trip = self.trips.feed(snap) or self._bus_checks(snap)
        commanded: dict[str, int] = {}
        hit = None
        if trip is None and self.contact is not None and self.state is State.HOLDING and not self.frozen and not self.shadow:
            hit = self.contact.check(snap)
        if hit is not None:  # freeze in place: goal := present everywhere, torque stays on
            commanded = self._freeze_in_place(snap)
            self.contact_hit = hit
            self._event("contact", hit.joint, f"{hit.detail} -> FROZEN (goal := present, torque on)")
        elif trip is None and goals:
            commanded = self.clamp_goals(goals, snap)
            blocked = self._predicted_collision(commanded, snap) if (self.collision and not self.shadow) else None
            if blocked:
                commanded = self._freeze_in_place(snap)
                self.collision_hit = blocked
                self._event("collision_predicted", None, f"{blocked} -> FROZEN (goal not written; goal := present)")
            else:
                for joint, g in commanded.items():
                    if not self.shadow:
                        self._write(self.cfg.joints[joint], "Goal_Position", g)
                if self.contact is not None and not self.shadow:
                    self.contact.update(commanded, snap.t_start_ns)
        if self.hub is not None:  # publish before tripping, so the log holds the tripping sample
            self.hub.publish(Sample(self.cfg.name, self.seq, snap, commanded or None))
        self.seq += 1
        if trip is not None:
            self._trip(trip)
        return StepResult(snap, commanded)

    def _freeze_in_place(self, snap: ArmSnapshot) -> dict[str, int]:
        commanded = self.clamp_goals({j: s.position for j, s in snap.joints.items() if s is not None}, snap)
        for joint, g in commanded.items():
            self._write(self.cfg.joints[joint], "Goal_Position", g)
        if self.contact is not None:
            self.contact.update(commanded, snap.t_start_ns)
        self.frozen = True
        return commanded

    def _predicted_collision(self, commanded: dict[str, int], snap: ArmSnapshot) -> str | None:
        """Table check of the goal pose vs the measured pose (margin rule). Returns the violation text or None."""
        measured = {j: s.position for j, s in snap.joints.items() if s is not None}
        if len(measured) < len(self.cfg.joints):
            return None  # no complete reading: the comm check handles it; don't guess geometry
        goal_ticks = {**measured, **self.last_goals, **commanded}
        ref = self.collision.check_ticks(measured)
        rep = self.collision.check_ticks(goal_ticks)
        for w in rep.warnings:  # self-collision: warn-only, once per pair per session
            pair = w.split(" ")[0] + w.split(" ")[2]
            if pair not in self._self_warned:
                self._self_warned.add(pair)
                self._event("self_collision_warning", None, w)
        v = self.collision.table_violations(rep, ref)
        return v[0] if v else None

    def clamp_goals(self, goals: dict[str, float], snap: ArmSnapshot) -> dict[str, int]:
        """present +/- max_step, then the software limits. A joint outside its limits may only
        move back toward them (never further out, never a clamp-jump to the limit)."""
        s, out = self.safety, {}
        for joint, goal in goals.items():
            if joint not in self.cfg.joints:
                raise SafetyError(f"unknown joint {joint!r}")
            st = snap.joints[joint]
            if st is None:  # no fresh position: can't validate, so don't move this joint
                self.clamps[(joint, "no_present")] += 1
                continue
            p, g = st.position, int(round(goal))
            stepped = min(max(g, p - s.max_step), p + s.max_step)
            if stepped != g:
                self.clamps[(joint, "max_step")] += 1
            lo, hi = s.limits[joint]
            if p > hi:
                final = min(max(stepped, lo), p)
            elif p < lo:
                final = min(max(stepped, p), hi)
            else:
                final = min(max(stepped, lo), hi)
            if final != stepped:
                self.clamps[(joint, "limit")] += 1
            out[joint] = final
        return out

    def _bus_checks(self, snap: ArmSnapshot) -> Trip | None:
        """Checks that need the bus: packet error flags, and Torque_Enable still on."""
        t = snap.t_start_ns
        # Packet error byte: voltage/overheat bits come from the glitchy ADC and are covered by the
        # debounced Status/V/degC checks in TripChecker; overload, overcurrent and angle trip at once.
        serious = sum(n for (_, flag), n in self.bus.stats.servo_errors.items() if flag not in ("voltage", "overheat"))
        if serious > self._servo_errors_seen:
            flags = {f"{sid}:{flag}": n for (sid, flag), n in self.bus.stats.servo_errors.items()}
            self._servo_errors_seen = serious
            return Trip(t, None, "servo_error", serious, f"packet error flags {flags}")
        if self.state is State.HOLDING and (
            self._last_torque_check_ns is None or (t - self._last_torque_check_ns) / 1e9 >= self.safety.torque_check_s
        ):
            self._last_torque_check_ns = t
            for joint, sid in self.cfg.joints.items():
                try:
                    if read(self.bus, sid, "Torque_Enable") != 1:
                        return Trip(t, joint, "torque_off", 0, "Torque_Enable found cleared while holding")
                except BusError:
                    pass  # a failed read is counted by the comm check on the next snapshots
        return None

    def _trip(self, trip: Trip) -> None:
        self._event("trip", trip.joint, f"{trip.reason}={trip.value}: {trip.detail}")
        self.disable()
        self.state = State.TRIPPED
        raise SafetyTrip(trip)

    def report(self) -> dict:
        """JSON-able summary for the run directory (safety_events.json)."""
        return {
            "state": self.state.value,
            "shadow": self.shadow,
            "cycles": self.seq,
            "events": [asdict(e) for e in self.events],
            "clamps": {f"{j}:{kind}": n for (j, kind), n in self.clamps.items()},
            "safety": asdict(self.safety),
        }
