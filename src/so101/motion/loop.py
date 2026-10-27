"""Fixed-rate control loop around SafeArm.step(), plus a linear ramp trajectory.

Same timing scheme as telemetry.poller (absolute monotonic deadlines; overruns counted; if more than
a period behind, reschedule from now instead of bursting). The loop thread is the only user of the
bus (SafeArm owns it). A SafetyTrip propagates to the caller after SafeArm has turned torque off.
"""

import threading
import time
from collections.abc import Callable
from dataclasses import dataclass

from so101.safety.safearm import SafeArm

GoalFn = Callable[[float], dict[str, float] | None]  # seconds since loop start -> goals (or None = hold)


@dataclass
class LoopStats:
    cycles: int = 0
    overruns: int = 0
    skipped_slots: int = 0
    duration_s: float = 0.0
    frozen: bool = False


def control_loop(arm: SafeArm, rate_hz: float, goal_fn: GoalFn, duration_s: float,
                 stop: threading.Event | None = None, on_tick: Callable[[float], None] | None = None,
                 freeze: threading.Event | None = None) -> LoopStats:  # fmt: skip
    """Call arm.step(goal_fn(t)) at rate_hz until duration_s, `stop` is set, or a trip raises.
    `on_tick(t)` runs before each step (fault injection in demos/tests).

    If `freeze` is set, or SafeArm froze itself (contact detected): freeze once (goal := present, torque
    stays on), then keep monitoring with no goals and IGNORE duration_s: the arm holds until `stop` is
    set (Ctrl-C) or a trip."""
    period = int(1e9 / rate_hz)
    stats, stop = LoopStats(), stop or threading.Event()
    t0 = next_t = time.monotonic_ns()
    try:
        while not stop.is_set():
            t = (time.monotonic_ns() - t0) / 1e9
            if (freeze is not None and freeze.is_set()) or arm.frozen:
                if not arm.frozen:
                    arm.freeze()  # user freeze (Ctrl-C); SafeArm freezes itself on contact
                stats.frozen = True
                arm.step()
            else:
                if t >= duration_s:
                    break
                if on_tick:
                    on_tick(t)
                arm.step(goal_fn(t))
            stats.cycles += 1
            next_t += period
            now = time.monotonic_ns()
            if now > next_t:
                stats.overruns += 1
                behind = (now - next_t) // period
                if behind >= 1:
                    stats.skipped_slots += behind
                    next_t = now
                continue
            stop.wait((next_t - now) / 1e9)
    finally:
        stats.duration_s = (time.monotonic_ns() - t0) / 1e9
    return stats


# Minimum-jerk point-to-point profile s(tau) = 10 tau^3 - 15 tau^4 + 6 tau^5 (zero velocity and acceleration at
# both ends). Its peak speed is 1.875x the average, so the move takes 1.875x longer than a linear ramp at the
# same `speed`, which keeps the PEAK at `speed` (inside the caps). Smooth starts/stops excite less structural
# wobble than the linear ramp's instant stop (user observation, step 6; encoders showed no ringing).
MINJERK_STRETCH = 1.875


def _minjerk(tau: float) -> float:
    return tau * tau * tau * (10 - 15 * tau + 6 * tau * tau)


@dataclass(frozen=True)
class Segment:
    start: dict[str, float]
    target: dict[str, float]
    speed: float  # ticks/s: constant speed (linear) or peak speed (minjerk); each joint finishes on its own
    easing: str = "linear"  # linear | minjerk

    def _reach(self, j: str) -> float:
        if self.speed <= 0:
            return 0.0
        base = abs(self.target[j] - self.start[j]) / self.speed
        return base * MINJERK_STRETCH if self.easing == "minjerk" else base

    @property
    def duration_s(self) -> float:
        return max((self._reach(j) for j in self.target), default=0.0)

    def at(self, t: float) -> dict[str, float]:
        out = {}
        for j, goal in self.target.items():
            s, reach = self.start[j], self._reach(j)
            tau = 1.0 if reach == 0 or t >= reach else max(t, 0.0) / reach
            out[j] = s + (goal - s) * (_minjerk(tau) if self.easing == "minjerk" else tau)
        return out


def sequence(segments: list[Segment], settle_s: float = 0.0) -> tuple[GoalFn, float]:
    """Chain segments (each followed by settle_s of holding its target). Returns (goal_fn, total_s)."""
    starts, t = [], 0.0
    for seg in segments:
        starts.append(t)
        t += seg.duration_s + settle_s

    def goal_fn(now: float) -> dict[str, float] | None:
        for seg, t_start in zip(reversed(segments), reversed(starts)):
            if now >= t_start:
                return seg.at(now - t_start)
        return None

    return goal_fn, t


def out_and_back(start: dict[str, float], limits: dict[str, tuple[int, int]], amplitude: float,
                 speed: float, settle_s: float = 0.3) -> tuple[GoalFn, float]:  # fmt: skip
    """One joint at a time: move `amplitude` ticks toward the middle of its range, then back.
    Used by the sim demo and the step 5 checks, so both command the same thing."""
    segs, pose = [], {j: float(p) for j, p in start.items()}
    for j, (lo, hi) in limits.items():
        direction = 1 if (lo + hi) / 2 > pose[j] else -1
        out = {**pose, j: pose[j] + direction * amplitude}
        segs += [Segment(pose, out, speed), Segment(out, pose, speed)]
    return sequence(segs, settle_s)
