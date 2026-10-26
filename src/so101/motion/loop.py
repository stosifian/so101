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


def control_loop(arm: SafeArm, rate_hz: float, goal_fn: GoalFn, duration_s: float,
                 stop: threading.Event | None = None, on_tick: Callable[[float], None] | None = None) -> LoopStats:  # fmt: skip
    """Call arm.step(goal_fn(t)) at rate_hz until duration_s, `stop` is set, or a trip raises.
    `on_tick(t)` runs before each step (fault injection in demos/tests)."""
    period = int(1e9 / rate_hz)
    stats, stop = LoopStats(), stop or threading.Event()
    t0 = next_t = time.monotonic_ns()
    try:
        while not stop.is_set():
            t = (time.monotonic_ns() - t0) / 1e9
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


@dataclass(frozen=True)
class Segment:
    start: dict[str, float]
    target: dict[str, float]
    speed: float  # ticks/s, along each joint (each joint ramps at this speed, finishing when done)

    @property
    def duration_s(self) -> float:
        return max((abs(self.target[j] - self.start[j]) / self.speed for j in self.target), default=0.0)

    def at(self, t: float) -> dict[str, float]:
        out = {}
        for j, goal in self.target.items():
            s = self.start[j]
            reach = abs(goal - s) / self.speed if self.speed > 0 else 0.0
            frac = 1.0 if reach == 0 or t >= reach else max(t, 0.0) / reach
            out[j] = s + (goal - s) * frac
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
