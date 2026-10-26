"""Per-joint sweep plans (step 6). Pure functions: ticks in, waypoint paths out. No hardware.

generic:  `n` evenly spaced waypoints across the middle `fraction` of the joint's software limits,
          visited from the end nearest the start, out to the far end and back.
anchored: `steps` waypoints of `step` ticks from the start toward the middle of the range, out and
          back. For gravity-loaded joints whose rest pose is far from mid-range (wrist_flex,
          elbow_flex, shoulder_lift): the first runs stay near rest.

A joint that starts outside its limits (elbow, gripper at rest) first moves inward to `margin` ticks
inside the limit (allowed by SafeArm's back-toward-the-range rule), and the sweep ends there, not at
the rest pose: once inside the range a joint can't leave it (parking decision, 2026-10-26).
"""

from dataclasses import dataclass

from so101.motion.loop import GoalFn, Segment, sequence

GRAVITY_JOINTS = ("wrist_flex", "elbow_flex", "shoulder_lift")


@dataclass(frozen=True)
class SweepPlan:
    joint: str
    start: float
    entry: float | None  # in-range point reached first, if the start was outside the limits
    path: list[float]  # waypoints in visiting order (holds happen at each)
    home: float  # where the sweep ends (start, or entry if the start was outside)


def entry_point(start: float, lo: int, hi: int, margin: int = 20) -> float | None:
    if start > hi:
        return float(hi - margin)
    if start < lo:
        return float(lo + margin)
    return None


def generic_waypoints(lo: int, hi: int, fraction: float, n: int) -> list[float]:
    if not 0 < fraction <= 1 or n < 2:
        raise ValueError("generic: 0 < fraction <= 1 and n >= 2")
    c, h = (lo + hi) / 2, fraction * (hi - lo) / 2
    return [round(c - h + 2 * h * k / (n - 1)) for k in range(n)]


def anchored_waypoints(start: float, lo: int, hi: int, step: int, steps: int) -> list[float]:
    if step <= 0 or steps < 1:
        raise ValueError("anchored: step > 0 and steps >= 1")
    direction = 1 if (lo + hi) / 2 > start else -1
    return [round(start + direction * step * k) for k in range(1, steps + 1)]


def plan_sweep(joint: str, start: float, lo: int, hi: int, profile: str, *, fraction: float = 0.5,
               n: int = 7, step: int = 150, steps: int = 4, margin: int = 20) -> SweepPlan:  # fmt: skip
    entry = entry_point(start, lo, hi, margin)
    home = entry if entry is not None else float(start)
    if profile == "generic":
        wps = generic_waypoints(lo, hi, fraction, n)
        if abs(home - wps[-1]) < abs(home - wps[0]):
            wps.reverse()
        path = wps + wps[-2::-1]  # out to the far end, back to the near end
    elif profile == "anchored":
        wps = anchored_waypoints(home, lo, hi, step, steps)
        path = wps + wps[-2::-1]
    else:
        raise ValueError(f"unknown profile {profile!r}")
    outside = [w for w in path if not lo <= w <= hi]
    if outside:
        raise ValueError(f"{joint}: waypoints {outside} outside limits [{lo}, {hi}]")
    return SweepPlan(joint, float(start), entry, [float(w) for w in path], home)


def sweep_goals(plan: SweepPlan, speed: float, hold_s: float, entry_speed: float = 50.0) -> tuple[GoalFn, float]:
    """Goal function for control_loop: (entry ramp) -> every waypoint with a hold -> back home."""
    j, segs, pos = plan.joint, [], plan.start
    if plan.entry is not None:
        segs.append(Segment({j: pos}, {j: plan.entry}, entry_speed))
        pos = plan.entry
    for w in [*plan.path, plan.home]:
        segs.append(Segment({j: pos}, {j: w}, speed))
        pos = w
    return sequence(segs, settle_s=hold_s)
