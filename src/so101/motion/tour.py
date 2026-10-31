"""Pose tours: a list of waypoints visited with all joints moving together (joint-space min-jerk).

A waypoint is either joint ticks or a tool-point target (base frame, metres) solved by IK, warm-started
from the previous waypoint so consecutive solutions stay close (no flips). Every segment is checked
before anything is written: IK success, joint limits (outside-start rule), and the table at 20 ms samples
along the interpolated path with the inside-margin rule against the segment's start. A failed check
refuses the whole tour and names the waypoint, so a tour either runs completely as planned or not at all.
Self-collision is reported as warnings (warn-only, as in SafeArm).
"""

import math
from dataclasses import dataclass, field

import numpy as np
import yaml

from so101.kinematics.collision import CollisionModel
from so101.kinematics.fk import CHAIN_JOINTS, DEFAULT_TCP, Chain, fk
from so101.kinematics.ik import solve_ik
from so101.kinematics.mapping import Mapping
from so101.motion.loop import Segment

ALL_JOINTS = [*CHAIN_JOINTS, "gripper"]


@dataclass
class Waypoint:
    name: str
    ticks: dict[str, float] | None = None  # joint target (any subset; others keep their value)
    pos_m: list[float] | None = None  # tool point target, base frame
    approach: list[float] | None = None  # world direction for the tool z axis (fingers), e.g. [0, 0, -1]
    roll: float | None = None  # wrist_roll ticks to hold during IK (default: keep current)
    gripper: float | None = None  # gripper ticks at this waypoint (default: keep current)
    dwell_s: float | None = None  # hold after arriving (default: tour dwell)
    speed: float | None = None  # peak ticks/s for the move INTO this waypoint (default: tour speed)
    start_pose: bool = False  # go back to the tour's starting pose, clamped into the software limits
    capture: bool = True  # capture_tour.py: take photos here (False = a via point that only sets the approach)
    shots: int | None = None  # capture_tour.py: photos at this waypoint (default: --shots)
    desired_ticks: dict[str, float] | None = None  # capture_tour.py: after settling, nudge the goal once so the encoder
    # READING lands here (step 10b encoder correction; at most --max-correct-ticks per joint)
    meta: dict | None = None  # free-form tags carried into the manifest (e.g. step 10b target id + method)


@dataclass
class TourStep:
    waypoint: Waypoint
    segment: Segment
    dwell_s: float
    tool_mm: list[float]
    min_table_mm: float
    warnings: list[str] = field(default_factory=list)


@dataclass
class TourPlan:
    steps: list[TourStep]
    ok: bool
    reason: str = ""

    @property
    def duration_s(self) -> float:
        return sum(s.segment.duration_s + s.dwell_s for s in self.steps)

    def goal_fn(self):
        starts, t = [], 0.0
        for s in self.steps:
            starts.append(t)
            t += s.segment.duration_s + s.dwell_s

        def fn(now: float) -> dict[str, float] | None:
            for s, t0 in zip(reversed(self.steps), reversed(starts)):
                if now >= t0:
                    return s.segment.at(now - t0)
            return None

        return fn

    def phase_fn(self):
        """now -> (index, name, 'moving'|'dwell')"""
        bounds, t = [], 0.0
        for i, s in enumerate(self.steps):
            bounds.append((t, t + s.segment.duration_s, i))
            t += s.segment.duration_s + s.dwell_s

        def fn(now: float) -> tuple[int, str, str]:
            for t0, t1, i in reversed(bounds):
                if now >= t0:
                    return i, self.steps[i].waypoint.name, "moving" if now < t1 else "dwell"
            return 0, self.steps[0].waypoint.name, "moving"

        return fn


def load_tour(path) -> tuple[list[Waypoint], dict]:
    d = yaml.safe_load(open(path))
    return [Waypoint(**w) for w in d["waypoints"]], {k: v for k, v in d.items() if k != "waypoints"}


def _ik_multi_seed(chain: Chain, pos, q0: np.ndarray, limits_rad: np.ndarray, approach, roll: float,
                   tol_ang: float = math.radians(3.0)):
    """DLS IK is local: from a folded pose it can stall in a minimum. Try the current pose first, then a grid of
    seeds across the joint ranges; of the successes keep the one closest to the current pose (smallest move).
    Approach tolerance 3°: the fixed-jaw tip sits off the arm's plane, so an exactly radial approach off-axis
    is not reachable with wrist_roll held; a tour doesn't need the direction to 1°."""
    best = solve_ik(chain, pos, q0, limits_rad, approach=approach, fixed={4: roll}, tol_ang=tol_ang)
    if best.success:
        return best
    lo, hi = limits_rad[:, 0], limits_rad[:, 1]
    fallback = best
    for f_lift in (0.3, 0.5, 0.7):
        for f_elbow in (0.3, 0.5, 0.7):
            for f_wrist in (0.3, 0.5, 0.7):
                seed = q0.copy()
                for i, f in ((1, f_lift), (2, f_elbow), (3, f_wrist)):
                    seed[i] = lo[i] + f * (hi[i] - lo[i])
                r = solve_ik(chain, pos, seed, limits_rad, approach=approach, fixed={4: roll}, tol_ang=tol_ang)
                if r.success and (not best.success or np.abs(r.q - q0).sum() < np.abs(best.q - q0).sum()):
                    best = r
    return best if best.success else fallback


def _within(t: float, lim: tuple[int, int], start: float) -> bool:
    lo, hi = min(lim[0], start), max(lim[1], start)  # outside-start rule
    return lo - 0.5 <= t <= hi + 0.5


def plan_tour(waypoints: list[Waypoint], start_ticks: dict[str, float], *, chain: Chain, mapping: Mapping,
              limits_ticks: dict[str, tuple[int, int]], limits_rad: np.ndarray, collision: CollisionModel,
              speed: float = 150.0, dwell_s: float = 2.0, sample_s: float = 0.02) -> TourPlan:  # fmt: skip
    cur = {j: float(start_ticks[j]) for j in ALL_JOINTS}
    steps: list[TourStep] = []
    for w in waypoints:
        tgt = dict(cur)
        if w.start_pose:
            tgt = {j: float(min(max(start_ticks[j], limits_ticks[j][0]), limits_ticks[j][1])) for j in ALL_JOINTS}
        if w.ticks:
            tgt.update({j: float(v) for j, v in w.ticks.items()})
        if w.pos_m is not None:
            q0 = np.array([mapping.joints[j].to_rad(cur[j]) for j in CHAIN_JOINTS])
            roll = mapping.joints["wrist_roll"].to_rad(w.roll if w.roll is not None else cur["wrist_roll"])
            r = _ik_multi_seed(chain, np.array(w.pos_m), q0, limits_rad, None if w.approach is None else np.array(w.approach), roll)
            if not r.success:
                return TourPlan(steps, False, f"{w.name}: IK failed ({r.reason}, {r.pos_err_m * 1000:.1f} mm, "
                                              f"{math.degrees(r.ang_err_rad):.1f}° off)")  # fmt: skip
            tgt.update({j: mapping.joints[j].to_ticks(v) for j, v in zip(CHAIN_JOINTS, r.q)})
        if w.gripper is not None:
            tgt["gripper"] = float(w.gripper)
        tgt = {j: float(round(v)) for j, v in tgt.items()}
        for j in ALL_JOINTS:
            if not _within(tgt[j], limits_ticks[j], cur[j]):
                return TourPlan(steps, False, f"{w.name}: {j} {tgt[j]:.0f} outside limits {limits_ticks[j]}")
        seg = Segment(dict(cur), tgt, w.speed or speed, "minjerk")
        ref = collision.check_ticks(cur)
        n = max(2, int(math.ceil(seg.duration_s / sample_s)) + 1)
        worst, warns = float("inf"), set()
        for t in np.linspace(0, seg.duration_s, n):
            rep = collision.check_ticks(seg.at(float(t)))
            worst = min(worst, rep.min_table)
            bad = collision.table_violations(rep, ref)
            if bad:
                return TourPlan(steps, False, f"{w.name}: path hits the table margin at t={t:.2f} s of {seg.duration_s:.2f} s: {bad[0]}")
            warns.update(rep.warnings)
        q = np.array([mapping.joints[j].to_rad(tgt[j]) for j in CHAIN_JOINTS])
        p = fk(chain, q, tcp=DEFAULT_TCP)[:3, 3]
        steps.append(TourStep(w, seg, w.dwell_s if w.dwell_s is not None else dwell_s, (p * 1000).round(1).tolist(),
                              worst * 1000, sorted(warns)))  # fmt: skip
        cur = tgt
    return TourPlan(steps, True)
