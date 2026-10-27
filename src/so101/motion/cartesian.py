"""Straight-line Cartesian moves of the tool point (step 9b). Planned and validated BEFORE torque-on.

plan_line(): the TCP moves along the segment p0 -> p1 with a min-jerk time profile (peak speed `speed`),
sampled at the control rate. Each sample is solved with IK warm-started from the previous one, so joints
change smoothly. The plan is refused (with the reason) if any sample fails IK, leaves the joint limits,
needs a joint jump larger than `max_dq_per_cycle`, or hits the table in the collision model.
Execution is the normal SafeArm loop: plan.goals_fn(t) -> joint goals in ticks.
"""

import math
from dataclasses import dataclass, field

import numpy as np

from so101.kinematics.collision import CollisionModel
from so101.kinematics.fk import CHAIN_JOINTS, DEFAULT_TCP, Chain, fk
from so101.kinematics.ik import solve_ik
from so101.kinematics.mapping import Mapping
from so101.motion.loop import MINJERK_STRETCH, _minjerk


@dataclass
class CartesianPlan:
    ok: bool
    reason: str
    times: np.ndarray = field(default_factory=lambda: np.zeros(0))  # s, per sample
    qs: np.ndarray = field(default_factory=lambda: np.zeros((0, 5)))  # rad, per sample (CHAIN_JOINTS)
    tcp_planned: np.ndarray = field(default_factory=lambda: np.zeros((0, 3)))  # FK of qs, metres
    p0: np.ndarray = field(default_factory=lambda: np.zeros(3))
    p1: np.ndarray = field(default_factory=lambda: np.zeros(3))
    max_dq: float = 0.0  # largest joint change between consecutive samples, rad
    max_line_dev_m: float = 0.0  # planned TCP distance from the ideal line (IK residual)
    min_table_m: float = float("inf")
    duration_s: float = 0.0
    mapping: Mapping | None = None

    def goals_ticks(self, i: int) -> dict[str, int]:
        return {j: int(round(self.mapping.joints[j].to_ticks(self.qs[i, k]))) for k, j in enumerate(CHAIN_JOINTS)}

    def goals_fn(self):
        """For control_loop: t (s) -> joint goals in ticks (holds the last sample after the end)."""
        def fn(t: float) -> dict[str, int]:
            i = int(np.searchsorted(self.times, t, side="right")) - 1
            return self.goals_ticks(min(max(i, 0), len(self.times) - 1))
        return fn


def line_point_distance(p0, p1, pts) -> np.ndarray:
    d = p1 - p0
    L = np.linalg.norm(d)
    if L == 0:
        return np.linalg.norm(pts - p0, axis=1)
    u = d / L
    v = pts - p0
    along = np.clip(v @ u, 0, L)
    return np.linalg.norm(v - np.outer(along, u), axis=1)


def plan_line(chain: Chain, mapping: Mapping, q_start: np.ndarray, p1, limits: np.ndarray, *, speed: float = 0.03,
              rate_hz: float = 100.0, tcp: str = DEFAULT_TCP, approach=None, hold_roll: bool = True,
              max_dq_per_cycle: float = math.radians(2.0), collision: CollisionModel | None = None,
              gripper_rad: float | None = None) -> CartesianPlan:  # fmt: skip
    """Plan a straight TCP line from the current pose's TCP to p1 (metres, base frame)."""
    q_start = np.asarray(q_start, float)
    p0 = fk(chain, q_start, tcp=tcp)[:3, 3]
    p1 = np.asarray(p1, float)
    L = float(np.linalg.norm(p1 - p0))
    duration = L / speed * MINJERK_STRETCH if L > 0 else 0.0
    n = max(2, int(math.ceil(duration * rate_hz)) + 1)
    times = np.linspace(0.0, duration, n)
    fixed = {4: q_start[4]} if hold_roll else None
    qs, q = np.zeros((n, 5)), q_start.copy()
    plan = CartesianPlan(False, "", times, qs, np.zeros((n, 3)), p0, p1, duration_s=duration, mapping=mapping)
    for i, t in enumerate(times):
        target = p0 + (p1 - p0) * _minjerk(t / duration if duration > 0 else 1.0)
        r = solve_ik(chain, target, q, limits, tcp=tcp, approach=approach, fixed=fixed)
        if not r.success:
            plan.reason = f"IK failed at sample {i}/{n} (t={t:.2f} s, target {np.round(target * 1000, 1)} mm): {r.reason}, " \
                          f"pos err {r.pos_err_m * 1000:.1f} mm"  # fmt: skip
            return plan
        if i:
            dq = float(np.max(np.abs(r.q - q)))
            plan.max_dq = max(plan.max_dq, dq)
            if dq > max_dq_per_cycle:
                plan.reason = f"joint jump {math.degrees(dq):.1f}° at sample {i} (limit {math.degrees(max_dq_per_cycle):.1f}°)"
                return plan
        qs[i], q = r.q, r.q
        plan.tcp_planned[i] = fk(chain, r.q, tcp=tcp)[:3, 3]
    plan.max_line_dev_m = float(line_point_distance(p0, p1, plan.tcp_planned).max())
    if collision is not None:
        path = [{**dict(zip(CHAIN_JOINTS, qq)), **({"gripper": gripper_rad} if gripper_rad is not None else {})}
                for qq in qs]  # fmt: skip
        reports = [collision.check(qd) for qd in path]
        plan.min_table_m = min(r.min_table for r in reports)
        for i, rep in enumerate(reports):  # margin rule against the start pose
            v = collision.table_violations(rep, reports[0])
            if v:
                plan.reason = f"table collision at sample {i} (t={times[i]:.2f} s): {v[0]}"
                return plan
    plan.ok, plan.reason = True, "ok"
    return plan
