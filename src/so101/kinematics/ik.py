"""Inverse kinematics (step 9a): damped least squares on our calibrated FK. Radians and metres.

Target: a tool-point position, optionally with an approach direction (a tool-frame axis that should point
along a given world direction, e.g. fingers down). The SO-101 has 5 joints, so position + approach
direction (5 constraints) is the natural target; wrist_roll can also be held fixed.

Update (Wampler / Nakamura-Hanafusa damped least squares):
    dq = J^T (J J^T + lambda^2 I)^-1 e,   e = [p* - p ; w (a x a*)]
with the orientation rows weighted by w (metres per radian), lambda raised automatically when a step does
not reduce the error (near-singular poses, e.g. the arm stretched), and every step clamped to the joint
limits. Solving starts from the current pose, so the result stays near it (no sudden flips).

LeRobot's SO-101 IK uses the placo library on its own URDF (lerobot RobotKinematics). We need IK on *our*
calibrated FK and tick mapping, so model and real agree; DLS on fk.py is the standard choice.
"""

import math
from dataclasses import dataclass

import numpy as np

from so101.kinematics.fk import DEFAULT_TCP, Chain, FKParams, fk, geometric_jacobian


@dataclass
class IKResult:
    q: np.ndarray
    success: bool
    pos_err_m: float
    ang_err_rad: float  # approach-direction error (0 if no approach requested)
    iters: int
    reason: str  # converged | max_iter | stalled (unreachable or blocked by limits)


def limits_rad(mapping, limits_ticks: dict[str, tuple[int, int]], joints: list[str]) -> np.ndarray:
    """(n, 2) joint limits in radians from software limits in ticks (sign-aware)."""
    out = []
    for j in joints:
        a, b = (mapping.joints[j].to_rad(t) for t in limits_ticks[j])
        out.append((min(a, b), max(a, b)))
    return np.array(out)


def solve_ik(chain: Chain, target_pos, q0, limits: np.ndarray, *, tcp: str | np.ndarray = DEFAULT_TCP,
             approach: np.ndarray | None = None, tool_axis=(0.0, 0.0, 1.0), w_ori: float = 0.05,
             fixed: dict[int, float] | None = None, params: FKParams | None = None,
             tol_pos: float = 5e-4, tol_ang: float = math.radians(1.0), max_iter: int = 300,
             max_step: float = 0.2) -> IKResult:  # fmt: skip
    """Solve for 5 joint angles. q0: start (normally the current pose). limits: (5, 2) rad; a joint that
    starts outside its limits may stay where it is but never moves further out. fixed: {joint index: value}
    holds joints (e.g. {4: roll}). approach: world unit vector the tool axis should point along."""
    q = np.array(q0, float)
    if fixed:
        for i, v in fixed.items():
            q[i] = v
    lo = np.minimum(limits[:, 0], q)  # outside-start rule: the start is allowed, further out is not
    hi = np.maximum(limits[:, 1], q)
    q = np.clip(q, lo, hi)
    active = [i for i in range(5) if not fixed or i not in fixed]
    p_star = np.asarray(target_pos, float)
    a_star = None if approach is None else np.asarray(approach, float) / np.linalg.norm(approach)
    axis = np.asarray(tool_axis, float)

    def error(qq):
        T = fk(chain, qq, params, tcp)
        e_p = p_star - T[:3, 3]
        if a_star is None:
            return e_p, 0.0, T
        a = T[:3, :3] @ axis
        e_o = np.cross(a, a_star)
        ang = math.atan2(np.linalg.norm(e_o), float(np.dot(a, a_star)))
        return np.concatenate([e_p, w_ori * e_o]), ang, T

    lam = 1e-3
    e, ang, _ = error(q)
    cost = float(e @ e)
    for it in range(1, max_iter + 1):
        pos_err = float(np.linalg.norm(e[:3]))
        if pos_err < tol_pos and (a_star is None or ang < tol_ang):
            return IKResult(q, True, pos_err, ang, it - 1, "converged")
        J = geometric_jacobian(chain, q, params, tcp)
        Jt = J[:3] if a_star is None else np.vstack([J[:3], w_ori * J[3:]])
        Ja = Jt[:, active]
        dq_a = Ja.T @ np.linalg.solve(Ja @ Ja.T + lam**2 * np.eye(Ja.shape[0]), e)
        n = np.linalg.norm(dq_a)
        if n > max_step:
            dq_a *= max_step / n
        q_new = q.copy()
        q_new[active] += dq_a
        q_new = np.clip(q_new, lo, hi)
        e_new, ang_new, _ = error(q_new)
        c_new = float(e_new @ e_new)
        if c_new < cost:
            q, e, ang, cost = q_new, e_new, ang_new, c_new
            lam = max(lam / 2, 1e-6)
        else:
            lam *= 4  # more damping: smaller, safer steps
            if lam > 1e3:
                return IKResult(q, False, float(np.linalg.norm(e[:3])), ang, it, "stalled")
    return IKResult(q, False, float(np.linalg.norm(e[:3])), ang, max_iter, "max_iter")
