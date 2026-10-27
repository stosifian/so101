"""Parameterized forward kinematics for the SO-101 (step 8a). Math: docs/fk_math.md.

Local product-of-exponentials on the MJCF chain:
    T_{i-1,i}(q_i; theta) = T_i^fix  exp([xi_i])  Rot_z(q_i + delta_i)
    T_tool(q; theta)      = prod_i T_{i-1,i}  T_site  exp([xi_tool])
All parameters zero = the MuJoCo model exactly. Units: radians in, metres out (kinematics/ only).
Also exports the equivalent space-frame PoE (screws S_i, home pose M) for comparison and teaching.
"""

from dataclasses import dataclass, field

import mujoco
import numpy as np

from so101.kinematics.model import JOINTS

CHAIN_JOINTS = JOINTS[:5]  # the gripper joint moves only the moving jaw, not the tool site
TOOL_SITE = "gripperframe"


# --- SO(3) / SE(3) helpers ---------------------------------------------------------------------


def skew(w: np.ndarray) -> np.ndarray:
    return np.array([[0.0, -w[2], w[1]], [w[2], 0.0, -w[0]], [-w[1], w[0], 0.0]])


def quat_to_R(q: np.ndarray) -> np.ndarray:
    w, x, y, z = np.asarray(q, float) / np.linalg.norm(q)
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - w * z), 2 * (x * z + w * y)],
        [2 * (x * y + w * z), 1 - 2 * (x * x + z * z), 2 * (y * z - w * x)],
        [2 * (x * z - w * y), 2 * (y * z + w * x), 1 - 2 * (x * x + y * y)],
    ])  # fmt: skip


def homog(R: np.ndarray, p: np.ndarray) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3], T[:3, 3] = R, p
    return T


def rot_z(theta: float) -> np.ndarray:
    c, s = np.cos(theta), np.sin(theta)
    T = np.eye(4)
    T[:2, :2] = [[c, -s], [s, c]]
    return T


def se3_exp(xi: np.ndarray) -> np.ndarray:
    """exp of the twist xi = (omega, v): Rodrigues for the rotation, left Jacobian J(omega) for the translation."""
    w, v = np.asarray(xi[:3], float), np.asarray(xi[3:], float)
    phi = np.linalg.norm(w)
    W = skew(w)
    if phi < 1e-8:  # Taylor: avoid 0/0
        R, J = np.eye(3) + W + W @ W / 2, np.eye(3) + W / 2 + W @ W / 6
    else:
        a, b, c = np.sin(phi) / phi, (1 - np.cos(phi)) / phi**2, (phi - np.sin(phi)) / phi**3
        R, J = np.eye(3) + a * W + b * W @ W, np.eye(3) + b * W + c * W @ W
    return homog(R, J @ v)


def adjoint(T: np.ndarray) -> np.ndarray:
    """Ad_T for twists ordered (omega, v)."""
    R, p = T[:3, :3], T[:3, 3]
    A = np.zeros((6, 6))
    A[:3, :3], A[3:, 3:], A[3:, :3] = R, R, skew(p) @ R
    return A


# --- chain + parameters ------------------------------------------------------------------------


@dataclass(frozen=True)
class Chain:
    fixed: list[np.ndarray]  # T_i^fix, 4x4, for CHAIN_JOINTS
    site: np.ndarray  # T_site, 4x4, in the last joint's body frame


@dataclass
class FKParams:
    delta: np.ndarray = field(default_factory=lambda: np.zeros(5))  # joint zero offsets, rad
    xi: np.ndarray = field(default_factory=lambda: np.zeros((5, 6)))  # link placement corrections (omega, v)
    xi_tool: np.ndarray = field(default_factory=lambda: np.zeros(6))  # tool transform correction


def chain_from_mujoco(m: mujoco.MjModel) -> Chain:
    fixed = []
    for j in CHAIN_JOINTS:
        jid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, j)
        b = m.jnt_bodyid[jid]
        if np.any(m.jnt_pos[jid]) or not np.allclose(m.jnt_axis[jid], [0, 0, 1]) or m.qpos0[m.jnt_qposadr[jid]] != 0:
            raise ValueError(f"{j}: expected joint at body origin, axis z, ref 0 (docs/fk_math.md)")
        fixed.append(homog(quat_to_R(m.body_quat[b]), m.body_pos[b].copy()))
    sid = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_SITE, TOOL_SITE)
    last_body = m.jnt_bodyid[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, CHAIN_JOINTS[-1])]
    if m.site_bodyid[sid] != last_body:
        raise ValueError(f"{TOOL_SITE} must sit on the {CHAIN_JOINTS[-1]} body")
    return Chain(fixed, homog(quat_to_R(m.site_quat[sid]), m.site_pos[sid].copy()))


def frames(chain: Chain, q, params: FKParams | None = None) -> tuple[list[np.ndarray], np.ndarray]:
    """Joint frames T_{0,i} (each AFTER its fixed transform and correction, BEFORE its rotation, so the
    frame's z-axis is joint i's axis) and the tool pose. q: 5 angles (rad), CHAIN_JOINTS order."""
    p = params or FKParams()
    T = np.eye(4)
    joint_frames = []
    for i in range(5):
        T = T @ chain.fixed[i] @ se3_exp(p.xi[i])
        joint_frames.append(T)
        T = T @ rot_z(q[i] + p.delta[i])
    return joint_frames, T @ chain.site @ se3_exp(p.xi_tool)


def fk(chain: Chain, q, params: FKParams | None = None) -> np.ndarray:
    """Tool pose T_base->tool (4x4) for joint angles q (5, rad)."""
    return frames(chain, q, params)[1]


def geometric_jacobian(chain: Chain, q, params: FKParams | None = None) -> np.ndarray:
    """6x5: rows (linear; angular) velocity of the tool point in the base frame per joint rate."""
    jf, T = frames(chain, q, params)
    p = T[:3, 3]
    J = np.zeros((6, 5))
    for i, F in enumerate(jf):
        z, o = F[:3, 2], F[:3, 3]
        J[:3, i], J[3:, i] = np.cross(z, p - o), z
    return J


# --- space-frame product of exponentials (equivalent form) -------------------------------------


def space_screws(chain: Chain, params: FKParams | None = None) -> tuple[np.ndarray, np.ndarray]:
    """(S, M): S[i] = Ad_{T_{0,i}(0)} (0,0,1, 0,0,0), M = T_tool(0). Includes params (delta shifts the home)."""
    p = params or FKParams()
    jf, M = frames(chain, -p.delta, p)  # q = -delta puts every joint at its geometric zero
    S = np.array([adjoint(F) @ np.array([0, 0, 1, 0, 0, 0.0]) for F in jf])
    return S, M


def fk_space(S: np.ndarray, M: np.ndarray, q) -> np.ndarray:
    """Space-frame PoE: T = exp([S1] q1) ... exp([S5] q5) M. With space_screws(chain, params), pass the
    geometric angles q + params.delta (the zero offsets are folded into the home pose)."""
    T = np.eye(4)
    for i in range(len(S)):
        T = T @ se3_exp(S[i] * q[i])
    return T @ M
