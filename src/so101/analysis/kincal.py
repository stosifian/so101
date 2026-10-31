"""Kinematic calibration from camera observations of the gripper tag (Phase 2 step 10).

Model, per shot i:  T_cam_tag,i = R_i · X_run · F(q_i; θ) · Y, where R_i = the iPad's pose MEASURED in the same photo
(T_cam_ipad; identity if no reference), X_run = T_ipad_base, one per run (absorbs a bump of the iPad or base between
runs), Y = T_link_tag, F = forward kinematics to
the gripper body with corrections θ = FKParams: joint zero offsets `delta` (shoulder_lift, elbow_flex) and link placement twists `xi`).
Objective: the tag's 4 corner reprojection errors in pixels (as so101.analysis.handeye), plus weak priors on
θ (σ: 3° offsets, 2° / 2 mm link twists) so directions the data can't see stay at zero instead of drifting.
What a fixed camera + one tag can't separate (left out of the fit): the pan zero offset and the first link's
twist (both = a rotation/shift of X), the wrist_roll offset and the tool twist (= Y, roll never varies here).
`identifiability` reports the SVD of the prior-scaled Jacobian: singular values near zero = parameter
combinations the pose set does not determine.
Because R_i is measured per photo, camera motion (tripod creep, a bumped table) cancels exactly: everything is
tag relative to the iPad, as in steps 5b and 8 (phase-2 D1). Accuracy is reported on HELD-OUT pose groups
(k-fold): the distance between the model's predicted tag centre and the PnP-measured one, expressed along the
camera axes (mm). The PnP centre carries ~0.1 mm depth noise per shot.
Units: X, Y in mm; FK internally in metres (θ twists: rad, m).
"""

from dataclasses import dataclass, field

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from so101.analysis.handeye import T_of, fit as handeye_fit
from so101.analysis.pose import Camera, tag_object_points
from so101.kinematics.fk import CHAIN_JOINTS, Chain, FKParams, fk

OFFSET_JOINTS = [1, 2]  # shoulder_lift, elbow_flex. Not separable here: pan offset (= X), and with wrist_roll held
# constant the wrist_flex AND roll offsets (a fixed rotation before a fixed roll commutes into Y): a first fit gave a
# meaningless 35° wrist offset that held-out data didn't support. Vary roll in a grid to make them observable.
TWIST_LINKS = [1, 2, 3, 4]  # xi[i]: placement of joint i's frame (xi[0] = X); xi_tool = Y
SIG_DELTA, SIG_W, SIG_V = np.radians(3), np.radians(2), 0.002
PITCH_JOINTS = [1, 2, 3]  # shoulder_lift, elbow_flex, wrist_flex: the joints gravity loads (Shot.tau order)
SIG_COMP = 0.05  # prior on a compliance: 0.05 rad per N·m (2.9°/N·m)
TAU_KNEE = 0.75  # N·m, "stiff_thr": the shoulder_lift load above which the step 10 errors jump (chosen FROM the
# step 10 results, kincal_gravity.py: post hoc, stated as such in the report)


@dataclass
class Shot:
    group: str  # physical pose (repeats of the same commanded pose share a group)
    q: np.ndarray  # 5 joint angles (rad) from the encoder readings
    corners: np.ndarray  # (4, 2) refined px
    T_pnp: np.ndarray  # per-shot PnP pose (mm): init + accuracy reference
    label: str = ""
    run: str = ""
    T_ref: np.ndarray = field(default_factory=lambda: np.eye(4))  # T_cam_ipad measured in this photo (mm)
    tau: np.ndarray = field(default_factory=lambda: np.zeros(3))  # static gravity torque (N·m) on PITCH_JOINTS at q (model masses)


@dataclass
class Model:
    variant: str  # a key of VARIANTS
    X: dict  # run -> T_ref_base (mm); ref = iPad (or the camera, without a reference)
    Y: np.ndarray
    params: FKParams = field(default_factory=FKParams)
    tag_scale: float = 1.0  # true tag size / nominal ("full" only; measured against the arm's nominal link lengths)
    compliance: np.ndarray = field(default_factory=lambda: np.zeros(3))  # rad per N·m on PITCH_JOINTS (stiff_* only)

    def q_eff(self, s: "Shot") -> np.ndarray:
        return effective_q(s, self.variant, self.compliance)

    def T_cam_tag(self, chain: Chain, s: "Shot") -> np.ndarray:
        F = fk(chain, self.q_eff(s), self.params, tcp=np.eye(4))
        F[:3, 3] *= 1000
        return s.T_ref @ self.X[s.run] @ F @ self.Y


VARIANTS = {  # name: (zero-offset joints, link twists?, tag scale?, compliance mode)
    "nominal": ([], False, False, None),
    "offsets": (OFFSET_JOINTS, False, False, None),
    "links": (OFFSET_JOINTS, True, False, None),
    "full": ([*OFFSET_JOINTS, 3, 4], True, True, None),
    "stiff_lin": (OFFSET_JOINTS, True, False, "lin"),  # links + linear compliance: Δθ_j = c_j · τ_j (lift, elbow, wrist)
    "stiff_thr": (OFFSET_JOINTS, True, False, "thr"),  # links + Δθ_lift = c · max(0, τ_lift − TAU_KNEE)
}


def _spec(variant: str) -> tuple[list[int], bool, bool, str | None]:
    """(zero-offset joints, link twists?, tag scale?, compliance mode). "full" adds the wrist_flex and wrist_roll
    offsets, which are observable once wrist_roll varies between poses (step 10 grid), and the tag's scale."""
    return VARIANTS[variant]


def _n_comp(mode: str | None) -> int:
    return {None: 0, "lin": 3, "thr": 1}[mode]


def effective_q(s: "Shot", variant: str, compliance: np.ndarray) -> np.ndarray:
    """Encoder angles + the gravity deflection the variant models (zero for geometry-only variants)."""
    mode = _spec(variant)[3]
    if mode is None:
        return s.q
    q = s.q.copy()
    if mode == "lin":
        q[PITCH_JOINTS] += compliance * s.tau
    else:
        q[1] += compliance[0] * max(0.0, abs(s.tau[0]) - TAU_KNEE) * np.sign(s.tau[0])
    return q


def _n_theta(variant: str) -> int:
    oj, tw, sc, cm = _spec(variant)
    return len(oj) + (6 * len(TWIST_LINKS) if tw else 0) + (1 if sc else 0) + _n_comp(cm)


def _unpack(p: np.ndarray, variant: str, runs: list[str]) -> tuple[dict, np.ndarray, FKParams, float, np.ndarray]:
    nx = 6 * len(runs)
    X = {r: T_of(p[6 * k : 6 * k + 3], p[6 * k + 3 : 6 * k + 6]) for k, r in enumerate(runs)}
    Y = T_of(p[nx : nx + 3], p[nx + 3 : nx + 6])
    oj, tw, sc, cm = _spec(variant)
    fp, th = FKParams(), p[nx + 6 :]
    k = 0
    fp.delta[oj] = th[k : k + len(oj)]
    k += len(oj)
    if tw:
        fp.xi[TWIST_LINKS] = th[k : k + 6 * len(TWIST_LINKS)].reshape(len(TWIST_LINKS), 6)
        k += 6 * len(TWIST_LINKS)
    scale = 1.0
    if sc:
        scale = 1.0 + th[k]
        k += 1
    comp = np.zeros(3)
    comp[: _n_comp(cm)] = th[k : k + _n_comp(cm)]
    return X, Y, fp, scale, comp


def _T6(T: np.ndarray) -> np.ndarray:
    return np.r_[Rotation.from_matrix(T[:3, :3]).as_rotvec(), T[:3, 3]]


def _pack(m: Model, runs: list[str]) -> np.ndarray:
    oj, tw, sc, cm = _spec(m.variant)
    th = ([m.params.delta[oj]] + ([m.params.xi[TWIST_LINKS].ravel()] if tw else []) + ([[m.tag_scale - 1.0]] if sc else [])
          + ([m.compliance[: _n_comp(cm)]] if cm else []))  # fmt: skip
    return np.r_[*[_T6(m.X[r]) for r in runs], _T6(m.Y), *th]


SIG_SCALE = 0.005  # prior on the tag scale: 0.5 %


def _prior_sigmas(variant: str) -> np.ndarray:
    oj, tw, sc, cm = _spec(variant)
    return np.array([SIG_DELTA] * len(oj) + ([SIG_W, SIG_W, SIG_W, SIG_V, SIG_V, SIG_V] * len(TWIST_LINKS) if tw else [])
                    + ([SIG_SCALE] if sc else []) + [SIG_COMP] * _n_comp(cm))  # fmt: skip


def _residuals(p, shots, chain, cam, obj, variant, prior_w, runs) -> np.ndarray:
    X, Y, fp, scale, comp = _unpack(p, variant, runs)
    o = obj * scale
    out = []
    for s in shots:
        F = fk(chain, effective_q(s, variant, comp), fp, tcp=np.eye(4))
        F[:3, 3] *= 1000
        T = s.T_ref @ X[s.run] @ F @ Y
        proj = cv2.projectPoints(o, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], cam.K, cam.dist)[0].reshape(-1, 2)
        out.append((proj - s.corners).ravel())
    sig = _prior_sigmas(variant)
    return np.concatenate(out + ([prior_w * p[len(p) - len(sig) :] / sig] if len(sig) else []))


def fit(shots: list[Shot], chain: Chain, cam: Camera, tag_mm: float, variant: str, prior_w: float = 1.0) -> tuple[Model, float]:
    """Fit X (per run), Y (+ θ for the variant). Returns (model, corner RMS px on these shots). Init: the hand-eye
    fit in the camera frame, X_run = (mean measured T_cam_ref of the run)⁻¹ · T_cam_base."""
    obj = tag_object_points(tag_mm)
    runs = sorted({s.run for s in shots})
    Fs = []
    for s in shots:
        F = fk(chain, s.q, tcp=np.eye(4))
        F[:3, 3] *= 1000
        Fs.append(F)
    he = handeye_fit(Fs, [s.corners for s in shots], [s.T_pnp for s in shots], cam, tag_mm)
    X0 = {}
    for r in runs:
        Rs = [s.T_ref for s in shots if s.run == r]
        Tm = np.eye(4)
        Tm[:3, :3] = Rotation.from_matrix([T[:3, :3] for T in Rs]).mean().as_matrix()
        Tm[:3, 3] = np.mean([T[:3, 3] for T in Rs], axis=0)
        X0[r] = np.linalg.inv(Tm) @ he.X
    p0 = np.r_[_pack(Model("nominal", X0, he.Y), runs), np.zeros(_n_theta(variant))]
    r = least_squares(_residuals, p0, args=(shots, chain, cam, obj, variant, prior_w, runs), method="lm", x_scale="jac", max_nfev=5000)
    X, Y, fp, scale, comp = _unpack(r.x, variant, runs)
    n = 8 * len(shots)
    return Model(variant, X, Y, fp, scale, comp), float(np.sqrt(np.mean(r.fun[:n] ** 2)))


def errors_mm(model: Model, shots: list[Shot], chain: Chain) -> np.ndarray:
    """(n, 3) predicted − PnP-measured tag centre, camera frame (mm). The PnP pose was solved with the nominal tag
    size; its translation scales with the size, so it's rescaled by the fitted tag scale."""
    return np.array([model.T_cam_tag(chain, s)[:3, 3] - model.tag_scale * s.T_pnp[:3, 3] for s in shots])


def identifiability(shots: list[Shot], chain: Chain, cam: Camera, tag_mm: float, model: Model) -> dict:
    """Singular values of the corner-residual Jacobian w.r.t. θ (prior-scaled: a unit step = 1σ of the prior),
    with the X (per run) and Y directions projected out. Small values = combinations the data barely constrains."""
    variant = model.variant
    if variant == "nominal":
        return {}
    obj = tag_object_points(tag_mm)
    runs = sorted({s.run for s in shots})
    p = _pack(model, runs)
    sigs = _prior_sigmas(variant)
    nfix = len(p) - len(sigs)
    sig = np.r_[np.ones(nfix), sigs]
    args = (shots, chain, cam, obj, variant, 0.0, runs)
    base = _residuals(p, *args)[: 8 * len(shots)]
    J = np.zeros((len(base), len(p)))
    for k in range(len(p)):
        h = 1e-6 if k < nfix else 1e-3 * sig[k]
        d = np.zeros(len(p))
        d[k] = h
        J[:, k] = (_residuals(p + d, *args)[: 8 * len(shots)] - base) / h
    Jx, Jt = J[:, :nfix], J[:, nfix:] * sig[nfix:]
    Q, _ = np.linalg.qr(Jx)
    Jt = Jt - Q @ (Q.T @ Jt)  # remove what X, Y can absorb
    sv = np.linalg.svd(Jt, compute_uv=False)
    return {"singular_px_per_sigma": sv.tolist(), "n_params": Jt.shape[1]}


def kfold(shots: list[Shot], chain: Chain, cam: Camera, tag_mm: float, variant: str, k: int = 5, seed: int = 0) -> np.ndarray:
    """Held-out errors (n, 3) mm: pose GROUPS are split into k folds; each fold predicted by a model fitted on the rest."""
    groups = sorted({s.group for s in shots})
    rng = np.random.default_rng(seed)
    rng.shuffle(groups)
    fold = {g: i % k for i, g in enumerate(groups)}
    err = np.full((len(shots), 3), np.nan)
    for f in range(k):
        train = [s for s in shots if fold[s.group] != f]
        test = [i for i, s in enumerate(shots) if fold[s.group] == f]
        m, _ = fit(train, chain, cam, tag_mm, variant)
        err[test] = errors_mm(m, [shots[i] for i in test], chain)
    return err


def gravity_torques(arm_model, mapping, ticks: dict) -> np.ndarray:
    """Static gravity torque (N·m) on PITCH_JOINTS at the pose given by encoder ticks (MuJoCo model masses)."""
    import mujoco

    m, d = arm_model.m, arm_model.d
    d.qpos[:] = 0
    for j, v in mapping.to_q(ticks).items():
        d.qpos[arm_model.qadr[j]] = v
    d.qvel[:] = 0
    mujoco.mj_forward(m, d)
    return np.array([d.qfrc_bias[m.jnt_dofadr[mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_JOINT, CHAIN_JOINTS[i])]] for i in PITCH_JOINTS])


def angles_from_ticks(mapping, ticks: dict) -> np.ndarray:
    return np.array([mapping.joints[j].to_rad(ticks[j]) for j in CHAIN_JOINTS])
