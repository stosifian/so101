"""Target poses from one image (PnP) with first-order covariance, and the gripper tag's pose relative to the iPad.

Frames: T_cam_x maps points in frame x to the camera frame (x right, y down, z along the optical axis).
Tag frame (OpenCV ArUco convention): origin at the tag centre, x right, y up as printed, z out of the tag.
iPad frame: the ChArUco board's (origin at the top-left square corner, mm with the display pitch, nominal
until step 3). Relative pose T_ipad_tag = T_cam_ipad⁻¹ T_cam_tag: what the camera's own motion cancels from.

Covariance: corner noise σ (px, isotropic, independent) → Cov(rvec, tvec) = σ² (JᵀJ)⁻¹ at the solution, J from
cv2.projectPoints. Intrinsics are held fixed (they are common to every frame of a run, so they bias the
absolute pose, not its change over time). Relative-pose covariance by numerical propagation through the
composition, the two detections treated as independent. Uncertainty is anisotropic: depth along the
line of sight is ~f/size worse than lateral, and a frame change rotates that ellipsoid, so keep full covariances.

A single square tag has two IPPE solutions (the tilt flip). Both are polished with LM and the one with the
lower reprojection error wins; `ambiguity` = best/second error ratio (near 1 = the flip is unresolved).
LeRobot has no camera-pose code; nothing to copy.
"""

import json
from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np

from so101.config import CONFIG_DIR

INTRINSICS = CONFIG_DIR / "intrinsics_a7iv.json"  # step 4 result (user-accepted 2026-10-28)
CALIBRATED_MAX_Y = 3700  # no calibration views below this image row (step 4)


@dataclass(frozen=True)
class Camera:
    K: np.ndarray
    dist: np.ndarray  # k1 k2 p1 p2 k3

    @classmethod
    def load(cls, path: Path = INTRINSICS) -> "Camera":
        d = json.loads(Path(path).read_text())
        return cls(np.array(d["K"], float), np.array([d["dist"][k] for k in ("k1", "k2", "p1", "p2", "k3")], float))


@dataclass
class Pose:
    R: np.ndarray  # (3, 3) T_cam_x rotation
    t: np.ndarray  # (3,) mm
    cov: np.ndarray  # (6, 6) over (rotation rad, t mm): rvec for `solve`, a small rotation in the pose's own axes for `relative`
    rms_px: float  # reprojection RMS
    ambiguity: float = 0.0  # IPPE: best/second reprojection error (tags only)

    @property
    def rvec(self) -> np.ndarray:
        return cv2.Rodrigues(self.R)[0].ravel()

    def matrix(self) -> np.ndarray:
        T = np.eye(4)
        T[:3, :3], T[:3, 3] = self.R, self.t
        return T


def tag_object_points(size_mm: float) -> np.ndarray:
    """Outer black-border corners in the tag frame, in OpenCV's order (TL, TR, BR, BL), as SOLVEPNP_IPPE_SQUARE wants."""
    h = size_mm / 2
    return np.array([[-h, h, 0], [h, h, 0], [h, -h, 0], [-h, -h, 0]], float)


def _reproj(obj: np.ndarray, img: np.ndarray, rvec: np.ndarray, tvec: np.ndarray, cam: Camera) -> tuple[np.ndarray, np.ndarray]:
    p, J = cv2.projectPoints(obj, rvec, tvec, cam.K, cam.dist)
    return p.reshape(-1, 2) - img, J[:, :6]


def solve(obj: np.ndarray, img: np.ndarray, cam: Camera, sigma_px: float, square: bool = False) -> Pose:
    """PnP for a planar target. square=True: obj from `tag_object_points`, both IPPE solutions tried."""
    obj, img = np.asarray(obj, np.float64), np.asarray(img, np.float64)
    flag = cv2.SOLVEPNP_IPPE_SQUARE if square else cv2.SOLVEPNP_IPPE
    n, rvecs, tvecs, _ = cv2.solvePnPGeneric(obj, img, cam.K, cam.dist, flags=flag)
    cands = []
    for rv, tv in zip(rvecs[:n], tvecs[:n]):
        rv, tv = cv2.solvePnPRefineLM(obj, img, cam.K, cam.dist, rv.copy(), tv.copy(),
                                      (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_COUNT, 100, 1e-12))  # fmt: skip
        r, _ = _reproj(obj, img, rv, tv, cam)
        cands.append((float(np.sqrt((r**2).sum(1).mean())), rv.ravel(), tv.ravel()))
    cands.sort(key=lambda c: c[0])
    rms, rv, tv = cands[0]
    _, J = _reproj(obj, img, rv, tv, cam)
    cov = sigma_px**2 * np.linalg.inv(J.T @ J)
    amb = cands[0][0] / cands[1][0] if len(cands) > 1 and cands[1][0] > 0 else 0.0
    return Pose(cv2.Rodrigues(rv)[0], tv, cov, rms, amb)


def _compose(p_tag: np.ndarray, p_ref: np.ndarray) -> np.ndarray:
    """(rvec, t) of T_ref_tag = T_cam_ref⁻¹ T_cam_tag, from the two camera-frame poses as 6-vectors."""
    Rt, Rr = cv2.Rodrigues(p_tag[:3])[0], cv2.Rodrigues(p_ref[:3])[0]
    R = Rr.T @ Rt
    return np.concatenate([cv2.Rodrigues(R)[0].ravel(), Rr.T @ (p_tag[3:] - p_ref[3:])])


def relative(tag: Pose, ref: Pose, eps: float = 1e-6) -> Pose:
    """T_ref_tag with covariance propagated from both poses (independent detections)."""
    a, b = np.concatenate([tag.rvec, tag.t]), np.concatenate([ref.rvec, ref.t])
    f0 = _compose(a, b)
    R0 = cv2.Rodrigues(f0[:3])[0]

    def f(x: np.ndarray) -> np.ndarray:  # rotation as a small delta in R0's axes, so σ reads directly as angles
        g = _compose(x[:6], x[6:])
        return np.concatenate([rotation_delta(R0, cv2.Rodrigues(g[:3])[0]), g[3:]])

    x0, J = np.concatenate([a, b]), np.zeros((6, 12))
    for k in range(12):
        d = np.zeros(12)
        d[k] = eps
        J[:, k] = (f(x0 + d) - f(x0 - d)) / (2 * eps)
    C = np.zeros((12, 12))
    C[:6, :6], C[6:, 6:] = tag.cov, ref.cov
    return Pose(R0, f0[3:], J @ C @ J.T, float("nan"))


def rotation_delta(R0: np.ndarray, R: np.ndarray) -> np.ndarray:
    """Small rotation from R0 to R as a rotation vector (rad) in R0's own axes. (Not cv2.Rodrigues: it rounds
    rotations below ~1e-5 rad to zero, which kills finite-difference Jacobians.)"""
    from scipy.spatial.transform import Rotation

    return Rotation.from_matrix(R0.T @ R).as_rotvec()
