"""Fixed camera, tag on the gripper: fit where the camera is (T_cam_base) and where the tag sits on the gripper body
(T_link_tag) from poses with joint readings + a tag detection. Phase 2 step 6 (planning) and a preview of step 10.

Model, per pose i:  T_cam_tag,i = X · F(q_i) · Y,  X = T_cam_base, Y = T_link_tag, F = forward kinematics to the
gripper (last chain) body (nominal kinematics; step 10 fits those too). Fit: Levenberg-Marquardt on the tag's
4 corner REPROJECTION errors in pixels (no per-pose PnP needed; noise is where it's measured, so soft poses
weigh what their pixels weigh). Initialisation: Y's rotation from a grid of axis-aligned candidates (each gives
X_i = T_cam_tag,i Y⁻¹ F_i⁻¹; the most consistent X_i set wins), translations zero, then LM.
Units: mm and rad. Residuals contain the kinematic model's errors, so they are an upper bound on detection noise.
LeRobot has no hand-eye calibration; OpenCV's calibrateRobotWorldHandEye solves the linear form, the
pixel-space LM here is what we report.
"""

from dataclasses import dataclass
from itertools import permutations, product

import cv2
import numpy as np
from scipy.optimize import least_squares
from scipy.spatial.transform import Rotation

from so101.analysis.pose import Camera, tag_object_points


def T_of(rvec, t) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = Rotation.from_rotvec(rvec).as_matrix()
    T[:3, 3] = t
    return T


def inv(T: np.ndarray) -> np.ndarray:
    R, t = T[:3, :3], T[:3, 3]
    out = np.eye(4)
    out[:3, :3], out[:3, 3] = R.T, -R.T @ t
    return out


@dataclass
class HandEye:
    X: np.ndarray  # T_cam_base (mm)
    Y: np.ndarray  # T_link_tag (mm)
    rms_px: float
    per_pose_px: np.ndarray  # RMS corner error per pose
    tag_mm: float

    def T_cam_tag(self, F: np.ndarray) -> np.ndarray:
        return self.X @ F @ self.Y

    def predict(self, F: np.ndarray, cam: Camera) -> dict:
        """What the camera sees for gripper-body pose F (base frame, mm): corners (px), distance (mm), tilt of the
        tag normal to the line of sight (deg; 90 = edge-on, > 90 = facing away), facing (bool)."""
        T = self.T_cam_tag(F)
        obj = tag_object_points(self.tag_mm)
        px = cv2.projectPoints(obj, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], cam.K, cam.dist)[0].reshape(-1, 2)
        ray = T[:3, 3] / np.linalg.norm(T[:3, 3])
        n = T[:3, 2]  # tag z: out of the printed face, towards a camera that sees it
        tilt = float(np.degrees(np.arccos(np.clip(-n @ ray, -1, 1))))
        return {"corners_px": px, "distance_mm": float(np.linalg.norm(T[:3, 3])), "tilt_deg": tilt, "facing": tilt < 90}

    def as_dict(self) -> dict:
        cam_in_base = inv(self.X)
        return {"T_cam_base": self.X.tolist(), "T_link_tag": self.Y.tolist(), "rms_px": self.rms_px,
                "per_pose_px": self.per_pose_px.tolist(), "tag_mm": self.tag_mm,
                "camera_position_in_base_mm": cam_in_base[:3, 3].tolist(),
                "camera_optical_axis_in_base": cam_in_base[:3, 2].tolist(),
                "tag_offset_on_link_mm": self.Y[:3, 3].tolist()}  # fmt: skip


def _corner_residuals(p: np.ndarray, Fs, corners, cam: Camera, obj: np.ndarray) -> np.ndarray:
    X, Y = T_of(p[0:3], p[3:6]), T_of(p[6:9], p[9:12])
    out = []
    for F, c in zip(Fs, corners):
        T = X @ F @ Y
        proj = cv2.projectPoints(obj, cv2.Rodrigues(T[:3, :3])[0], T[:3, 3], cam.K, cam.dist)[0].reshape(-1, 2)
        out.append((proj - c).ravel())
    return np.concatenate(out)


def _axis_rotations() -> list[np.ndarray]:
    """The 24 proper rotations that map axes onto ±axes."""
    out = []
    for perm in permutations(range(3)):
        for signs in product((1, -1), repeat=3):
            R = np.zeros((3, 3))
            for i, (j, s) in enumerate(zip(perm, signs)):
                R[i, j] = s
            if np.linalg.det(R) > 0:
                out.append(R)
    return out


def fit(Fs: list[np.ndarray], corners: list[np.ndarray], T_cam_tags: list[np.ndarray], cam: Camera,
        tag_mm: float) -> HandEye:  # fmt: skip
    """Fs: gripper-body poses (base frame, mm); corners: (4, 2) refined tag corners per pose; T_cam_tags: per-pose
    PnP poses (initialisation only)."""
    obj = tag_object_points(tag_mm)
    best = None
    for Ry in _axis_rotations():
        Y = np.eye(4)
        Y[:3, :3] = Ry
        Xs = [Tc @ inv(Y) @ inv(F) for Tc, F in zip(T_cam_tags, Fs)]
        Rm = Rotation.from_matrix([x[:3, :3] for x in Xs]).mean().as_matrix()
        spread = np.mean([np.linalg.norm(Rotation.from_matrix(Rm.T @ x[:3, :3]).as_rotvec()) for x in Xs])
        if best is None or spread < best[0]:
            best = (spread, Ry, Rm, np.mean([x[:3, 3] for x in Xs], axis=0))
    _, Ry, Rx, tx = best
    p0 = np.r_[Rotation.from_matrix(Rx).as_rotvec(), tx, Rotation.from_matrix(Ry).as_rotvec(), np.zeros(3)]
    r = least_squares(_corner_residuals, p0, args=(Fs, corners, cam, obj), method="lm", x_scale="jac", max_nfev=20000)
    res = r.fun.reshape(len(Fs), 8)
    per = np.sqrt((res**2).mean(axis=1))
    return HandEye(T_of(r.x[0:3], r.x[3:6]), T_of(r.x[6:9], r.x[9:12]), float(np.sqrt((r.fun**2).mean())), per, tag_mm)
