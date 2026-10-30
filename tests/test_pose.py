"""Pose + relative pose on synthetic projections (exact intrinsics, Gaussian corner noise)."""

import cv2
import numpy as np
import pytest

from so101.analysis.charuco import CharucoSpec
from so101.analysis.pose import Camera, relative, rotation_delta, solve, tag_object_points

CAM = Camera(np.array([[7007.0, 0, 3527.5], [0, 7006.0, 2319.3], [0, 0, 1]]), np.array([-0.027, 0.098, 0, 0, 0.154]))


def ipad_points() -> np.ndarray:
    return CharucoSpec.load().board().getChessboardCorners().astype(float)


def project(obj, rvec, tvec, noise=0.0, rng=None):
    p = cv2.projectPoints(obj, np.asarray(rvec, float), np.asarray(tvec, float), CAM.K, CAM.dist)[0].reshape(-1, 2)
    return p + (rng.normal(0, noise, p.shape) if noise else 0)


TAG_RV, TAG_T = np.radians([160, 15, 5]), np.array([-60.0, -40.0, 720.0])  # tag tilted ~25°, facing the camera
IPAD_RV, IPAD_T = np.radians([-150, 0, 2]), np.array([-20.0, 30.0, 760.0])


def test_tag_pose_exact():
    obj = tag_object_points(20.0)
    p = solve(obj, project(obj, TAG_RV, TAG_T), CAM, 0.05, square=True)
    assert np.allclose(p.t, TAG_T, atol=1e-6)
    assert np.allclose(rotation_delta(cv2.Rodrigues(TAG_RV)[0], p.R), 0, atol=1e-8)
    assert p.ambiguity < 1e-3  # noise-free: the flipped solution can't fit as well


def test_tag_covariance_matches_monte_carlo():
    obj, rng, sigma = tag_object_points(20.0), np.random.default_rng(0), 0.05
    ts = np.array([solve(obj, project(obj, TAG_RV, TAG_T, sigma, rng), CAM, sigma, square=True).t for _ in range(400)])
    pred = np.sqrt(np.diag(solve(obj, project(obj, TAG_RV, TAG_T), CAM, sigma, square=True).cov)[3:])
    assert np.allclose(ts.std(0), pred, rtol=0.2)
    assert pred[2] > 5 * pred[:2].max()  # depth is the weak axis


def test_relative_pose_cancels_camera_motion():
    tag, ipad = tag_object_points(20.0), ipad_points()
    def rel(cam_rv, cam_t):  # move the camera: apply the same rigid motion to both targets
        Rc = cv2.Rodrigues(np.asarray(cam_rv, float))[0]
        poses = []
        for obj, rv, t, sq in ((tag, TAG_RV, TAG_T, True), (ipad, IPAD_RV, IPAD_T, False)):
            R = Rc @ cv2.Rodrigues(rv)[0]
            poses.append(solve(obj, project(obj, cv2.Rodrigues(R)[0], Rc @ t + cam_t), CAM, 0.05, square=sq))
        return relative(*poses)
    a, b = rel([0, 0, 0], [0, 0, 0]), rel(np.radians([0.3, -0.2, 0.5]), [2.0, -1.0, 5.0])
    assert np.allclose(a.t, b.t, atol=1e-5)
    assert np.allclose(rotation_delta(a.R, b.R), 0, atol=1e-8)


def test_relative_covariance_matches_monte_carlo():
    tag, ipad, rng, sigma = tag_object_points(20.0), ipad_points(), np.random.default_rng(1), 0.05
    def one(noise):
        return relative(solve(tag, project(tag, TAG_RV, TAG_T, noise, rng), CAM, sigma, square=True),
                        solve(ipad, project(ipad, IPAD_RV, IPAD_T, noise, rng), CAM, sigma))  # fmt: skip
    ref = one(0.0)
    xs = np.array([np.concatenate([rotation_delta(ref.R, p.R), p.t]) for p in (one(sigma) for _ in range(400))])
    assert np.allclose(xs.std(0), np.sqrt(np.diag(ref.cov)), rtol=0.2)


@pytest.mark.parametrize("bad", [0.0, 0.05])
def test_ipad_pose(bad):
    obj, rng = ipad_points(), np.random.default_rng(2)
    p = solve(obj, project(obj, IPAD_RV, IPAD_T, bad, rng), CAM, 0.05)
    assert np.allclose(p.t, IPAD_T, atol=0.05)
