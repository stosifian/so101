"""AprilTag edge-line refinement on synthetic tags with known corners (anti-aliased, blurred, noisy)."""

import cv2
import numpy as np
import pytest

from so101.analysis.apriltag import detect_tags

BITS = cv2.aruco.generateImageMarker(cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11), 2, 8, borderBits=1)


def render(tag_px, dx, dy, ang, tilt=0.0, blur=1.2, noise=1.0, gamma=1.0, seed=0, S=16, SH=4):
    """Tag 2 (8 x 8 cells incl. border), pixel centres at integers. tilt: a true projective tilt about x.
    Returns the image and the 4 true outer corners (TL, TR, BR, BL)."""
    rng = np.random.default_rng(seed)
    W = H = int(tag_px * 2.2)
    c, s = np.cos(ang), np.sin(ang)
    cell = tag_px / 8

    def P(u, v):
        x = np.array([u - 4, v - 4]) * cell
        x = x / (1 + tilt * x[1] / tag_px)
        return np.array([[c, -s], [s, c]]) @ x + [W / 2 + dx, H / 2 + dy]

    big = np.full((H * S, W * S), 255, np.uint8)
    for r in range(8):
        for q in range(8):
            if BITS[r, q] < 128:
                poly = np.array([P(q, r), P(q + 1, r), P(q + 1, r + 1), P(q, r + 1)])
                cv2.fillPoly(big, [np.round(((poly + 0.5) * S - 0.5) * (1 << SH)).astype(np.int32)], 0, shift=SH)
    img = cv2.resize(big, (W, H), interpolation=cv2.INTER_AREA).astype(np.float32) / 255
    img = cv2.GaussianBlur(img, (0, 0), blur)
    img = 0.1 + 0.8 * img + rng.normal(0, noise / 255, img.shape)
    img = np.clip(img, 0, 1) ** (1 / gamma)
    return (img * 255).astype(np.uint8), np.array([P(0, 0), P(8, 0), P(8, 8), P(0, 8)])


@pytest.mark.parametrize("tag_px,tilt", [(110, 0.0), (180, 0.0), (180, 0.3), (400, 0.0)])
def test_refined_corners_match_truth(tag_px, tilt):
    errs = []
    for k in range(6):
        img, true = render(tag_px, 0.1 * k - 0.3, 0.37 - 0.1 * k, 0.05 * k - 0.1, tilt=tilt, seed=k)
        d = detect_tags(img)
        assert set(d) == {2}
        errs.append(d[2].corners - true)
        assert d[2].edge_rms < 0.08
    # rasterisation grows the black by 0.5/16 px per edge, plus noise; ~0.03 px mean, ≤ 0.1 px worst corner
    assert np.abs(np.array(errs)).max() < 0.1
    assert np.abs(np.array(errs).mean(axis=0)).max() < 0.06


def test_gamma_encoded_image_biases_edges_inward():
    """Why the refinement must run on linear data: on a gamma-2.2 image the tag shrinks by ~0.4 px per edge (blur 1.2)."""
    img_lin, true = render(180, 0.2, -0.1, 0.0, blur=1.2)
    img_gam, _ = render(180, 0.2, -0.1, 0.0, blur=1.2, gamma=2.222)
    side = lambda c: np.linalg.norm(c[1] - c[0])  # noqa: E731
    grow = side(detect_tags(img_gam)[2].corners) - side(true)
    assert grow < -0.4  # two edges, each pulled inwards (the mid-level crossing moves to the dark side)
    assert abs(side(detect_tags(img_gam, linear=img_lin)[2].corners) - side(true)) < 0.1


def test_id_filter():
    img, _ = render(180, 0, 0, 0)
    assert detect_tags(img, ids=[5]) == {}
