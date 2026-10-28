import cv2
import numpy as np

from so101.analysis.charuco import CharucoSpec, detect, render
from so101.analysis.intrinsics import calibrate, coverage_grid

SPEC = CharucoSpec.load("ipad_charuco")


def test_render_is_pixel_exact_and_detects_every_corner():
    img = render(SPEC)
    assert img.shape == (1640, 2360) and set(np.unique(img)) == {0, 255}
    b = SPEC.board("px")
    ref = b.generateImage((SPEC.squares_x * SPEC.square_px, SPEC.squares_y * SPEC.square_px), marginSize=0, borderBits=1)
    ox, oy = SPEC.offset_px
    assert (img[oy : oy + ref.shape[0], ox : ox + ref.shape[1]] == ref).all()
    d = detect(img, b)
    assert len(d.ids) == SPEC.n_corners == 96
    o, i = b.matchImagePoints(d.corners.reshape(-1, 1, 2).astype(np.float32), d.ids.reshape(-1, 1))
    err = i.reshape(-1, 2) - (o.reshape(-1, 3)[:, :2] + [ox, oy])
    assert np.allclose(err, -0.5, atol=1e-3)  # OpenCV pixel-centre convention, otherwise exact


def synthetic_views(K, dist, size, n=12, seed=0):
    """Warp the rendered board into a pinhole+distortion camera at random tilted poses."""
    rng = np.random.default_rng(seed)
    screen = render(SPEC)
    p = SPEC.pitch_mm
    W, H = size
    # dense remap: for every camera pixel, undistort → ray → hit the board plane → screen pixel
    u, v = np.meshgrid(np.arange(W, dtype=np.float32), np.arange(H, dtype=np.float32))
    rays = cv2.undistortPoints(np.stack([u.ravel(), v.ravel()], 1).reshape(-1, 1, 2), K, dist).reshape(-1, 2)
    rays = np.hstack([rays, np.ones((len(rays), 1))])
    ox, oy = SPEC.offset_px
    views = []
    for _ in range(n):
        rvec = rng.uniform(-0.5, 0.5, 3) * [1, 1, 0.3]
        R, _ = cv2.Rodrigues(rvec)
        c = np.array([SPEC.squares_x * SPEC.square_mm, SPEC.squares_y * SPEC.square_mm, 0]) / 2
        t = np.array([rng.uniform(-60, 60), rng.uniform(-40, 40), rng.uniform(380, 520)]) - R @ c
        nrm = R[:, 2]
        s = (nrm @ t) / (rays @ nrm)
        X = (rays * s[:, None] - t) @ R  # board-frame mm
        mx = (X[:, 0] / p + ox - 0.5).reshape(H, W).astype(np.float32)
        my = (X[:, 1] / p + oy - 0.5).reshape(H, W).astype(np.float32)
        img = cv2.remap(screen, mx, my, cv2.INTER_LINEAR, borderValue=255)
        views.append(cv2.GaussianBlur(img, (0, 0), 1.0))
    return views


def test_calibration_recovers_synthetic_intrinsics():
    size = (1600, 1100)
    K = np.array([[1500.0, 0, 805.0], [0, 1500.0, 545.0], [0, 0, 1]])
    dist = np.array([-0.08, 0.05, 0.0, 0.0, 0.0])
    b = SPEC.board()
    dets = {f"v{i}": detect(img, b) for i, img in enumerate(synthetic_views(K, dist, size))}
    cal = calibrate(dets, b, size)
    assert cal.rms_px < 0.5  # synthetic images alias (bilinear sampling of a 1-bit screen at ~0.3 px per display px)
    assert abs(cal.K[0, 0] / 1500 - 1) < 0.01 and abs(cal.K[1, 1] / 1500 - 1) < 0.01
    assert abs(cal.K[0, 2] - 805) < 5 and abs(cal.K[1, 2] - 545) < 5
    assert abs(cal.dist[0] - dist[0]) < 0.02


def test_coverage_grid():
    g = coverage_grid(np.array([[0, 0], [99, 99], [10, 99]]), (100, 100), nx=2, ny=2)
    assert g.tolist() == [[1, 0], [1, 1]]
