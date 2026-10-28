"""Camera intrinsics from ChArUco views (Phase 2 step 4).

`calibrate` = OpenCV pinhole + k1 k2 p1 p2 k3 (calibrateCameraExtended: per-view RMS + parameter std devs).
`residual_figure` draws what moiré/glare would show up in: detected-vs-reprojected residual vectors across
the image, residuals by position on the board (a display-pixel beat pattern repeats in board coordinates),
per-view RMS, and corner coverage of the frame.
"""

from dataclasses import dataclass

import cv2
import numpy as np

from so101.analysis.charuco import Detection

DIST_NAMES = ["k1", "k2", "p1", "p2", "k3"]


@dataclass
class Intrinsics:
    image_size: tuple[int, int]  # (width, height)
    K: np.ndarray
    dist: np.ndarray
    rms_px: float
    per_view_rms: np.ndarray
    std: dict  # parameter -> 1-sigma
    views: list[str]
    residuals: list[np.ndarray]  # per view (N, 2): detected - reprojected
    image_points: list[np.ndarray]
    object_points: list[np.ndarray]

    def as_dict(self) -> dict:
        return {"image_size": list(self.image_size), "K": self.K.tolist(), "dist": dict(zip(DIST_NAMES, self.dist.ravel().tolist())),
                "fx": self.K[0, 0], "fy": self.K[1, 1], "cx": self.K[0, 2], "cy": self.K[1, 2], "rms_px": self.rms_px,
                "std": self.std, "views": [{"image": v, "rms_px": float(e), "n_corners": int(len(p))}
                                           for v, e, p in zip(self.views, self.per_view_rms, self.image_points)]}  # fmt: skip


def calibrate(dets: dict[str, Detection], board: cv2.aruco.CharucoBoard, image_size: tuple[int, int],
              min_corners: int = 12) -> Intrinsics:  # fmt: skip
    names, obj, img = [], [], []
    for name, d in dets.items():
        if d is None or len(d.ids) < min_corners:
            continue
        o, i = board.matchImagePoints(d.corners.reshape(-1, 1, 2).astype(np.float32), d.ids.reshape(-1, 1))
        names.append(name)
        obj.append(o.reshape(-1, 3).astype(np.float32))
        img.append(i.reshape(-1, 2).astype(np.float32))
    if len(names) < 5:
        raise ValueError(f"need >= 5 usable views, have {len(names)}")
    crit = (cv2.TERM_CRITERIA_COUNT + cv2.TERM_CRITERIA_EPS, 200, 1e-12)
    rms, K, dist, rvecs, tvecs, std_in, _, per_view = cv2.calibrateCameraExtended(
        obj, img, image_size, None, None, criteria=crit)
    res = []
    for o, i, rv, tv in zip(obj, img, rvecs, tvecs):
        p, _ = cv2.projectPoints(o, rv, tv, K, dist)
        res.append(i - p.reshape(-1, 2))
    std = dict(zip(["fx", "fy", "cx", "cy", *DIST_NAMES], std_in.ravel()[:9].tolist()))
    return Intrinsics(image_size, K, dist.ravel(), float(rms), per_view.ravel(), std, names, res, img, obj)


def residual_figure(cal: Intrinsics, square_size: float, path, arrow_scale: float = 2000.0) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    W, H = cal.image_size
    pts = np.vstack(cal.image_points)
    res = np.vstack(cal.residuals)
    obj = np.vstack(cal.object_points)
    fig, ax = plt.subplots(2, 2, figsize=(14, 10))
    a = ax[0, 0]
    a.quiver(pts[:, 0], pts[:, 1], res[:, 0], res[:, 1], np.linalg.norm(res, axis=1), angles="xy", scale_units="xy",
             scale=1 / arrow_scale, cmap="viridis", width=0.0015)  # fmt: skip
    a.set(xlim=(0, W), ylim=(H, 0), aspect="equal", title=f"residuals in the image (arrows ×{arrow_scale:g})")
    a = ax[0, 1]
    # per-corner residuals pooled over views: moiré/glare stuck to the board shows as hot corners
    corner_idx = np.round(obj[:, :2] / square_size).astype(int)
    sc = a.scatter(corner_idx[:, 0] + 0.15 * np.random.default_rng(0).standard_normal(len(obj)),
                   corner_idx[:, 1] + 0.15 * np.random.default_rng(1).standard_normal(len(obj)),
                   c=np.linalg.norm(res, axis=1), s=6, cmap="magma")  # fmt: skip
    fig.colorbar(sc, ax=a, label="|residual| px")
    a.set(aspect="equal", title="|residual| by corner on the board (all views)", xlabel="corner x", ylabel="corner y")
    a.invert_yaxis()
    a = ax[1, 0]
    order = np.argsort(cal.views)
    a.bar(range(len(order)), cal.per_view_rms[order])
    a.axhline(cal.rms_px, color="k", lw=0.8, ls="--", label=f"overall {cal.rms_px:.3f} px")
    a.set(title="RMS per view", xlabel="view", ylabel="px")
    a.legend()
    a = ax[1, 1]
    a.scatter(pts[:, 0], pts[:, 1], s=2)
    a.set(xlim=(0, W), ylim=(H, 0), aspect="equal", title=f"coverage: {len(pts)} corners in {len(cal.views)} views")
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def coverage_grid(points: np.ndarray, image_size: tuple[int, int], nx: int = 6, ny: int = 4) -> np.ndarray:
    """Count of corners per cell of an nx × ny grid over the frame (row 0 = top)."""
    W, H = image_size
    g = np.zeros((ny, nx), int)
    if len(points):
        ix = np.clip((points[:, 0] / W * nx).astype(int), 0, nx - 1)
        iy = np.clip((points[:, 1] / H * ny).astype(int), 0, ny - 1)
        np.add.at(g, (iy, ix), 1)
    return g
