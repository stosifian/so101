"""AprilTag (36h11) detection on our RAW images, with edge-line corner refinement.

OpenCV's ArUco detector finds and decodes the tags; its corner refinements are biased for an outer tag corner
(a black square corner on white, not a checkerboard saddle): SUBPIX pulls each corner ~0.17 px inwards along
both axes (a scale bias of ~0.2 % on a 180 px tag), APRILTAG returns pixel-corner coordinates (+0.5 px).
`refine_corners` instead fits a straight line to each outer edge of the black border (sub-pixel mid-level
crossings along the edge normal) and intersects adjacent lines: unbiased for symmetric blur, and every edge
pixel contributes, not just the corner neighbourhood. Straight edges are exact under perspective; lens
distortion bends a 180 px edge by < 0.01 px here. LeRobot has no fiducial code; nothing to copy.

Corner order (OpenCV): top-left, top-right, bottom-right, bottom-left of the tag as printed. Coordinates are
image px with pixel centres at integers (OpenCV convention). Tag size = outer edge of the black border.
"""

from dataclasses import dataclass

import cv2
import numpy as np

FAMILIES = {"tag36h11": cv2.aruco.DICT_APRILTAG_36h11}


@dataclass
class TagDetection:
    id: int
    corners: np.ndarray  # (4, 2) refined, image px
    raw: np.ndarray  # (4, 2) OpenCV's unrefined corners
    edge_rms: float  # RMS of edge points about their fitted lines (px): flatness/print/blur check


def detect_tags(gray: np.ndarray, family: str = "tag36h11", ids: list[int] | None = None,
                linear: np.ndarray | None = None) -> dict[int, TagDetection]:  # fmt: skip
    """All tags of `family` (optionally only `ids`), each refined. A tag whose edges can't be fitted is dropped.
    gray: 8-bit image for detection. linear: the same image proportional to light (`charuco.load_linear`) for the
    refinement; always pass it for RAW images: refining on gamma-encoded gray shrinks the tag by 0.2-0.6 px per
    edge depending on blur, i.e. a pose- and focus-dependent scale (depth) bias."""
    p = cv2.aruco.DetectorParameters()
    p.cornerRefinementMethod = cv2.aruco.CORNER_REFINE_NONE
    det = cv2.aruco.ArucoDetector(cv2.aruco.getPredefinedDictionary(FAMILIES[family]), p)
    cs, found, _ = det.detectMarkers(gray)
    out = {}
    for c, i in zip(cs, [] if found is None else found.ravel()):
        if ids is not None and int(i) not in ids:
            continue
        raw = c.reshape(4, 2).astype(float)
        r = refine_corners(gray if linear is None else linear, raw)
        if r is not None:
            out[int(i)] = TagDetection(int(i), r[0], raw, r[1])
    return out


def _bilinear(img: np.ndarray, x: np.ndarray, y: np.ndarray) -> np.ndarray:
    x0, y0 = np.floor(x).astype(int), np.floor(y).astype(int)
    fx, fy = x - x0, y - y0
    x0, y0 = np.clip(x0, 0, img.shape[1] - 2), np.clip(y0, 0, img.shape[0] - 2)
    a, b = img[y0, x0], img[y0, x0 + 1]
    c, d = img[y0 + 1, x0], img[y0 + 1, x0 + 1]
    return (a * (1 - fx) + b * fx) * (1 - fy) + (c * (1 - fx) + d * fx) * fy


def refine_corners(gray: np.ndarray, corners: np.ndarray, n_samples: int = 40, end_frac: float = 0.12,
                   half_width: float | None = None) -> tuple[np.ndarray, float] | None:  # fmt: skip
    """Edge-line refinement of a tag's 4 outer corners. Returns (corners (4, 2), edge RMS px) or None.

    For each edge: sample points along it (skipping `end_frac` at both ends, where the neighbouring edge
    bends the profile), take an intensity profile along the normal (±half_width, default 0.4 cell so it
    stays inside the black border and the white quiet zone), and locate the crossing of the level halfway
    between the dark and light plateaus. A total-least-squares line through the crossings is the edge."""
    img = gray.astype(np.float32)
    side = np.mean([np.linalg.norm(corners[k] - corners[(k + 1) % 4]) for k in range(4)])
    hw = half_width if half_width is not None else 0.4 * side / 8
    if hw < 3:
        return None
    centre = corners.mean(axis=0)
    lines, resid = [], []
    offs = np.linspace(-hw, hw, int(np.ceil(2 * hw * 4)) + 1)  # 0.25 px steps
    for k in range(4):
        a, b = corners[k], corners[(k + 1) % 4]
        t = (b - a) / np.linalg.norm(b - a)
        n = np.array([-t[1], t[0]])
        if np.dot(centre - (a + b) / 2, n) > 0:
            n = -n  # normal points outwards (dark -> light)
        pts = []
        for s in np.linspace(end_frac, 1 - end_frac, n_samples):
            p0 = a + s * (b - a)
            prof = _bilinear(img, p0[0] + offs * n[0], p0[1] + offs * n[1])
            q = max(2, len(prof) // 5)
            lo, hi = prof[:q].mean(), prof[-q:].mean()
            if hi - lo < 0.3 * hi:  # weak contrast (occluded, glare); relative, so 8-bit and linear both work
                continue
            mid = (lo + hi) / 2
            j = np.nonzero((prof[:-1] < mid) & (prof[1:] >= mid))[0]
            if len(j) != 1:  # noise or a second edge in the window
                continue
            j = j[0]
            u = offs[j] + (mid - prof[j]) / (prof[j + 1] - prof[j]) * (offs[j + 1] - offs[j])
            pts.append(p0 + u * n)
        if len(pts) < n_samples // 2:
            return None
        pts = np.array(pts)
        m = pts.mean(axis=0)
        _, _, vt = np.linalg.svd(pts - m)
        d = vt[0]
        nrm = np.array([-d[1], d[0]])
        resid.append((pts - m) @ nrm)
        lines.append((m, d))
    out = np.empty((4, 2))
    for k in range(4):  # corner k = edge (k-1) ∩ edge k
        (m1, d1), (m2, d2) = lines[k - 1], lines[k]
        st = np.linalg.solve(np.column_stack([d1, -d2]), m2 - m1)
        out[k] = m1 + st[0] * d1
    return out, float(np.sqrt(np.mean(np.concatenate(resid) ** 2)))
