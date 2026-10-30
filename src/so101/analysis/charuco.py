"""ChArUco boards in display pixels (the iPad target, Phase 2 D1) + detection on our RAW images.

The board spec lives in config/targets/<name>.yaml and is the single source of truth for generation and
detection (never hard-code sizes). Units: `square_px`/`marker_px` are DISPLAY pixels; mm come from the
display pitch, which is nominal until the caliper check (D2/D4). Intrinsics do not depend on that scale.

Image frame: rawpy's full visible frame (7028 x 4688 for the A7 IV), no lens corrections. The principal
point is defined in this frame, so every later step must decode with `load_gray`.
"""

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml

from so101.config import REPO_ROOT

TARGETS_DIR = REPO_ROOT / "config" / "targets"


@dataclass(frozen=True)
class CharucoSpec:
    name: str
    dictionary: str
    squares_x: int
    squares_y: int
    square_px: int
    marker_px: int
    screen_px: tuple[int, int]  # (width, height) of the display
    offset_px: tuple[int, int]  # board top-left on the screen
    pitch_mm: float  # display pixel pitch (nominal unless `pitch_measured`)
    pitch_measured: bool = False

    @classmethod
    def load(cls, name: str = "ipad_charuco") -> "CharucoSpec":
        d = yaml.safe_load((TARGETS_DIR / f"{name}.yaml").read_text())
        return cls(name=name, dictionary=d["dictionary"], squares_x=d["squares_x"], squares_y=d["squares_y"],
                   square_px=d["square_px"], marker_px=d["marker_px"], screen_px=tuple(d["screen_px"]),
                   offset_px=tuple(d["offset_px"]), pitch_mm=d["pitch_mm"], pitch_measured=d.get("pitch_measured", False))  # fmt: skip

    @property
    def square_mm(self) -> float:
        return self.square_px * self.pitch_mm

    @property
    def n_corners(self) -> int:
        return (self.squares_x - 1) * (self.squares_y - 1)

    def board(self, unit: str = "mm") -> cv2.aruco.CharucoBoard:
        """Board with lengths in mm (default) or display px. Object points: origin at the top-left square corner."""
        k = self.pitch_mm if unit == "mm" else 1.0
        d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, self.dictionary))
        b = cv2.aruco.CharucoBoard((self.squares_x, self.squares_y), self.square_px * k, self.marker_px * k, d)
        b.setLegacyPattern(False)
        return b


def render(spec: CharucoSpec) -> np.ndarray:
    """The full-screen image, pixel-exact: every square and marker cell is a whole number of display pixels."""
    d = cv2.aruco.getPredefinedDictionary(getattr(cv2.aruco, spec.dictionary))
    cells = d.markerSize + 2
    if spec.marker_px % cells or (spec.square_px - spec.marker_px) % 2:
        raise ValueError(f"marker_px must be a multiple of {cells} and centred on whole pixels")
    w, h = spec.squares_x * spec.square_px, spec.squares_y * spec.square_px
    ox, oy = spec.offset_px
    if ox + w > spec.screen_px[0] or oy + h > spec.screen_px[1]:
        raise ValueError("board does not fit the screen")
    board = spec.board("px")
    ref = board.generateImage((w, h), marginSize=0, borderBits=1)  # only used to read which squares are black
    img = np.full((spec.screen_px[1], spec.screen_px[0]), 255, np.uint8)
    s, probe = spec.square_px, 2
    for r in range(spec.squares_y):
        for c in range(spec.squares_x):
            if ref[r * s + probe, c * s + probe] < 128:
                img[oy + r * s : oy + (r + 1) * s, ox + c * s : ox + (c + 1) * s] = 0
    for pts, mid in zip(board.getObjPoints(), board.getIds().ravel()):
        x0, y0 = int(round(pts[0][0])), int(round(pts[0][1]))
        m = cv2.aruco.generateImageMarker(d, int(mid), spec.marker_px, borderBits=1)
        img[oy + y0 : oy + y0 + spec.marker_px, ox + x0 : ox + x0 + spec.marker_px] = m
    return img


def load_gray(path: Path) -> np.ndarray:
    """RAW → 8-bit gray, camera WB, no auto-brightness, no lens corrections (rawpy full visible frame)."""
    import rawpy

    with rawpy.imread(str(path)) as r:
        rgb = r.postprocess(no_auto_bright=True, use_camera_wb=True, output_bps=8)
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY)


def load_linear(path: Path) -> np.ndarray:
    """RAW → float32 gray proportional to light (gamma 1, 16-bit, camera WB, no auto-brightness, no lens
    corrections; same frame as `load_gray`). For edge-based sub-pixel work: a gamma curve moves an edge's
    mid-level crossing towards the dark side by a blur-dependent amount (0.2-0.6 px on a tag edge)."""
    import rawpy

    with rawpy.imread(str(path)) as r:
        rgb = r.postprocess(no_auto_bright=True, use_camera_wb=True, output_bps=16, gamma=(1, 1))
    return cv2.cvtColor(rgb, cv2.COLOR_RGB2GRAY).astype(np.float32)


def to_display(linear: np.ndarray) -> np.ndarray:
    """Linear gray (`load_linear`) → 8-bit with a 2.2 gamma, close to `load_gray`, for detectors that expect it."""
    return np.clip(255 * (linear / 65535) ** (1 / 2.222), 0, 255).astype(np.uint8)


@dataclass
class Detection:
    ids: np.ndarray  # (N,) ChArUco corner ids
    corners: np.ndarray  # (N, 2) image px
    n_markers: int


def detect(gray: np.ndarray, board: cv2.aruco.CharucoBoard) -> Detection | None:
    det = cv2.aruco.CharucoDetector(board)
    cc, ci, mc, mi = det.detectBoard(gray)
    if ci is None or len(ci) < 6:
        return None
    return Detection(ci.ravel().astype(int), cc.reshape(-1, 2).astype(float), 0 if mi is None else len(mi))
