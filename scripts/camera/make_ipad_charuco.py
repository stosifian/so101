"""Render the iPad ChArUco (config/targets/ipad_charuco.yaml) as a pixel-exact PNG at the screen's native size.

    uv run scripts/camera/make_ipad_charuco.py

Then AirDrop the PNG to the iPad, open it in Photos, landscape, tap once to hide the UI. The image is exactly
2360 x 1640, so Photos draws it 1:1 (don't pinch-zoom). The script re-detects the rendered image as a check.
"""

import cv2

from so101.analysis.charuco import TARGETS_DIR, CharucoSpec, detect, render


def main() -> None:
    spec = CharucoSpec.load("ipad_charuco")
    img = render(spec)
    out = TARGETS_DIR / f"{spec.name}_{spec.screen_px[0]}x{spec.screen_px[1]}.png"
    cv2.imwrite(str(out), img)
    d = detect(img, spec.board("px"))
    n = 0 if d is None else len(d.ids)
    print(f"wrote {out.relative_to(TARGETS_DIR.parent.parent)}: {spec.squares_x}x{spec.squares_y} squares of {spec.square_px} px "
          f"(~{spec.square_mm:.2f} mm nominal), {spec.dictionary}; self-check {n}/{spec.n_corners} corners")
    if n != spec.n_corners:
        raise SystemExit("self-check failed")


if __name__ == "__main__":
    main()
