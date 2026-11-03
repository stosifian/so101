"""Step 10b validation GIF, with the left part of the frame cropped off (people in the background; read-only).

    uv run scripts/camera/validation_gif.py data/raw/20261031-135444_capture_follower

One frame per arrival (first shot of each pair): the photo, a circle at the tag coloured by method, and the landing
error from data/derived/<run>/command_accuracy.json. Everything left of --crop-left (fraction of the width) is cropped off.
"""

import argparse
import json
import re
from pathlib import Path

import cv2
import numpy as np
from PIL import Image

from so101.analysis.apriltag import detect_tags
from so101.analysis.charuco import load_linear, to_display
from so101.config import REPO_ROOT

COLORS = {"nominal": (150, 150, 150), "cal": (214, 120, 42), "cal_corr": (110, 158, 31)}  # BGR


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", type=Path)
    ap.add_argument("--width", type=int, default=900)
    ap.add_argument("--crop-left", type=float, default=0.255, help="crop off x < this fraction of the width")
    ap.add_argument("--frame-ms", type=int, default=172)
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()

    acc = json.loads((REPO_ROOT / "data/derived" / args.run.name / "command_accuracy.json").read_text())["methods"]
    files = [f for f in sorted((args.run / "images").glob("*.ARW")) if not f.stem.endswith("_s1")]
    frames = []
    for f in files:
        lin = load_linear(f)
        g = to_display(lin)
        s = args.width / g.shape[1]
        im = cv2.cvtColor(cv2.resize(g, None, fx=s, fy=s, interpolation=cv2.INTER_AREA), cv2.COLOR_GRAY2BGR)
        x0 = int(args.crop_left * im.shape[1])
        im = np.ascontiguousarray(im[:, x0:])
        w = im.shape[1]

        name = f.stem.split("_", 2)[-1].rsplit("_s", 1)[0]
        m = re.match(r"T(\d+)_(nominal|cal_corr|cal)$", name)
        txt, col = name, (200, 200, 200)
        if m:
            e = np.linalg.norm(acc[m.group(2)]["errors_mm"][str(int(m.group(1)))])
            col = COLORS[m.group(2)]
            txt = f"target {int(m.group(1)):02d}  {m.group(2):8s}  landed {e:.1f} mm from target"
        t = detect_tags(g, ids=[2], linear=lin).get(2)
        if t is not None:
            c = t.corners.mean(0) * s
            cv2.circle(im, (int(c[0]) - x0, int(c[1])), 22, col, 2)
        cv2.rectangle(im, (0, 0), (w, 30), (0, 0, 0), -1)
        cv2.putText(im, txt, (10, 21), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1, cv2.LINE_AA)
        frames.append(Image.fromarray(cv2.cvtColor(im, cv2.COLOR_BGR2RGB)))

    out = args.out or REPO_ROOT / "docs/figures" / f"validation_10b_{args.run.name.split('_')[0]}.gif"
    frames[0].save(out, save_all=True, append_images=frames[1:], duration=args.frame_ms, loop=0, optimize=True)
    print(len(frames), "frames ->", out, round(out.stat().st_size / 1e6, 1), "MB")


if __name__ == "__main__":
    main()
