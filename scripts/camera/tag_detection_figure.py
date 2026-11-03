"""Figure: the AprilTag edge-line corner refinement on one RAW photo (read-only).

    uv run scripts/camera/tag_detection_figure.py data/raw/20261030-113927_stability_camera/images/img_0000.ARW

Left: the tag crop (linear RAW, display gamma) with the sub-pixel edge crossings, the fitted edge lines, the
refined corners (+) and OpenCV's unrefined corners (o). Top right: one corner, zoomed. Bottom right: each edge
crossing's distance from its fitted line along the edge. Title: edge-line RMS, the tag's PnP reprojection RMS and
the mean refined-vs-OpenCV corner shift. Same detection path as scripts/camera/target_pose.py.
Writes data/derived/<run>/tag_detection_<image>.png (or --out).
"""

import argparse
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np

from so101.analysis.apriltag import detect_tags, refine_corners
from so101.analysis.charuco import load_linear, to_display
from so101.analysis.pose import INTRINSICS, Camera, solve, tag_object_points
from so101.config import REPO_ROOT

EDGE_COLORS = ["#2a6fdb", "#e08a1e", "#3a9d5d", "#8e44ad"]


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("image", type=Path)
    ap.add_argument("--tag-id", type=int, default=2)
    ap.add_argument("--tag-mm", type=float, default=20.0)
    ap.add_argument("--corner", type=int, default=0, help="corner to zoom (0 = top-left as printed)")
    ap.add_argument("--out", type=Path)
    args = ap.parse_args()

    lin = load_linear(args.image)
    gray = to_display(lin)
    tags = detect_tags(gray, ids=[args.tag_id], linear=lin)
    if args.tag_id not in tags:
        raise SystemExit(f"tag {args.tag_id} not found in {args.image}")
    t = tags[args.tag_id]
    edges: list = []
    refine_corners(lin, t.raw, edges_out=edges)  # same call as detect_tags, keeping the edge points
    pose = solve(tag_object_points(args.tag_mm), t.corners, Camera.load(INTRINSICS), 0.05, square=True)
    shift = np.linalg.norm(t.corners - t.raw, axis=1).mean()
    side = np.mean([np.linalg.norm(t.corners[k] - t.corners[(k + 1) % 4]) for k in range(4)])

    fig = plt.figure(figsize=(12, 6.8))
    gs = fig.add_gridspec(2, 2, width_ratios=[1.25, 1], height_ratios=[1, 1])
    ax_img, ax_zoom, ax_res = fig.add_subplot(gs[:, 0]), fig.add_subplot(gs[0, 1]), fig.add_subplot(gs[1, 1])

    def draw(ax, half: float, centre: np.ndarray, dot: float, line_ext: float) -> None:
        x0, x1 = int(centre[0] - half), int(centre[0] + half)
        y0, y1 = int(centre[1] - half), int(centre[1] + half)
        ax.imshow(gray[y0:y1, x0:x1], cmap="gray", extent=(x0 - 0.5, x1 - 0.5, y1 - 0.5, y0 - 0.5),
                  interpolation="nearest")  # fmt: skip
        for (pts, m, d), c in zip(edges, EDGE_COLORS):
            ends = m + np.outer([-line_ext, line_ext], d)
            ax.plot(ends[:, 0], ends[:, 1], color=c, lw=1.2, alpha=0.9)
            ax.scatter(pts[:, 0], pts[:, 1], s=dot, color=c, edgecolors="white", linewidths=0.4, zorder=3)
        ax.scatter(t.raw[:, 0], t.raw[:, 1], s=60, facecolors="none", edgecolors="#c0392b", linewidths=1.5,
                   zorder=4, label="OpenCV corner (unrefined)")  # fmt: skip
        ax.scatter(t.corners[:, 0], t.corners[:, 1], s=80, marker="+", color="#ffd400", linewidths=1.8, zorder=5,
                   label="refined corner (edge-line intersection)")  # fmt: skip
        ax.set_xlim(x0 - 0.5, x1 - 0.5)
        ax.set_ylim(y1 - 0.5, y0 - 0.5)
        ax.set_xticks([])
        ax.set_yticks([])

    centre = t.corners.mean(axis=0)
    draw(ax_img, 0.75 * side, centre, 14, 0.7 * side)
    ax_img.legend(loc="lower center", bbox_to_anchor=(0.5, -0.1), ncol=2, fontsize=8, frameon=False)
    ax_img.set_title(f"tag {args.tag_id} ({args.tag_mm:g} mm ≈ {side:.0f} px): edge crossings and fitted lines",
                     fontsize=10, loc="left")  # fmt: skip
    draw(ax_zoom, 12, t.corners[args.corner], 30, 0.7 * side)
    ax_zoom.set_title(f"corner {args.corner}, zoomed (1 square = 1 pixel)", fontsize=10, loc="left")

    for k, ((pts, m, d), c) in enumerate(zip(edges, EDGE_COLORS)):
        nrm = np.array([-d[1], d[0]])
        s = (pts - m) @ d
        ax_res.plot(s - s.min(), (pts - m) @ nrm, "o-", ms=3, lw=0.8, color=c,
                    label=f"edge {k}: RMS {np.sqrt(np.mean(((pts - m) @ nrm) ** 2)):.3f} px")  # fmt: skip
    ax_res.axhline(0, color="0.5", lw=0.8)
    ax_res.set_xlabel("position along edge (px)")
    ax_res.set_ylabel("distance from fitted line (px)")
    ax_res.set_title("edge crossings vs their fitted lines", fontsize=10, loc="left")
    lo, hi = ax_res.get_ylim()
    ax_res.set_ylim(lo, hi + 0.4 * (hi - lo))  # room for the legend above the data
    ax_res.legend(fontsize=8, ncol=2, frameon=False, loc="upper left")
    ax_res.grid(alpha=0.25)
    ax_res.spines[["top", "right"]].set_visible(False)

    fig.suptitle(f"{args.image.parent.parent.name}/{args.image.name}:  edge-line RMS {t.edge_rms:.3f} px · "
                 f"tag reprojection RMS {pose.rms_px:.3f} px · refined vs OpenCV corner shift {shift:.2f} px",
                 fontsize=11)  # fmt: skip
    fig.tight_layout()
    out = args.out or REPO_ROOT / "data/derived" / args.image.parent.parent.name / f"tag_detection_{args.image.stem}.png"
    out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out, dpi=150)
    print(out)


if __name__ == "__main__":
    main()
