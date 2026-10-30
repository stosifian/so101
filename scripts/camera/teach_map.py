"""3D map of hand-taught poses (step 6a): one dot per pose at the tool point (forward kinematics from the joint
readings, arm base frame), coloured by how well the camera sees the gripper tag there.

    uv run scripts/camera/teach_map.py data/raw/<teach_run> [data/raw/<teach_run> ...]
    uv run scripts/camera/teach_map.py data/raw/*_teach_follower --out docs/figures/step6a_teach_map

Colours: green = sharp (edge-line RMS <= 0.13 px), orange = usable (<= 0.18 px), red = not usable (tag not
found, softer than 0.18 px, in the uncalibrated region, or the arm moved during the capture). Thresholds as in
scripts/readonly/teach_poses.py. Reads data/derived/<run_id>/teach.json for each run (teach_poses.py writes it; --analyze rebuilds it).
Writes <out>.html (interactive: rotate, zoom, hover for label/sharpness/distance/tilt/ticks; plotly.js from a
CDN) and <out>.png (static, for the write-up). Default out: data/derived/teach_map/teach_map. Read-only.
"""

import argparse
import json
import sys
from pathlib import Path

import numpy as np

from so101.config import REPO_ROOT

SHARP, USABLE = 0.13, 0.30
CLASSES = [("sharp", "#1f9e6e"), ("usable", "#eb8c34"), ("not usable", "#d43f3a")]


def classify(p: dict) -> tuple[str, str]:
    """(class, reason)"""
    t = p["tag"]
    if not t["found"]:
        return "not usable", "tag not found"
    if p["moved"]:
        return "not usable", "arm moved during capture"
    if t["uncalibrated"]:
        return "not usable", "uncalibrated region"
    if t["edge_rms"] > USABLE:
        return "not usable", "soft focus"
    return ("sharp" if t["edge_rms"] <= SHARP else "usable"), ""


def load(runs: list[Path]) -> list[dict]:
    pts = []
    for r in runs:
        f = REPO_ROOT / "data" / "derived" / r.name / "teach.json"
        if not f.exists():
            print(f"skip {r.name}: no teach.json (run teach_poses.py --analyze {r})", file=sys.stderr)
            continue
        for p in json.loads(f.read_text())["poses"]:
            c, why = classify(p)
            t = p["tag"]
            hover = (f"<b>{p['label']}</b> ({r.name[:15]})<br>{c}{' (' + why + ')' if why else ''}<br>"
                     + (f"edge {t['edge_rms']:.3f} px, {t['distance_mm']:.0f} mm from camera, tilt {t['tilt_deg']:.0f}°<br>" if t["found"] else "")
                     + f"tool {', '.join(f'{v:.0f}' for v in p['tool_mm'])} mm<br>"
                     + " ".join(f"{j[:5]} {v}" for j, v in p["ticks"].items()))  # fmt: skip
            pts.append({"xyz": p["tool_mm"], "cls": c, "label": p["label"], "hover": hover})
    return pts


def write_html(pts: list[dict], path: Path, title: str) -> None:
    traces = []
    for name, col in CLASSES:
        s = [p for p in pts if p["cls"] == name]
        traces.append({"type": "scatter3d", "mode": "markers+text", "name": f"{name} ({len(s)})",
                       "x": [p["xyz"][0] for p in s], "y": [p["xyz"][1] for p in s], "z": [p["xyz"][2] for p in s],
                       "text": [p["label"] for p in s], "textposition": "top center", "textfont": {"size": 10},
                       "hovertext": [p["hover"] for p in s], "hoverinfo": "text",
                       "marker": {"size": 7, "color": col, "line": {"color": "#ffffff", "width": 1}}})  # fmt: skip
    traces.append({"type": "scatter3d", "mode": "markers+text", "name": "arm base (origin)", "x": [0], "y": [0], "z": [0],
                   "text": ["base"], "marker": {"size": 5, "color": "#6b6a63", "symbol": "diamond"}, "hoverinfo": "name"})  # fmt: skip
    xs = np.array([p["xyz"] for p in pts] + [[0, 0, 0]])
    lo, hi = xs.min(0) - 50, xs.max(0) + 50
    traces.append({"type": "mesh3d", "name": "table", "x": [lo[0], hi[0], hi[0], lo[0]], "y": [lo[1], lo[1], hi[1], hi[1]],
                   "z": [-2.4] * 4, "i": [0, 0], "j": [1, 2], "k": [2, 3], "color": "#b9b7ad", "opacity": 0.25,
                   "hoverinfo": "name", "showlegend": True})  # fmt: skip
    layout = {"title": {"text": title, "font": {"size": 14}}, "margin": {"l": 0, "r": 0, "t": 40, "b": 0},
              "legend": {"x": 0.01, "y": 0.98},
              "scene": {"aspectmode": "data", "xaxis": {"title": "x forward (mm)"}, "yaxis": {"title": "y left (mm)"},
                        "zaxis": {"title": "z up (mm)"}}}  # fmt: skip
    path.write_text(f"""<!doctype html><html><head><meta charset="utf-8"><title>Teach map</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<script src="https://cdn.jsdelivr.net/npm/plotly.js-dist-min@2.35.2/plotly.min.js"></script>
<style>html,body{{margin:0;height:100%;background:#fff;font-family:system-ui,sans-serif}}#p{{width:100%;height:100vh}}</style>
</head><body><div id="p"></div><script>
Plotly.newPlot("p", {json.dumps(traces)}, {json.dumps(layout)}, {{responsive: true}});
</script></body></html>""")


def write_png(pts: list[dict], path: Path, title: str) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig = plt.figure(figsize=(14, 5.2))
    views = [("3D", None), ("top (x–y)", (0, 1)), ("side (x–z)", (0, 2))]
    for k, (name, axes) in enumerate(views):
        ax = fig.add_subplot(1, 3, k + 1, projection="3d" if axes is None else None)
        for cls, col in CLASSES:
            s = np.array([p["xyz"] for p in pts if p["cls"] == cls]).reshape(-1, 3)
            if not len(s):
                continue
            if axes is None:
                ax.scatter(s[:, 0], s[:, 1], s[:, 2], c=col, s=40, edgecolors="white", label=cls, depthshade=False)
            else:
                ax.scatter(s[:, axes[0]], s[:, axes[1]], c=col, s=40, edgecolors="white", label=cls)
        lab = ["x forward (mm)", "y left (mm)", "z up (mm)"]
        if axes is None:
            ax.scatter([0], [0], [0], c="#6b6a63", marker="D", s=30)
            ax.set_xlabel(lab[0]), ax.set_ylabel(lab[1]), ax.set_zlabel(lab[2])
        else:
            ax.scatter([0], [0], c="#6b6a63", marker="D", s=30)
            ax.annotate("base", (0, 0), xytext=(4, 4), textcoords="offset points", fontsize=8, color="#6b6a63")
            for p in pts:
                ax.annotate(p["label"], (p["xyz"][axes[0]], p["xyz"][axes[1]]), xytext=(4, 3), textcoords="offset points", fontsize=7, color="#1f1f1d")
            ax.set_xlabel(lab[axes[0]]), ax.set_ylabel(lab[axes[1]])
            ax.set_aspect("equal", adjustable="datalim")
            ax.grid(color="#e4e3dd", lw=0.6)
            if axes == (0, 2):
                ax.axhline(-2.4, color="#b9b7ad", lw=1)
        ax.set_title(name, loc="left", fontsize=10)
    fig.axes[0].legend(loc="upper left", fontsize=8)
    fig.suptitle(title, x=0.01, ha="left", fontsize=11)
    fig.tight_layout()
    fig.savefig(path, dpi=130)
    plt.close(fig)


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", type=Path, nargs="+")
    ap.add_argument("--out", type=Path, default=REPO_ROOT / "data" / "derived" / "teach_map" / "teach_map")
    args = ap.parse_args()
    pts = load(args.runs)
    if not pts:
        sys.exit("no poses")
    args.out.parent.mkdir(parents=True, exist_ok=True)
    n = {c: sum(p["cls"] == c for p in pts) for c, _ in CLASSES}
    title = f"Hand-taught poses: tool point, arm base frame. {n['sharp']} sharp, {n['usable']} usable, {n['not usable']} not usable"
    write_html(pts, args.out.with_suffix(".html"), title)
    write_png(pts, args.out.with_suffix(".png"), title)
    print(f"wrote {args.out.with_suffix('.html')} + .png ({len(pts)} poses: {n})")


if __name__ == "__main__":
    main()
