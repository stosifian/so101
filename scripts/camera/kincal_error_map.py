"""Step 10: 3D map of the held-out kinematic error, one arrow per pose (scripts/camera/kinematic_fit.py data).

    uv run scripts/camera/kincal_error_map.py data/raw/<run> [data/raw/<run> ...] [--scale 40]

Tag measured relative to the iPad in each photo, one iPad<-base transform for all runs (as kinematic_fit.py's default).
Per pose group: the tag centre (arm base frame, from the full-data fit) and the mean HELD-OUT error vector
(measured - predicted, k-fold by pose group, as kinematic_fit.py), rotated into the base frame (x forward,
y left, z up), drawn `--scale` times longer than life. Variants `nominal` and `links` side by side (toggle in
the HTML legend). Uses the detection cache of kinematic_fit.py (run it first). Writes
data/derived/kincal/error_map.html (interactive; arrow colour = error magnitude) and error_map.png.
"""

import argparse
import json
import sys
from collections import defaultdict
from pathlib import Path

import numpy as np
from rich.console import Console

from so101.analysis import kincal
from so101.analysis.pose import Camera, solve, tag_object_points
from so101.config import REPO_ROOT
from so101.kinematics.fk import chain_from_mujoco
from so101.kinematics.mapping import load_mapping
from so101.kinematics.model import ArmModel

sys.path.insert(0, str(Path(__file__).parent))
from kinematic_fit import OUT, detections, group_of  # noqa: E402


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("runs", type=Path, nargs="+")
    ap.add_argument("--variants", default="nominal,links")
    ap.add_argument("--scale", type=float, default=40.0, help="arrow length = error × scale")
    ap.add_argument("--folds", type=int, default=5)
    ap.add_argument("--replot", action="store_true", help="redraw from data/derived/kincal/error_map.json (no refit)")
    args = ap.parse_args()
    console = Console()
    if args.replot:
        layers = json.loads((OUT / "error_map.json").read_text())
        write_html(layers, args.scale, OUT / "error_map.html")
        write_png(layers, args.scale, OUT / "error_map.png")
        write_dots(layers, OUT / "error_dots")
        console.print(f"redrew {(OUT / 'error_map.html').relative_to(REPO_ROOT)}, error_dots.html + .png")
        return
    cam = Camera.load()
    mapping = load_mapping("follower")
    arm = ArmModel(mapping.model_path)
    chain = chain_from_mujoco(arm.m)
    obj = tag_object_points(20.0)
    shots = []
    for run in args.runs:
        for r in detections(run, 2, console):
            c = np.array(r["corners"])
            if r.get("T_cam_ipad") is None:
                continue
            shots.append(kincal.Shot(group_of(run.name, r["label"]), kincal.angles_from_ticks(mapping, r["ticks"]), c,
                                     solve(obj, c, cam, 0.05, square=True).matrix(), f"{run.name[:15]}:{r['label']}",
                                     "all", np.array(r["T_cam_ipad"]), kincal.gravity_torques(arm, mapping, r["ticks"])))  # fmt: skip  (one iPad<-base transform, as kinematic_fit.py)
    layers = {}
    for v in args.variants.split(","):
        model, _ = kincal.fit(shots, chain, cam, 20.0, v)
        e_cam = kincal.kfold(shots, chain, cam, 20.0, v, args.folds)  # predicted − measured
        by = defaultdict(list)
        for s, e in zip(shots, e_cam):
            T_cam_base = s.T_ref @ model.X[s.run]
            T_base_tag = np.linalg.inv(T_cam_base) @ model.T_cam_tag(chain, s)
            by[s.group].append((T_base_tag[:3, 3], -T_cam_base[:3, :3].T @ e, s.label))  # arrow: measured − predicted, base frame
        pts = []
        for g, rows in by.items():
            P = np.mean([r[0] for r in rows], axis=0)
            E = np.mean([r[1] for r in rows], axis=0)
            pts.append({"group": g, "label": rows[0][2].split("#")[0], "n": len(rows), "p": P.tolist(), "e": E.tolist()})
        layers[v] = pts
        mags = np.array([np.linalg.norm(p["e"]) for p in pts])
        console.print(f"{v}: {len(pts)} poses, mean |error| {mags.mean():.2f} mm, max {mags.max():.2f} mm")
    OUT.mkdir(parents=True, exist_ok=True)
    write_html(layers, args.scale, OUT / "error_map.html")
    write_png(layers, args.scale, OUT / "error_map.png")
    (OUT / "error_map.json").write_text(json.dumps(layers))
    write_dots(layers, OUT / "error_dots")
    console.print(f"wrote {(OUT / 'error_map.html').relative_to(REPO_ROOT)} + error_map.png, error_dots.html + .png")


def write_html(layers: dict, k: float, path: Path) -> None:
    vmax = max(np.linalg.norm(p["e"]) for pts in layers.values() for p in pts)
    traces = []
    for i, (v, pts) in enumerate(layers.items()):
        vis = True if i == len(layers) - 1 else "legendonly"
        P, E = np.array([p["p"] for p in pts]), np.array([p["e"] for p in pts])
        mag = np.linalg.norm(E, axis=1)
        hover = [f"<b>{p['label']}</b> ({p['n']} shots)<br>error {m:.2f} mm: dx {e[0]:+.2f}, dy {e[1]:+.2f}, dz {e[2]:+.2f} (base frame)"
                 for p, e, m in zip(pts, E, mag)]  # fmt: skip
        traces.append({"type": "scatter3d", "mode": "markers", "name": f"{v}: poses", "legendgroup": v, "visible": vis,
                       "x": P[:, 0].tolist(), "y": P[:, 1].tolist(), "z": P[:, 2].tolist(), "hovertext": hover, "hoverinfo": "text",
                       "marker": {"size": 3, "color": mag.tolist(), "colorscale": "Viridis", "cmin": 0, "cmax": vmax}})  # fmt: skip
        xs, ys, zs = [], [], []
        for p, e in zip(P, E):
            q = p + k * e
            xs += [p[0], q[0], None]
            ys += [p[1], q[1], None]
            zs += [p[2], q[2], None]
        traces.append({"type": "scatter3d", "mode": "lines", "name": f"{v}: error × {k:g}", "legendgroup": v, "visible": vis,
                       "x": xs, "y": ys, "z": zs, "hoverinfo": "skip", "line": {"color": "#6b6a63", "width": 3}})  # fmt: skip
        tips = P + k * E
        traces.append({"type": "cone", "name": f"{v}: direction", "legendgroup": v, "visible": vis, "showlegend": False,
                       "x": tips[:, 0].tolist(), "y": tips[:, 1].tolist(), "z": tips[:, 2].tolist(),
                       "u": E[:, 0].tolist(), "v": E[:, 1].tolist(), "w": E[:, 2].tolist(), "anchor": "tip", "sizemode": "absolute",
                       "sizeref": 8, "colorscale": "Viridis", "cmin": 0, "cmax": vmax, "showscale": i == len(layers) - 1,
                       "colorbar": {"title": "|error| mm"}, "hovertext": hover, "hoverinfo": "text"})  # fmt: skip
    traces.append({"type": "scatter3d", "mode": "markers+text", "name": "base", "x": [0], "y": [0], "z": [0], "text": ["base"],
                   "marker": {"size": 4, "color": "#1f1f1d", "symbol": "diamond"}, "hoverinfo": "name"})  # fmt: skip
    layout = {"title": {"text": f"Held-out kinematic error at the tag, relative to the iPad (arrows: measured − predicted, × {k:g}); toggle variants in the legend",
                        "font": {"size": 13}}, "margin": {"l": 0, "r": 0, "t": 40, "b": 0}, "legend": {"x": 0.01, "y": 0.95},
              "scene": {"aspectmode": "data", "xaxis": {"title": "x forward (mm)"}, "yaxis": {"title": "y left (mm)"}, "zaxis": {"title": "z up (mm)"}}}  # fmt: skip
    path.write_text(f"""<!doctype html><html><head><meta charset="utf-8"><title>Kinematic error map</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<script src="https://cdn.jsdelivr.net/npm/plotly.js-dist-min@2.35.2/plotly.min.js"></script>
<style>html,body{{margin:0;height:100%;background:#fff;font-family:system-ui,sans-serif}}#p{{width:100%;height:100vh}}</style>
</head><body><div id="p"></div><script>
Plotly.newPlot("p", {json.dumps(traces)}, {json.dumps(layout)}, {{responsive: true}});
</script></body></html>""")


def write_dots(layers: dict, stem: Path) -> None:
    """Dots only, coloured by |held-out error| (mm), one shared colour scale for all variants."""
    vmax = max(np.linalg.norm(p["e"]) for pts in layers.values() for p in pts)
    traces = []
    for i, (v, pts) in enumerate(layers.items()):
        P, E = np.array([p["p"] for p in pts]), np.array([p["e"] for p in pts])
        mag = np.linalg.norm(E, axis=1)
        traces.append({"type": "scatter3d", "mode": "markers", "name": f"{v} (mean {mag.mean():.2f}, max {mag.max():.2f} mm)",
                       "visible": True if i == len(layers) - 1 else "legendonly",
                       "x": P[:, 0].tolist(), "y": P[:, 1].tolist(), "z": P[:, 2].tolist(),
                       "hovertext": [f"<b>{p['label']}</b> ({p['n']} shots)<br>|error| {m:.2f} mm" for p, m in zip(pts, mag)],
                       "hoverinfo": "text", "marker": {"size": 7, "color": mag.tolist(), "colorscale": "Viridis", "cmin": 0, "cmax": vmax,
                       "line": {"color": "#ffffff", "width": 1}, "showscale": i == len(layers) - 1, "colorbar": {"title": "|error| mm"}}})  # fmt: skip
    traces.append({"type": "scatter3d", "mode": "markers+text", "name": "base", "x": [0], "y": [0], "z": [0], "text": ["base"],
                   "marker": {"size": 4, "color": "#1f1f1d", "symbol": "diamond"}, "hoverinfo": "name"})  # fmt: skip
    layout = {"title": {"text": "Held-out kinematic error magnitude at the tag, relative to the iPad, per pose (toggle variants in the legend)", "font": {"size": 13}},
              "margin": {"l": 0, "r": 0, "t": 40, "b": 0}, "legend": {"x": 0.01, "y": 0.95},
              "scene": {"aspectmode": "data", "xaxis": {"title": "x forward (mm)"}, "yaxis": {"title": "y left (mm)"}, "zaxis": {"title": "z up (mm)"}}}  # fmt: skip
    stem.with_suffix(".html").write_text(f"""<!doctype html><html><head><meta charset="utf-8"><title>Kinematic error dots</title>
<meta name="viewport" content="width=device-width, initial-scale=1">
<script src="https://cdn.jsdelivr.net/npm/plotly.js-dist-min@2.35.2/plotly.min.js"></script>
<style>html,body{{margin:0;height:100%;background:#fff;font-family:system-ui,sans-serif}}#p{{width:100%;height:100vh}}</style>
</head><body><div id="p"></div><script>
Plotly.newPlot("p", {json.dumps(traces)}, {json.dumps(layout)}, {{responsive: true}});
</script></body></html>""")
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    vs = list(layers)
    fig = plt.figure(figsize=(15, 4.8 * len(vs)))
    for r, v in enumerate(vs):
        P = np.array([p["p"] for p in layers[v]])
        mag = np.linalg.norm(np.array([p["e"] for p in layers[v]]), axis=1)
        o = np.argsort(mag)  # draw the large errors last (on top)
        for c, (name, ax_ix) in enumerate([("3D", None), ("top (x–y)", (0, 1)), ("side (x–z)", (0, 2))]):
            ax = fig.add_subplot(len(vs), 3, 3 * r + c + 1, projection="3d" if ax_ix is None else None)
            if ax_ix is None:
                sc = ax.scatter(P[o, 0], P[o, 1], P[o, 2], c=mag[o], cmap="viridis", vmin=0, vmax=vmax, s=40, edgecolors="white", linewidths=0.5, depthshade=False)
                ax.set_xlabel("x (mm)"), ax.set_ylabel("y (mm)"), ax.set_zlabel("z (mm)")
            else:
                a, b = ax_ix
                sc = ax.scatter(P[o, a], P[o, b], c=mag[o], cmap="viridis", vmin=0, vmax=vmax, s=40, edgecolors="white", linewidths=0.5)
                ax.set_aspect("equal", adjustable="datalim"), ax.grid(color="#e4e3dd", lw=0.6)
                ax.set_xlabel(["x forward", "y left", "z up"][a] + " (mm)"), ax.set_ylabel(["x forward", "y left", "z up"][b] + " (mm)")
                if c == 2:
                    fig.colorbar(sc, ax=ax, label="|error| (mm)")
            ax.set_title(f"{v}: {name} (mean {mag.mean():.2f}, max {mag.max():.2f} mm)", loc="left", fontsize=9)
    fig.tight_layout()
    fig.savefig(stem.with_suffix(".png"), dpi=110)
    plt.close(fig)


def write_png(layers: dict, k: float, path: Path) -> None:
    import matplotlib

    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    vs = list(layers)
    vmax = max(np.linalg.norm(p["e"]) for pts in layers.values() for p in pts)
    fig = plt.figure(figsize=(15, 5 * len(vs)))
    for r, v in enumerate(vs):
        P, E = np.array([p["p"] for p in layers[v]]), np.array([p["e"] for p in layers[v]])
        mag = np.linalg.norm(E, axis=1)
        cmap = plt.get_cmap("viridis")
        for c, (name, ax_ix) in enumerate([("3D", None), ("top (x–y)", (0, 1)), ("side (x–z)", (0, 2))]):
            ax = fig.add_subplot(len(vs), 3, 3 * r + c + 1, projection="3d" if ax_ix is None else None)
            cols = cmap(mag / vmax)
            if ax_ix is None:
                ax.quiver(P[:, 0], P[:, 1], P[:, 2], k * E[:, 0], k * E[:, 1], k * E[:, 2], colors=cols, arrow_length_ratio=0.25, linewidth=1.2)
                ax.scatter(P[:, 0], P[:, 1], P[:, 2], c=mag, cmap=cmap, vmin=0, vmax=vmax, s=10, depthshade=False)
                ax.set_xlabel("x (mm)"), ax.set_ylabel("y (mm)"), ax.set_zlabel("z (mm)")
            else:
                a, b = ax_ix
                ax.quiver(P[:, a], P[:, b], k * E[:, a], k * E[:, b], mag, cmap=cmap, clim=(0, vmax), angles="xy", scale_units="xy", scale=1, width=0.004)
                sc = ax.scatter(P[:, a], P[:, b], c=mag, cmap=cmap, vmin=0, vmax=vmax, s=10)
                ax.set_aspect("equal", adjustable="datalim"), ax.grid(color="#e4e3dd", lw=0.6)
                ax.set_xlabel(["x forward", "y left", "z up"][a] + " (mm)"), ax.set_ylabel(["x forward", "y left", "z up"][b] + " (mm)")
                if c == 2:
                    fig.colorbar(sc, ax=ax, label="|error| (mm)")
            ax.set_title(f"{v}: {name} (arrows × {k:g}; mean {mag.mean():.2f}, max {mag.max():.2f} mm)", loc="left", fontsize=9)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


if __name__ == "__main__":
    main()
