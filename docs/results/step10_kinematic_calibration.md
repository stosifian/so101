# Phase 2 step 10: kinematic calibration and accuracy at the tag

*2026-10-30. SO-101 follower; Sony A7 IV at ~0.6 m; gripper AprilTag (36h11 ID 2, 20 mm nominal) measured
relative to the iPad ChArUco in every photo.*

## Question

How well does the arm's kinematic model predict where the gripper actually is, from the encoder readings alone? How
much does a calibration fitted on camera measurements improve that? The plan's target is residuals below 1 mm
(`docs/phase-2-plan.md`, step 10).

## Data

| Run | What | Shots used | Distinct poses | Role |
|---|---|---|---|---|
| `20261030-154331_capture_follower` | step 6 dry run (sector tour) | 18 | 18 | fit |
| `20261030-162649_capture_follower` | step 6 rerun (hand-eye-planned wrist angles) | 28 | 28 | fit |
| `20261030-170704_capture_follower` | step 8 repeatability + backlash | 92 | 4 | fit |
| `20261030-190841_capture_follower` | step 10 grid (wrist roll 0 / ±25°; stopped at p39) | 83 | 42 | fit |
| `20261030-202915_capture_follower` | **independent test**: grid points the grid run never reached, minus 4 above 0.75 N·m shoulder load | 34 | 17 | **test only, never fitted** |

The fit uses **221 shots / 92 poses**. Excluded:
- shots where a joint moved > 3 ticks during the capture;
- shots where the tag wasn't found;
- the hand-held teach runs, where the arm wasn't commanded.

## Method

- **Measurement:** the tag's 4 corners, refined by fitting straight lines to its edges on the linear (gamma 1) RAW image. The iPad pose is measured in the same photo, and everything is expressed **relative to the iPad**, so camera motion cancels. The iPad moved ≤ 12 µm within each run and ≤ 50 µm / 40″ between runs over 3.5 h, so camera motion was not a factor at the mm level.
- **Model per shot:** T_cam_tag = T_cam_iPad (measured) · X (iPad ← base) · F(q; θ) · Y (gripper body ← tag).
  - F is the forward kinematics from the encoder angles.
  - θ is the variant's corrections.
  - One X is shared by all runs. One X per run was tried and overfit: CV lateral 1.03 vs 0.89 mm for `links`.
- **Objective:** the tag-corner reprojection error in pixels. Weak priors on θ keep directions the data can't determine at zero:
  - offsets σ 3°;
  - link rotations σ 2° and translations σ 2 mm;
  - tag scale σ 0.5 %;
  - compliance σ 2.9°/N·m.
- **Not separable with one fixed camera and one tag:** the pan zero offset (absorbed into X), and the first link's correction and the tool correction (absorbed into X and Y). The wrist-flex and roll offsets need wrist-roll variation, which the grid run supplied.
- **Gravity torque** on shoulder lift, elbow and wrist flex at each pose comes from the MuJoCo model's masses (static `qfrc_bias`).
- **Validation:**
  1. 5-fold cross-validation **by pose group**: repeats of a pose stay in the same fold.
  2. An **independent test run** that no fit ever saw.
- **Error definition:** measured tag centre − model-predicted tag centre, in mm along the camera axes (x right, y down, z depth). "Lateral" = x, y.

## Variants

Each variant is saved separately in `config/kinematics/follower_<variant>.json` and loaded with
`so101.kinematics.calibrated.load("<variant>")`. Each file holds:
- the parameters and the transforms they were fitted with;
- the runs used for fitting and testing;
- the reference frame;
- the git sha;
- all the metrics below.

| Variant | Arm parameters | Adds |
|---|---|---|
| nominal | 0 | CAD geometry; only X (iPad ← base) and Y (body ← tag) fitted |
| offsets | 2 | + zero offsets of shoulder_lift, elbow_flex |
| links | 26 | + zero offsets (lift, elbow) + placement corrections of links 1–4 (6 each: 3 rotations, 3 translations) |
| full | 29 | links + wrist_flex and wrist_roll zero offsets + tag scale |
| **stiff_lin** | 29 | links + linear gravity compliance on lift, elbow, wrist: Δθ_j = c_j · τ_j |
| stiff_thr | 27 | links + shoulder_lift compliance only above 0.75 N·m (threshold chosen *from these results*: post hoc) |

## Results

All values in mm at the tag, relative to the iPad.
- **CV** = cross-validated (held-out poses, 5-fold by pose group).
- **Inner** = shoulder-lift gravity torque |τ| ≤ 0.75 N·m; **far** = above it (the far-forward reach).
- **Test** = the independent run (17 poses, mean of each pose's 2 shots).

| variant | n θ | fit px | CV lat | CV 3D | CV 3D max | CV inner 3D | CV far 3D | test lat | test 3D | test max | test sub-mm lat |
|---|---|---|---|---|---|---|---|---|---|---|---|
| nominal | 0 | 12.5 | 1.57 | 1.98 | 6.09 | 1.75 | 3.80 | 2.02 | 2.18 | 3.56 | 4/17 |
| offsets | 2 | 11.3 | 1.43 | 2.06 | 6.60 | 1.77 | 4.19 | 1.81 | 2.80 | 3.98 | 3/17 |
| links | 26 | 6.9 | 0.89 | 1.66 | 5.41 | 1.47 | 3.19 | 1.01 | 1.34 | 2.23 | 10/17 |
| full | 29 | 6.9 | 0.89 | 1.67 | 5.44 | 1.48 | 3.21 | 1.02 | 1.35 | 2.26 | 10/17 |
| **stiff_lin** | 29 | **6.2** | **0.74** | **1.07** | **4.94** | **0.85** | **2.48** | **0.79** | **1.14** | 2.32 | **13/17** |
| stiff_thr | 27 | 6.7 | 0.91 | 1.79 | 5.19 | 1.62 | 3.19 | 0.92 | 1.28 | 2.39 | 12/17 |

Per camera axis (RMS, mm: x / y / z = depth):

| variant | CV x / y / z | test x / y / z | test sub-mm 3D |
|---|---|---|---|
| nominal | 1.10 / 1.12 / 1.21 | 0.73 / 1.89 / 0.82 | 4/17 |
| offsets | 0.98 / 1.05 / 1.48 | 0.51 / 1.74 / 2.13 | 0/17 |
| links | 0.52 / 0.72 / 1.41 | 0.71 / 0.72 / 0.88 | 5/17 |
| full | 0.53 / 0.72 / 1.42 | 0.72 / 0.73 / 0.89 | 5/17 |
| **stiff_lin** | **0.37 / 0.64 / 0.77** | **0.53 / 0.58 / 0.82** | **9/17** |
| stiff_thr | 0.57 / 0.70 / 1.55 | 0.69 / 0.61 / 0.89 | 7/17 |

Per pose, cross-validated (92 poses, 3D):

| variant | sub-mm | median | max |
|---|---|---|---|
| nominal | 6 / 92 | 1.74 | 6.09 |
| links | 36 / 92 | 1.22 | 5.41 |
| **stiff_lin** | **59 / 92** | **0.77** | 4.94 |

Fitted parameters (selected):

| variant | zero offsets (deg) | compliance (deg per N·m) | tag scale |
|---|---|---|---|
| offsets | lift +1.45, elbow −0.03 | — | — |
| links | lift +0.86, elbow −0.13 | — | — |
| full | lift +0.86, elbow −0.25, wrist_flex +1.67, wrist_roll 0.00 | — | 1.0084 |
| stiff_lin | lift −0.18, elbow +0.26 | lift −2.07, elbow −0.34, wrist +12.97 | — |
| stiff_thr | lift +1.02, elbow −0.05 | lift +2.45 (above 0.75 N·m) | — |

## Findings

1. **The calibrated model (`stiff_lin`) halves the error.** On the independent test, lateral RMS goes from 2.02 to **0.79 mm** and 3D from 2.18 to **1.14 mm**. Across the 92 cross-validated poses, sub-mm 3D goes from 6 to 59.
2. **Link geometry matters; zero offsets alone don't.** `offsets` is no better than nominal in cross-validation and worse on the test (3D 2.80 mm). The gain comes from the link corrections (`links`).
3. **Gravity compliance helps across the workspace, not only at the edge.** `stiff_lin` improves the inner workspace too (CV 3D 1.47 → 0.85 mm). A rank correlation of |error| against load had suggested gravity only mattered at the far edge. It missed this because the gravity term acts on the error's *direction*, not just its size.
4. **The threshold idea does not hold up.** `stiff_thr` is no better than `links` in cross-validation (3D 1.79 vs 1.66 mm).
5. **Wrist offsets and tag scale add nothing** (`full` ≈ `links`).
6. **What remains is mostly depth along the camera's line of sight,** varying in sign by pose (error-map arrows), plus the far-forward band (CV far 3D 2.48 mm for `stiff_lin`).
7. **Against the target:** < 1 mm is **met laterally** (0.74 CV, 0.79 test) and **for the median pose** (0.77 mm 3D). 3D RMS is just above 1 mm (1.07 CV, 1.14 test). It is **not met in the far-forward band.**

## Caveats

- **Scale is nominal** (tag 20 mm, iPad 264 ppi; D4). Absolute lengths carry ≤ 0.3 %. Re-processing with a measured scale needs no new captures.
- **The wrist compliance (13°/N·m) is suspect as a physical number.** The wrist torque barely varies (0.09–0.13 N·m), so it's a ~0.5° effect that follows the wrist's tilt against gravity, and it can't be told apart from a geometric error that depends on wrist tilt. The lift and elbow compliances are better constrained. Step 9 (payload at fixed poses) would separate the two.
- **Accuracy is at the tag, not the tool tip** (D13). The tip adds the predicted-orientation error × ~10 cm, plus the tag-to-tip offset error.
- **The far-forward band was under-sampled.** The grid run stopped at p39 when contact detection, which isn't gravity-aware, falsely tripped at the heaviest pose, and the test set excluded the band. Its numbers rest on ~12 poses.
- **The 0.75 N·m "inner / far" split was chosen after seeing the data.**
- **The single 20 mm tag limits depth.** That's the main residual direction.

## Figures

- `docs/figures/step10_error_dots.html` / `.png`: per-pose |error| (nominal, links, stiff_lin).
- `docs/figures/step10_error_map.html` / `.png`: error vectors, ×40 (they line up with the camera's line of sight).
- `docs/figures/step10_error_vs_gravity_links.png`: |error| vs joint angles and gravity torque (`links`).
- `docs/figures/step10_heldout_per_shot.png`: held-out error per shot by variant.
- `docs/figures/step10_variant_comparison.md`: the comparison table as generated.

## Reproduce

```
uv run scripts/camera/kinematic_fit.py data/raw/20261030-190841_capture_follower data/raw/20261030-162649_capture_follower \
    data/raw/20261030-154331_capture_follower data/raw/20261030-170704_capture_follower \
    --test data/raw/20261030-202915_capture_follower --variants nominal,offsets,links,full,stiff_lin,stiff_thr --save
uv run scripts/camera/kincal_error_map.py <same four runs> --variants nominal,links,stiff_lin
uv run scripts/camera/kincal_gravity.py <same four runs>
```
Read-only on raw data; detections are cached in `data/derived/kincal/cache/`.
