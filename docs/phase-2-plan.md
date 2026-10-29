# Phase 2 plan (camera metrology)

The source of truth for Phase 2. **Update this file every time the plan changes:** edit the table and
add a dated entry to the deviation log, with the reason. `docs/ROADMAP.md` only summarizes it.
Spec: `so101-metrology-plan.md` §3 (Phase 2), §6–§9. Targets + generator: `so101-metrology/`.

Last updated: 2026-10-28 (revision 8).

## Current plan

| # | Step | Done when | Status | Arm moves? | Changed vs baseline |
|---|------|-----------|--------|------------|---------------------|
| 0 | Setup: this plan; camera interface (gphoto2 + mock backends, behind `Sensor`); opencv + rawpy in the env | Mock camera passes tests | **done 2026-10-28**: `so101.sensors` (`Sensor`, `Camera`, `Gphoto2Camera` via the CLI with a per-capture timeout, `MockCamera`), `scripts/camera/tether_check.py`, opencv 5.0.0 + rawpy 0.27.1; 224 tests | no | camera code in `src/so101/` (D3) |
| 1 | **Tethering:** 50 consecutive captures over gphoto2; record body + lens firmware | 50/50, no hang/disconnect; else Sony Camera Remote SDK behind the same interface | **done 2026-10-28** (`20261028-131445_tether_camera`): **50/50**, no hang; 5.9 s per capture (median; p95 5.96, max 6.09) for a 67.6 MB uncompressed RAW; body firmware 3.01, gphoto2 2.5.32. gphoto2 accepted; Sony SDK not needed | no | — |
| 2 | **Camera settings** (spec §6): M, fixed ISO + WB, MF taped, IBIS off, RAW (decoded without lens corrections), f/11, 1/60 s, EFC, save to PC, auto-review + power save off | Menus photographed; settings hash in the run header | **done 2026-10-28**: settings in `config/camera_a7iv.yaml`: f/11, 1/30 s, ISO 1600, Daylight, RAW, MF, sdram (read back over USB every run); SteadyShot off, lens comp. off, Mechanical + e-front curtain, auto review off (user-confirmed; Power Save Start Time greyed out while tethered). Exposure from test shots `…133453`/`142131`/`142714`: iPad whites 61–91 %, unclipped. Glare: dark cloth behind the camera → black squares 0.3–1.7 % (`…143317`), one residual streak away from corners. Stability: 3 shots / 10 s, shift ≤ 0.07 px, residual 0.03 px RMS (`20261028-143956_stability_camera`). Menu photos: optional | no | — |
| 3 | **Targets + scale:** iPad ChArUco (pixel-exact, native resolution); caliper the gripper print; iPad scale via print-vs-iPad fit (multi-view, relative pose free, not assumed coplanar) or caliper jaws imaged on the screen | iPad and gripper-target scales known to ≤ 0.05 % | **deferred** until a caliper is available (D4); step 4 goes first |  no | iPad replaces the printed reference board (D1); caliper anchors absolute scale (D2) |
| 4 | **Intrinsics:** ~30 iPad views at working distance (~0.5 m), focus taped, views covering the frame corners, tilt ≤ ~40° | Reprojection 0.1–0.2 px; no moiré pattern in detected-vs-reprojected corner plots; f ≈ 6840 px | **done 2026-10-28** (user-accepted): runs `20261028-170956` + `20261028-172433_intrinsics_camera`, 41 views (dropped 2 blurred: 170956/img_0024 3.98 px, 172433/img_0003 0.94 px). **RMS 0.166 px**; fx 7007.4 / fy 7006.2 (±1.5; +2.6 % vs infinity focus, as expected at 0.7 m); cx 3527.5 / cy 2319.3 (±0.6/0.9); k1 −0.0269, k2 +0.098, k3 +0.154, p1/p2 < 0.001. No moiré (no hot board corners). Coverage: whole width, top edge, but **nothing below y ≈ 3700** (bottom 21 %: the table blocks the board there unless it comes closer than the focused range, user): measurements must stay inside the calibrated region. Result: `data/derived/combined_20261028-170956+20261028-172433/drop_…/intrinsics.json`. Tools: hands-free capture, multi-run analysis | no | — |
| 5 | **Rig stability:** static iPad + gripper target 1–2 h; dial-indicator depth check | Tripod/table drift ≪ 0.25 mm | **done 2026-10-28, usable (user)** (`20261028-182722_stability_camera`, 41 shots / 90 s, iPad at ~0.78 m): smooth linear drift, board dy +0.32 px/h (≈ 35 µm/h), dx +0.14 px/h, rotation +11″/h, scale +45 ppm/h; scatter about the trend 0.035 px; detection residual 0.044 → 0.068 px (not removed by a homography). All ≪ the 0.25 mm budget (≤ ~40 µm/h; ≤ ~10 µm over a 15-min run). Event at ~21 min (dy, rotation, scale jump together; cause unknown). **Camera vs iPad not separable** (scene too dark for background tracking: evening, panel off). Carried: light the static scene in long runs (thermal drift) so background tracking works; re-shoot a reference pose periodically in long runs; dial-indicator depth check | no | — |
| 5b | **Gripper target + iPad static stability, 20–30 min** (user): target mounted (card on the wrist-camera mount plate), (a) arm torque off resting on its stops, then (b) torque on holding one measurement pose. Measure the target's 6-DoF pose relative to the iPad (PnP with the step 4 intrinsics; nominal scale is fine for stability) over time; static scene lit so background tracking works | Relative-pose noise and drift ≪ 0.25 mm (target ~tens of µm over 30 min); (b) − (a) = what servo holding adds | planned | (b) yes | added (D7) |
| 6 | **Dry run:** 20 poses, move → settle → snapshot → capture → snapshot (spec §7) | 0 missed captures; manifest ↔ files 1:1; stillness check never fires | planned | yes | — |
| 7 | **Ring-down:** worst-case wobble (extended arm, lift/elbow moves) | `dwell_s` set from measurement | planned | yes | — |
| 8 | **Repeatability + backlash:** ~30 returns, same vs opposite approach | Spread + backlash per joint | planned | yes | — |
| 9 | **Gravity sag per joint**, with/without payload | Joint stiffnesses fitted | planned | yes | — |
| 10 | **Accuracy grid + kinematic fit** | Residuals < 1 mm | planned | yes | — |
| 11 | **Thermal drift:** loaded hold 1–2 h, temperatures median-filtered | Drift vs temperature | planned | yes | load the joints (ROADMAP) |
| 12 | Write-up + Phase 3 choice | Report + decision | planned | no | — |

### Error budget (sub-mm goal)
- Intrinsics set the floor for every pose: corner quality (0.1–0.2 px, no moiré) matters more than iPad size.
- Gripper position relative to the iPad: depth error ≈ (*s*_grip − *s*_iPad) × Z → 0.05 % mismatch = 0.25 mm at 0.5 m.
- Common scale error *s*: error ≈ *s* × distance from the iPad → ≤ 0.15 mm over 300 mm at 0.05 %.
- Relative tests (repeatability, backlash, sag, drift) are insensitive to scale (< 0.02 mm at *s* = 0.2 %).

### Hardware facts
- Camera: Sony A7 IV (firmware 3.01) + SEL35F18F (no lens stabilization; IBIS off is enough). f ≈ 6840 px; ~73 µm per camera pixel at 0.5 m.
- iPad Air M1: 2360 × 1640, 264 ppi (96.2 µm pitch, nominal, rounded by Apple), LCD (no PWM flicker), screen ≈ 227 × 158 mm.
  At 0.5 m one display pixel ≈ 1.3 camera pixels → moiré risk; f/11 diffraction blur (~3 px) helps.

### Open items
- Intrinsics are extrapolated below y ≈ 3700 px (no calibration data): flag any detection outside the calibrated region (convex hull of calibration corners) once pose estimation exists.
- Throughput: 5.9 s per RAW (spec estimated 1–3 s) → ~20 s/pose at 3 images + 2 s settle, ~180 poses/h (spec: 240). Lossless-compressed RAW or 2 images/pose if this limits a run.
- The first tether frames clip at 16383 (scene-dependent): set exposure on the actual target (iPad) before step 4.
- Lens firmware (MENU → Setup → Setup Option → Version); body = 3.01 (from gphoto2).
- Gripper target: spec §8 prefers a ~60 mm ChArUco (AprilTag outer corners shift under blur); `so101-metrology/` has AprilTag grids (20 mm). Decide before step 3.
- `gen_targets.py` rewrites `targets_spec.json` with `measured_*` = null: keep measured scales in a separate file.
- iPad as world reference: dedicated for the campaign, fixed to the table, auto-lock off, on power, fixed brightness matched to the room.
- Cardstock changes size with humidity (~0.1 %): measure scale on the day, or mount on glass/aluminum.

## Baseline (revision 1, 2026-10-28)
Spec §3 Phase 2 + `docs/ROADMAP.md` "To do in Phase 2": instrument characterization (intrinsics from iPad,
targets caliper-measured, iPad↔board cross-check, mount stability, depth check vs dial indicator) →
repeatability/backlash → accuracy grid → per-joint sag → dynamics → thermal drift. Printed (or aluminum)
reference ChArUco board on the table defines the world frame.

## Deviation log
Newest last. Each entry: what changed, why, evidence.

| # | Date | Change | Why | Evidence |
|---|------|--------|-----|----------|
| D1 | 2026-10-28 | The iPad's on-screen ChArUco is both the intrinsics target and the world reference; the printed reference board is dropped (user) | Flat rigid glass, no humidity change, pattern set in software; removes the board↔gripper scale chain's third member | Discussion 2026-10-28; error budget above |
| D2 | 2026-10-28 | A caliper anchors absolute scale (≤ 0.05 %); the iPad print fit alone gives only the printer's x/y ratio | Apple's 264 ppi is rounded (~0.1–0.3 %) → up to ~0.6 mm error on the accuracy grid | Error budget above |
| D3 | 2026-10-28 | Camera code lives in `src/so101/` behind the `Sensor` interface, not the spec §10 `so101_metrology/` layout | One package; Phase 1 interfaces (`Arm`, `SafeArm`, manifest) are reused | CLAUDE.md conventions |
| D4 | 2026-10-28 | Caliper-dependent work deferred (no caliper today, user): step 3's scale measurements and anything that needs absolute scale (step 10; absolute depth). Step 4 (intrinsics, including generating the pixel-exact iPad ChArUco) runs before step 3 | Intrinsics don't depend on board scale, so the iPad calibration is valid without the caliper; relative tests (steps 5–9, 11) are scale-insensitive | Error budget above |
| D5 | 2026-10-28 | Working distance ~0.38 m (15 in, user) instead of the spec's 0.5 m; framing and focus set on the Phase 1 pose tour before intrinsics | Field ~0.39 m wide, 56 µm/px, ~1.7 camera px per iPad pixel (less moiré), DoF ~±4 cm; intrinsics are only valid at the focus they are shot at | `scripts/motion/pose_tour.py` |
| D6 | 2026-10-28 | Working distance now ~0.7 m (supersedes D5's 0.38 m; user): arm at the table's SW corner, camera ~0.7 m away on a table-top tripod, iPad ~25 cm north of the arm, all in one frame | DoF at f/11: ~114 mm (≤ 4 px blur) to ~330 mm (≤ 8 px); frame ~685 × 457 mm, ~98 µm/px; pose noise still ≪ budget (lateral ~5 µm, depth ~0.06 mm for a 60 mm target at 0.05 px). Intrinsics must be shot at this focus | setup photo 2026-10-28 |
| D7 | 2026-10-28 | Step 5b added: static gripper-target + iPad stability (20–30 min), arm resting then holding, before any motion measurements (user) | Step 5 measured the rig with the iPad only; the quantity Phase 2 measures is the target's pose relative to the iPad, so its noise floor and drift (small tags, mount, servo holding) need their own baseline | step 5 run `20261028-182722` |
