# Phase 2 plan (camera metrology)

The source of truth for Phase 2. **Update this file every time the plan changes:** edit the table and
add a dated entry to the deviation log, with the reason. `docs/ROADMAP.md` only summarizes it.
Spec: `so101-metrology-plan.md` §3 (Phase 2), §6–§9. Targets + generator: `so101-metrology/`.

Last updated: 2026-10-28 (revision 6).

## Current plan

| # | Step | Done when | Status | Arm moves? | Changed vs baseline |
|---|------|-----------|--------|------------|---------------------|
| 0 | Setup: this plan; camera interface (gphoto2 + mock backends, behind `Sensor`); opencv + rawpy in the env | Mock camera passes tests | **done 2026-10-28**: `so101.sensors` (`Sensor`, `Camera`, `Gphoto2Camera` via the CLI with a per-capture timeout, `MockCamera`), `scripts/camera/tether_check.py`, opencv 5.0.0 + rawpy 0.27.1; 224 tests | no | camera code in `src/so101/` (D3) |
| 1 | **Tethering:** 50 consecutive captures over gphoto2; record body + lens firmware | 50/50, no hang/disconnect; else Sony Camera Remote SDK behind the same interface | **done 2026-10-28** (`20261028-131445_tether_camera`): **50/50**, no hang; 5.9 s per capture (median; p95 5.96, max 6.09) for a 67.6 MB uncompressed RAW; body firmware 3.01, gphoto2 2.5.32. gphoto2 accepted; Sony SDK not needed | no | — |
| 2 | **Camera settings** (spec §6): M, fixed ISO + WB, MF taped, IBIS off, RAW (decoded without lens corrections), f/11, 1/60 s, EFC, save to PC, auto-review + power save off | Menus photographed; settings hash in the run header | **done 2026-10-28**: settings in `config/camera_a7iv.yaml`: f/11, 1/30 s, ISO 1600, Daylight, RAW, MF, sdram (read back over USB every run); SteadyShot off, lens comp. off, Mechanical + e-front curtain, auto review off (user-confirmed; Power Save Start Time greyed out while tethered). Exposure from test shots `…133453`/`142131`/`142714`: iPad whites 61–91 %, unclipped. Glare: dark cloth behind the camera → black squares 0.3–1.7 % (`…143317`), one residual streak away from corners. Stability: 3 shots / 10 s, shift ≤ 0.07 px, residual 0.03 px RMS (`20261028-143956_stability_camera`). Menu photos: optional | no | — |
| 3 | **Targets + scale:** iPad ChArUco (pixel-exact, native resolution); caliper the gripper print; iPad scale via print-vs-iPad fit (multi-view, relative pose free, not assumed coplanar) or caliper jaws imaged on the screen | iPad and gripper-target scales known to ≤ 0.05 % | **deferred** until a caliper is available (D4); step 4 goes first |  no | iPad replaces the printed reference board (D1); caliper anchors absolute scale (D2) |
| 4 | **Intrinsics:** ~30 iPad views at working distance (~0.5 m), focus taped, views covering the frame corners, tilt ≤ ~40° | Reprojection 0.1–0.2 px; no moiré pattern in detected-vs-reprojected corner plots; f ≈ 6840 px | **tooling done 2026-10-28**: `config/targets/ipad_charuco.yaml` (13×9 squares of 168 display px ≈ 16.2 mm, DICT_6X6_250, 96 corners) → pixel-exact PNG (`scripts/camera/make_ipad_charuco.py`, corners exact to design); `scripts/camera/calibrate_intrinsics.py` capture (live corner count + coverage map) / analyze (`data/derived/<run>/intrinsics.json` + residual plots); synthetic test recovers f within 1 %. Awaiting ~30 views | no | — |
| 5 | **Rig stability:** static iPad + gripper target 1–2 h; dial-indicator depth check | Tripod/table drift ≪ 0.25 mm | planned | no | — |
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
