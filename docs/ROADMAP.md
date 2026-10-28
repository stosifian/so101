# Roadmap

## Phase 1 — bring-up & function (inside the 30-day return window)

**Plan, status and deviation log: [`docs/phase-1-plan.md`](phase-1-plan.md)** (source of truth; updated on every change).

| # | Step | Status |
|---|------|--------|
| 0–2 | Bootstrap, ports, register dump | done 2026-10-26 |
| 3 | Torque-off telemetry + limit sweeps | done 2026-10-26 |
| 4 | SafeArm + mock/sim tests | done 2026-10-26 |
| 5 | First powered move (5a shadow → 5b hold → 5c move → 5d stick-slip) | done 2026-10-26 |
| 6 | Per-joint sweeps: 6a contact detection → 6b sweeps from a clear pose → 6c `wait_until_settled` → 6d timing | done 2026-10-27 |
| 7 | Calibration: ticks → MuJoCo Menagerie model mapping | done 2026-10-27 (by eye) |
| 8 | Parameterized FK + MuJoCo collision model + table plane (base underside) validated by touches | done 2026-10-27 |
| 9 | IK + collision-checked motion + Cartesian demo (wrist_roll limit kept; wide sweeps deferred) | done 2026-10-27 |
| 10 | Endurance/thermal loop, gripper squeeze (current scale), manifest | done 2026-10-27 |
| 11 | Acceptance write-up, go/no-go | done 2026-10-27: GO (user); teleop check dropped |

## Phase 2 — camera metrology (spec §5–§9)

**Plan, status and deviation log: [`docs/phase-2-plan.md`](phase-2-plan.md)** (source of truth; updated on every change).
Key decision (D1, 2026-10-28): the iPad's on-screen ChArUco is both the intrinsics target and the world reference; a caliper anchors absolute scale (D2).

| # | Step | Status |
|---|------|--------|
| 0–2 | Camera interface, gphoto2 tethering (50 captures), settings | planned |
| 3–5 | Target scales, intrinsics (iPad), rig stability | planned |
| 6–11 | Dry run, ring-down, repeatability/backlash, sag, accuracy grid, thermal drift | planned |
| 12 | Write-up + Phase 3 choice | planned |


**Phase 1 must already provide:**
- `motion.wait_until_settled()` that uses both a position tolerance and the STS3215 `Moving` register, and takes an `approach_dir` (for backlash).
- `Arm.snapshot()`: ticks/load/temp/voltage for all joints, monotonic timestamp (before/after each image).
- A manifest with optional camera fields; the Phase 2 capture loop extends it, not replaces it.
- A parameterized FK. The kinematic fit adjusts link offsets, joint zero offsets, and the target→wrist transform (6 DoF).
- Phase 1 thermal-loop data kept as a drift baseline.

**Phase 1 copies LeRobot patterns by default (CLAUDE.md). Re-check these before Phase 2 relies on them:**
- LeRobot defaults target teleop, not metrology: PID (P=16), accel and speed profiles, settle logic. Measure rather than inherit.
- Copied patterns are pinned to a LeRobot commit. Don't silently re-sync to a newer version mid-campaign; it changes the baseline.
- Servo V/T registers on IDs 4–6 glitch in ~1 % of samples (servo ADC, both arms; step 3). Median-filter temperature/voltage before the thermal-drift analysis.
- Anything LeRobot lacks (Moving-based settling with `approach_dir`, parameterized FK, kinematic fit) is ours and gets its own tests.
- If a copied piece limits accuracy (e.g. sync-read timing skew across joints, clamping behaviour), replace it behind its interface (`ServoBus`/`Arm`/`SafeArm`/`Sensor`) and record why in the decision log.

**To do in Phase 2:**
- **Thermal drift needs real heating:** the Phase 1 endurance loop (step 10c) held repeatability within ±3 ticks for 15 min but warmed the servos only ≤ 1 °C (low loads). For the thermal-drift run, load the joints (e.g. hold the arm extended against gravity at ~20–30 % load, or longer/faster cycles) and track the reference pose as temperature rises; the 10c run is the cold baseline.
- **Measure structural ring-down** (camera or IMU): the end effector wobbles after stops while joint encoders already read steady (step 6). Set `wait_until_settled(dwell_s=…)` from the measured ring-down; 0.5 s is a placeholder. **Wobble depends on the moving joint and the pose** (user, step 6c: hardly any on pan with the arm folded, noticeable on lift with the arm extended; lever arm and inertia), so measure the worst case (extended arm, lift/elbow moves) or make the dwell depend on reach.
- **Base reference check (stock clamps kept, user decision 2026-10-26):** a firm hand push shifted the arm in the stock clamps (step 6a). Phase 2 applies no such forces, but an unnoticed shift (bump, cable tug, contact) would silently invalidate calibration and the table plane. Mark the base outline on the table, and at the start of every session and after any contact re-touch the reference point recorded at Phase 1 step 8. If it differs, re-clamp and re-measure before trusting world-frame numbers.
- gphoto2 ↔ A7 IV validation (50 consecutive captures). Sony Remote SDK is the fallback behind the same `Sensor` interface.
- Camera settings checklist (spec §6). Photograph the menus.
- Generate ChArUco targets in OpenCV, print, and caliper-measure them.
- iPad intrinsics → mount-stability run → 20-pose dry run.
- Run order: repeatability → backlash → grid accuracy → per-joint sag → dynamics → thermal drift.
- Open items in spec §11.

## Phase 3 — chosen from Phase 2 results (spec §3)

- **Deferred from Phase 1 step 9 (D16): wide lift/elbow sweeps** (lift past vertical where gravity flips, near-horizontal loads; elbow beyond 53°), now safe to run with `joint_sweep.py` once the collision model is wired into it (~10 lines). Input for the stiffness-compensation layer: the Cartesian demo's 8.5 mm off-line error was mostly gravity sag.
- **Grasping near the table needs a grasp mode in the collision model** (user, 2026-10-27): per-body table margins / allowed contacts (fingertips may approach or touch the table during a grasp; wrist, forearm and camera mount keep a real margin), like MoveIt's allowed collision matrix. `so101.kinematics.collision` already reports distances per link. How close the fingertips can safely go depends on Phase 2's calibration accuracy (by-eye mapping error is measured at step 8c). Contact detection also needs the gripper exempt or separately thresholded while grasping (step 10).
- `sensors/` takes a BMI270 IMU (reuse serial code from `~/Desktop/Python/imu-diff-sim`) or a wrist camera via the `Sensor` protocol.
- A stiffness-model compensation layer would sit between `kinematics` and `motion` (corrected IK).
- Policy/VLA route: an optional LeRobot adapter wraps `SafeArm`. The core deliberately doesn't depend on LeRobot normalization.
