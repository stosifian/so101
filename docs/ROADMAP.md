# Roadmap

## Phase 1 — bring-up & function (inside the 30-day return window)

**Plan, status and deviation log: [`docs/phase-1-plan.md`](phase-1-plan.md)** (source of truth; updated on every change).

| # | Step | Status |
|---|------|--------|
| 0–2 | Bootstrap, ports, register dump | done 2026-10-26 |
| 3 | Torque-off telemetry + limit sweeps | done 2026-10-26 |
| 4 | SafeArm + mock/sim tests | done 2026-10-26 |
| 5 | First powered move (5a shadow → 5b hold → 5c move → 5d stick-slip) | done 2026-10-26 |
| 6 | Per-joint sweeps: 6a contact detection → 6b sweeps from a clear pose → 6c `wait_until_settled` → 6d timing | in progress |
| 7 | Calibration: ticks → MuJoCo Menagerie model mapping | planned |
| 8 | Parameterized FK + MuJoCo collision model + measured table plane | planned |
| 9 | IK + collision-checked motion; relax wrist_roll limit; wide sweeps; Cartesian demo | planned |
| 10 | Endurance/thermal loop, gripper squeeze (current scale), manifest | planned |
| 11 | Acceptance write-up, go/no-go | planned |

## Phase 2 — camera metrology (spec §5–§9)

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
- **Measure structural ring-down** (camera or IMU): the end effector wobbles after stops while joint encoders already read steady (step 6). Set `wait_until_settled(dwell_s=…)` from the measured ring-down; 0.5 s is a placeholder.
- **Base reference check (stock clamps kept, user decision 2026-10-26):** a firm hand push shifted the arm in the stock clamps (step 6a). Phase 2 applies no such forces, but an unnoticed shift (bump, cable tug, contact) would silently invalidate calibration and the table plane. Mark the base outline on the table, and at the start of every session and after any contact re-touch the reference point recorded at Phase 1 step 8. If it differs, re-clamp and re-measure before trusting world-frame numbers.
- gphoto2 ↔ A7 IV validation (50 consecutive captures). Sony Remote SDK is the fallback behind the same `Sensor` interface.
- Camera settings checklist (spec §6). Photograph the menus.
- Generate ChArUco targets in OpenCV, print, and caliper-measure them.
- iPad intrinsics → mount-stability run → 20-pose dry run.
- Run order: repeatability → backlash → grid accuracy → per-joint sag → dynamics → thermal drift.
- Open items in spec §11.

## Phase 3 — chosen from Phase 2 results (spec §3)

- `sensors/` takes a BMI270 IMU (reuse serial code from `~/Desktop/Python/imu-diff-sim`) or a wrist camera via the `Sensor` protocol.
- A stiffness-model compensation layer would sit between `kinematics` and `motion` (corrected IK).
- Policy/VLA route: an optional LeRobot adapter wraps `SafeArm`. The core deliberately doesn't depend on LeRobot normalization.
