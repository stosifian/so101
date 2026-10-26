# Roadmap

## Phase 1 — bring-up & function (inside the 30-day return window)

| # | Step | Status |
|---|------|--------|
| 0 | Repo bootstrap (uv env, git, notes, permissions) | done 2026-10-26 |
| 1 | Port discovery, follower/leader mapping | done 2026-10-26 |
| 2 | Read-only register dump (all 12 servos) | done 2026-10-26 (runs 20261026-112606_regdump_*) |
| 3 | Torque-off live readout + telemetry (Rerun, TUI, parquet); record safe tick limits; leader by-hand sweep | |
| 4 | Safety layer (`SafeArm`) + mock tests. Must include: goal:=present + speed/accel caps before torque-on; software limits (no firmware limits set); temp trip well below the 70 °C firmware limit | |
| 5 | First powered move (one joint, slow) | |
| 6 | Per-joint sweeps with logging | |
| 7 | Home calibration (own zero offsets, no EEPROM) | |
| 8 | FK from URDF (parameterized) | |
| 9 | IK (DLS, joint-limit aware) + Cartesian demo | |
| 10 | 15-min endurance/thermal loop + gripper squeeze | |
| 11 | Phase 1 acceptance write-up, go/no-go | |

## Phase 2 — camera metrology (spec §5–§9)

**Phase 1 must already provide:**
- `motion.wait_until_settled()` that uses both a position tolerance and the STS3215 `Moving` register, and takes an `approach_dir` (for backlash).
- `Arm.snapshot()`: ticks/load/temp/voltage for all joints, monotonic timestamp (before/after each image).
- A manifest with optional camera fields; the Phase 2 capture loop extends it, not replaces it.
- A parameterized FK. The kinematic fit adjusts link offsets, joint zero offsets, and the target→wrist transform (6 DoF).
- Phase 1 thermal-loop data kept as a drift baseline.

**To do in Phase 2:**
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
