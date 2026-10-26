# Roadmap

## Phase 1 — bring-up & function (inside the 30-day return window)

| # | Step | Status |
|---|------|--------|
| 0 | Repo bootstrap (uv env, git, notes, permissions) | done 2026-10-26 |
| 1 | Port discovery, follower/leader mapping | done 2026-10-26 |
| 2 | Read-only register dump (all 12 servos) | done 2026-10-26 (runs 20261026-112606_regdump_*) |
| 3 | Torque-off live readout + telemetry (Rerun, TUI, parquet); record safe tick limits; leader by-hand sweep | done 2026-10-26 (sweeps 20261026-130808_telemetry_follower, 20261026-131126_telemetry_leader) |
| 4 | Safety layer (`SafeArm`) + mock tests. Must include: goal:=present + speed/accel caps before torque-on; software limits (no firmware limits set); temp trip well below the 70 °C firmware limit; **one owner per bus**: SafeArm's loop claims the follower bus (no poller runs) and does read → safety check → write → publish; safety checks run inline, not as a hub consumer; samples carry the commanded goal; temp/voltage trips debounced (single-sample ADC glitches on IDs 4–6, both arms, step 3); **starting outside limits** (rest pose: elbow_flex, gripper) must be allowed, with moves permitted only back toward the range, never a clamp-jump | done 2026-10-26 (sim-tested: `scripts/sim/safearm_demo.py`; 122 tests) |
| 5 | First powered move, staged: 5a shadow mode (torque off, no writes) → 5b torque-on hold → 5c one joint, small, slow. Verify `Moving` semantics, tracking, overshoot, jitter (P 32 vs 16); compare with SimBus. **Parking:** joints end at their soft limit; at torque-off the elbow settles the last ~4° (2878 → 2927) onto its rest stop, so keep a hand on it | |
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

**Phase 1 copies LeRobot patterns by default (CLAUDE.md). Re-check these before Phase 2 relies on them:**
- LeRobot defaults target teleop, not metrology: PID (P=16), accel and speed profiles, settle logic. Measure rather than inherit.
- Copied patterns are pinned to a LeRobot commit. Don't silently re-sync to a newer version mid-campaign; it changes the baseline.
- Servo V/T registers on IDs 4–6 glitch in ~1 % of samples (servo ADC, both arms; step 3). Median-filter temperature/voltage before the thermal-drift analysis.
- Anything LeRobot lacks (Moving-based settling with `approach_dir`, parameterized FK, kinematic fit) is ours and gets its own tests.
- If a copied piece limits accuracy (e.g. sync-read timing skew across joints, clamping behaviour), replace it behind its interface (`ServoBus`/`Arm`/`SafeArm`/`Sensor`) and record why in the decision log.

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
