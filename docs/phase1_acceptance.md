# Phase 1 acceptance: SO-101 follower bring-up (go/no-go)

Date: 2026-10-27. Scope: steps 0–10 of `docs/phase-1-plan.md` (deviation log D1–D16). Evidence = run IDs under
`data/raw/`, details in `docs/hardware_log.md`. Test suite: 219 tests passing.

## Recommendation: GO (keep the arm)

All 12 servos are healthy, the follower moves accurately and repeatably under a safety layer that has been
proven on the real arm (including two real collisions it now prevents or catches), and the Phase 2 metrology
prerequisites are in place. No hardware defect was found that warrants a return. One item to inspect before
the return window closes: the wrist servos (IDs 4–6) show occasional single-sample sensor glitches and rare
missed replies, consistent with wiring or grounding at the end of the chain.

## What was verified

| Area | Result | Evidence |
|---|---|---|
| Servo health | 12/12 STS3215 respond, firmware 3.10, follower 11.8–11.9 V, leader 4.9 V, no error flags; factory homing offsets present, no firmware angle limits | `20261026-112606_regdump_*` |
| Communication | 6-servo telemetry read ≈ 3.0 ms; 100 Hz loop holds; 15-min endurance: 0 failed reads in 96k samples | `20261026-121235`, `20261027-142432_endurance` |
| Safety layer | SRAM-only write whitelist (EEPROM writes impossible); torque-on with goal := present + caps, read back: **0 ticks of motion at torque-on**; debounced trips; two-stage stop (freeze, then torque off) | `20261026-144830_hold`, `…145026` push test |
| Contact detection | Following error by joint role → freeze. Live push test froze at 14 ticks; replay catches the table contact 2 s before peak load; caught a real gripper–table contact on day 2 | `20261026-170706_hold`, `20261027-091714_sweep` |
| Collision model | MuJoCo geometry check on the calibrated model: all 3 real table contacts flagged (−2 to −20 mm), normal motion ≥ 27 mm clear; runtime freeze before a colliding goal is written | `tests/test_collision.py` |
| Motion | P=32: 0 holding jitter. Stick-slip at 50 ticks/s, smooth at 150. Stiffness ≈ 0.85 % load per tick on every joint | `20261026-145837_move`, `…154001` |
| Settle + repeatability | Same-direction repeatability 0–2 ticks (pan, 6c) and ≤ ±3 ticks over 30 visits in 15 min, no drift | `20261027-093309_settle`, `…142432_endurance` |
| Calibration | Tick→model mapping (by eye + table-touch refinement): tool height error 1.5 mm RMS at table level (max 3.7 mm, leave-one-out) | `20261027-124750_model_check` |
| Kinematics | Parameterized FK = MuJoCo to machine precision; DLS IK with limits; Cartesian line: commanded path within 0.6 mm | `docs/fk_math.md`, `docs/ik_math.md`, `20261027-135238_cartesian` |
| Gripper | Torque-capped squeeze to 37.6 % load (46 ticks short); `Present_Current` scale plausible above ~10 % load | `20261027-141349_squeeze` |
| Thermal | 15-min loop: ≤ +1 °C (low load); 55 °C trip never approached | `20261027-142432_endurance` |

## Known limitations (with numbers)

| Limitation | Size | Handling |
|---|---|---|
| Gravity sag + following lag (P-only servos) | Cartesian line measured up to 8.5 mm off (−8.4 mm vertical sag, 7 mm lag) | Phase 3 stiffness compensation |
| Direction-dependent stopping (hysteresis) | 2–6 ticks unloaded, 8–20 ticks on gravity joints | `approach(approach_dir)` for metrology |
| Stick-slip at low speed | 51 % of samples stalled at 50 ticks/s, 12 % at 150 | Default motion 150 ticks/s |
| Servo V/°C readings glitch | Single samples; up to 103 °C on a driven joint | Debounced trips; median displays |
| Calibration (by eye + 1 refinement) | ~1.5–2.5 mm at the tool | Phase 2 camera fit |
| Self-collision blind spot | Folded-pose gripper↔shoulder contact unresolved (model says 13–14 mm) | wrist_roll limit 2550 kept; warn-only |
| Encoders don't see structural wobble | Visible at stops on extended moves | Settle dwell; measure ring-down in Phase 2 |
| Wrist reach near the table | Fingers vertical at 8 cm height exceed wrist_flex limit | Tilt ~25° |

## Phase 2 prerequisites

| Obligation | Status |
|---|---|
| `Arm.snapshot()` (all joints, monotonic before/after) | done (step 3a) |
| `wait_until_settled(approach_dir)` using `Moving` + steadiness | done (step 6c) |
| Parameterized FK (zero offsets, link twists, tool transform) | done (step 8a) |
| Run manifest with optional camera fields | done (step 10d) |
| Drift baseline | done as a cold repeatability baseline (step 10c); thermal run needs load (Phase 2 to-do) |

## Carried forward

- Phase 2: base reference check each session; measure structural ring-down; thermal drift under load.
- Phase 3: collision grasp mode; wide lift/elbow sweeps (D16); gravity/stiffness compensation.
- Open: gripper lower limit (2082) above its closed reading (≈ 2048), kept for now (user); wrist servo
  wiring check (IDs 4–6).
