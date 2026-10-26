# Phase 1 plan (bring-up & function)

The source of truth for Phase 1. **Update this file every time the plan changes:** edit the table and
add a dated entry to the deviation log, with the reason. `docs/ROADMAP.md` only summarizes it.

Last updated: 2026-10-26 (revision 2).

## Current plan

| # | Step | Status | Arm moves? | Changed vs baseline |
|---|------|--------|------------|---------------------|
| 0 | Repo bootstrap (uv env, git, notes, permissions) | done 2026-10-26 | no | — |
| 1 | Port discovery, follower/leader mapping | done 2026-10-26 | no | — |
| 2 | Read-only register dump (all 12 servos) | done 2026-10-26 (`20261026-112606_regdump_*`) | no | — |
| 3 | Torque-off live readout + telemetry (poller/hub, parquet, TUI, Rerun); by-hand limit sweeps, follower + leader | done 2026-10-26 (`20261026-130808_telemetry_follower`, `20261026-131126_telemetry_leader`) | no | + bus ownership, ADC-glitch investigation (D1) |
| 4 | `SafeArm` + mock tests: SRAM-only write whitelist; goal := present + speed/accel caps before torque-on, all read back; software limits; start-outside-limits rule; debounced trips; one owner per bus; shadow mode; SimBus | done 2026-10-26 (122 tests at the time) | no | + shadow mode, SimBus, debounced trips (D2) |
| 5 | First powered move, staged: 5a shadow → 5b hold (+ push test) → 5c one joint → 5d stick-slip experiment | done 2026-10-26 (5c `20261026-145837_move_follower`) | yes | staged into 4 parts; two-stage Ctrl-C (freeze) (D3, D4) |
| 6 | Per-joint sweeps with logging | **in progress**: pan, wrist_roll, gripper, wrist_flex (anchored), elbow (anchored) done; shoulder_lift hit the table | yes | split into 6a–6d (D6) |
| 6a | **Contact detection** in SafeArm: following error (≈35 ticks moving / ≈12 holding, 0.3 s) + holding drift (> 10 ticks with no new goal) → **freeze**. Thresholds from replaying today's runs (normal sweeps pass, both contacts trigger). Then a controlled hardware test (slow pan into a soft obstacle must freeze). Fold in: `read_block` length check, per-sample error-byte logging | next | one slow test | **new** (D6) |
| 6b | Finish sweeps from a **pose clear of the table**, placed by hand (torque off); `--hold-at-end` waits for the user to support the arm before torque-off. Goals: shoulder_lift's real gravity profile, wider elbow / wrist_flex ranges | planned | yes | changed: not from rest pose (D6) |
| 6c | `wait_until_settled(tol, approach_dir)` using `Moving` + position tolerance (≥ ±3 ticks; hysteresis 2–13 ticks measured), with tests | planned | yes | **scheduled** (Phase 2 obligation, previously unscheduled) (D7) |
| 6d | TUI vs no-TUI loop-timing comparison; close step 6 | planned | yes | — |
| 7 | **Calibration = mapping ticks to model angles:** per-joint sign + zero offset (software, no EEPROM) between ticks and the MuJoCo Menagerie SO-101 model, from known physical poses, checked on ≥ 2 poses; refuse to start if the arm's pose doesn't fit the mapping | planned | to reference poses | reframed (was "own zero offsets") (D8) |
| 8 | **Parameterized FK** (own chain code, because Phase 2 fits its parameters; checked against MuJoCo) **+ collision model:** MuJoCo self-collision + table; **table plane measured** by touching the gripper tip to ≥ 3 points (FK → plane, ~10 mm margin). Required test cases: today's two contacts | planned | by hand only | + collision model, table plane, MuJoCo (D5, D8) |
| 9 | IK (DLS, joint-limit aware) + **collision-checked motion:** every plan checked before torque-on; SafeArm checks each goal at runtime and freezes on a predicted contact. Then relax the rest-pose wrist_roll limit where it's clear; wide shoulder_lift / elbow sweeps; Cartesian demo | planned | yes | + collision checks, wide sweeps moved here (D5, D6) |
| 10 | 15-min endurance / thermal loop (Phase 2 drift baseline) + gripper squeeze (doubles as the `Present_Current` scale check); start the run **manifest** (optional camera fields for Phase 2) | planned | yes | + current check, manifest scheduled (D7) |
| 11 | Phase 1 acceptance write-up, go/no-go before the return window closes; optional leader→follower teleop check | planned | optional | — |

### Guardrail (scope)
Collision work in Phase 1 is **validation only**: table plane + self-collision from the Menagerie model, used to
refuse a plan before torque-on or freeze at runtime. **No collision-avoiding path planning in Phase 1.**

### Phase 2 obligations (from ROADMAP) and where they land
| Obligation | Where | Status |
|---|---|---|
| `Arm.snapshot()` (all joints, monotonic before/after timestamps) | step 3a | done |
| `wait_until_settled(tol, approach_dir)` using `Moving` + tolerance | 6c | planned |
| Parameterized FK | 8 | planned |
| Run manifest with optional camera fields | 10 | planned |
| Thermal-loop data as drift baseline | 10 | planned |

### Open items carried along (details: `notes/claude/2026-10-26.md` → Open questions)
- `read_block` length check + per-sample packet error-byte logging → folded into 6a
- `Present_Current` scale unverified → step 10 squeeze (or a PSU ammeter)
- Loop stalls up to ~100 ms when a wrist servo misses a reply (SDK timeout) → before step 9's timing-sensitive work
- SimBus has no acceleration profile → only if needed

## Baseline (revision 1, 2026-10-26 start of day)
| # | Step |
|---|------|
| 0 | Repo bootstrap |
| 1 | Port discovery, follower/leader mapping |
| 2 | Read-only register dump |
| 3 | Torque-off live readout + telemetry; record safe tick limits; leader by-hand sweep |
| 4 | Safety layer (`SafeArm`) + mock tests (goal := present + caps before torque-on; software limits; temp trip below 70 °C) |
| 5 | First powered move (one joint, slow) |
| 6 | Per-joint sweeps with logging |
| 7 | Home calibration (own zero offsets, no EEPROM) |
| 8 | FK from URDF (parameterized) |
| 9 | IK (DLS, joint-limit aware) + Cartesian demo |
| 10 | 15-min endurance/thermal loop + gripper squeeze |
| 11 | Phase 1 acceptance write-up, go/no-go |

## Deviation log
Newest last. Each entry: what changed, why, evidence.

| # | Date | Change | Why | Evidence |
|---|------|--------|-----|----------|
| D1 | 2026-10-26 | Step 3 added bus ownership (`claim`/`release`) and an ADC-glitch investigation | Concurrent bus use can cross replies; V/°C readings glitched on IDs 4–6 | A/B test: block vs per-register reads return the same glitched values |
| D2 | 2026-10-26 | Step 4 added shadow mode, SimBus and debounced trips; voltage/overheat Status bits debounced | Real glitches would false-trip an immediate trip; replay found the leader gripper's Status bit following its voltage glitches | replay fixture `telemetry_20261026-121235_both.parquet` |
| D3 | 2026-10-26 | Step 5 staged as 5a shadow / 5b hold / 5c move / 5d stick-slip | First writes to real hardware; each stage adds one risk | runs `…142826_shadow`, `…144830_hold`, `…145837_move`, `…154001/154057_move` |
| D4 | 2026-10-26 | Two-stage Ctrl-C: 1st = freeze (hold, torque on), 2nd = torque off | Torque-off drops gravity-loaded joints, which is bad when a collision is imminent | freeze verified on hardware: 13 ticks / 100 ms stop (`…155559_move`) |
| D5 | 2026-10-26 | Collision model + checks added to steps 8–9; library changed pinocchio/coal → **MuJoCo + Menagerie SO-101** | Two contacts in step 6; SO-ARM community uses MuJoCo for exactly this (robocurve #26); LeRobot has none | `…160843_sweep` (gripper vs shoulder), `…162655_sweep` (wrist vs table) |
| D6 | 2026-10-26 | Step 6 split: 6a contact detection, 6b sweeps from a clear pose with hold-at-end; wide lift/elbow sweeps moved to step 9 | The load trip didn't catch the table contact (39 % < 50 %); the rest pose drives the wrist into the table when lifting | following error 35–57 vs normal ≈ 20 ticks; holding elbow pushed 21 ticks |
| D7 | 2026-10-26 | Scheduled `wait_until_settled` (6c) and the manifest (10) | Phase 2 obligations that weren't scheduled in any step | `Moving` semantics and hysteresis measured in 5c/6 |
| D8 | 2026-10-26 | Step 7 reframed as the tick→model mapping + refuse-to-start check; table plane measurement added to step 8 | A collision model is only valid if ticks map correctly onto it (rig-dependent zeros, robocurve #26) | — |
| D9 | 2026-10-26 | Decisions inside steps: keep P = 32; motion default ~150 ticks/s; parking at the soft limit; wrist_roll max 3438 → 2550 (rest-pose limit) | Measured: 0 jitter at P=32, stick-slip depends on speed; contact at roll ≈ 2645 in rest pose | hardware log, `config/arm_follower.yaml` |
