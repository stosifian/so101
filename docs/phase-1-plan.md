# Phase 1 plan (bring-up & function)

The source of truth for Phase 1. **Update this file every time the plan changes:** edit the table and
add a dated entry to the deviation log, with the reason. `docs/ROADMAP.md` only summarizes it.

Last updated: 2026-10-27 (revision 3).

## Current plan

| # | Step | Status | Arm moves? | Changed vs baseline |
|---|------|--------|------------|---------------------|
| 0 | Repo bootstrap (uv env, git, notes, permissions) | done 2026-10-26 | no | — |
| 1 | Port discovery, follower/leader mapping | done 2026-10-26 | no | — |
| 2 | Read-only register dump (all 12 servos) | done 2026-10-26 (`20261026-112606_regdump_*`) | no | — |
| 3 | Torque-off live readout + telemetry (poller/hub, parquet, TUI, Rerun); by-hand limit sweeps, follower + leader | done 2026-10-26 (`20261026-130808_telemetry_follower`, `20261026-131126_telemetry_leader`) | no | + bus ownership, ADC-glitch investigation (D1) |
| 4 | `SafeArm` + mock tests: SRAM-only write whitelist; goal := present + speed/accel caps before torque-on, all read back; software limits; start-outside-limits rule; debounced trips; one owner per bus; shadow mode; SimBus | done 2026-10-26 (122 tests at the time) | no | + shadow mode, SimBus, debounced trips (D2) |
| 5 | First powered move, staged: 5a shadow → 5b hold (+ push test) → 5c one joint → 5d stick-slip experiment | done 2026-10-26 (5c `20261026-145837_move_follower`) | yes | staged into 4 parts; two-stage Ctrl-C (freeze) (D3, D4) |
| 6 | Per-joint sweeps with logging | **done 2026-10-27**: all six joints swept (lift from a cleared pose to 79°); contact detection, settle/approach, timing done. Wide ranges and over-vertical lift deferred to step 9 (collision-checked) | yes | split into 6a–6d (D6) |
| 6a | **Contact detection** in SafeArm: following error (≈35 ticks moving / ≈12 holding, 0.3 s) + holding drift (> 10 ticks with no new goal) → **freeze**. Thresholds from replaying today's runs (normal sweeps pass, both contacts trigger). Then a controlled hardware test (slow pan into a soft obstacle must freeze). Fold in: `read_block` length check, per-sample error-byte logging | code done 2026-10-26 (replay: table contact frozen at 5.2 s by idle-elbow drift; 0 false contacts on 5 normal runs; gentle contact not caught, as documented). **Hardware test passed** (`20261026-170706_hold_follower`: hand push on idle pan froze at 14 ticks, held, torque off on Ctrl-C). Optional: moving-case test | one slow test | **new** (D6) |
| 6b | Finish sweeps from a **pose clear of the table**, placed by hand (torque off); `--hold-at-end` keeps holding after the sweep until the user supports the arm and presses Ctrl-C. Goals: shoulder_lift's real gravity profile, wider elbow / wrist_flex ranges | **done 2026-10-27**: shoulder_lift gravity profile to 79° (gripper cleared by hand); wider elbow/wrist_flex skipped (step 9) | yes | changed: not from rest pose (D6) |
| 6c | `wait_until_settled` (`Moving` == 0 + steady position; error to goal reported, `tol` optional) + `approach(approach_dir)` (overshoot to the far side) + **dwell after encoder settle** (structural wobble the encoders can't see) + **min-jerk ramps** (default) | **done 2026-10-27** (`20261027-093309_settle_follower`: 12/12 settled ≈ 0.21 s; same-direction repeatability 0–2 ticks; hysteresis 3–6) | yes | scheduled (D7); + dwell, min-jerk (D11) |
| 6d | TUI vs no-TUI loop-timing comparison; close step 6 | **done 2026-10-27**: TUI costs timing (overruns 4 → 21 / 30 s, p99 period 11.6 → 14.0 ms, max 13 → 28 ms). No TUI for timing-sensitive runs | yes | — |
| 7 | **Calibration = mapping ticks to model angles:** per-joint sign + zero offset (software, no EEPROM) between ticks and the MuJoCo Menagerie SO-101 model, from known physical poses, checked on ≥ 2 poses; refuse to start if the arm's pose doesn't fit the mapping | **done 2026-10-27** (verified by eye): zeros from single-joint references + live nudges, signs +1; 3 mixed validation poses lined up (`20261027-103553_model_check_follower`). Stops reach 7–11° past the model's ranges (lift, wrist_flex), so the plausibility check uses model ∪ measured range | by hand only | reframed (was "own zero offsets") (D8) |
| 8 | **Parameterized FK** (own chain code, because Phase 2 fits its parameters; checked against MuJoCo) **+ collision model:** MuJoCo self-collision + table; **table plane = base underside (z = −2.4 mm in the model frame)**; gripper-tip touches on ≥ 4 table points then **validate** FK + mapping (height residuals = calibration error at the tool) and the base's flatness. Required test cases: today's two contacts. Also record one **base reference touch point** (for the Phase 2 base check) | **8a, 8b done 2026-10-27**: FK = MuJoCo to machine precision (+ space-PoE export, `docs/fk_math.md`); collision model flags all 3 real table contacts, normal sweeps clear, self-collision warn-only (folded-pose gripper↔shoulder contact is a documented blind spot), 0.5 ms/check. **8c done 2026-10-27**: base rocking found + fixed (corner mount); 8 touches → lift zero refined +1.03° (touch error 4.4 → 1.5 mm RMS, max 3.7). Table margin stays 0 (rest pose sits +2.3 mm). **Step 8 done** | by hand only | + collision model, table plane, MuJoCo (D5, D8) |
| 9 | IK (DLS, joint-limit aware) + **collision-checked motion:** every plan checked before torque-on; SafeArm checks each goal at runtime and freezes on a predicted contact. **Table-margin rule:** a part already inside the margin may only move away from the table (like the outside-limits rule), so a few-mm margin doesn't refuse the rest pose. ~~Relax the rest-pose wrist_roll limit~~ (dropped, D15); wide shoulder_lift / elbow sweeps; Cartesian demo | **9a done 2026-10-27**: DLS IK on our FK, TCP = fixed-jaw tip (user), position + approach direction, joint limits with the outside-start rule. Fingers straight down near the table exceeds the wrist_flex soft limit (+72.7°): use ~25° tilt. **9b + 9c done** (sim-tested): min-jerk straight-line plans validated before torque-on (IK, limits, joint continuity, table with 3 mm margin + inside-margin rule); SafeArm checks every goal against the table and freezes on a predicted contact. **9d Cartesian demo done** (`20261027-135238_cartesian_follower`: commanded line exact, measured max 8.5 mm off, mostly gravity sag; Phase 3 compensation). wrist_roll keeps 2550 (D15); wide lift/elbow sweeps deferred to Phase 3 prep (D16). **Step 9 done 2026-10-27** | yes | + collision checks, wide sweeps moved here (D5, D6) |
| 10 | 15-min endurance / thermal loop (Phase 2 drift baseline) + gripper squeeze (doubles as the `Present_Current` scale check); start the run **manifest** (optional camera fields for Phase 2). **Before the squeeze: gripper-specific contact detection** (own holding threshold, or exempt while squeezing): closing on an object looks exactly like a holding contact and would freeze | **10a done 2026-10-27**: Torque_Limit whitelisted (SRAM, 1–1000, restored to 100 % on exit), gripper contact exemption, per-joint load trip (gripper 80 %). **10b squeeze done** (`20261027-141349`: load to 37.6 % at 46 ticks; current scale plausible above ~10 % load). **10d manifest done** (`runlog/manifest.py`, JSONL, optional camera block for Phase 2). **10c done** (`20261027-142432_endurance_follower`: 15 min, 30/30 reference visits settled, repeatability ≤ ±3 ticks, no drift; ≤ +1 °C, so not yet a thermal test). **Step 10 done** | yes | + current check, manifest scheduled (D7); + gripper contact handling (D13) |
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
| D10 | 2026-10-26 | Step 8 also records a base reference touch point; no new mounting step (stock clamps kept, user decision) | A hand push shifted the arm in its clamps (6a test); Phase 2 has no such forces, but an unnoticed shift would invalidate world-frame data, so Phase 2 re-touches the reference each session | `20261026-170706_hold_follower`; ROADMAP Phase 2 to-do |
| D11 | 2026-10-27 | 6c adds min-jerk ramps (default; `--easing linear` reproduces step 5–6 runs) and a dwell after encoder settle | User saw end-effector wobble at every stop; encoders showed no ringing (structural flex beyond the encoders), excited by the linear ramp's instant stop | `20261027-092438_sweep_follower` encoder analysis |
| D12 | 2026-10-27 | Step 8 table plane comes from the model (base underside, z = −2.4 mm) instead of being measured; tip touches become a validation of FK + mapping | User: the base is mounted flat on the table, so the base plane is the table plane (table assumed flat). Turns 8c into the first quantitative check of the by-eye calibration | `models/so101_menagerie` base mesh |
| D13 | 2026-10-27 | Step 10 gains gripper-specific contact-detection handling before the squeeze test; Phase 3 to-do: collision grasp mode | User asked whether the table check would block picking small objects; grasping is an intended contact for both the table check and contact detection | ROADMAP Phase 3 |
| D14 | 2026-10-27 | shoulder_lift zero refined from table touches (2222 → 2210, +1.03°), a 1-parameter mini calibration before Phase 2; base re-mounted at a table corner | Touches showed reach-proportional height error (max 8.5 mm after the mount fix); one lift offset explains it (leave-one-out 1.8 mm), more parameters overfit | `20261027-124750_model_check_follower` |
| D15 | 2026-10-27 | Dropped "relax the rest-pose wrist_roll limit (2550) where clear" from step 9; the limit stays for all of Phase 1 (user) | The collision model can't resolve that contact (puts gripper and shoulder 13–14 mm apart where they touched, step 8b), so relaxing would rely on a check known to be blind there | `tests/test_collision.py` known-miss test |
| D16 | 2026-10-27 | Wide lift/elbow sweeps (lift past vertical, near-horizontal loads) deferred from step 9 to Phase 3 prep (user) | Not needed for Phase 1's go/no-go; their value is gravity-compensation input, and Phase 2's camera runs measure sag more precisely | ROADMAP Phase 3 |
