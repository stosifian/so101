# SO-101 follower: bring-up and function results (Phase 1)

*2026-10-26 → 2026-10-28. SO-101 follower (6 × Feetech STS3215, 12 V) and leader (5 V), driven from macOS over
USB serial. Compiled 2026-10-31 from the repo's records: `docs/phase1_acceptance.md` (authoritative),
`docs/phase-1-plan.md` (steps, deviations D1–D18), `docs/hardware_log.md`, `notes/claude/2026-10-26…28.md`,
`notes/LESSONS_LEARNED.md`. Numbers are quoted from those documents; none were recomputed for this report. Phase 1
produced no saved figures, so this report is text and tables.*

## Headline

> **The follower arm passed bring-up: all 12 servos healthy, motion accurate and repeatable at the encoder (±3 ticks
> over 15 min) under a safety layer proven on the real arm, including two real collisions it now prevents or
> catches. Decision 2026-10-27: GO, keep the arm.** The main limitation found is the P-only servos: they hold
> short of their goals under gravity (a commanded Cartesian line sagged up to 8.4 mm), which Phase 2 quantified at
> the gripper and largely calibrated out.

| Area | Result | Evidence |
|---|---|---|
| Servo health | 12/12 STS3215, firmware 3.10; follower 11.8–11.9 V, leader 4.9 V; no error flags; factory homing offsets present; **no firmware angle limits** | `20261026-112606_regdump_*` |
| Communication | 6-servo telemetry read ≈ 3.0 ms; 100 Hz loop holds; 15-min endurance: 0 failed reads in 96k samples | `20261026-121235`, `20261027-142432_endurance` |
| Safe torque-on | goal := present + speed/accel caps, read back: **0 ticks of motion at torque-on** | `20261026-144830_hold` |
| Contact detection | following error by joint role → freeze; live push test froze at 14 ticks; caught a real gripper–table contact on day 2 | `20261026-170706_hold`, `20261027-091714_sweep` |
| Collision model | MuJoCo geometry check flags all 3 real table contacts (−2 to −20 mm); normal motion ≥ 27 mm clear; runtime freeze before a colliding goal is written | `tests/test_collision.py` |
| Motion quality | P = 32: 0 holding jitter; stick-slip at 50 ticks/s (51 % stalled), smooth at 150 (12 %) | `20261026-145837_move`, `…154001` |
| Stiffness | ≈ 0.85 % load per tick on every joint (pan push, elbow sag, gripper 0.82) | 5b, 6, 10b |
| Repeatability (encoder) | same direction 0–2 ticks (pan); **≤ ±3 ticks over 30 visits in 15 min, no drift** | `20261027-093309_settle`, `…142432_endurance` |
| Calibration (tick → model) | by eye + one table-touch refinement: tool height error **1.5 mm RMS** at table level (max 3.7 mm, leave-one-out 1.8) | `20261027-124750_model_check` |
| Kinematics | parameterized FK = MuJoCo to machine precision; DLS IK with limits; Cartesian line commanded within 0.6 mm | `docs/fk_math.md`, `docs/ik_math.md`, `20261027-135238_cartesian` |
| Gripper | torque-capped squeeze to 37.6 % load (46 ticks short); `Present_Current` trustworthy above ~10 % load | `20261027-141349_squeeze` |
| Thermal (15 min, low load) | ≤ +1 °C; 55 °C trip never approached | `20261027-142432_endurance` |
| All-joint pose tour | 17 waypoints, all joints together, 4.6 min; no freeze, trip or contact; end-of-dwell error ≤ 14 ticks (elbow); peak load 17.6 % | `20261028-155951_tour` |

---

## 1. Hardware and communication (steps 1–3)

| Item | Finding |
|---|---|
| Port map | both driver boards enumerate as identical CH343 (`1a86:55d3`); only the board serial distinguishes them: follower `5B90103255` (12 V), leader `5B90101307` (5 V) |
| Factory state | non-zero homing `Offset` on all servos (Seeed-calibrated); `Min/Max_Angle_Limit` 0/4095; **at power-up Goal_Position = 0, Goal_Speed = 0, Acceleration = 0 (unlimited)**: naive torque-on would drive a joint toward tick 0 at full speed |
| Leader shoulder_lift | different EEPROM fingerprint (Phase 12, Max V 8.0, Protection_Current 500): the 1/345 gear variant, matching the follower |
| Telemetry glitches | single-sample V/°C outliers on **IDs 4–6** of both arms (0.2–1.6 % of samples): servo-side ADC noise, not the read path (A/B test). Temperature spikes up to 103 °C on a driven joint while moving |
| Loop timing (no TUI) | period p50 / p99 / max 10.0 / 11.6 / 13 ms; the TUI costs timing (p99 14.0, max 28 ms) |

**Lessons:**

- **"Port exists ≠ servos reachable"** (LESSONS #1). The leader was silent until the driver→motor cable was reconnected.

- **The repo moved out of the iCloud-synced Desktop** (LESSONS #2). iCloud hid the editable install's `.pth` file, which Python 3.12 then skips.

## 2. Safety layer (steps 4–5)

| Feature | Detail |
|---|---|
| Write whitelist | SRAM registers only; EEPROM writes impossible from scripts |
| Torque-on sequence | goal := present, speed/accel caps, all read back, then torque; Mode writes re-check Torque_Enable |
| Software limits | the only position guard (no firmware limits); a pose outside them is allowed at start but can't move further out |
| Trips | debounced temperature/voltage (ADC glitches), immediate overload/overcurrent/angle; one owner per bus |
| Stop | two-stage Ctrl-C: 1st = freeze (hold), 2nd = torque off (D4). Measured stop 13 ticks / 100 ms at 150 ticks/s |
| Staging | 5a shadow (0 writes, 60 s live) → 5b hold (+ push test) → 5c one joint → 5d stick-slip (D3) |

- **Replay before hardware:** replaying recorded telemetry through the trip checker found a false trip before any hardware test. The leader gripper's Status voltage bit follows its ADC glitches.

## 3. Sweeps, contact detection, settling (step 6)

| Joint | Hysteresis (approach direction) | Holding load | Notes |
|---|---|---|---|
| shoulder_pan | 3–5 ticks | ±2.4 % (friction) | |
| wrist_roll | 2–5 ticks | ±2.4 % | **gripper base touched the shoulder near 2645–2664** in rest pose → wrist_roll limit 3438 → 2550 (D9) |
| gripper | 2–6 ticks | 0…±4 % | |
| wrist_flex | 4–5 ticks | ↓ +3.2…+4.8 % | |
| elbow_flex | **8–13 ticks** | ↓ +7…+12 % | sag ≈ load / 0.86 %/tick |
| shoulder_lift | **14–20 ticks** (largest) | gravity ≈ 10–11 %, friction ±6–7 % | **wrist hit the table** lifting from rest; clean gravity profile from a cleared pose to 79° |

- **Contact detection (6a):** freeze on following error, moving / holding / idle (config: 40 / 25 / 10 ticks, 0.3 s). In replay it catches the table contact ~2 s before the 39 % load peak, with no false triggers on 5 normal runs. Gentle contact is not caught (documented).
- **Settling (6c):** `wait_until_settled` + `approach(dir)` + min-jerk ramps + a dwell after the encoder settles. Encoder settle ≈ 0.21 s. **End-effector wobble at stops is invisible to the encoders** (structural flex, D11).

- **Two real contacts were first misread** (cable twist, gravity), and corrected by what the user saw. Rule kept: ask what was physically seen before explaining odd telemetry.

## 4. Calibration and kinematics (steps 7–9)

| Item | Result |
|---|---|
| Tick → model mapping (step 7) | per-joint zero + sign (all +1) vs the MuJoCo Menagerie SO-101, by eye + live nudges; 3 validation poses lined up; stops reach 7–11° past the model's ranges (lift, wrist_flex) |
| FK (8a) | own local product-of-exponentials chain = MuJoCo to machine precision; parameterized (zero offsets, link twists, tool) for Phase 2 fitting; space-PoE exported |
| Collision model (8b) | MuJoCo geometry: table = base underside (z = −2.4 mm); self-collision warn-only; 0.5 ms/check; folded-pose gripper↔shoulder contact is a known blind spot (model 13–14 mm apart) |
| Table touches (8c) | base rocking found and fixed (corner mount); 8 touches → shoulder_lift zero 2222 → 2210 (+1.03°): height error 4.4 → **1.5 mm RMS** (max 3.7). More parameters overfit |
| IK (9a) | DLS, joint-limit aware, tool = fixed-jaw tip, position + approach direction; fingers straight down near the table exceed wrist_flex → ~25° tilt |
| Collision-checked motion (9b–c) | every plan checked before torque-on (IK, limits, continuity, table with 3 mm margin + inside-margin rule); runtime freeze on a predicted contact |
| Cartesian line (9d) | 80 mm line: commanded within 0.6 mm; **measured up to 8.5 mm off (−8.4 mm vertical sag, ≤ 7.1 mm lag)**, RMS 4.7 mm |

## 5. Endurance, gripper, manifest (step 10)

| Item | Result |
|---|---|
| Endurance loop (15 min, 30 cycles) | 30/30 settled; error at the reference ≤ ±3 ticks (std ≤ 0.85); trend ≤ 0.06 ticks/min; 0 bus failures in 96k samples |
| Thermal | ≤ +1 °C (median load 0–2.4 %): a repeatability baseline, **not** a thermal test |
| Gripper squeeze | contact from goal ≈ 2220; at 2100: 46 ticks short, 37.6 % load, ≈ 436 mA; stiffness 0.82 %/tick |
| `Present_Current` | ≈ 0 below ~10 % load; ≈ 19 mA/% above: trust current above ~10 %, load below |
| Run manifest | JSONL per pose, snapshots before/after, optional camera block (used by every Phase 2 capture) |

## 6. Acceptance and the post-acceptance tour (steps 11–12)

- **GO (user, 2026-10-27).** One item was flagged for before the return window closed: inspect the wrist servos' (IDs 4–6) wiring, given the sensor glitches and rare missed replies at the end of the chain. Teleop check dropped (D17).
- **Step 12 (D18), all-joint pose tour** (`config/tours/pick_and_place.yaml`):
  - 17 waypoints, every joint moving together;
  - no freeze, trip or contact; 0 missed reads;
  - end-of-dwell error ≤ 14 ticks (elbow, gravity), median 1–3 on every joint;
  - temperatures 33–37 °C;
  - doubled as the Phase 2 depth-of-field check.

## 7. Known limitations at acceptance

| Limitation | Size | Handling then |
|---|---|---|
| Gravity sag + following lag (P-only servos) | Cartesian line up to 8.5 mm off (−8.4 mm sag, 7 mm lag) | Phase 3 stiffness compensation |
| Direction-dependent stopping | 2–6 ticks unloaded, 8–20 ticks on gravity joints | `approach(dir)` for metrology |
| Stick-slip at low speed | 51 % stalled at 50 ticks/s, 12 % at 150 | default 150 ticks/s |
| Servo V/°C glitches | single samples, up to 103 °C on a driven joint | debounced trips, median displays |
| Calibration (by eye + 1 refinement) | ~1.5–2.5 mm at the tool | Phase 2 camera fit |
| Self-collision blind spot | folded-pose gripper↔shoulder | wrist_roll limit 2550 |
| Wobble invisible to encoders | visible at stops on extended moves | dwell; Phase 2 ring-down |
| Wrist reach near the table | fingers vertical at 8 cm exceed wrist_flex | ~25° tilt |

## 8. Revisited in Phase 2

| Phase 1 finding | What Phase 2 measured at the gripper (`docs/results/phase2_report.md`) |
|---|---|
| Cartesian line sagged −8.4 mm (encoder-derived + by-eye model) | **consistent:** commanding with the nominal model lands the tag ~7.5 mm low (step 10b) |
| Calibration ~1.5–2.5 mm at the tool (table touches) | **understated away from the table:** the nominal model is ~2 mm RMS off at the tag across the workspace, up to ~6 mm at far reach; camera calibration brings it to ~0.8 mm lateral (step 10) |
| Repeatability ≤ ±3 ticks at the encoder | **same story, now in mm:** 0.7–0.9 mm at the tag, set by ±1–2 tick landings (step 8) |
| Hysteresis 2–20 ticks ("backlash") | **it is the servo deadband**, visible to the encoder; hidden gear backlash ≲ 1 tick (step 8) |
| Wobble invisible to encoders; ring-down "for Phase 2" | **not measurable with this camera** (1/30 s, ~6 s per shot): deferred as a setup limitation (D11); a 3 s settle verified |
| "Thermal drift needs load" | **done (step 11):** a 45-min loaded hold drifts one encoder tick (~0.4 mm), otherwise ~1 µm/min |
| Contact thresholds tuned near rest | **false trip at full reach** (p39, step 10 grid): not gravity-aware |

## 9. Inconsistencies found in the records

- **Contact-detection thresholds:** the plan's step 6a row says "≈35 ticks moving / ≈12 holding". The 10-26 notes and the safety config say moving 40 / holding 25 / idle 10. The plan row records the first proposal; the config values (40 / 25 / 10) are what ran, and what this report quotes.
- **Stiffness:** quoted as 0.85 %/tick (acceptance), 0.86 (elbow sag, notes) and 0.82 (gripper). They're the same spring within measurement; the report uses ≈ 0.85.

- **Elbow position after its sweep:** first logged as back at 2927, later corrected in the notes to ~2860 (friction held it). The hardware log has the corrected value.

## Data and code

| Step | Key runs | Code |
|---|---|---|
| 2 | `20261026-112606_regdump_follower`, `…_leader` | `scripts/readonly/dump_registers.py` |
| 3 | `20261026-130808_telemetry_follower`, `20261026-131126_telemetry_leader` | `scripts/readonly/live_readout.py` |
| 5 | `20261026-142826_shadow`, `…144830_hold`, `…145837_move`, `…154001/154057_move` | `so101.safety.SafeArm`, `scripts/motion/` |
| 6 | `20261026-160702…162655_sweep`, `20261027-092126/092438_sweep`, `20261026-170706_hold`, `20261027-093309_settle` | `scripts/motion/joint_sweep.py`, `settle_check.py` |
| 7–8 | `20261027-103553/124750_model_check` | `so101.kinematics` (`mapping`, `fk`, `collision`) |
| 9 | `20261027-135238_cartesian` | `so101.kinematics.ik`, `scripts/motion/cartesian_line.py` |
| 10 | `20261027-141349_squeeze`, `20261027-142432_endurance` | `scripts/motion/gripper_squeeze.py`, `endurance_loop.py`, `so101.runlog.manifest` |
| 12 | `20261028-155951_tour` | `scripts/motion/pose_tour.py` |

Acceptance: `docs/phase1_acceptance.md`. Plan and deviations D1–D18: `docs/phase-1-plan.md`. Hardware facts: `docs/hardware_log.md`.
