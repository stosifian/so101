# Hardware log

Facts about *this* physical unit. Update as they are verified, and link run IDs for evidence.

## Port mapping
| Arm | Serial port | Board serial | PSU | Verified |
|-----|-------------|--------------|-----|----------|
| Follower (white) | /dev/cu.usbmodem5B901032551 | 5B90103255 (CH343, 1a86:55d3) | 12 V | 2026-10-26: IDs 1–6 model 777 @ 11.8–11.9 V |
| Leader (black) | /dev/cu.usbmodem5B901013071 | 5B90101307 (CH343, 1a86:55d3) | 5 V | 2026-10-26: IDs 1–6 model 777 @ 4.9 V (after reconnecting driver→motor cable) |

## Servo inventory (from register dumps)
| Arm | ID | Joint | Model | Firmware | Voltage | Notes |
|-----|----|-------|-------|----------|---------|-------|
| Follower | 1–6 | all | 777 | 3.10 | 11.8–11.9 V | identical EEPROM config: P/D/I 32/32/0, Phase 12, Max V 14.0, Max T 70 °C, Protection_Current 310 |
| Leader | 1,3,4,5,6 | pan, elbow, wrist×2, gripper | 777 | 3.10 | 4.9 V | Phase **92**, Max V 12.0, Max T 80 °C, Protection_Current 300, Min startup force 0 (low-gear variants) |
| Leader | 2 | shoulder_lift | 777 | 3.10 | 4.9 V | Phase **12**, Max V 8.0, Max T 70 °C, Protection_Current 500, startup force 16. The 1/345 variant: EEPROM fingerprint differs even though Model is the same |

**Factory state (both arms, 2026-10-26):**
- `Offset` (homing offset, EEPROM) is already **non-zero** on every servo, so Seeed calibrated before shipping. `Present_Position` already includes these offsets. Follower: -1835, 1690, -1197, -1660, -1470, 640.
- `Min/Max_Angle_Limit` = 0/4095 on all servos. **There are no firmware position limits**, so software limits are the only guard.
- `Lock` = 1 (EEPROM write-protected), Mode 0 (position), Torque_Enable 0, Status 0, Baud index 0 (1 Mbps), Return_Delay 0.
- SRAM at power-up: **Goal_Position = 0, Goal_Speed = 0, Acceleration = 0.** Speed 0 and accel 0 mean "unlimited" on STS servos.
- Self-protection: Unloading_Condition 44 (unload torque on overheat, overcurrent, overload). Overload is >80 % for 2.0 s, after which torque drops to 20 %.

## Register dumps
| Date | Run ID | Arm | Purpose |
|------|--------|-----|---------|
| 2026-10-26 | 20261026-112606_regdump_follower | follower | factory baseline, before any calibration/writes |
| 2026-10-26 | 20261026-112606_regdump_leader | leader | factory baseline |

## Joint ranges (by-hand sweep, torque off, 2026-10-26)
Ticks as reported by `Present_Position` (factory homing offsets applied). Rest = arm folded on its stand.
| Joint | Follower swept | Follower limit (±50) | Follower rest | Leader swept | Leader rest |
|-------|----------------|----------------------|---------------|--------------|-------------|
| shoulder_pan | 919–3454 | 969–3404 | 2176 | 740–3182 | ~1750–2100 (moved) |
| shoulder_lift | 1003–3208 | 1053–3158 | 1121 | 1003–3135 | 1014 |
| elbow_flex | 715–2928 | 765–2878 | 2927 (**outside limit**) | 717–2916 | 2883–2914 |
| wrist_flex | 946–3025 | 996–2975 | 2945 | 851–3187 | — |
| wrist_roll | 291–3488 | 341–**2550** (rest-pose contact, see Collisions) | 1197 | 170–3974 | — |
| gripper | 2032–3497 | 2082–3447 | ~2045–2126 (**closed is outside limit**) | 2042–3279 | 2047 |

Runs: follower `20261026-130808_telemetry_follower`, leader `20261026-131126_telemetry_leader`. Proposals: `data/derived/<run_id>/limits_proposed.json`.
- shoulder_lift lower stop (1003) and elbow_flex lower stop (715/717) match across arms, so the factory homing offsets line up. shoulder_pan stops differ by ~200–270 ticks.
- No wrap through 0/4095 on any joint. Leader wrist_roll is nearly a full turn.

## Per-joint sweeps (step 6, P=32, 150 ticks/s, hold 1 s; steady state = last 0.5 s of each hold)
Error = position − goal; ↑ = approached from below. Hysteresis = err↓ − err↑ (approach-direction effect).
| Joint | Run | Profile / travel | err ↑ | err ↓ | Hysteresis | Holding load | Jitter | Stuck while moving | Notes |
|-------|-----|------------------|-------|-------|------------|--------------|--------|--------------------|-------|
| shoulder_pan | 20261026-160702_sweep_follower | generic, 1578–2795 (107°) | −1…−2 | +2…+3 | 3–5 ticks (0.26–0.44°), uniform across range | ±2.4 % (friction only) | 0 | 5 % | no trips, lag 20 ticks |
| wrist_roll | 20261026-160843_sweep_follower | generic, 1115–2664 (136°) | −1…−2 | +1…+3 | 2–5 ticks | ±2.4 % | 0–1 | 5 % | **gripper base touched the shoulder near 2645–2664 (arm in rest pose)**; held 5–21 ticks short at 5–7.6 % load; no trip |
| gripper | 20261026-161842_sweep_follower | generic, 2145–3106 (84°) | −1…−3 | +1…+4 | 2–6 ticks | 0…±4 % | 0 | 8 % | no contact seen in load; lag 18 ticks |
| wrist_flex | 20261026-162422_sweep_follower | anchored 4×150 from rest, 2853→2253 (53°) | −1…0 | +3…+5 | 4–5 ticks | ↓ +3.2…+4.8 %, ↑ 0 % (gravity asymmetry) | 0 | 8 % | arm in rest pose; no contact |
| elbow_flex | 20261026-162529_sweep_follower | anchored, entered range at 2858, 2858→2258 (53°) | +1…+2 | +8…+14 | **8–13 ticks (0.7–1.1°)** | ↓ +7…+12 %, ↑ 0…2.4 % | 0 | 15 % (stall 171 ms) | sag ≈ load / 0.86 %/tick (same stiffness as pan 5b); after torque-off it stayed at ~2857–2863 (friction), it did **not** fall back to the 2927 rest stop |
| shoulder_lift | 20261026-162655_sweep_follower | anchored 4×150 from rest, 1074→1674 | −2 (to 1374) | +1…+2 | 4 ticks (to 1374) | 2.4 % (to 1374) | 0–1 | 12 % | **wrist hit the table (seen by the user) from lift ≈ 1398**: up to 1674 the readings are contact, not gravity (see Collisions). Lift's gravity profile is still unknown |

## Collisions / contact
- **wrist_roll vs shoulder (step 6, run 20261026-160843_sweep_follower):** with the arm in rest pose (elbow 2927, wrist_flex 2853), rolling the wrist up to ~2645–2664 brings the gripper base against the shoulder (seen by the user). Load during the approach (~6.4 %) looked the same as normal 150 ticks/s motion, so **gentle contact is not detectable from load at these speeds; it has to be prevented by limits or geometry.** Pose-dependent: the per-joint hand sweep could not find it. **Action: wrist_roll max lowered 3438 → 2550 in config (≈8° before contact). That's a rest-pose limit applied in every pose for now; FK collision checking (steps 8–9) should relax it where the pose allows.**

- **Wrist vs table during shoulder_lift (step 6, run 20261026-162655_sweep_follower):** lifting the shoulder from rest pose with the elbow folded (≈2857) swings the wrist down into the table. Contact from lift ≈ 1398 (≈ 28° above rest): the elbow, which had no new goal, was pushed back 2857 → 2836 and wrist_flex −5 ticks. Lift's error grew to 35–57 ticks while ramping (normal ≈ 20). Holding at 1524 going up it sat 20 ticks short at 16 % load, versus −1 tick / 0 % at the same position going down. It peaked at 1626 (goal 1674, 48 short) at **39 % load for ~3 s, below the 50 % load trip**, so nothing tripped. Contact released on the way down near 1600. Gentle at first, sustained at the end; the load trip alone can't catch it.

## Telemetry quirks
- **V/°C ADC glitches on IDs 4–6 (both arms):** ~0.2–1.6 % of samples read low voltage / high temperature for a single sample. The value really is in the servo register (block and single-register reads agree), so it's servo-side. Debounce any trip on V/°C. Positions have shown no spikes.
- **Status bit 0 (voltage) follows the ADC glitches:** the leader gripper sets Status=0x01 on exactly the samples where its voltage glitches to 3.6–3.9 V, below the 4.0 V `Min_Voltage_Limit` that every servo has (27 of ~264k samples across all step 3 runs; never on the follower, whose glitches only reach ~10.4 V). The voltage/overheat Status and error-byte bits are therefore debounced in SafeArm; overload/overcurrent/angle bits trip at once.
- **Temperature spikes on the moving joint while it moves** (step 5–6 runs): single samples up to 60–103 °C on whichever joint is being driven (pan: 44 spikes in its sweep, max 67; wrist_roll: 37, max 103), next sample back at the 34–36 °C median. Never on a joint at rest, and larger than the at-rest ADC glitches (≤ +12 °C, IDs 4–6). Likely motor-drive (PWM) noise on the temperature ADC. The debounced trip (≥ 55 °C continuously for 0.5 s) ignores them; Rerun shows raw values, the TUI medians.
- **Occasional failed reads on IDs 4–6 only** (step 5a, run 20261026-142826_shadow_follower): 3 of ~36k reads ("no status packet" ×2, "incorrect status packet" ×1), all absorbed by retries. Each failed attempt costs the SDK's ~34 ms timeout (`LATENCY_TIMER` 16 ms × 2 + 2), so the loop stalls up to ~100 ms. Same three end-of-chain servos as the ADC glitches, which points further toward wrist cabling/ground.
- **`Moving` = "servo still executing its move"** (confirmed step 5c, run 20261026-145837_move_follower): 1 in 99 % of samples while the goal ramps, drops to 0 34–120 ms after the ramp ends, **while the position is still 1–3 ticks short of goal**. Stays 0 when back-driven by hand (torque off, and torque on with a fixed goal). Settle detection needs `Moving == 0` **and** a position tolerance ≥ ±3 ticks.
- **First commanded move (step 5c, shoulder_pan ±100 ticks @ 50 ticks/s, P=32, no load):** tracking lag median 5 / max 8 ticks; load while moving ~4 % (≈ 0.85 %/tick × lag); no overshoot; settles within ±2 ticks in 30–80 ms; **holding jitter 0 ticks**. Other joints moved 0 ticks.
- **Freeze (1st Ctrl-C) on the real arm** (run 20261026-155559_move_follower, shoulder_pan ramping at 150 ticks/s): it overran the freeze point by 13 ticks (≈1.1°) within 100 ms while decelerating at the accel cap (predicted ~11), came back to it within 300 ms, then held with 0 ticks of movement for 6 s until the 2nd Ctrl-C.
- **Stick-slip at low speed (shoulder_pan, P=32, I=0):** at 50 ticks/s the joint sticks until its error reaches ~5–8 ticks, then jumps (staircase in the position plot). Dead zone is only 1 tick, so it's friction. Stuck share while > 4 ticks off goal: **stream 50 t/s 51 %**, **stream 150 t/s 12 %** (run 20261026-154001), **servo-profile 50 t/s 57 %** (one goal per move, Goal_Speed 50; run 20261026-154057). So it depends on speed, not on streaming. In servo mode `Moving` stays 1 for the whole move.
- **Direction-dependent steady-state error (shoulder_pan):** stops short of goal by −1 tick approaching upward and +2…+3 approaching downward, holding 2–3 % load there (P-only, I=0, so friction wins at small error). This is the backlash/approach-direction effect Phase 2's `approach_dir` is for.
- **Holding stiffness (P=32, torque on, step 5b, run 20261026-145026_hold_follower):** shoulder_pan pushed by hand deviates up to ~9 ticks; `Present_Load` ≈ 0.85 % per tick of position error, same sign as the deviation; returns when released. `Present_Current` stays tiny (≤ 4 units ≈ 26 mA at 8 % load). At rest on its stops the arm shows 0 load / 0 current with torque on.
- **`Present_Speed` resolution is 50 ticks/s** (values seen: ±50, −350, −600). Derive speed from position for analysis.
- **Telemetry read:** 15-byte block (addr 56–70) per servo, ~3.0 ms for 6 servos at 1 Mbps; poll limit ~330 Hz. Encoder noise at rest: 0 ticks.

## Physical setup
- Base clamping:
- Photos:
