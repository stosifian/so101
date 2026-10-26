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
| wrist_roll | 291–3488 | 341–3438 | 1197 | 170–3974 | — |
| gripper | 2032–3497 | 2082–3447 | ~2045–2126 (**closed is outside limit**) | 2042–3279 | 2047 |

Runs: follower `20261026-130808_telemetry_follower`, leader `20261026-131126_telemetry_leader`. Proposals: `data/derived/<run_id>/limits_proposed.json`.
- shoulder_lift lower stop (1003) and elbow_flex lower stop (715/717) match across arms, so the factory homing offsets line up. shoulder_pan stops differ by ~200–270 ticks.
- No wrap through 0/4095 on any joint. Leader wrist_roll is nearly a full turn.

## Telemetry quirks
- **V/°C ADC glitches on IDs 4–6 (both arms):** ~0.2–1.6 % of samples read low voltage / high temperature for a single sample. The value really is in the servo register (block and single-register reads agree), so it's servo-side. Debounce any trip on V/°C. Positions have shown no spikes.
- **`Moving` stays 0 when back-driven by hand** with torque off; `Present_Speed` does show hand motion. Behaviour under torque: verify at step 5.
- **Telemetry read:** 15-byte block (addr 56–70) per servo, ~3.0 ms for 6 servos at 1 Mbps; poll limit ~330 Hz. Encoder noise at rest: 0 ticks.

## Physical setup
- Base clamping:
- Photos:
