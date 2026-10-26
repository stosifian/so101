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

## Physical setup
- Base clamping:
- Photos:
