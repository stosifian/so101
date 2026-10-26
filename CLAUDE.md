# SO-101 project — working context

Spec: `so101-metrology-plan.md` (hardware, Phase 2 metrology spec, conventions).
Roadmap + Phase 2/3 handoff: `docs/ROADMAP.md`. Hardware facts/port map: `docs/hardware_log.md`.

## Current phase
**Phase 1 — bring-up.** Progress is tracked in the step table in `docs/ROADMAP.md`.
Working style: Claude scaffolds one step at a time with a walkthrough; the user reviews and runs it on hardware; advance only after the checkpoint passes.

## Hard rules
- **Never run anything that can enable torque or move the arm without the user's explicit go-ahead.** Those scripts live in `scripts/motion/` (permission "ask" rule in `.claude/settings.json`) and require `--arm-moves`. `scripts/readonly/` scripts must never write torque/goal registers.
- All motion goes through `so101.safety.SafeArm`; never write goal/torque registers from scripts directly.
- Re-check `Torque_Enable` after any write to `Mode` (it was silently cleared on Shoggoth).
- Never swap power supplies: follower 12 V, leader 5 V.
- **Torque-enable sequence:** at power-up Goal_Position=0, Goal_Speed=0 and Acceleration=0 (unlimited). Before `Torque_Enable=1`, always write Goal_Position := Present_Position and set speed/accel caps, or the joint slams toward tick 0 at full speed.
- No EEPROM writes (IDs, offsets, limits) without explicit user approval and a register dump before + after.

## Conventions
- Hardware layer speaks raw ticks (0–4095). Radians/mm only in `kinematics/` and `analysis/`.
- Host monotonic clock (`time.monotonic_ns`) for all timing.
- `data/raw/<run_id>/` is immutable; analysis writes to `data/derived/`.
- Every run gets a header (git sha, package versions, config hashes) via `so101.runlog`.
- Live diagnostics where possible: Rerun (`--viz`) and rich TUI (`--tui`), alongside the always-on parquet logger.
- Env: `uv` with Python 3.12 arm64. Run things with `uv run ...`. Do not use the anaconda base (x86_64).

## Notes (for the write-up)
- `notes/claude/YYYY-MM-DD.md` — Claude's daily log (Done / Surprises / Learnings / Decisions / Numbers / Open questions / Write-up hooks). Update at the end of each session.
- `notes/mine/YYYY-MM-DD.md` — **the user's own notes. Claude creates the day's template only, never edits it afterwards.**
- `notes/LESSONS_LEARNED.md` — cross-day gotchas (Symptom / What was going on / Why / How found / Fix / Takeaway).
