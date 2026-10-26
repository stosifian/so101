<!-- Approved plan from the 2026-10-26 planning session. Living status is in ROADMAP.md. -->
# SO-101 Project — Codebase Architecture + Phase 1 Plan

## Context
A three-phase project on a Seeed SO-ARM101 Pro (6× Feetech STS3215 follower at 12 V, leader at 5 V):
1. **Bring-up & function** (inside the 30-day return window): connect, move, joint-space control, FK/IK, thermals, acceptance.
2. **Camera metrology** (A7 IV + ChArUco) to characterize repeatability, accuracy, sag, backlash, thermal drift. Full spec in `so101-metrology-plan.md`.
3. **TBD from Phase 2 results**: a compensation study, a use-case/policy, or added sensing such as IMUs.

The working directory currently has only `so101-metrology-plan.md`. It is not a git repo. One USB serial device is already visible (`/dev/cu.usbmodem5B901032551`).

The goal is an architecture that Phase 2/3 can extend without rewriting Phase 1 code. The user codes alongside in Phase 1 (I scaffold in small steps, they review and run on hardware, then we iterate). The project also needs a daily notes system for the eventual write-up.

**Decisions made with the user:**
- Phase 1 mode: I scaffold each step with a walkthrough. The user reviews and runs it, and we move on only after a checkpoint passes.
- FK and IK are both in Phase 1. They come from a *parameterized* model built on the SO-101 URDF, so the same model becomes the Phase 2 kinematic-calibration model.
- Notes: the user types `notes/mine/YYYY-MM-DD.md`. I keep my own, more complete, `notes/claude/YYYY-MM-DD.md`.
- Env: `uv` with native arm64 Python 3.12 and a lockfile. Avoid the x86_64/Rosetta anaconda base.

## Architecture principles (what makes it phase-proof)
- **Layered, with one-way dependencies:** `hw → safety → motion → (kinematics, sensors) → capture → analysis`. Each layer has a small interface plus a mock, so higher layers are testable without hardware.
- **Raw ticks at the hardware boundary.** Radians and mm appear only in `kinematics/` and `analysis/` (spec §4).
- **All motion goes through `SafeArm`.** No script ever talks to the bus directly for writes.
- **One run/manifest format from day 1.** Phase 1 sweeps and the thermal loop are logged as runs in `data/raw/<run_id>/`, with a header and JSONL rows. These are the same writer and schema that Phase 2 extends with camera fields. Phase 1 data is then directly comparable later, e.g. as a thermal baseline.
- **A generic `Sensor` protocol.** The camera (Phase 2) and IMU/wrist-cam (Phase 3) plug in through it. A capture loop only needs `snapshot()` / `capture()`.
- **Telemetry is a stream with several consumers.** A `TelemetryPoller` runs a background thread that sync-reads every joint (ticks, load, temp, voltage, moving flag, current) at a fixed rate with monotonic timestamps. It publishes each `Snapshot` to subscribers, which are independent and can be switched on or off per script:
  - **Disk logger** (always on during runs): `data/raw/<run_id>/telemetry.parquet` (chunked writes) is the canonical data for after-the-fact analysis.
  - **Live plots via Rerun** (`rerun-sdk`, `--viz` flag): per-joint time series of commanded vs measured ticks, tracking error, load, temperature and voltage, plus a live 3D view of the arm from the URDF+FK. Phase 2 adds camera frames and detected target poses to the same viewer. Each session also saves a `.rrd` recording next to the run, so a live session can be replayed exactly. LeRobot uses Rerun too, so it fits the ecosystem.
  - **Terminal dashboard** (`rich`, `--tui` flag): a compact per-joint table with color thresholds (temp, load, tracking error), bus error counters and poll rate/latency. Useful when the Rerun viewer isn't open.
  - **Safety watchdog:** `SafeArm` subscribes to the same stream for stall and over-temp checks, so what you see live is exactly what the safety logic sees.
- **Debug hooks:** bus-level counters (timeouts, checksum errors, retries per servo), per-cycle sync-read latency, and an optional raw-packet trace (`--trace-bus`) to a log file. Python `logging` writes to the console and to `run.log` in each run dir.
- **Config, not code, for hardware facts:** ports, IDs, joint limits, speed caps and the home/zero calibration live in `config/*.yaml`, with versions recorded in run headers.
- **LeRobot is optional.** It is used only for `lerobot-find-port` and as a reference. The servo layer uses `feetech-servo-sdk` (`scservo_sdk`) directly, per spec §4. Shoggoth lesson #3 applies: the dist name differs from the import name, and installs must be verified from a neutral cwd.

## Repository layout
```
so101/
  pyproject.toml, uv.lock, README.md (pinned versions), CLAUDE.md (conventions + current phase)
  so101-metrology-plan.md          # existing spec (kept as-is, referenced)
  config/
    arm_follower.yaml              # port, ids, limits (ticks), speed/accel caps, stall thresholds
    arm_leader.yaml
    calibration/<date>_home.yaml   # our own zero offsets (ticks↔rad), NOT written to EEPROM
  src/so101/
    hw/
      registers.py                 # STS3215 control-table addresses (single source of truth)
      bus.py                       # ServoBus protocol + FeetechBus (scservo_sdk) + MockBus
      arm.py                       # Arm: joint names↔ids, sync read/write ticks, state snapshot
      dump.py                      # read-only full register dump → JSON
    safety.py                      # SafeArm: limits, speed caps, stall/tracking fault, torque-off, Mode→Torque recheck
    motion.py                      # move_to(ticks), wait_until_settled(tol, N polls, 'moving' flag), sweeps, approach_dir
    kinematics/
      model.py                     # parameterized chain (link offsets as params) loaded from URDF
      fk.py, ik.py                 # FK; damped-least-squares IK with joint limits
      convert.py                   # ticks↔rad using calibration file
    sensors/
      base.py                      # Sensor protocol, CaptureResult
      (phase 2) camera_gphoto2.py, camera_sony_sdk.py, camera_mock.py
      (phase 3) imu_bmi270.py, ...
    telemetry/
      poller.py                    # background sync-read thread → Snapshot pub/sub
      viz_rerun.py                 # live Rerun plots + 3D arm; saves .rrd
      tui.py                       # rich live table
      plots.py                     # post-hoc matplotlib figures from telemetry.parquet
    runlog/
      run.py                       # create run dir, run header (versions, config hashes, git sha)
      manifest.py                  # JSONL writer; schema from spec §7, camera fields optional
    (phase 2) capture/loop.py, capture/poses.py, analysis/{detect,calib,reports}.py
  scripts/
    readonly/                      # never enable torque: find_port.py, dump_registers.py, monitor.py
    motion/                        # can move the arm (always prompts): jog.py, sweep.py, thermal_loop.py, ik_demo.py
  tests/                           # pytest, MockBus-based: safety, manifest integrity, FK/IK round-trip
  data/raw/<run_id>/               # immutable (git-ignored); data/derived/ for analysis outputs
  notes/
    claude/YYYY-MM-DD.md           # my daily log
    mine/YYYY-MM-DD.md             # user's own notes — I create a blank template, never edit after
    LESSONS_LEARNED.md             # cross-day gotchas (Symptom / Cause / How found / Fix / Takeaway, shoggoth style)
  docs/
    ROADMAP.md                     # Phase 2/3 handoff notes (see below)
    hardware_log.md                # port mapping, serials, photos index, register dumps index
```

## Notes system
- **`notes/claude/YYYY-MM-DD.md`** is written by me at the end of each session and at notable moments. Sections: *Done*, *Surprises*, *Learnings*, *Decisions (+why)*, *Numbers* (measured values with links to run IDs), *Open questions*, *Write-up hooks* (things likely worth a paragraph or figure).
- **`notes/mine/YYYY-MM-DD.md`**: I create it from a light template (date, a few prompts such as "what surprised me" and "what I'd tell a friend") at the start of each day. After that I never touch it, so the prose stays yours.
- **`notes/LESSONS_LEARNED.md`** holds gotchas that outlive a single day.
- I'll also save a memory entry so future sessions know this two-track convention.

## Phase 1: step-by-step with checkpoints
Each step follows the same loop: I write the code, then walk through it, then you review and run it, then we record the results in notes. We move on only after the checkpoint passes.

| # | Step | Deliverable | Your checkpoint |
|---|------|-------------|-----------------|
| 0 | Repo bootstrap | `git init`, uv env (Py 3.12 arm64), pyproject, `.gitignore` (data/raw), CLAUDE.md, notes/docs skeleton, day-1 note files | `uv run python -c "import scservo_sdk"` from a neutral cwd |
| 1 | Port discovery | `lerobot-find-port` (via `uvx`) or a tiny pyserial lister; record follower/leader ports in config + hardware_log | Port mapping recorded, PSU labels checked (12 V follower / 5 V leader) |
| 2 | **Read-only** register dump | `hw/registers.py`, `hw/bus.py` (FeetechBus read path + MockBus), `scripts/dump_registers.py` → `data/raw/<run>/registers.json` | All 12 servos respond, IDs 1–6, voltage/temp/Mode/Torque_Enable sane |
| 3 | Torque-off live readout + telemetry | `hw/arm.py` sync read, `telemetry/` (poller, Rerun, TUI, parquet logger), `scripts/readonly/monitor.py --viz --tui` | Move each joint by hand, see ticks change; **record safe min/max ticks per joint** into `arm_follower.yaml`; leader by-hand sweep done here |
| 4 | Safety layer | `safety.py` SafeArm + pytest with MockBus (limit clamp, stall trip, Mode→Torque recheck) | Tests pass; review fault behavior together |
| 5 | First powered move | `motion.py` move_to + wait_until_settled, low speed cap; `jog.py` single joint ±small step | One joint at a time, arm supported, PSU switch in reach |
| 6 | Per-joint sweeps | `scripts/sweep.py` logging cmd/meas ticks, load, temp via `runlog/` | Smooth full-range sweeps; plots of tracking error and load per joint |
| 7 | Home calibration | `config/calibration/*_home.yaml` (our zero offsets; no EEPROM writes; dump registers before/after if LeRobot calibration is ever used) | Known pose → expected angles |
| 8 | FK | URDF (TheRobotStudio SO-ARM100 repo, SO-101) → `kinematics/model.py`, `fk.py`; tests vs URDF reference | FK tip position vs ruler/tape at a few poses (cm-level sanity) |
| 9 | IK | `ik.py` (DLS, joint-limit aware), `scripts/ik_demo.py` with a slow, previewed Cartesian move | Reaches marked targets on the table within a few mm by eye/ruler |
| 10 | Endurance & thermals | `scripts/thermal_loop.py` 15-min loop + short gripper squeeze, temp/voltage logged | No overheating; temps plotted; acceptance pass/fail summary |
| 11 | Phase 1 wrap-up | `docs/phase1_acceptance.md` + notes | Go/no-go before the return window closes |

Every motion step from 5 onward runs with the poller, live Rerun plots and the parquet logger active by default. From step 8, the Rerun 3D view shows the FK-predicted arm next to live telemetry.

Library choices: `feetech-servo-sdk`, `pyserial`, `numpy`, `scipy` (IK/least squares), `pyyaml`, `rerun-sdk` (live viz), `rich` (TUI), `pyarrow` (parquet), `matplotlib` (post-hoc figures), `pytest`. I'll use a small URDF parser such as `yourdfpy` or `urchin`, or hand-transcribe the joint origins. Our own FK keeps link parameters as explicit variables for Phase 2 calibration. Heavy dependencies like pinocchio are deliberately avoided.

## Handoff notes for future sessions (go into `docs/ROADMAP.md` + CLAUDE.md)
**Phase 2 will need, and Phase 1 must already provide:**
- `motion.wait_until_settled()` using both the position tolerance and the STS3215 "Moving" register, plus an `approach_dir` argument (needed for backlash tests).
- `Arm.snapshot()` that returns ticks, load, temp and voltage for all joints with a monotonic timestamp. The capture loop calls it before and after each image.
- A manifest whose camera fields are optional, so the Phase 2 capture loop extends it rather than replacing it.
- A parameterized FK. The kinematic fit adjusts link offsets, joint zero offsets and the target-to-wrist transform (6 DoF).
- Phase 1 thermal-loop data, kept as a baseline for Phase 2 drift.

**Phase 2 to-dos (not started in Phase 1):**
- Validate gphoto2 with the A7 IV (50 consecutive captures), with the Sony SDK as fallback behind the same `Sensor` interface.
- Camera settings checklist (spec §6).
- Generate and print the ChArUco targets and caliper-measure them.
- iPad intrinsics, then a mount-stability run, then a dry run (20 poses).
- Run order: repeatability → backlash → grid accuracy → per-joint sag → dynamics → thermal.
- Open items from spec §11.

**Phase 3 hooks:**
- `sensors/` already supports adding a BMI270 IMU (reusing serial code from `imu-diff-sim`) or a wrist camera.
- A stiffness-model compensation layer would sit between `kinematics` and `motion`, i.e. a corrected IK.
- For a policy/VLA route, an optional LeRobot adapter could wrap `SafeArm`. Phase 1 deliberately doesn't depend on LeRobot normalization.

## Verification
- `uv run pytest` runs the MockBus-based tests (safety trips, limit clamping, manifest 1:1 integrity, FK/IK round-trip `fk(ik(x)) ≈ x`). These run without hardware.
- Each Phase 1 step has a hardware checkpoint (table above). The user runs it and the outputs land in `data/raw/<run_id>/`.
- Telemetry tests: on MockBus, the poller hits its target rate, timestamps are monotonic, the parquet row count matches the poll count, and a slow subscriber (e.g. the viz) can't stall the safety watchdog or the logger. To get that, each subscriber has its own queue, and viz drops frames rather than blocking.
- Every run header records the git sha, package versions and config hashes, which a test checks.

## Working mode & permissions
- **Auto mode for everything except commands that can move the robot.** I proceed without asking for file edits, installs, tests, and read-only hardware scripts (port listing, register dump, torque-off readout).
- **Structural guard:** any script that can enable torque or command motion lives in `scripts/motion/` (jog, sweep, thermal_loop, ik_demo, gripper squeeze). Read-only scripts live in `scripts/readonly/`.
- In step 0 I add project `.claude/settings.json` **ask** rules for `uv run*scripts/motion/*` and `python*scripts/motion/*`. Those commands always prompt you, even in auto mode.
- In code, `SafeArm.enable_torque()` and motion entry points also require an explicit `--arm-moves` CLI flag. This is a second guard against accidentally running a motion script.
- You switch the session itself into auto mode with shift+tab; I can't change that for you.
- I'll save this preference to memory so future sessions follow it.

## First session after approval
Steps 0–2 only: bootstrap, port mapping, and the read-only register dump. I pause for your review after each one.
