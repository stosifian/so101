# SO-101 metrology

Software to measure how accurately an SO-101 arm goes where it's told, using a camera you probably already own,
an iPad as the reference, and a printed AprilTag. Results and the story are in
[`docs/writeup_draft.md`](docs/writeup_draft.md).

This README is for repeating the experiment on your own arm. Everything here was built for one follower arm, one
Sony A7 IV and one iPad Air, so expect to redo the per-device steps (ports, limits, joint mapping, intrinsics,
tours) for yours.

## What you need

| | Used here | Notes |
|---|---|---|
| Arm | SO-101 follower, 6 × Feetech STS3215, 12 V | Never swap the follower (12 V) and leader (5 V) supplies |
| Computer | macOS, Apple silicon | Linux should work; port names and `ptpcamerad` notes are macOS-specific |
| Camera | Sony A7 IV + 35 mm lens, tethered over USB | Any camera `gphoto2` can drive and that shoots RAW |
| Reference | iPad Air (2360 × 1640) showing a ChArUco board | Other screens need a new board config (below) |
| Target | 20 mm AprilTag 36h11, printed, stiff backing | Taped flat to the gripper |
| Software | [`uv`](https://docs.astral.sh/uv/), `gphoto2` (`brew install gphoto2`) | |

## Install

```bash
uv sync                 # creates .venv (Python 3.12) from uv.lock
uv run pytest           # hardware-free tests
```

Every script prints its full usage with `--help`; the docstring at the top of each file has examples.

## Safety model (read this first)

- `scripts/readonly/` never writes torque or goal registers. Safe with the arm powered.
- `scripts/motion/` can move the arm and refuses to run without `--arm-moves`. Most accept `--sim` to run against
  a simulated bus first. Support the arm and keep the 12 V switch in reach.
- All motion goes through `so101.safety.SafeArm`: before torque-on it sets every goal to the current position and
  caps speed and acceleration (the servos power up with goal 0 at unlimited speed), enforces software joint limits,
  trips on voltage/temperature/contact, and checks planned paths against the table in MuJoCo.
- Ctrl-C once = freeze in place; twice = torque off (the arm drops, so support it).
- No EEPROM writes anywhere. Limits live in config, not in the servos.

## Doing it on your arm

Runs are written to `data/raw/<run_id>/` (never modified) with a header of git sha, versions and config hashes;
analysis goes to `data/derived/`. Both are git-ignored.

### 1. Bring up the arm

```bash
uv run scripts/readonly/find_port.py --probe          # find the follower's serial port
uv run scripts/readonly/dump_registers.py --arm follower
uv run scripts/readonly/live_readout.py --tui --arm follower --record-limits   # move the arm by hand, torque off
```

- Put the port in `config/arm_follower.yaml`, and paste the proposed limits there after reviewing them.
- Map ticks to joint angles: `uv run scripts/readonly/model_check.py` shows the MuJoCo model posed from your arm
  in Rerun. Line the arm up with the model's reference pose and edit `config/calibration/follower_mapping.yaml`.
- First motion: `scripts/motion/single_joint_move.py --arm-moves --sim`, then without `--sim`, then
  `scripts/motion/pose_tour.py`.

### 2. Set up the camera and reference

```bash
uv run scripts/camera/make_ipad_charuco.py            # pixel-exact board PNG for the iPad
uv run scripts/camera/tether_check.py -n 50           # capture reliability over USB
```

- Show the PNG full screen in Photos, 1:1 (no zoom). For a different screen, edit `config/targets/ipad_charuco.yaml`.
- Camera: manual everything, RAW, focus set at the working distance and taped. The settings used are in
  `config/camera_a7iv.yaml`. Our code reads camera settings but never changes them.
- On macOS, `killall ptpcamerad` after plugging in the camera.

### 3. Calibrate the camera

```bash
uv run scripts/camera/calibrate_intrinsics.py capture
uv run scripts/camera/calibrate_intrinsics.py analyze data/raw/<run_id>
```

~30–40 views of the iPad board across the whole frame, tilted up to ~40°. Aim for ≲ 0.2 px reprojection RMS.
`analyze` writes `data/derived/<run_id>/intrinsics.json`; copy it to `config/intrinsics_a7iv.json`, which every
pose script reads. It's only valid at that focus setting.

### 4. Check the measurement floor

```bash
uv run scripts/camera/stability_check.py --board apriltag --tag-ids 2 -n 21 --interval 90   # 30 min, arm at rest
uv run scripts/camera/target_pose.py data/raw/<run_id>
```

This is the tag's pose relative to the iPad over time. Expect µm-level noise across the image and ~10× worse in
depth. If it drifts, fix the rig before measuring the arm.

### 5. Plan a workspace and capture

```bash
uv run scripts/readonly/teach_poses.py                         # pose the unpowered arm at the extremes
uv run scripts/sim/plan_sector_tour.py data/raw/<teach_run>    # grid of poses the camera can see -> config/tours/
uv run scripts/motion/capture_tour.py --arm-moves --sim --yes  # dry run
uv run scripts/motion/capture_tour.py --arm-moves --tour config/tours/<tour>.yaml
```

At each waypoint the tour settles, records joint readings, takes a photo, and records the readings again.
`scripts/camera/handeye_fit.py` fits where the camera and tag are relative to the arm; the planner uses it to
turn the tag toward the camera.

### 6. Analyse

| Question | Plan the tour | Analyse |
|---|---|---|
| Repeatability, backlash | `scripts/sim/plan_step8_tour.py` | `scripts/camera/repeatability.py` |
| Model accuracy (calibration) | step 5 grid, plus a separate test run | `scripts/camera/kinematic_fit.py <runs> --variants nominal,links,stiff_lin --test <test_run> --save` |
| Commanding accuracy | `scripts/camera/servo_droop_fit.py <runs>`, then `scripts/sim/plan_step10b_tour.py` | `scripts/camera/command_accuracy.py` |
| Hold drift, thermal | `config/tours/step11_*.yaml` | `scripts/camera/thermal_drift.py` |

If the iPad or arm base moves between sessions, `scripts/camera/reanchor.py` refits only the iPad ← base transform.
Re-anchor on poses where the model fits well.

## Where things live

| Path | What |
|---|---|
| `src/so101/hw/` | Servo bus (Feetech SDK), register map, simulated bus |
| `src/so101/safety/` | `SafeArm`, trips, contact detection |
| `src/so101/kinematics/` | MuJoCo model, FK/IK, collision checks, calibrated model loading |
| `src/so101/motion/` | Tours, settling, calibrated IK (gravity bend + servo droop feedforward) |
| `src/so101/analysis/` | AprilTag/ChArUco detection, PnP, intrinsics, hand-eye, kinematic calibration |
| `src/so101/telemetry/` | Parquet logging, TUI, Rerun live plots |
| `config/` | Per-arm, per-camera and per-experiment settings, tours, fitted models |
| `models/so101_menagerie/` | MuJoCo SO-101 model |
| `docs/` | Write-up, math notes (`pnp_math.md`, `calibration_math.md`, `fk_math.md`, `ik_math.md`), plans |

## Pinned versions

Exact versions are in `uv.lock`, and every run header records them. Baseline at bootstrap:

| Component | Version |
|-----------|---------|
| Python | 3.12.15 (arm64) |
| uv | 0.13.5 |
