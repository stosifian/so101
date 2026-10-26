# SO-101 Metrology Project — Plan & Spec

Context file for Claude Code. Describes the hardware, goals, conventions, and the
capture-loop spec for a quasi-static metrology characterization of an SO-101 arm.

---

## 1. Goal

Characterize how accurately a low-cost SO-101 follower arm positions its end effector,
and decompose the error into its sources (encoder/backlash, gravity sag, compliance,
thermal drift, settling). Phase 2 output is a self-contained write-up + dataset + tools.
Phase 3 is chosen *from* phase 2 results (see §3).

Guiding principles:
- **The arm is a measurement subject, not a demo.** No teleoperation required.
- **Every measurement traceable** to raw sensor values and a logged, scripted motion.
- **Characterize the instrument before the arm** (camera, targets, reference).
- **Precision vs. bias are tracked separately.**

---

## 2. Hardware

### Arm (Seeed SO-ARM101 Pro, pre-assembled)
- **Follower (white):** 6× Feetech STS3215, 1:345 gearing, **12 V**. Encoder: 12-bit, 4096 ticks/rev (~0.088°/tick).
- **Leader (black):** mixed gearing, **5 V**. Not used for phase 1–2 except a quick acceptance check.

  | Leader joint | Gear |
  |---|---|
  | 1 shoulder_pan | 1/191 |
  | 2 shoulder_lift | 1/345 |
  | 3 elbow_flex | 1/191 |
  | 4 wrist_flex | 1/147 |
  | 5 wrist_roll | 1/147 |
  | 6 gripper | 1/147 |

- **Never swap power supplies** (12 V on the leader can damage its motors). Arms/bricks are labeled.
- Base clamped with 2 bar clamps directly to bare table; re-tighten before each run.
- Host: MacBook Pro M1 Pro 16 GB, macOS. Powered USB hub.

### Measurement instruments
- **Primary camera:** Sony A7 IV (ILCE-7M4) + **35 mm prime** (do not use the 24-70 zoom).
  - Sensor 36 mm / 7008 px → ~5.1 µm pitch → **f ≈ 6800 px**. HFOV ≈ 54°.
  - On sturdy tripod with weighted legs.
- **Secondary (cross-check only):** kit 32×32 mm USB camera, ~90° from the A7 IV. Resolution TBD — verify.
- **Dial indicator** 0.01 mm + magnetic base on a steel plate clamped to the table (single-axis repeatability).
- **iPad:** screen-filling ChArUco for intrinsic calibration (square size computed exactly from display dims).
- **Fixed reference board** on the table near the workspace (foam-board print or AndyMark aluminum ChArUco).
- **BMI270 + microcontroller** (from imu-diff-sim) — optional, for link IMUs later.
- **Phone:** slow-motion video for latency/settling checks only. Not a metrology camera (AF/OIS/lens switching).

### The ELP stereo camera is NOT available (mounted in Shoggoth).

---

## 3. Phases

### Phase 1 — Bring-up & acceptance (within 30-day return window)
Pass/fail inspection. Exit criteria:
- All 12 servos respond with correct IDs and expected voltages (follower ~12 V, leader ~5 V).
- Every follower joint sweeps full range smoothly (no grinding/stalls).
- No overheating during a 15-min continuous loop (watch gripper servo; no long grip holds).
- Leader: by-hand sweep with position readout of all 6 servos, then set aside.

Steps:
1. Unbox/inspect/photograph; check joint-3 wiring and seated 3-pin cables.
2. `lerobot-find-port` for each arm; record mapping.
3. **Read-only** register dump of all servos (model, firmware, voltage, temp, position, Mode, Torque_Enable). Save as JSON.
4. Only if IDs are wrong: `lerobot-setup-motors` (requires per-motor connection — avoid on assembled arms).
5. If using LeRobot calibration: dump registers **before and after** (calibration may write offsets/limits to EEPROM).
6. Implement safety layer (§5) **before** first scripted move.
7. Acceptance tests on follower: slow per-joint sweeps logging commanded/measured ticks, load, temp; 15-min loop; short gripper squeeze.

### Phase 2 — Characterization (~3–4 weeks, time-boxed)
1. **Instrument characterization:** A7 IV intrinsics (iPad), target square sizes (calipers), iPad↔aluminum-board scale cross-check, mount stability (static target over 1–2 h), depth-axis check vs dial indicator.
2. **Repeatability:** ~30 returns to same pose; same vs opposite approach direction → **backlash**.
3. **Accuracy:** commanded vs measured over a workspace grid (needs kinematic model; add later).
4. **Gravity sag:** error vs pose, with/without payload. Measure **per joint** (vary one joint at a time) so joint stiffnesses can be fit.
5. **Dynamics:** settling time, overshoot, end-to-end latency.
6. **Thermal drift:** fixed pose over 1–2 h warm-up; log servo temps.

### Phase 3 — decided by phase 2 results
| Phase 2 finding | Phase 3 direction |
|---|---|
| Large arm error (mm-level, sag/backlash dominated) | Compensation study: model-only vs gripper IMU vs per-link IMUs vs wrist camera + fixed markers |
| Small, consistent arm error | Policy/perception project (VLA or sensing), using phase 2 data to relate arm accuracy to task success |
| Errors drift (thermal/wear) | Online calibration-drift monitoring |

Note: a single end-effector IMU measures only the **sum** of pitch-joint errors (shoulder/elbow/wrist pitch axes are parallel) and is blind to base yaw → cannot correct position alone. Repeatable sag can be compensated with a stiffness model (Δθ_i ≈ τ_gravity,i / k_i) without any sensor; a sensor is only justified for the non-repeatable part.

---

## 4. Conventions

- **Units:** encoder **raw ticks** (0–4095) at the hardware layer; degrees/radians only in analysis. Lengths in mm.
- **Bypass LeRobot normalization** for metrology; talk to servos via the Feetech SDK (or a thin wrapper). LeRobot is optional for bring-up only.
- **Joint-space commands first.** Cartesian/IK only for the accuracy grid, later.
- **Pin versions:** record LeRobot version/commit, Python version, camera firmware, gphoto2 version in the repo README and in every manifest header.
- **Clock:** host monotonic clock for all timing. Never use camera EXIF time for sync.
- **Unique IDs** for every ArUco/AprilTag across gripper, links, and reference board.
- **Raw data is immutable.** Analysis reads raw; never overwrite.

---

## 5. Safety layer (required before any scripted motion)

- Software joint limits tighter than mechanical stops; trajectories must avoid table, tripod, camera.
- Speed and acceleration caps via servo registers. Run every new trajectory once at low speed under observation.
- **Stall/fault check:** if load spikes or position stops tracking command → stop, disable torque.
- Gripper: no sustained high-force holds (overheating risk).
- 12 V supply switch within reach as E-stop. **Support the arm before disabling torque/power** (it drops).
- Re-check `Torque_Enable` after any write to the `Mode` register (on Shoggoth, writing Mode silently cleared Torque_Enable).

---

## 6. Camera control (A7 IV)

### Control path
1. **Primary: gphoto2 / python-gphoto2.** A7 IV support is community-maintained → validate first.
2. **Fallback: Sony Camera Remote SDK** (official, C++, lists ILCE-7M4 on macOS 14.1+). Wrap behind the same interface.

Keep camera access behind an interface (e.g. `Camera.capture(path) -> CaptureResult`) so backends are swappable; include a **mock camera** for dry runs.

### First test
```bash
brew install gphoto2
# Camera: USB Connection = PC Remote. On macOS the system photo daemon may claim the device:
killall ptpcamerad   # or PTPCamera on older macOS
gphoto2 --auto-detect
gphoto2 --capture-image-and-download --filename "test_%n.%C"
```
Accept gphoto2 if ~50 consecutive captures succeed without disconnect/hang. Record camera firmware version.

### Camera settings checklist (set once; photograph menus for the record)
- [ ] Manual exposure (M), fixed ISO (~100–400), fixed white balance
- [ ] Manual focus, focus ring taped. Recalibrate if anything moves.
- [ ] **IBIS OFF** (sensor shift moves the principal point)
- [ ] **Lens corrections OFF** (distortion/shading/CA) or shoot RAW and convert with no corrections
- [ ] Aperture f/11–f/16 (DoF ≈ ±5 cm at f/8, ≈ ±10 cm at f/16 @ 0.5 m; diffraction ~4 px at f/16 — acceptable)
- [ ] Shutter ≥ 1/60 s (multiple of 1/120 s) or flicker-free lighting — avoid LED flicker banding
- [ ] Electronic front curtain shutter
- [ ] Save to PC (+card backup optional), RAW, auto-review off, power save off
- [ ] USB-C / dummy-battery power for long runs; 20–30 min warm-up before drift tests

---

## 7. Capture loop spec (quasi-static)

Per pose:
1. **Command** joint targets (ticks) with speed/accel limits.
2. **Wait for motion complete:** position within tolerance of target AND servo "moving" status false, for N consecutive polls.
3. **Settle:** fixed dwell (start 2 s; shorten after phase 2 settling measurements). Table/tripod must settle too.
4. **Snapshot servo state BEFORE:** position, load, temperature, voltage — all joints.
5. **Trigger capture;** wait until file download completes. Record filename + host time at request and at completion.
6. **Snapshot servo state AFTER.** If any joint moved > threshold between before/after → mark sample **invalid** (creep during exposure).
7. Repeat capture **2–3× per pose** (spread = measurement noise at that pose).
8. Append one manifest row per image.

### Manifest schema (JSONL, one object per image)
```json
{
  "run_id": "str",
  "pose_id": "str",
  "repeat_idx": 0,
  "approach_dir": "pos|neg|na",
  "cmd_ticks": [0, 0, 0, 0, 0, 0],
  "before": {"ticks": [], "load": [], "temp_c": [], "voltage": []},
  "after":  {"ticks": [], "load": [], "temp_c": [], "voltage": []},
  "valid": true,
  "invalid_reason": null,
  "image_path": "str",
  "t_cmd": 0.0, "t_settled": 0.0, "t_capture_req": 0.0, "t_capture_done": 0.0,
  "camera_settings_hash": "str",
  "notes": ""
}
```
Run header (separate file per run): software versions, camera firmware, lens, aperture/shutter/ISO,
target IDs + **measured** square sizes, calibration file hashes, room temperature if available.

### Throughput estimate
~1–3 s capture+transfer per RAW + ~2 s settle → ~15 s/pose at 3 images → ~240 poses/h.

### Dry run (before any real data)
20 poses: zero missed captures, manifest↔file 1:1, stillness check never fires on a stationary clamped arm.

---

## 8. Targets

- **Gripper target:** ChArUco ~60 mm, 7×7 squares of ~8 mm (36 inner corners), on 2 mm aluminum (~19 g) or 3 mm acrylic (~13 g).
  - ChArUco preferred over AprilTag for precision: saddle-point corners are blur-invariant; AprilTag outer corners shift under blur.
  - Mount **rigidly** to wrist or fixed jaw (M3 screws / thin CA). No foam tape. Not the moving jaw.
  - Keep the same target for all tests; record its mass (it contributes to sag).
  - Target-to-wrist transform = 6 unknowns → estimate in kinematic calibration, don't hand-measure.
  - Grazing angles degrade pose → keep target within ~45–60° of camera axis, or add a second face (L-bracket) with distinct IDs; calibrate multi-face geometry by bundle adjustment.
- **If AprilTags:** 2×2 grid of ~25 mm tag36h11 per face (sized for the low-res USB camera, ≥4–5 px/cell).
- **Optional link markers:** small AprilTags on upper arm/forearm (on glass slides epoxied to brackets) → visual joint-angle measurement.
- **Fixed reference board** near workspace depth: defines world frame, cancels tripod drift, monitors stability.
- **Printing:** generate in OpenCV with the same dictionary/layout as detection; vector PDF at exact mm; 100% scale, matte paper, 600–1200 dpi laser; thin spray adhesive. **Measure square size with calipers and use measured values.**

---

## 9. Measurement math (reference)

- Lateral: σ_lat ≈ (Z/f)·σ_px → 0.5 m / 6800 px × 0.1 px ≈ **7 µm**
- Depth (monocular PnP on known target): ΔZ ≈ (Z²/(f·L))·Δs → Z=0.5 m, L=60 mm, Δs≈0.05 px → **~0.03 mm**
  - 1 mm depth change ≈ 1.6 px change in target size (816.0 → 817.6 px).
- **Depth bias:** depth ∝ calibrated f and target size → 0.1% scale error ≈ 0.5 mm at 0.5 m.
  - Mitigate: measure relative to the fixed reference board at similar depth (residual ≈ 0.1% × separation), iPad↔board cross-check, dial-indicator check along the camera axis.
- **Test accuracy ratio:** instrument ≥ 4× better than the quantity measured (arm errors ~1 mm → need ≤ 0.25 mm per axis).
- Orient camera so the effect of interest is **lateral** in the image when possible (e.g. horizontal view for vertical sag).

---

## 10. Suggested code layout

```
so101_metrology/
  hw/
    servo_bus.py        # Feetech SDK wrapper: raw ticks, register read/write, dumps
    safety.py           # limits, speed caps, stall detection, torque-off
  camera/
    base.py             # Camera interface + CaptureResult
    gphoto2_cam.py      # primary backend
    sony_sdk_cam.py     # fallback backend (stub until needed)
    mock_cam.py         # dry runs / CI
  capture/
    loop.py             # move -> settle -> verify -> capture -> verify -> log
    manifest.py         # JSONL writer, run header
    poses.py            # pose sets: repeatability, backlash, per-joint sag, grid
  analysis/
    detect.py           # ChArUco/AprilTag detection, PnP
    calib.py            # intrinsics, target verification, kinematic fit
    reports.py          # figures/tables for the write-up
  tests/                # unit tests incl. safety + manifest integrity (mock hw)
data/raw/<run_id>/      # immutable: images + manifest + header
```

---

## 11. Verify before trusting (open items)

- [ ] gphoto2 reliably drives the A7 IV on current firmware (else Sony SDK)
- [ ] Kit USB camera resolution/focal length (affects tag sizing)
- [ ] Whether the small white printed part in the kit is a wrist-camera mount
- [ ] Whether LeRobot calibration writes EEPROM offsets in the pinned version
- [ ] Clamp count (kit lists 4) and table stiffness
- [ ] SO-101 URDF availability (TheRobotStudio SO-ARM100 repo) for the accuracy grid

---

## References
- LeRobot SO-101 docs: https://huggingface.co/docs/lerobot/so101.md
- Sony Camera Remote SDK: https://support.d-imaging.sony.co.jp/app/sdk/en/index.html
- OpenCV calibration patterns: https://docs.opencv.org/5.0/tutorials/calib3d/camera_calibration_pattern/camera_calibration_pattern.html
- calib.io pattern generator: https://calib.io/pages/camera-calibration-pattern-generator
